#include "mcal/cmsis_i2c.h"
#include "mcal/cmsis_uart.h"
#include <cstdint>
#include <cstdio>

namespace mcal {
  void CMSIS_I2C::init() {
    disable_peripheral();

    en_clks();
    setup_pins();
    init_dma();
    
    en_peripheral();
    wait_to_stabilize();
  }

  void CMSIS_I2C::en_clks() {
    // enable GPIOB, DMA1 and I2C1 clks to power peripherals
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;       // RM0090: 6.3.13: bit 21
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;       // RM0090: 6.3.10: bit 21
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;      // RM0090: 6.3.10: bit 1 
    reset();
  }

  void CMSIS_I2C::setup_pins() {
    // set pins PB6/7 to AF (alt func) mode
    // RM0090: 8.4.1 
    GPIOB->MODER &= ~((3 << 12) | (3 << 14)); // clear target bits
    GPIOB->MODER |=   (2 << 12) | (2 << 14);  // 0b10 sets pins to AF mode
  
    // specify pin AFs to be AF4 (i2c1)
    // RM0090: 8.4.9: AFRL now called AFR[0]
    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // clear PB 6/7 AF 
    GPIOB->AFR[0] |=   (0x4 << 24) | (0x4 << 28);  // set each pin to AF4

    // RM0090: 8.4.2: set pb6/7 as output open-drain pins
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);

    // SW enabled pullups for pb6/7, corresponding with i2c standard
    GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
    GPIOB->PUPDR |= (1 << 12) | (1 << 14);
  
    // RM0090: 27.6.1/2: bit 0 to disable peripheral
    I2C1->CR1  =  0x8000; // software reset i2c1
    I2C1->CR1 &= ~0x8000; // out of reset
    I2C1->CR2 = 16;       // APB1 frequency (MHz) from HSI
    
    set_fast_mode();
  }

  void CMSIS_I2C::init_dma() {
    DMA1_Stream0->CR &= ~(1 << 0);                // disable DMA stream
    while(DMA1_Stream0->CR & (1 << 0));           // wait until stream fully disabled
  
    DMA1->LIFCR = 0x3F;                           // low interrupt flag clear register
                                                  // TC: transfer complete
                                                  // HT: half transfer (not used)
                                                  // DME: direct mode error - FIFO issue
                                                  // FE: FIFO error - overflow/underflow
    
    DMA1_Stream0->CR = (1 << 25) |                // RM090: 10.5.5 - select stream channel
                       (1 << 10) |                // set memory increment (MINC) mode
                       (2 << 16);                 // set priority level to HIGH (0b10)
  
    DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;      // set peripheral memory (source) addr
    I2C1->CR2 |= 0x0400;                          // enable I2C DMA 
  }

  bool CMSIS_I2C::read_burst(std::uint8_t dev_addr,
                             std::uint8_t reg_addr,
                             std::uint8_t* buffer,
                             std::uint16_t bytes) {
    if(bytes == 0) return false;
    volatile int tmp;

    // ----------- write phase: send reg addr ------------//
    send_start();

    I2C1->DR = (dev_addr << 1) | 0;               // transmit imu addr (dir bit set to write)
    while(!(I2C1->SR1 & 2));                      // wait until addr flag is set
    tmp = I2C1->SR1;                              // clear addr flag
    tmp = I2C1->SR2;                              // clear addr flag
    while(!(I2C1->SR1 & 0x80));                   // wait until data register is empty

    I2C1->DR = reg_addr;
    while(!(I2C1->SR1 & 0x80));                   // wait until data register is empty
    // ----------- write phase: send reg addr ------------//

    // ----------- read phase: repeated start -------------//
    I2C1->CR1 |= 0x100;                           // set to repeated start generation
    while(!(I2C1->SR1 & 1));                      // wait until start flag is set
    I2C1->DR = (dev_addr << 1) | 1;               // transmit imu addr (dir bit set to read)
    while(!(I2C1->SR1 & 2));                      // wait until addr flag is set
    // ----------- read phase: repeated start -------------//

    // TODO: replace with DMA
    // ----------- cpu-driven byte read ----------------------//
    I2C1->CR1 |= (1 << 10);                       // enable ACK
    tmp = I2C1->SR1;                              // clear addr flag
    tmp = I2C1->SR2;                              // clear addr flag

    for(uint16_t i = 0; i < bytes; i++) {
      if(i == bytes - 1) {
        I2C1->CR1 &= ~(1 << 10);                  // disable ACK for last byte
        I2C1->CR1 |= 0x200;                       // generate STOP
      }

      while(!(I2C1->SR1 & 0x40));                 // wait for RxNE
      buffer[i] = I2C1->DR;                       // read byte
    }
    // ----------- cpu-driven byte read ----------------------//

    return true;
  }

  std::uint8_t CMSIS_I2C::read_reg(std::uint8_t dev_addr,
                                   std::uint8_t reg_addr) {
    volatile int tmp; 

    send_start();

    I2C1->DR = (dev_addr << 1) | 0;  // transmit imu addr (with dir bit set to write)
    while(!(I2C1->SR1 & 2));         // wait until addr flag is set 
    tmp = I2C1->SR2;                 // clear addr flag
    while(!(I2C1->SR1 & 0x80));      // wait until data register is empty
    
    I2C1->DR = reg_addr;
    while(!(I2C1->SR1 & 0x80));      // wait until data register is empty
  
    I2C1->CR1 |= 0x100;              // generate restart
    while(!(I2C1->SR1 & 1));         // wait until start flag is set
  
    I2C1->DR = (dev_addr << 1) | 1;  // transmit imu addr (with dir bit set to read)
    while(!(I2C1->SR1 & 2));         // wait until addr flag is set 
  
    I2C1->CR1 &= ~0x400;             // disable ACKs
    tmp = I2C1->SR2;                 // clear addr flag
  
    send_stop();

    while(!(I2C1->SR1 & 0x40));      // wait for RXnE (receive buffer not empty) flag
    return I2C1->DR;
  }


  void CMSIS_I2C::write_reg(std::uint8_t dev_addr,
                            std::uint8_t reg_addr,
                            std::uint8_t value) {
    volatile int tmp; 

    send_start();

    I2C1->DR = (dev_addr << 1) | 0;  // transmit imu addr
    while(!(I2C1->SR1 & 2));         // wait until addr flag is set 
    tmp = I2C1->SR1;                 // clear addr flag
    tmp = I2C1->SR2;                 // clear addr flag
    while(!(I2C1->SR1 & 0x80));      // wait until data register is empty

    I2C1->DR = reg_addr;
    while(!(I2C1->SR1 & 0x80));      // wait until data register is empty

    I2C1->DR = value;                // write data
    while(!(I2C1->SR1 & 4));         // wait until transfer is completed

    send_stop();
  }


  void CMSIS_I2C::set_fast_mode() {
    // TODO: review this math

    /*
    TIMINGR: controls bus timing characteristics
      I2C1->TIMINGR = 0x00503D5A;

    ex: how long clock high/low periods last, setup/hold times, etc.

    TIMINGR bit fields (32-bit register):
      PRESC  [31:28] : prescaler (divides APB1 clock)
      SCLDEL [23:20] : data setup time
      SDADEL [19:16] : data hold time
      SCLH    [15:8] : SCL high period
      SCLL     [7:0] : SCL low period

    breaking down 0x00503D5A:
      PRESC  = 0x0 (bits 31-28): no prescaling
      SCLDEL = 0x5 (bits 23-20): setup time
      SDADEL = 0x0 (bits 19-16): hold time
      SCLH   = 0x3D (bits 15-8): SCL high = 61 cycles
      SCLL   = 0x5A (bits 7-0) : SCL low = 90 cycles

    example:
      target: 400kHz I2C from 16MHz APB1
      period: 16MHz / 400kHz = 40 APB1 cycles per I2C cycle
      setup/hold times: Meet I2C fast-mode specs 
    */ 

    // fast mode (bit 15) + (16Hz / (3 × 400Khz) ≈ 13) 
    I2C1->CCR = (1 << 15) | 13;   

    /*
    *
    * TRISE = (max_rise_time * APB1_freq) + 1
    *       = (300ns x 16MHz) + 1 = 5.8
    *
    */
    I2C1->TRISE = 6;
    
  }


//  void CMSIS_I2C::init_dma() {
//    //TODO: review all logic 
//    DMA1_Stream0->CR &= ~(1 << 0);                 // disable stream first
//    while(DMA1_Stream0->CR & (1 << 0));            // wait until disabled
//
//    DMA1->LIFCR |= (0x3F << 0);                    // clear all Stream0 flags
//  
//    // Configure DMA1 Stream0 Channel 1 for I2C1_RX
//    DMA1_Stream0->CR = 0;                           // Reset control register
//    DMA1_Stream0->CR |= (1 << 25);                  // Channel 1 (I2C1_RX)
//    DMA1_Stream0->CR |= (0 << 6);                   // Peripheral-to-memory
//    DMA1_Stream0->CR |= (1 << 10);                  // Memory increment
//    DMA1_Stream0->CR |= (0 << 11);                  // Peripheral no increment
//    DMA1_Stream0->CR |= (0 << 13);                  // Memory data size: byte
//    DMA1_Stream0->CR |= (0 << 11);                  // Peripheral data size: byte
//    DMA1_Stream0->CR |= (1 << 4);                   // Transfer complete interrupt
//    DMA1_Stream0->CR |= (2 << 16);                  // High priority
//
//    // Set peripheral address (I2C1 data register)
//    DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;
//
//    // Use direct mode for I2C (FIFO disabled)
//    DMA1_Stream0->FCR = 0;  // Direct mode enabled (default)
//
//    // Enable DMA interrupt in NVIC
//    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
//    NVIC_SetPriority(DMA1_Stream0_IRQn, 1);
//  }
  
  void CMSIS_I2C::disable_peripheral() {
    I2C1->CR1 &= ~(1 << 0);
  }

  void CMSIS_I2C::en_peripheral() {
    I2C1->CR1 |= 0x0001;
  }

  void CMSIS_I2C::reset() {
    //RM0090: 27.6.1: SWRST
    I2C1->CR1 |= (1 << 15);
    wait_to_stabilize();
    I2C1->CR1 &= ~(1 << 15);
  }

  void CMSIS_I2C::wait_to_stabilize() {
    for(volatile std::uint32_t i = 0; i < 1000; i++); 
  }

  void CMSIS_I2C::send_start() {
    while(I2C1->SR2 & 2);   // wait until bus not busy
    I2C1->CR1 |= (1 << 8);  // generate start flag
  
    while(!(I2C1->SR1 & 1)); // wait until start flag is set
  }

  void CMSIS_I2C::send_stop() {
    I2C1->CR1 |= (1 << 9);  
  }
  
  bool CMSIS_I2C::wait_flag(std::uint32_t flag) {
    while(!(I2C1->SR1 & flag));

    return true;
  }
  
  void CMSIS_I2C::send_addr(std::uint8_t addr, bool read) {
    I2C1->DR = (addr << 1) | read;
    wait_flag(1 << 1);

    // clear addr by reading SR1 and then SR2. TODO: review why
    (void)I2C1->SR1;
    (void)I2C1->SR2;
  }
}

