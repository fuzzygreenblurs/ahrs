#include "cmsis_i2c.h"

namespace mcal {
  void CMSIS_I2C::init() {
    disable_peripheral()

    en_clks();
    reset();
    setup_pins();
    set_fast_mode();
    init_dma();
    
    en_peripheral();
    wait_to_stabilize();
  }

  void en_clks() {
    // enable GPIOB, DMA1 and I2C1 clks to power peripherals
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;       // RM0090: 6.3.13: bit 21  
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;       // RM0090: 6.3.10: bit 21
    RCC->APB1ENR |= RCC_AHB1ENR_GPIOBEN;      // RM0090: 6.3.10: bit 1 
  }

  void setup_pins() {
    // set pins pb6/7 to AF (alt func) mode
    // RM0090: 8.4.1 
    GPIOB->MODER &= ~((3 << 12) | (3 << 14)); // clear target bits
    GPIOB->MODER |= (2 << 12) | (2 << 14);    // 0b10 sets pins to AF mode
  
    // specify pin AFs to be AF4 (i2c1)
    // RM0090: 8.4.9: AFRL now called AFR[0]
    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // clear pb6/7 AF 
    GPIOB->AFR[0] |= (0x4 << 24) | (0x4 << 28);    // set each pin to AF4

    // RM0090: 8.4.2: set pb6/7 as output open-drain pins
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);

    // SW enabled pullups for pb6/7, corresponding with i2c standard
    GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
    GPIOB->PUPDR |= (1 << 12) | (1 << 14);
  
    // RM0090: 27.6.1/2: bit 0 to disable peripheral
    I2C1->CR1 = 0;
    I2C1->CR2 = 42;
    
  }

  void set_fast_mode() {
    // TODO: review this math

    /*
    TIMINGR: controls bus timing characteristics
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
      target: 400kHz I2C from 42MHz APB1
      period: 42MHz / 400kHz = 105 APB1 cycles per I2C cycle
      split: ~61 cycles high + 90 cycles low = 151 total (includes overhead)
      setup/hold times: Meet I2C fast-mode specs 
  */
  
  // fast mode (bit 15) + 400kHz (42MHz / (3 × 35) ≈ 400kHz)
  I2C1->CCR = (1 << 15) | 35;   
  I2C1->TRISE = 14;
  
  // I2C1->TIMINGR = 0x00503D5A;
  }

  void init_dma() {

    DMA1_Stream0->CR &= ~(1 << 0);                 // disable stream first
    while(DMA1_Stream0->CR & (1 << 0));            // wait until disabled

    DMA1->LIFCR |= (0x3F << 0);                    // clear all Stream0 flags
  
    // Configure DMA1 Stream0 Channel 1 for I2C1_RX
    DMA1_Stream0->CR = 0;                           // Reset control register
    DMA1_Stream0->CR |= (1 << 25);                  // Channel 1 (I2C1_RX)
    DMA1_Stream0->CR |= (0 << 6);                   // Peripheral-to-memory
    DMA1_Stream0->CR |= (1 << 10);                  // Memory increment
    DMA1_Stream0->CR |= (0 << 11);                  // Peripheral no increment
    DMA1_Stream0->CR |= (0 << 13);                  // Memory data size: byte
    DMA1_Stream0->CR |= (0 << 11);                  // Peripheral data size: byte
    DMA1_Stream0->CR |= (1 << 4);                   // Transfer complete interrupt
    DMA1_Stream0->CR |= (2 << 16);                  // High priority

    // Set peripheral address (I2C1 data register)
    DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;

    // Use direct mode for I2C (FIFO disabled)
    DMA1_Stream0->FCR = 0;  // Direct mode enabled (default)

    // Enable DMA interrupt in NVIC
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream0_IRQn, 1);
  }
  

  void CMSIS_I2C::disable_peripheral() {
    I2C1->CR1 &= ~(1 << 0);
  }

  void CMSIS_I2C::en_peripheral() {
    I2C1->CR1 |= (1 << 0);
  }

  void reset() {
    //RM0090: 27.6.1: SWRST
    I2C->CR1 |= (1 << 15);
    wait_to_stabilize();
    I2C->CR1 &= ~(1 << 15);
  }

  void wait_to_stabilize() {
    for(volatile std::uint32_t i = 0; i < 1000; i++); 
  }

  void send_start() {
    uint32_t timeout = 10000;

    I2C1->CR1 |= (1 << 8); // send start bit
    while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) timeout--;
    if(timeout == 0) return;
  }

  void send_address()

}

