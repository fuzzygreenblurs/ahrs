#include "mcal/cmsis_uart.h"
#include "stm32f4xx.h"
#include <cstring>

namespace mcal {
  void CMSIS_UART::init() {
    // TESTING: Use attitude_estimator's exact init sequence
    RCC->AHB1ENR |= 1;        // powers GPIOA port peripheral
    RCC->APB1ENR |= 0x20000;  // powers USART2 peripheral

    GPIOA->AFR[0] &= ~0x0F00;
    GPIOA->AFR[0] |=  0x0700; // set AF for PA2 to AF7 (USART2_TX only)

    GPIOA->MODER  &= ~0x0030;
    GPIOA->MODER  |=  0x0020; // set PA2 to AF mode 
  
                              
  /*
   * PCL1:     cycles/sec
   * baud:     bits/sec
   * USARTDIV: cycles/sample
   *
   * in order to sync the sender/receiver using async clocks, the receiver might attempt to oversample
   * the same bit. for example, by sampling 16 times per bit, the receiver will simply select the "middle"
   * sample. thus, the sender stm32 will enforce the correct baud rate to a sufficiently high tolerance, 
   * based on the duration of a single sample (for a 16 sample/bit rate)
   *
   * This can be calculated by:
   * cycles/bit = PCL1 / baud
   * cycles/sample = (cycles/bit) / (samples/bit) -> this is USARTDIV
   *
   * USARTDIV = (16MHz / 115200) / (16) = 8.681 cycles/sample
   *
   * BRR (RM0090: 30.6.3):
   *  mantissa (integer component) = floor(USARTDIV) = 8
   *  fraction                     = (USARTDIV - mantissa) x 16
   *                               = 0.681 * 16 = 10.8
   *                               ~ 11
   *
   * BRR = (mantissa << 4) | fraction
   *     = (8 << 4) | 11
   *     = 0x0x008B
   *
   * ---------------------------------------------------------
   * before configure_sysclk() [before startup]:
   *     - SYSCLK = HSI
   *     - HSI is a 16MHz clk (see clk_config for details)
   * after configure_sysclk():
   *     - PLL enabled
   *     - SYSCLK = PLL = 168MHz
   * ---------------------------------------------------------
   *
   */

    USART2->BRR  = 0x008B;              
    USART2->CR1  = 0x0008;           // enable tx, 8-bit data
    USART2->CR2  = 0x0000;           // 1 stop bit
    USART2->CR3  = 0x0000;           // no flow control
    USART2->CR1 |= 0x2000;           // enable USART2
  }

  void CMSIS_UART::blocking_transmit(const char* str) {
    while(*str) {
      transmit_byte(*str++);
    }

    wait_tx_complete();
  }

  void CMSIS_UART::transmit_byte(uint8_t byte) {
    while(!(USART2->SR & 0x0080));  // Wait for TX empty
    USART2->DR = (byte & 0xFF);
  } 

  void CMSIS_UART::wait_tx_complete() {
    // for(volatile uint32_t i = 0; i < 2000000; i++);
    while(!(USART2->SR & 0x0040));  // wait for TC flag (bit 6)
  }
}
