#ifndef CMSIS_UART_H
#define CMSIS_UART_H

#include <cstdint>

namespace mcal {
  class CMSIS_UART {
    public:
      void init();
      void blocking_transmit(const char* str);
      void dma_transmit(const char* str, uint16_t len);
      bool is_tx_busy() const;

    private:
      void en_clks();
      void configure_pins();
      void set_baud(uint32_t baud);
      void enable_uart();

      void init_dma_tx();
      void transmit_byte(uint8_t byte);
      void wait_tx_complete();
  };
}

#endif
