#ifndef CMSIS_I2C_H
#define CMSIS_I2C_H

#include "interfaces/i_i2c.h"
#include "stm32f4xx.h"

namespace mcal {
  class CMSIS_I2C : public AHRS::II2C {
    public:
      CMSIS_I2C() = default;

      void init();

      std::uint8_t read_reg(std::uint8_t dev_addr,
                            std::uint8_t reg_addr) override;

      void write_reg(std::uint8_t dev_addr,
                             std::uint8_t reg_addr,
                             std::uint8_t value) override;

      bool read_burst(std::uint8_t dev_addr,
                              std::uint8_t reg_addr, 
                              std::uint8_t* buffer,
                              std::uint16_t length) override;

    private:
      void disable_peripheral();
      void en_peripheral();
      void init_dma();
      void en_clks();
      void setup_pins();
      void set_fast_mode();
      void reset();
      void wait_to_stabilize();
      
      void send_start();
      void send_addr(std::uint8_t addr, bool read);
      void send_stop();
      bool wait_flag(std::uint32_t flag);
  };
}

#endif
