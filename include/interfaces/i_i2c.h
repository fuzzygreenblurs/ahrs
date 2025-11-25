#ifndef I_I2C_H
#define I_I2C_H


namespace AHRS {
  class II2C {
    public:
      virtual ~II2C() = default;
      virtual std::uint8_t read_reg(std::uint8_t dev_addr,
                                    std::uint8_t reg_addr) = 0;

      virtual void write_reg(std::uint8_t dev_addr,
                             std::uint8_t reg_addr,
                             std::uint8_t value) = 0;

      virtual bool read_burst(std::uint8_t dev_addr,
                              std::uint8_t reg_addr, 
                              std::uint8_t* buffer,
                              std::uint16_t length) = 0; 
  };
}


#endif
