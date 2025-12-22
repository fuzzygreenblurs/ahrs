#ifndef MPU9250_H
#define MPU9250_H

#include <cstdint>
#include <Eigen/Core>
#include "interfaces/i_imu.h"
#include "interfaces/i_i2c.h"

namespace drivers {
  class MPU9250 : public AHRS::IIMU {
    public:
      explicit MPU9250(AHRS::II2C& i2c_interface);
      ~MPU9250() override = default;

      bool init();
      bool calibrate_gyro(std::uint16_t samples = 1000);
      bool calibrate_accel(std::uint16_t samples = 1000);

      // IIMU interface implementation
      Eigen::Vector3f read_gyro() override;
      Eigen::Vector3f read_accel() override;
      Eigen::Vector3f read_mag() override { return Eigen::Vector3f::Zero(); }
      float get_sample_rate() const override { return sample_rate_; }
      bool has_mag() const override { return false; }

      // configuration
      void set_gyro_range(std::uint8_t range);   
      void set_accel_range(std::uint8_t range); 
      void set_sample_rate(std::uint16_t rate);

    private:
      // I2C interface reference
      AHRS::II2C& i2c_;

      // Device addresses
      static constexpr std::uint8_t MPU9250_ADDR = 0x68;
      static constexpr std::uint8_t AK8963_ADDR = 0x0C;

      // MPU9250 Registers
      static constexpr std::uint8_t WHO_AM_I = 0x75;
      static constexpr std::uint8_t PWR_MGMT_1 = 0x6B;
      static constexpr std::uint8_t PWR_MGMT_2 = 0x6C;
      static constexpr std::uint8_t CONFIG = 0x1A;
      static constexpr std::uint8_t GYRO_CONFIG = 0x1B;
      static constexpr std::uint8_t ACCEL_CONFIG = 0x1C;
      static constexpr std::uint8_t ACCEL_CONFIG2 = 0x1D;
      static constexpr std::uint8_t SMPLRT_DIV = 0x19;
      static constexpr std::uint8_t INT_PIN_CFG = 0x37;
      static constexpr std::uint8_t INT_ENABLE = 0x38;
      static constexpr std::uint8_t ACCEL_XOUT_H = 0x3B;
      static constexpr std::uint8_t GYRO_XOUT_H = 0x43;

      // AK8963 Magnetometer Registers
      static constexpr std::uint8_t AK8963_WHO_AM_I = 0x00;
      static constexpr std::uint8_t AK8963_CNTL1 = 0x0A;
      static constexpr std::uint8_t AK8963_CNTL2 = 0x0B;
      static constexpr std::uint8_t AK8963_ST1 = 0x02;
      static constexpr std::uint8_t AK8963_HXL = 0x03;
      static constexpr std::uint8_t AK8963_ST2 = 0x09;

      Eigen::Vector3f gyro_bias_{0.0f, 0.0f, 0.0f};
      Eigen::Vector3f accel_bias_{0.0f, 0.0f, 0.0f};

      float gyro_scale_  = 131.0f;    // default 250 dps
      float accel_scale_ = 16384.0f;  // default 2g

      float sample_rate_ = 1000.0f;   // default 1kHz

      bool verify_device();
      void read_raw_accel_gyro(std::int16_t* data);
      bool read_raw_mag(std::int16_t* mx, std::int16_t* my, std::int16_t* mz);
  };

}

#endif
