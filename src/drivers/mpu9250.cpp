#include "drivers/mpu9250.h"

namespace drivers {

MPU9250::MPU9250(AHRS::II2C& i2c_interface)
  : i2c_(i2c_interface) {
}

bool MPU9250::init() {
  // verify device id 
  if (!verify_device()) {
    return false;
  }

  // give MPU9250 time to power up (100ms)
  for(volatile std::uint32_t i = 0; i < 2000000; i++);

  // wake up MPU9250 (clear sleep bit)
  i2c_.write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x00);
  for(volatile std::uint32_t i = 0; i < 200000; i++);

  // set DLPF 
  i2c_.write_reg(MPU9250_ADDR, CONFIG, 0x03);

  // set sample rate to 500Hz (1kHz / (1 + 1))
  i2c_.write_reg(MPU9250_ADDR, SMPLRT_DIV, 0x01);
  sample_rate_ = 500.0f;

  // configure gyroscope to 250dps
  i2c_.write_reg(MPU9250_ADDR, GYRO_CONFIG, 0x00);
  gyro_scale_ = 131.0f;

  // configure accelerometer (+/-2g default)
  i2c_.write_reg(MPU9250_ADDR, ACCEL_CONFIG, 0x00);
  accel_scale_ = 16384.0f;

  return true;
}

bool MPU9250::verify_device() {
  std::uint8_t who_am_i = i2c_.read_reg(MPU9250_ADDR, WHO_AM_I);

  // MPU9250 can report 0x71 or 0x73 depending on revision
  // mpu6050 dummy clause. TODO: move to 650 specific driver
  if (who_am_i != 0x71 && who_am_i != 0x73 && who_am_i != 0x68 && who_am_i != 0x75) {
    return false;
  }

  return true;
}

void MPU9250::set_gyro_range(std::uint8_t range) {
  i2c_.write_reg(MPU9250_ADDR, GYRO_CONFIG, range << 3);

  switch(range) {
    case 0:  gyro_scale_ = 131.0f; break;     // 250  dps
    case 1:  gyro_scale_ =  65.5f; break;     // 500  dps
    case 2:  gyro_scale_ =  32.8f; break;     // 1000 dps
    case 3:  gyro_scale_ =  16.4f; break;     // 2000 dps
    default: gyro_scale_ = 131.0f; break;
  }
}

void MPU9250::set_accel_range(std::uint8_t range) {
  // range: 0=±2g, 1=±4g, 2=±8g, 3=±16g
  i2c_.write_reg(MPU9250_ADDR, ACCEL_CONFIG, range << 3);

  // Update scale factor
  switch(range) {
    case 0: accel_scale_ = 16384.0f; break;  // 2g
    case 1: accel_scale_ = 8192.0f; break;   // 4g
    case 2: accel_scale_ = 4096.0f; break;   // 8g
    case 3: accel_scale_ = 2048.0f; break;   // 16g
    default: accel_scale_ = 16384.0f; break;
  }
}

void MPU9250::set_sample_rate(std::uint16_t rate) {
  // sample rate = gyroscope output rate / (1 + SMPLRT_DIV)
  // gyroscope output rate = 1kHz (when DLPF is enabled)

  if (rate > 1000) rate = 1000;
  if (rate < 4) rate = 4;

  std::uint8_t divider = (1000 / rate) - 1;
  i2c_.write_reg(MPU9250_ADDR, SMPLRT_DIV, divider);
  sample_rate_ = 1000.0f / (1 + divider);
}

void MPU9250::read_raw_accel_gyro(std::int16_t* data) {
  std::uint8_t buffer[14];
  i2c_.read_burst(MPU9250_ADDR, ACCEL_XOUT_H, buffer, 14);

  data[0] = (std::int16_t)((buffer[0] << 8) | buffer[1]);   // ax
  data[1] = (std::int16_t)((buffer[2] << 8) | buffer[3]);   // ay
  data[2] = (std::int16_t)((buffer[4] << 8) | buffer[5]);   // az
  // Skip temperature: buffer[6:7]
  data[3] = (std::int16_t)((buffer[8] << 8) | buffer[9]);   // gx
  data[4] = (std::int16_t)((buffer[10] << 8) | buffer[11]); // gy
  data[5] = (std::int16_t)((buffer[12] << 8) | buffer[13]); // gz
}

Eigen::Vector3f MPU9250::read_accel() {
  std::int16_t raw[6];
  read_raw_accel_gyro(raw);

  // Convert to g and apply calibration
  Eigen::Vector3f accel;
  accel(0) = (raw[0] / accel_scale_) - accel_bias_(0);
  accel(1) = (raw[1] / accel_scale_) - accel_bias_(1);
  accel(2) = (raw[2] / accel_scale_) - accel_bias_(2);

  return accel;
}

Eigen::Vector3f MPU9250::read_gyro() {
  std::int16_t raw[6];
  read_raw_accel_gyro(raw);

  // Convert to deg/s and apply calibration
  Eigen::Vector3f gyro;
  gyro(0) = (raw[3] / gyro_scale_) - gyro_bias_(0);
  gyro(1) = (raw[4] / gyro_scale_) - gyro_bias_(1);
  gyro(2) = (raw[5] / gyro_scale_) - gyro_bias_(2);

  return gyro;
}


bool MPU9250::calibrate_gyro(std::uint16_t samples) {
  Eigen::Vector3f sum(0.0f, 0.0f, 0.0f);

  for (std::uint16_t i = 0; i < samples; i++) {
    std::int16_t raw[6];
    read_raw_accel_gyro(raw);

    sum(0) += raw[3] / gyro_scale_;
    sum(1) += raw[4] / gyro_scale_;
    sum(2) += raw[5] / gyro_scale_;

    for(volatile std::uint32_t j = 0; j < 20000; j++);
  }

  // average to get bias
  gyro_bias_ = sum / samples;

  return true;
}

bool MPU9250::calibrate_accel(std::uint16_t samples) {
  Eigen::Vector3f sum(0.0f, 0.0f, 0.0f);

  // collect samples
  for (std::uint16_t i = 0; i < samples; i++) {
    std::int16_t raw[6];
    read_raw_accel_gyro(raw);

    sum(0) += raw[0] / accel_scale_;
    sum(1) += raw[1] / accel_scale_;
    sum(2) += raw[2] / accel_scale_;

    for(volatile std::uint32_t j = 0; j < 20000; j++);
  }

  // average and remove gravity (assume Z-axis is up, +1g)
  accel_bias_ = sum / samples;
  accel_bias_(2) -= 1.0f;  // remove gravity on Z-axis

  return true;
}

}
