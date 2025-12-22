#include "stm32f4xx.h"
#include "mcal/cmsis_uart.h"
#include "mcal/cmsis_i2c.h"
#include "drivers/mpu9250.h"
#include "filters/madgwick.h"
#include "filters/ekf.h"
#include <cstdio>

int main() {
  mcal::CMSIS_UART uart;
  uart.init();
  uart.blocking_transmit("initializing i2c bus...\r\n");

  mcal::CMSIS_I2C i2c;
  i2c.init();

  drivers::MPU9250 imu(i2c);
  if(!imu.init()) {
    uart.blocking_transmit("IMU init failed\r\n");
    while(1);
  }

  uart.blocking_transmit("calibrating gyro...\r\n");
  imu.calibrate_gyro(5000);  
  uart.blocking_transmit("calibration completed.\r\n");

  // Initialize filters
  uart.blocking_transmit("initialize Madgwick...\r\n");
  AHRS::MadgwickFilter madgwick(0.5f);

  uart.blocking_transmit("initialize EKF...\r\n");
  AHRS::EKF ekf(1e-5f, 0.1f);  // Q_gyro, R_accel 

  uart.blocking_transmit("streaming attitude: \r\n");

  char buf[128];
  const float dt = 0.1f;  
  
  while(1) {
    auto accel_g = imu.read_accel();              // g (gravitational acceleration)
    auto gyro_deg = imu.read_gyro();              // deg/s

    auto gyro = gyro_deg * (3.14159f / 180.0f);   // deg/s to rad/s
    auto accel = accel_g * 9.81f;                 // g to m/s^2

    madgwick.update(gyro, accel, dt);
    ekf.update(gyro, accel, dt);

    auto euler_mad = madgwick.get_euler();
    auto euler_ekf = ekf.get_euler();

    // Convert to degrees (180/pi)
    float mad_roll  = euler_mad(0) * 57.2958f;
    float mad_pitch = euler_mad(1) * 57.2958f;
    float mad_yaw   = euler_mad(2) * 57.2958f;

    float ekf_roll  = euler_ekf(0) * 57.2958f;
    float ekf_pitch = euler_ekf(1) * 57.2958f;
    float ekf_yaw   = euler_ekf(2) * 57.2958f;

    snprintf(buf, sizeof(buf),
             "Roll:%.1f Pitch:%.1f Yaw:%.1f\r\n",
             mad_roll, mad_pitch, mad_yaw);
    uart.blocking_transmit(buf);

    for(volatile uint32_t i = 0; i < 60000; i++);  
  }
}

