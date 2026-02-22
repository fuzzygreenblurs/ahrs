## 6-DOF (Degree-of-Freedom) Attitude Estimator

This project is a first-principles design and implementation of a real-time Attitude Heading Reference System (AHRS), representing the state estimator for a self-stabilizing drone flight control loop. The system compares the performance of a Madgwick orientation filter and an Extended Kalman Filter, fusing 3-axis gyroscope and accelerometer sensor modalities to estimate 3D orientation. Quaternion representation is used to avoid gimbal lock singularities. Mathematical justification for these approaches can be found in the [project report](./6dof_attitude_estimation.pdf).

An MPU9250 IMU is used for this project, communicating over I2C with an STM32F446RE Nucleo board. The communication driver is implemented on bare-metal using CMSIS-CORE macros generated from STM32CubeIDE. 

## Future Work

1. This project focuses more on the driver logic and does not yet include calibration logic for the filter gains.
2. At the moment, only pitch and roll are tracked through these filters. In future, the included magnetometer telemetry should be employed to estimate yaw as well.
3. The current telemetry pipeline uses an interrupt based architecture that could be optimized for higher speeds.The STM32F446RE provides DMA with built-in circular buffer functionality, which would free the processor to run more frequent cycles of the filters themselves.
