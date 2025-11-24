#ifndef I_IMU_DATA_SOURCE
#define I_IMU_DATA_SOURCE

#include <Eigen/Core>

namespace AHRS {
  struct IMUReading {
    float timestamp;
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
    Eigen::Vector3f mag;
    float dt;
  };

  class IIMUDataSource {
    public:
      virtual ~IIMUDataSource() = default;
    
      virtual bool has_next() const = 0;
      virtual IMUReading get_next() = 0;
      
      virtual bool has_ground_truth() const = 0;
      virtual Eigen::Vector3f get_ground_truth_euler() = 0;
  };
}

#endif
