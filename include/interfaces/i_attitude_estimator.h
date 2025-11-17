#ifndef I_ATTITUDE_ESTIMATOR
#define I_ATTITUDE_ESTIMATOR

#include <stdexcept>
#include "math/quaternion.h"
     
namespace AHRS {
  class IAttitudeEstimator {
    public:
      virtual ~IAttitudeEstimator() = default;

      // 6DOF: for all IMUs
      virtual void update(const Eigen::Vector3f& gyro,
                          const Eigen::Vector3f& accel,
                          const float dt) = 0;

      // 9DOF: optional for added yaw 
      virtual void update(const Eigen::Vector3f& gyro,
                          const Eigen::Vector3f& accel, 
                          const Eigen::Vector3f& mag,
                          const float dt) {

        throw std::runtime_error("magnetometer fusion not supported.");
      }
     
      virtual bool supports_mag() const = 0;
      virtual Quaternion get_orientation() const = 0;
      virtual Eigen::Vector3f get_euler() const = 0;
  };
}

#endif
