#ifndef MADGWICK_H
#define MADGWICK_H

#include <Eigen/Core>
#include "math/quaternion.h"
#include "interfaces/i_attitude_estimator.h"

namespace AHRS {
  class MadgwickFilter : public IAttitudeEstimator {
    public:
      void update(const Eigen::Vector3f& gyro,
                  const Eigen::Vector3f& accel,
                  const float dt) override;
      
      bool supports_mag() const override { return false; }
      Quaternion get_orientation() const override;
      Eigen::Vector3f get_euler() const override;

    private:
      Quaternion q_;
      float B_; 
  };
}

#endif

