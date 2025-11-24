#ifndef MADGWICK_H
#define MADGWICK_H

#include <Eigen/Core>
#include "math/quaternion.h"
#include "interfaces/i_attitude_estimator.h"

namespace AHRS {
  class MadgwickFilter : public IAttitudeEstimator {
    public:
      MadgwickFilter(float beta= 0.1f); 

      void update(const Eigen::Vector3f& gyro,
                  const Eigen::Vector3f& accel) override;
      
      bool supports_mag() const override { return false; }
      Quaternion get_orientation() const override;
      Eigen::Vector3f get_euler() const override;

    private:
      Quaternion q_;
      float beta_;

      Eigen::Vector4f prediction(const Eigen::Vector3f& gyro) const;
      Eigen::Vector4f correction(const Eigen::Vector3f& accel) const;
      Eigen::Vector3f g_vec_error(const Eigen::Vector3f& accel) const;
      Eigen::Matrix<float, 3, 4> jacobian() const;
  };
}

#endif

