#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "math/quaternion.h"
#include "interfaces/i_attitude_estimator.h"

namespace AHRS {
  class EKF : public IAttitudeEstimator {
    public:
      EKF(float Q_gyro = 1e-3f, float R_accel = 0.1f);

      void update(const Eigen::Vector3f& gyro,
                  const Eigen::Vector3f& accel,
                  const float dt) override;

      Quaternion get_orientation() const override;

      Eigen::Vector3f get_euler() const override;

      bool supports_mag() const override { return false; }

      void set_process_noise(float Q_gyro);

      void set_accel_noise(float R_accel);
    private:
      static constexpr float GRAVITY = 9.81f;

      Quaternion q_;
      Eigen::Matrix3f P_;
      Eigen::Matrix3f Q_;
      Eigen::Matrix3f R_accel_;

      void predict_state(const Eigen::Vector3f& gyro, float dt);
      void predict_covariance(const Eigen::Vector3f& gyro, float dt);
      void update_accel(const Eigen::Vector3f& accel);

      Eigen::Vector3f predict_gravity() const;
      Eigen::Matrix3f compute_H_accel() const;
      Eigen::Matrix3f skew_symmetric(const Eigen::Vector3f& v) const;

      Quaternion delta_quaternion(const Eigen::Vector3f& delta_theta) const;
  };
}

#endif
