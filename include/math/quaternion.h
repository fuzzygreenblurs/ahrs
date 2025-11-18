#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Dense>
#include "math/dcm.h"

/*
  * note: Vector3 representations are compact but always have singularities
  * phttps://lisyarus.github.io/blog/posts/introduction-to-quaternions.html#section-some-notes-on-the-formula
  */

namespace AHRS {
  class Quaternion {
    public:
      static constexpr float TOLERANCE = 1e-6f;

      Quaternion();
      Quaternion(const Eigen::Vector4f& v);
    
      static Quaternion eye();
      static Quaternion from_axis_angle(const Eigen::Vector3f& axis, float angle);
      static Quaternion from_dcm(const DCM& m);
      DCM to_dcm() const;

      Quaternion operator*(const Quaternion& q) const;
      Quaternion operator+(const Quaternion& q) const;
      Quaternion conjugate() const;
      Quaternion inv() const;

      Quaternion normalized_dup() const;
      Quaternion& normalize();
      float norm() const;
      Eigen::Vector3f rotate(const Eigen::Vector3f& vec) const;

      Eigen::Vector3f to_euler_zyx() const;

      float w() const { return data_(0); }
      float x() const { return data_(1); }
      float y() const { return data_(2); }
      float z() const { return data_(3); }

    private:
      Eigen::Vector4f data_;
      Eigen::Vector4f to_vec4() const { return data_; };
  };
}

#endif

