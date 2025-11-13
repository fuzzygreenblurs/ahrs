#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Dense>
#include "dcm.h"

/*
  * note: Vector3 representations are compact but always have singularities
  * phttps://lisyarus.github.io/blog/posts/introduction-to-quaternions.html#section-some-notes-on-the-formula
  */

namespace ahrs {
  class Quaternion {
    public:
      static constexpr float TOLERANCE = 1e-6f;

      Quaternion();
      Quaternion(float w, float x, float y, float z);
    
      static Quaternion eye();
      static Quaternion from_axis_angle(const Eigen::Vector3f& axis, float angle);
      static Quaternion from_dcm(const DCM& m);
      DCM to_dcm() const;

      Quaternion operator*(const Quaternion& q) const;
      Quaternion conjugate() const;
      Quaternion inv() const;

      Quaternion normalized_dup() const;
      Quaternion& normalize();
      float norm() const;
      Eigen::Vector3f rotate(const Eigen::Vector3f& vec) const;

      double w() const { return w_; }
      double x() const { return x_; }
      double y() const { return y_; }
      double z() const { return z_; }

    private:
      double w_, x_, y_, z_;  
  };
}

#endif
