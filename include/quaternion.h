#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Dense>
#include "dcm.h"

namespace ahrs {
  class Quaternion {
    public:
      Quaternion();
      Quaternion(double w, double x, double y, double z);
    
      static Quaternion from_axis_angle(const Eigen::Vector3d& axis, double angle);
      static Quaternion from_dcm(const DCM& m);
      DCM to_dcm() const;
      static Quaternion eye();

      Quaternion operator*(const Quaternion& q) const;
      Quaternion conjugate() const;
      Quaternion inv() const;
      Quaternion normalized() const;
      Quaternion& normalize();
  
      // quaternion represents rotation: 
      // v' = q * v * q̄ (quaternion sandwich product) 
      Eigen::Vector3d rotate(const Eigen::Vector3d& vec) const;

      double w() const { return w_; }
      double x() const { return x_; }
      double y() const { return y_; }
      double z() const { return z_; }

    private:
      double w_, x_, y_, z_;  
  };
}

#endif
