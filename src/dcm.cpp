#include "dcm.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cassert>

namespace ahrs {
  using namespace Eigen;

  DCM::DCM() : R_(Matrix3f::Identity()) {}

  Matrix3f DCM::to_matrix() const {
    return R_; 
  }

  DCM::DCM(const Matrix3f& mat) {
    assert(is_so3(mat) && "invalid rotation matrix");
    R_ = mat;  
  }

  bool DCM::is_so3(const Matrix3f& mat) {
    float det = mat.determinant();
    if(abs(det - 1.0f) > TOLERANCE)
      return false;

    Matrix3f prod = mat.transpose() * mat;
    if((prod - Matrix3f::Identity()).norm() > TOLERANCE)
      return false;

    return true;
  }

  DCM DCM::operator*(const DCM& other) const {
    return DCM(R_ * other.R_); 
  }


  Vector3f DCM::operator*(const Vector3f& vec) const {
    return R_ * vec;  
  }

  DCM DCM::from_euler_zyx(float roll, float pitch, float yaw) {
    float cr = cos(roll),  sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw),   sy = sin(yaw);
    
    // derived from right-to-left multiplication of euler rotations
    // ZYX: roll THEN pitch THEN yaw
    Matrix3f R;
    R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
           -sp,            cp*sr,            cp*cr;

    return DCM(R);
  }

  Vector3f DCM::to_euler_zyx() const {
    float roll  = atan2(R_(2,1), R_(2,2));
    float pitch = asin(-R_(2,0));
    float yaw   = atan2(R_(1,0), R_(0,0));

    return Vector3f(roll, pitch, yaw);
  }

  DCM DCM::inv() const {
    return T();
  }

  DCM DCM::T() const {
    return DCM(R_.transpose());
  }
}

