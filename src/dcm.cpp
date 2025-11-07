#include "dcm.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace ahrs {
  using namespace Eigen;

  DCM::DCM() : R_(Matrix3f::Identity()) {}

  Matrix3f DCM::to_matrix() const {
    return R_; 
  }

  DCM::DCM(const Matrix3f& mat) {
    R_ = mat;  
  }

  bool DCM::is_so3() const {
    float det = R_.determinant();
    if(abs(det - 1.0f) > TOLERANCE)
      return false;

    Matrix3f prod = R_.transpose() * R_;
    if((prod - Matrix3f::Identity()).norm() > TOLERANCE)
      return false;

    return true;
  }
}
