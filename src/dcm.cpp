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
}
