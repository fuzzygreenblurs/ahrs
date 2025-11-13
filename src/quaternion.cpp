#include "quaternion.h"
#include "dcm.h"
#include <cassert>

namespace ahrs {
  using namespace Eigen;

  Quaternion::Quaternion() : w_(1.0f), x_(0.0f), y_(0.0f), z_(0.0f) {};

  // TODO: ensure incoming quaternion belongs to Spin(3)
  Quaternion::Quaternion(float w, float x, float y, float z)
    : w_(w), x_(x), y_(y), z_(z) {}

  Quaternion Quaternion::eye() {
    return Quaternion();
  }

  Quaternion Quaternion::conjugate() const {
    return Quaternion(w_, -x_, -y_, -z_);
  }

  Quaternion Quaternion::inv() const {
    return conjugate();
  }

  Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
      w_*q.w_ - x_*q.x_ - y_*q.y_ - z_*q.z_,  // w
      w_*q.x_ + x_*q.w_ + y_*q.z_ - z_*q.y_,  // x
      w_*q.y_ - x_*q.z_ + y_*q.w_ + z_*q.x_,  // y
      w_*q.z_ + x_*q.y_ - y_*q.x_ + z_*q.w_   // z
    );
  } 

  Vector3f Quaternion::rotate(const Vector3f& vec) const {
    // quaternion product sandwich: magic formula
    Quaternion v(0, vec(0), vec(1), vec(2));
    Quaternion prod = (*this) * v * conjugate();
    assert(abs(prod.w()) < 1e-5f && "must produce imaginary quaternion");
    return Vector3f(prod.x(), prod.y(), prod.z());
  }

  Quaternion Quaternion::from_axis_angle(const Vector3f& axis, float angle) {
    float half_angle = angle / 2.0f;
    float s = sin(half_angle);
    float c = cos(half_angle);

    Vector3f u_axis = axis.normalized();
    return Quaternion(c, s * u_axis(0), s * u_axis(1), s * u_axis(2));
  }

  float Quaternion::norm() const {
    return sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_); 
  }

  Quaternion& Quaternion::normalize() {
    float n = norm();
    assert(n > 1e-6f && "cannot normalize zero quaternion");
    w_ /= n; 
    x_ /= n; 
    y_ /= n; 
    z_ /= n; 

    return *this;
  }

  Quaternion Quaternion::normalized_dup() const {
    Quaternion q = *this;
    q.normalize();
    return q;
  }

  // see: https://motoq.github.io/doc/tnotes/dcmq.pdf
  // TODO: derive this formula
  DCM Quaternion::to_dcm() const {
    float w2 = w_*w_, x2 = x_*x_, y2 = y_*y_, z2 = z_*z_;
    float wx = w_*x_, wy = w_*y_, wz = w_*z_;
    float xy = x_*y_, xz = x_*z_, yz = y_*z_;

    Matrix3f R;
    R << w2+x2-y2-z2, 2*(xy - wz), 2*(xz + wy),
         2*(xy + wz), w2-x2+y2-z2, 2*(yz - wx),
         2*(xz - wy), 2*(yz + wx), w2-x2-y2+z2;

    return DCM(R);
  }

  // shepperd's method
  // https://ahrs.readthedocs.io/en/latest/quaternion/quaternion.from_DCM.html 
  // TODO: derive this formula
  Quaternion Quaternion::from_dcm(const DCM& dcm) {
    Matrix3f R = dcm.to_matrix();

    float trace = R(0,0) + R(1,1) + R(2,2);
    
    if(trace > 0) {
      // w is the largest component 
      float s = 0.5f / sqrt(trace + 1.0f);

      return Quaternion(
        0.25f / s,
        (R(2,1) - R(1,2)) * s,
        (R(0,2) - R(2,0)) * s,
        (R(1,0) - R(0,1)) * s
      );

    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
      // x is the largest component 
      float s = 2.0f * sqrt(1.0f + R(0,0) - R(1,1) - R(2,2));
      return Quaternion(
        (R(2,1) - R(1,2)) / s,
        0.25f * s,
        (R(0,1) + R(1,0)) / s, 
        (R(0,2) + R(2,0)) / s
      );

    } else if (R(1,1) > R(2,2)) {
      // y is the largest component 
      float s = 2.0f * sqrt(1.0f + R(1,1) - R(0,0) - R(2,2));
      return Quaternion(
        (R(0,2) - R(2,0)) / s,        
        (R(0,1) + R(1,0)) / s,       
        0.25f * s,                 
        (R(1,2) + R(2,1)) / s     
      );

    } else {
      // z is the largest component 
      float s = 2.0f * sqrt(1.0f + R(2,2) - R(0,0) - R(1,1));
      return Quaternion(
        (R(1,0) - R(0,1)) / s,        
        (R(0,2) + R(2,0)) / s,        
        (R(1,2) + R(2,1)) / s,        
        0.25f * s                     
      );
    }
  }
}
