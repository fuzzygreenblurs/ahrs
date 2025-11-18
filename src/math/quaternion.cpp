#include "math/quaternion.h"
#include "math/dcm.h"
#include <cassert>

namespace AHRS {
  using namespace Eigen;

  // TODO: ensure incoming quaternion belongs to Spin(3)
  Quaternion::Quaternion() : 
    data_(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f)) {};

  Quaternion::Quaternion(const Eigen::Vector4f& v) : 
    data_(v) {};

  Quaternion Quaternion::eye() {
    return Quaternion();
  }

  Quaternion Quaternion::conjugate() const {
    return Quaternion(Vector4f(data_(0), 
                              -data_(1), 
                              -data_(2),
                              -data_(3)));
  }

  Quaternion Quaternion::inv() const {
    return conjugate();
  }

  Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(data_ + q.data_);
  } 

  Quaternion Quaternion::operator*(const Quaternion& q) const {
    float w0 =   data_(0),   x0 = data_(1),   y0 = data_(2),   z0 = data_(3);
    float w1 = q.data_(0), x1 = q.data_(1), y1 = q.data_(2), z1 = q.data_(3);

    return Quaternion(
      Vector4f(w0*w1 - x0*x1 - y0*y1 - z0*z1,  // w
               w0*x1 + x0*w1 + y0*z1 - z0*y1,  // x
               w0*y1 - x0*z1 + y0*w1 + z0*x1,  // y
               w0*z1 + x0*y1 - y0*x1 + z0*w1)   // z
    );
  }  

  Vector3f Quaternion::rotate(const Vector3f& vec) const {
    // quaternion product sandwich: magic formula
    Quaternion v(Vector4f(0, vec(0), vec(1), vec(2)));
    Quaternion prod = (*this) * v * conjugate();
    assert(abs(prod.w()) < 1e-5f && "must produce imaginary quaternion");
    return Vector3f(prod.x(), prod.y(), prod.z());
  }

  Quaternion Quaternion::from_axis_angle(const Vector3f& axis, float angle) {
    float half_angle = angle / 2.0f;
    float s = sin(half_angle);
    float c = cos(half_angle);

    Vector3f u_axis = axis.normalized();
    return Quaternion(
      Vector4f(c, s * u_axis(0), s * u_axis(1), s * u_axis(2))
    );
  }

  float Quaternion::norm() const {
    return data_.norm(); 
  }

  Quaternion& Quaternion::normalize() {
    data_.normalize();
    assert(norm() > 1e-6f && "cannot normalize zero quaternion");
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
    float w2 = pow(data_(0), 2), 
          x2 = pow(data_(1), 2),
          y2 = pow(data_(2), 2),
          z2 = pow(data_(3), 2);

    float wx = data_(0) * data_(1),
          wy = data_(0) * data_(2),
          wz = data_(0) * data_(3),
          xy = data_(1) * data_(2),
          xz = data_(1) * data_(3),
          yz = data_(2) * data_(3);

    Matrix3f R;
    R << w2+x2-y2-z2, 2*(xy - wz), 2*(xz + wy),
         2*(xy + wz), w2-x2+y2-z2, 2*(yz - wx),
         2*(xz - wy), 2*(yz + wx), w2-x2-y2+z2;

    return DCM(R);
  }

  Vector3f Quaternion::to_euler_zyx() const {
    //TODO: derive this conversion
    float qw = data_(0),
          qx = data_(1),
          qy = data_(2),
          qz = data_(3);

    Vector3f rpy;
    rpy << atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)),
            asin(2*(qw*qy - qz*qx)),
           atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)); 

    return rpy;
  }
//
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
        Vector4f(
          0.25f / s,
          (R(2,1) - R(1,2)) * s,
          (R(0,2) - R(2,0)) * s,
          (R(1,0) - R(0,1)) * s)
      );

    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
      // x is the largest component 
      float s = 2.0f * sqrt(1.0f + R(0,0) - R(1,1) - R(2,2));
      return Quaternion(
        Vector4f(
          (R(2,1) - R(1,2)) / s,
          0.25f * s,
          (R(0,1) + R(1,0)) / s, 
          (R(0,2) + R(2,0)) / s
        )
      );

    } else if (R(1,1) > R(2,2)) {
      // y is the largest component 
      float s = 2.0f * sqrt(1.0f + R(1,1) - R(0,0) - R(2,2));
      return Quaternion(
        Vector4f(
          (R(0,2) - R(2,0)) / s,        
          (R(0,1) + R(1,0)) / s,       
          0.25f * s,                 
          (R(1,2) + R(2,1)) / s)
      );

    } else {
      // z is the largest component 
      float s = 2.0f * sqrt(1.0f + R(2,2) - R(0,0) - R(1,1));
      return Quaternion(
        Vector4f(
          (R(1,0) - R(0,1)) / s,        
          (R(0,2) + R(2,0)) / s,        
          (R(1,2) + R(2,1)) / s,        
          0.25f * s) 
      );
    }
  }
}
