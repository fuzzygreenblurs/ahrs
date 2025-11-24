#include "filters/madgwick.h"
#include "math/dcm.h"
#include "math/quaternion.h"

using namespace Eigen;

namespace AHRS {
  MadgwickFilter::MadgwickFilter(float beta) : q_(), beta_(beta) {}

  void MadgwickFilter::update(const Vector3f& gyro,
                              const Vector3f& accel) {
      
    Vector4f q_dot_pred = prediction(gyro);
    Vector4f q_dot_corr = correction(accel);
    Vector4f q_dot      = q_dot_pred - (beta_ * q_dot_corr);

    q_ = q_ + Quaternion(dt * q_dot);
    q_.normalize();
  }

  Vector4f MadgwickFilter::prediction(const Vector3f& gyro) const {
    Quaternion w(Vector4f(0, gyro(0), gyro(1), gyro(2)));
    Quaternion prod = q_ * w;
    return 0.5f * Eigen::Vector4f(prod.w(), 
                                  prod.x(), 
                                  prod.y(),
                                  prod.z());
  }

  Vector4f MadgwickFilter::correction(const Eigen::Vector3f& accel) const {
    Vector3f f = g_vec_error(accel);
    Matrix<float, 3, 4> J = jacobian();
    Vector4f gradient = J.transpose() * f;
    gradient.normalize(); 

    return gradient; 
  }

  Vector3f MadgwickFilter::g_vec_error(const Vector3f& accel) const {
    Vector3f a = accel.normalized();

    float w2 = pow(q_.w(),2), 
          x2 = pow(q_.x(),2),        
          y2 = pow(q_.y(),2),
          z2 = pow(q_.z(),2),
          xz = q_.x()*q_.z(),
          wy = q_.w()*q_.y(), 
          wx = q_.w()*q_.x(),
          yz = q_.y()*q_.z(); 

    Vector3f f;
    f << 2*(xz - wy) - a(0),
         2*(wx + yz) - a(1),
         (w2 - x2 - y2 + z2) - a(2);

    return f; 
  } 

  Matrix<float, 3, 4> MadgwickFilter::jacobian() const {
    float w = q_.w(), 
          x = q_.x(), 
          y = q_.y(), 
          z = q_.z();

    Matrix<float, 3, 4> J;
    J << -2*y,  2*z, -2*w, 2*x,
          2*x,  2*w,  2*z, 2*y,
          2*w, -2*x, -2*y, 2*z;

    return J;
  }


  Quaternion MadgwickFilter::get_orientation() const {
    return q_;
  }

  Vector3f MadgwickFilter::get_euler() const {
    // convert quaternion to euler angles
    return q_.to_euler_zyx();
  }
}
