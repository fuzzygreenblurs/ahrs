#include "filters/madgwick.h"
#include "math/dcm.h"
#include "math/quaternion.h"

using namespace Eigen;

namespace AHRS {
  MadgwickFilter::MadgwickFilter(float beta) : q_(), beta_(beta) {}

  void MadgwickFilter::update(const Vector3f& gyro,
                              const Vector3f& accel,
                              const float dt) {
      
    Vector4f q_dot_pred = predict(gyro);
    Vector4f q_dot_corr = correct(accel);
    Vector4f q_dot      = q_dot_pred - (beta_ * q_dot_corr);

    q_ = q_ + Quaternion(dt * q_dot);
    q_.normalize();
  }

  Vector4f MadgwickFilter::predict(const Vector3f& gyro) const {
    Quaternion w(Vector4f(0, gyro(0), gyro(1), gyro(2)));
    Quaternion prod = q_ * w;
    return 0.5f * Eigen::Vector4f(prod.w(), 
                                  prod.x(), 
                                  prod.y(),
                                  prod.z());
  }

  Vector4f MadgwickFilter::correct(const Eigen::Vector3f& accel) const {
    Vector3f f = g_vec_error(accel);
    Matrix<float, 3, 4> J = jacobian();
    Vector4f gradient = J.transpose() * f;
    gradient.normalize(); 

    return gradient; 
  }

  Vector3f MadgwickFilter::g_vec_error(const Vector3f& accel) const {
    Vector3f a = accel.normalized();

    float qw2 = pow(q_.w(),2), 
          qx2 = pow(q_.x(),2),        
          qy2 = pow(q_.y(),2),
          qz2 = pow(q_.z(),2),
          qxz = q_.x()*q_.z(),
          qwy = q_.w()*q_.y(), 
          qwx = q_.w()*q_.x(),
          qyz = q_.y()*q_.z(); 

    Vector3f f;
    f << 2*(qxz - qwy) - a(0),
         2*(qwx + qyz) - a(1),
         (qw2 - qx2 - qy2 + qz2) - a(2);

    return f; 
  } 

  Matrix<float, 3, 4> MadgwickFilter::jacobian() const {
    float q0 = q_.w(), 
          q1 = q_.x(), 
          q2 = q_.y(), 
          q3 = q_.z();

    Matrix<float, 3, 4> J;
    J << -2*q2,  2*q3, -2*q0, 2*q1,
          2*q1,  2*q0,  2*q3, 2*q2,
          2*q0, -2*q1, -2*q2, 2*q3;

    return J;
  }


//  Quaternion MadgwickFilter::get_orientation() const {
//    return q_;
//  }

  Vector3f MadgwickFilter::get_euler() const {
    // convert quaternion to euler angles
    return q_.to_euler_zyx();
  }
}
