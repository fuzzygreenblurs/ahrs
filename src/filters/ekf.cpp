#include "filters/ekf.h"

using namespace Eigen;

namespace AHRS {

  EKF::EKF(float Q_gyro, float R_accel)
    : q_(),                                   // initialize to identity quaternion [1, 0, 0, 0]
      P_(Matrix3f::Identity() * 1e-6f),       // small initial uncertainty
      Q_(Matrix3f::Identity() * Q_gyro),
      R_accel_(Matrix3f::Identity() * R_accel) {}

  void EKF::update(const Vector3f& gyro, const Vector3f& accel, const float dt) {
    predict_state(gyro, dt);
    predict_covariance(gyro, dt);

    update_accel(accel);
  }


  void EKF::predict_state(const Vector3f& gyro, float dt) {
    // gyro quaternion
    Vector4f omega_vec(0.0f, gyro(0), gyro(1), gyro(2));
    Quaternion omega_quat(omega_vec);

    // quaternion orientation derivative
    Quaternion q_dot = q_ * omega_quat;
    Vector4f q_dot_vec(q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z());
    q_dot_vec *= 0.5f;

    // new prior estimate quaternion
    Vector4f q_vec(q_.w(), q_.x(), q_.y(), q_.z());
    Vector4f q_new_vec = q_vec + q_dot_vec * dt;
    q_ = Quaternion(q_new_vec);

    q_.normalize();
  }

  void EKF::predict_covariance(const Vector3f& gyro, float dt) {
    Matrix3f omega_cross = skew_symmetric(gyro);

    // prior covariance derives from discrete-time state-transition matrix
    Matrix3f F = Matrix3f::Identity() - omega_cross * dt;
    P_ = F * P_ * F.transpose() + Q_ * dt;
    P_ = 0.5f * (P_ + P_.transpose());
  }


  void EKF::update_accel(const Vector3f& accel) {
    // compute innovation
    Vector3f accel_norm = accel.normalized();
    Vector3f gravity_pred_norm = predict_gravity().normalized();
    Vector3f innovation = accel_norm - gravity_pred_norm;
  
    // compute Kalman gain 
    Matrix3f H = compute_H_accel();
    Matrix3f S = H * P_ * H.transpose() + R_accel_;
    Matrix3f K = P_ * H.transpose() * S.inverse();

    // compute posterior attitude estimate (left multiplication)
    Vector3f delta_theta = K * innovation;
    Quaternion delta_q = delta_quaternion(delta_theta);
    q_ = delta_q * q_;
    q_.normalize();

    // compute posterior covariance
    Matrix3f I_KH = Matrix3f::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_accel_ * K.transpose();
    P_ = 0.5f * (P_ + P_.transpose());
  }

  Matrix3f EKF::skew_symmetric(const Vector3f& v) const {
    Matrix3f S;
    S <<     0.0f,  -v(2),   v(1),
             v(2),   0.0f,  -v(0),
            -v(1),   v(0),   0.0f;

    return S;
  }

  Quaternion EKF::delta_quaternion(const Vector3f& delta_theta) const {
    float theta = delta_theta.norm();

    if (theta < 1e-8f) {
      return Quaternion::eye();
    }

    // small angle assumption to avoid trig operations: 
    // q = [cos(θ/2), sin(θ/2)x, sin(θ/2)y, sin(θ/2)z]
    // sin(θ/2) ~ θ/2 

    if (theta < 0.1f) {
      Vector4f dq_vec(1.0f,
                      delta_theta(0) * 0.5f,
                      delta_theta(1) * 0.5f,
                      delta_theta(2) * 0.5f);

      Quaternion result(dq_vec);
      result.normalize();
      return result;
    }

    // for larger rotation angles
    float half_theta = theta * 0.5f;
    float w = cos(half_theta);
    float s = sin(half_theta) / theta;  // sin(θ/2) / θ

    Vector4f dq_vec(w,
                    delta_theta(0) * s,
                    delta_theta(1) * s,
                    delta_theta(2) * s);

    return Quaternion(dq_vec);
  }

  Vector3f EKF::predict_gravity() const {
    // gravity vector in inertial NED frame
    Vector3f g_inertial(0.0f, 0.0f, GRAVITY);

    // rotate to body frame using quaternion
    return q_.rotate(g_inertial);
  }

  Matrix3f EKF::compute_H_accel() const {
    Vector3f g_body = predict_gravity();
    return -skew_symmetric(g_body);
  }

  Quaternion EKF::get_orientation() const {
    return q_;
  }

  Vector3f EKF::get_euler() const {
    return q_.to_euler_zyx();
  }

  void EKF::set_process_noise(float Q_gyro) {
    Q_ = Matrix3f::Identity() * Q_gyro;
  }

  void EKF::set_accel_noise(float R_accel) {
    R_accel_ = Matrix3f::Identity() * R_accel;
  }
} 
