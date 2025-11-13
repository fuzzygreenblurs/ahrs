#include "quaternion.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include <iostream>

namespace ahrs {
  using namespace Eigen;

  TEST_GROUP(QUATERNION) {};
  
  TEST(QUATERNION, default_ctor) { 
    Quaternion q;
    CHECK(q.w() == 1.0f);
    CHECK(q.x() == 0.0f);
    CHECK(q.y() == 0.0f);
    CHECK(q.z() == 0.0f);
  }

  TEST(QUATERNION, create_eye_quaternion) { 
    Quaternion q = Quaternion::eye();
    CHECK(q.w() == 1.0f);
    CHECK(q.x() == 0.0f);
    CHECK(q.y() == 0.0f);
    CHECK(q.z() == 0.0f);
  }
  
  TEST(QUATERNION, hamiltonian_product) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion eye = Quaternion::eye();
    Quaternion prod = q * eye;

    CHECK(abs(prod.w() - q.w()) < Quaternion::TOLERANCE);
    CHECK(abs(prod.x() - q.x()) < Quaternion::TOLERANCE);
    CHECK(abs(prod.y() - q.y()) < Quaternion::TOLERANCE);
    CHECK(abs(prod.z() - q.z()) < Quaternion::TOLERANCE);
  }

  TEST(QUATERNION, conjugate) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion conj = q.conjugate();

    CHECK_EQUAL(q.w(), conj.w());
    CHECK_EQUAL(-q.x(), conj.x());
    CHECK_EQUAL(-q.y(), conj.y());
    CHECK_EQUAL(-q.z(), conj.z());
  }

  TEST(QUATERNION, invert) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion q_inv = q.inv();
    Quaternion prod = q * q_inv;

    CHECK(abs(prod.w() - 1.0f) < Quaternion::TOLERANCE);
    CHECK(abs(prod.x()) < Quaternion::TOLERANCE);
    CHECK(abs(prod.y()) < Quaternion::TOLERANCE);
    CHECK(abs(prod.z()) < Quaternion::TOLERANCE);
  }
  
  TEST(QUATERNION, rotate_vector) {
    // yaw by 90deg (about Z)
    float s = sin(M_PI/4);
    float c = sin(M_PI/4);
    Quaternion q(c, 0, 0, s);
  
    Vector3f v(1, 0, 0);
    Vector3f res = q.rotate(v);

    // after rotation, the resulsant vector should point
    // towards the y-axis
    Vector3f expected(0, 1, 0);
    CHECK(res.isApprox(expected, Quaternion::TOLERANCE));
  }

  TEST(QUATERNION, axis_angle_to_quat) {
    Vector3f axis(0, 0, 1);
    float angle = M_PI/2;

    Quaternion q = Quaternion::from_axis_angle(axis, angle);
   
    // half angle because of double-cover characteristic
    float s = sin(M_PI/4);
    float c = cos(M_PI/4);

    DOUBLES_EQUAL(c, q.w(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(0, q.x(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(0, q.y(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(s, q.z(), Quaternion::TOLERANCE);
  }

  TEST(QUATERNION, compute_length) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    float norm = q.norm();
    DOUBLES_EQUAL(1.0f, norm, Quaternion::TOLERANCE);
  }

  TEST(QUATERNION, normalize_in_place) {
    Quaternion q(2, 0, 0, 0);
    q.normalize();
    DOUBLES_EQUAL(1.0f, q.w(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(1.0f, q.norm(), Quaternion::TOLERANCE);
  }

  TEST(QUATERNION, generate_normalized_duplicate) {
    Quaternion q(2, 0, 0, 0);
    Quaternion dup = q.normalized_dup();
    DOUBLES_EQUAL(2.0f, q.w(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(1.0f, dup.w(), Quaternion::TOLERANCE);
  }

  TEST(QUATERNION, dcm_conversion_simple_yaw) {
    // perform 90deg yaw 
    float s = sin(M_PI/4);
    float c = cos(M_PI/4);

    Quaternion q(c, 0, 0, s);
    DCM R = q.to_dcm();

    Matrix3f expected; 
    expected << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;

    CHECK(R.to_matrix().isApprox(expected, DCM::TOLERANCE));
  }

  TEST(QUATERNION, euler_dcm_quaternion_roundtrip) {
    float roll = 0.1f, pitch = 0.2f, yaw = 0.3f;

    DCM dcm = DCM::from_euler_zyx(roll, pitch, yaw);
    Quaternion q = Quaternion::from_dcm(dcm); 
    DCM dcm_from_quat = q.to_dcm();

    Vector3f recovered_euler = dcm_from_quat.to_euler_zyx();

    DOUBLES_EQUAL(roll,  recovered_euler(0), DCM::TOLERANCE);
    DOUBLES_EQUAL(pitch, recovered_euler(1), DCM::TOLERANCE);
    DOUBLES_EQUAL(yaw,   recovered_euler(2), DCM::TOLERANCE);
  }
}

int main(int argc, char** argv) {
  return CommandLineTestRunner::RunAllTests(argc, argv);
}
