#include "dcm.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

namespace ahrs {
  using namespace Eigen;

  TEST_GROUP(TEST_DCM) {
    //setup teardown as needed
  };

  TEST(TEST_DCM, is_so3) {
    // invalid cases
    Matrix3f nonzero_det;
    nonzero_det << 2, 0, 0,
                   0, 1, 0,
                   0, 0, 1;
    CHECK_FALSE(DCM::is_so3(nonzero_det));

    Matrix3f non_orthogonal;
    non_orthogonal << 1, 0.1, 0,
                      0, 1  , 0,
                      0, 0  , 1;
    CHECK_FALSE(DCM::is_so3(nonzero_det));


    // valid cases  
    CHECK_TRUE(DCM::is_so3(Matrix3f::Identity()));

    Matrix3f valid;
    float c = cos(M_PI/4);
    float s = sin(M_PI/4);

    valid  << 1,0,0,
              0,c,-s,
              0,s,c;

    CHECK_TRUE(DCM::is_so3(valid));
  }

  TEST(TEST_DCM, default_ctor) {
    Matrix3f target = Matrix3f::Identity();

    DCM dcm;
    Matrix3f mat = dcm.to_matrix();

    CHECK(mat.isApprox(target, DCM::TOLERANCE));
  }

  TEST(TEST_DCM, parametrized_ctor) {

    Matrix3f valid;
    float c = cos(M_PI/4);
    float s = sin(M_PI/4);
    
    valid  << 1,0,0,
              0,c,-s,
              0,s,c;

    DCM dcm(valid);
    Matrix3f mat = dcm.to_matrix();
    
    CHECK(mat.isApprox(valid, DCM::TOLERANCE));
  }

  TEST(TEST_DCM, multiply_composes_rotation) {
    Matrix3f m1;
    m1 << 0, -1, 0,
          1,  0, 0,
          0,  0, 1; 

    DCM r1(m1);
    DCM r2(m1);

    DCM res = r1 * r2;
    
    Matrix3f exp;
    exp << -1,  0, 0,
                 0, -1, 0,
                 0,  0, 1; 

    CHECK(res.to_matrix().isApprox(exp, DCM::TOLERANCE));
  }

  TEST(TEST_DCM, mat_vec_multiply) {
    Matrix3f m;
    m << 0, -1, 0,
         1,  0, 0,
         0,  0, 1; 

    DCM d(m); 
    Vector3f v(1, 0, 0);
    Vector3f res = d * v;

    Vector3f exp(0, 1, 0); 
    CHECK(res.isApprox(exp, DCM::TOLERANCE));
  }

  TEST(TEST_DCM, mat_inversion) {
    Matrix3f m;
    m << 0, -1, 0,
         1,  0, 0,
         0,  0, 1; 

    DCM R(m);
    DCM prod = R * R.inv();
    CHECK(prod.to_matrix().isApprox(Matrix3f::Identity(),
                                    DCM::TOLERANCE));
     
  }

  TEST(TEST_DCM, mat_transpose) {
    Matrix3f m;
    m << 0, -1, 0,
         1,  0, 0,
         0,  0, 1; 

    DCM R(m);
    DCM RT = R.T();
    CHECK(RT.to_matrix().isApprox(m.transpose(), DCM::TOLERANCE));
  }

  TEST(TEST_DCM, euler_zyx_eye) {
    DCM R = DCM::from_euler_zyx(0, 0, 0);
    CHECK(R.to_matrix().isApprox(Matrix3f::Identity(), DCM::TOLERANCE));
  } 
    
  TEST(TEST_DCM, euler_zyx_90_yaw) {
    DCM R = DCM::from_euler_zyx(0, 0, M_PI/2);
    Matrix3f exp;
    exp << 0, -1, 0,
           1,  0, 0,
           0,  0, 1;
  
    CHECK(R.to_matrix().isApprox(exp, DCM::TOLERANCE));
  }

  TEST(TEST_DCM, dcm_eye_yields_zero_euler_angles) {
    DCM R = DCM::eye(); 
    Vector3f euler = R.to_euler_zyx();

    DOUBLES_EQUAL(0.0f, euler(0), DCM::TOLERANCE);
    DOUBLES_EQUAL(0.0f, euler(1), DCM::TOLERANCE);
    DOUBLES_EQUAL(0.0f, euler(2), DCM::TOLERANCE);
  }

  TEST(TEST_DCM, euler_roundtrip) {
    float r = 0.1f; 
    float p = 0.2f; 
    float y = 0.3f; 

    DCM R = DCM::from_euler_zyx(r, p, y);
    Vector3f euler = R.to_euler_zyx();
    DOUBLES_EQUAL(r, euler(0), DCM::TOLERANCE);
    DOUBLES_EQUAL(p, euler(1), DCM::TOLERANCE);
    DOUBLES_EQUAL(y, euler(2), DCM::TOLERANCE);
  }
}

