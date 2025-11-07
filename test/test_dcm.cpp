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

  TEST(TEST_DCM, assign_ctor) {
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
}


int main(int argc, char** argv) {
  return CommandLineTestRunner::RunAllTests(argc, argv);
}
