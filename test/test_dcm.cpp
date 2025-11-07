#include "dcm.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

namespace ahrs {
  using namespace Eigen;

  TEST_GROUP(TEST_DCM) {
    //setup teardown as needed
  };

  TEST(TEST_DCM, default_ctor) {
    Matrix3f target = Matrix3f::Identity();

    DCM dcm;
    Matrix3f mat = dcm.to_matrix();

    CHECK(mat.isApprox(target, DCM::TOLERANCE));
  
  }

  TEST(TEST_DCM, assign_ctor) {
    Matrix3f target;
    target << 0,1,2,
              3,4,5,
              6,7,8;

    DCM dcm(target);
    Matrix3f mat = dcm.to_matrix();
    
    CHECK(mat.isApprox(target, DCM::TOLERANCE));
  }
}


int main(int argc, char** argv) {
  return CommandLineTestRunner::RunAllTests(argc, argv);
}
