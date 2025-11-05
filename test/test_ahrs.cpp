#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

TEST_GROUP(BasicTests) {};

TEST(BasicTests, Addition) {
  int result = 4 + 1;
  CHECK_EQUAL(5, result);
}

int main(int argc, char** argv) {
  return CommandLineTestRunner::RunAllTests(argc, argv);
}
