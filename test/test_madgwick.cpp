#include "filters/madgwick.h"
#include "data/test_source.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include <iostream>
#include <fstream>

namespace AHRS {
  using namespace Eigen;

  TEST_GROUP(MADGWICK) {};

  TEST(MADGWICK, constructor_intializes_identity) {
    MadgwickFilter filter;
    Quaternion q = filter.get_orientation();

    DOUBLES_EQUAL(1.0f, q.w(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(0.0f, q.x(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(0.0f, q.y(), Quaternion::TOLERANCE);
    DOUBLES_EQUAL(0.0f, q.z(), Quaternion::TOLERANCE);
  }

  TEST(MADGWICK, quaternion_stays_normalized) {
    MadgwickFilter filter;
    Vector3f gyro_reading(0.1f, 0.2f, 0.3f);   // non-zero dummy reading 
    Vector3f accel_reading(0.0f, 0.0f, 9.81f); // dummy accel values

    for(int i = 0; i < 100; i++) {
      filter.update(gyro_reading, accel_reading, 0.01);
    }

    Quaternion q = filter.get_orientation();
    DOUBLES_EQUAL(1.0f, q.norm(), Quaternion::TOLERANCE);
  }

  TEST(MADGWICK, six_dof_stationary_stays_level) {
    MadgwickFilter filter;
    Vector3f gyro_reading(0.0f, 0.0f, 0.0f);
    Vector3f accel_reading(0.0f, 0.0f, 9.81f);

    for(int i = 0; i < 100; i++) {
      filter.update(gyro_reading, accel_reading, 0.01);
    }

    Vector3f euler = filter.get_euler();
    CHECK(abs(euler(0)) < 0.1f);
    CHECK(abs(euler(1)) < 0.1f);
  
    Quaternion q = filter.get_orientation();
    DOUBLES_EQUAL(1.0f, q.norm(), Quaternion::TOLERANCE);
  }

  TEST(MADGWICK, six_dof_vn100_integration_test) {
    TestSource data("data/");
    std::ofstream results("analysis/test_set/madgwick_6dof_estimate.csv");
    const char* header = "time,"
                         "roll_est,pitch_est,yaw_est,"
                         "roll_gt,pitch_gt,yaw_gt,"
                         "roll_error,pitch_error,yaw_error\n";
    results << header; 

    float time = 0.0;
    Vector3f accum_error(0, 0, 0);
    
    MadgwickFilter filter;
    int count = 0;
    while(data.has_next()) {
      auto reading = data.get_next();
      filter.update(reading.gyro, reading.accel, reading.dt);
      
      Vector3f est_euler = filter.get_euler(); 
      Vector3f gt_euler = data.get_ground_truth_euler();
      Vector3f error = est_euler - gt_euler;

      results << time << ","
              << est_euler(0) << "," << est_euler(1) << "," << est_euler(2) << ","
              <<  gt_euler(0) << "," <<  gt_euler(1) << "," <<  gt_euler(2) << ","
              <<     error(0) << "," <<     error(1) << "," <<     error(2) 
              << "\n";

      accum_error += error.cwiseAbs2();
      count++;
      time += reading.dt;
    }

    Vector3f rmse = (accum_error / count).cwiseSqrt();
    std::cout << "RMSE: r: " << rmse(0) << "p: " << rmse(1) << "y: " << rmse(2);
    results.close();
  }

  TEST(MADGWICK, beta_tuning) {
    std::vector<float> beta_values = {
      0.001f, 0.005f, 0.01f, 0.03f, 0.05f,
      0.1f, 0.2f, 0.3f, 0.5f, 0.7f, 1.0f 
    };

    std::ofstream results("analysis/test_set/beta_tuning_results.csv");
    results << "beta,rmse_roll,rmse_pitch,rmse_yaw,rmse_combined\n";
    
    float best_beta = 0.0f;
    float best_combined_rmse = 1e9f;

    for(float beta : beta_values) {
      TestSource data("data/");
      MadgwickFilter filter(beta);

      Vector3f accum_error(0,0,0);
      int count = 0;
      
      while(data.has_next()) {
        auto reading = data.get_next();
        filter.update(reading.gyro, reading.accel, reading.dt);
      
        Vector3f est_euler = filter.get_euler();
        Vector3f gt_euler = data.get_ground_truth_euler();
        Vector3f error = est_euler - gt_euler;

        accum_error += error.cwiseAbs2();
        count++;
      }

      Vector3f rmse = (accum_error/count).cwiseSqrt();
      float combined_rmse = sqrt(
        (pow(rmse(0), 2) + pow(rmse(1), 2)) / 2.0f
      );
      
      results << beta << "," 
              << rmse(0) << "," << rmse(1) << "," << rmse(2) << ","
              << combined_rmse << "\n";

      if(combined_rmse < best_combined_rmse) {
        best_combined_rmse = combined_rmse;     
        best_beta = beta;
      }

      std::cout << "\nbeta: " << beta
                << " |  roll RMSE: " << rmse(0)
                << " | pitch RMSE: " << rmse(1)
                << " |   combined: " << combined_rmse;

    }
    
    results.close();
    std::cout << "\n\noptimal beta: "   << best_beta
              << "with combined rmse: " << best_combined_rmse << "\n";
  }
}
