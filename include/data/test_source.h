#ifndef IMU_TEST_SOURCE_H
#define IMU_TEST_SOURCE_H

#include "interfaces/i_imu_data_source.h"
#include <vector>
#include <string>

namespace AHRS {
  using namespace std;
  using namespace Eigen;

  class TestSource : public IIMUDataSource {
    static constexpr float MAG_NOT_AVAILABLE = -9999.0f;

    public:
      TestSource(const string& data_dir);
      bool has_next() const override;
      IMUReading get_next() override;

      bool has_ground_truth() const override { return true; };
      Eigen::Vector3f get_ground_truth_euler() override;

      void reset();
      size_t size() const { return gyro_data_.size(); }

    private:
      void load_csv(const string& path,
                    vector<Vector3f>& data);

      void load_csv(const string& path,
                    vector<float>& data);

      vector<Vector3f> gyro_data_;
      vector<Vector3f> accel_data_;
      vector<Vector3f> gt_data_;
      vector<float> dt_data_;

      size_t current_idx_;
      float current_time_;
  };
}

#endif
