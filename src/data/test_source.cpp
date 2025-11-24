#include "data/test_source.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace AHRS {
  TestSource::TestSource(const string& data_dir)
    : current_idx_(0), current_time_(0.0) {
    
    load_csv(data_dir + "/omega.csv", gyro_data_);
    load_csv(data_dir + "/a.csv", accel_data_);
    load_csv(data_dir + "/euler_gt.csv", gt_data_);
    load_csv(data_dir + "/dt.csv", dt_data_);

    size_t n = gyro_data_.size();
    if(accel_data_.size() != n ||
          gt_data_.size() != n ||
          dt_data_.size() != n) {
      
      throw runtime_error("test set has mismatched dataset lengths."); 
    }
  }

  bool TestSource::has_next() const {
    return current_idx_ < gyro_data_.size();
  }

  IMUReading TestSource::get_next() {
    if(!has_next()) {
      throw runtime_error("no more data available."); 
    }

    IMUReading reading;
    reading.timestamp = current_time_;
    reading.gyro = gyro_data_[current_idx_];
    reading.accel = accel_data_[current_idx_];  
    reading.mag = Vector3f::Constant(MAG_NOT_AVAILABLE);
    reading.dt = dt_data_[current_idx_];

    current_time_ += reading.dt;
    current_idx_++;

    return reading;
  }

  Vector3f TestSource::get_ground_truth_euler() {
    if(current_idx_ == 0) { 
      throw runtime_error("current_idx_ must already be incremented by get_next");
    }

    return gt_data_[current_idx_ - 1]; 
  }

  void TestSource::reset() {
    current_idx_ = 0;
    current_time_ = 0; 
  }

  //TODO: consolidate these two functions
  void TestSource::load_csv(const string& filepath,
                            vector<Vector3f>& data) {
  
    ifstream file(filepath);
    if(!file.is_open()) {
      throw runtime_error("failed to open file: " + filepath);
    }

    data.clear();
    string line;
    while(getline(file, line)) {
      stringstream ss(line);
      float x,y,z;
      char delim;

      ss >> x >> delim >> y >> delim >> z;
      data.push_back(Vector3f(x, y, z));
    }
    
    file.close();
  }

  void TestSource::load_csv(const string& filepath,
                            vector<float>& data) {
    ifstream file(filepath); 
    if(!file.is_open()) {
      throw runtime_error("failed to open file: " + filepath);
    }

    data.clear();
    string line;
    while(getline(file, line)) {
      data.push_back(stof(line));
    }

    file.close();
  }
}

