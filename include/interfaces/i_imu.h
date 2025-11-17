#ifndef I_IMU
#define I_IMU

#include <Eigen/Core>

namespace AHRS {
  class IIMU {
    public:
      virtual ~IIMU() = default;
    
      virtual Eigen::Vector3f read_gyro()  = 0;
      virtual Eigen::Vector3f read_accel() = 0;
      virtual Eigen::Vector3f read_mag()   = 0;

      virtual float get_sample_rate() const = 0;
      virtual bool has_mag() const = 0;
  };
}

#endif
