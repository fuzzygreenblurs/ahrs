#ifndef DCM_H
#define DCM_H

#include <Eigen/Core>
#include <Eigen/Dense> 

namespace ahrs {
  class DCM {
    public:
      static constexpr float TOLERANCE = 1e-4f;
      static bool is_so3(const Eigen::Matrix3f& mat);

      DCM();
      DCM(const Eigen::Matrix3f& mat);
    
      static DCM from_euler_zyx(float roll, float pitch, float yaw);
      static DCM eye() { return DCM(); };
   
      DCM operator*(const DCM& other) const;
      Eigen::Vector3f operator*(const Eigen::Vector3f& vec) const;
      DCM inv() const;
      DCM T() const;                        
    
      Eigen::Vector3f to_euler_zyx() const;
      Eigen::Matrix3f to_matrix() const;

    private:
      Eigen::Matrix3f R_;
  };
}

#endif
