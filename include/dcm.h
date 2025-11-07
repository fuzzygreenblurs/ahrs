#ifndef DCM_H
#define DCM_H

#include <Eigen/Core>
#include <Eigen/Dense> 

namespace ahrs {
  using namespace Eigen;  // Eigen used globally across AHRS module
  class DCM {
    public:
      static constexpr float TOLERANCE = 1e-4f;

      DCM();
      DCM(const Matrix3f& mat);
    
      static DCM from_euler_zyx(float roll, float pitch, float yaw);
      static DCM eye() { return DCM(); };
   
      DCM operator*(const DCM& other) const;
      Vector3f operator*(const Vector3f& vec) const;
      DCM inv() const;
      DCM T() const;                        
    
      Vector3f to_euler_zyx() const;
      Matrix3f to_matrix() const;
      bool is_so3() const;

    private:
      Matrix3f R_;
  };
}

#endif
