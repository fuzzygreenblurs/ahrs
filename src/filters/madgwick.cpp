#include "filters/madgwick.h"
#include "math/dcm.h"
#include "math/quaternion.h"

using namespace Eigen;

void MadgwickFilter::update(const Vector3f& gyro,
                            const Vector3f& accel,
                            const float dt) {

  // compute gradient
  // update quaternion and normalize
}

Quaternion MadgwickFilter::get_orientation() const {
  return q_;
}

Vector3f MadgwickFilter::get_euler() const {
  // convert quaternion to euler angles
}
