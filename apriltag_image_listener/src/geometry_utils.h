#include "tf/tf.h"
#include <Eigen/Geometry>


inline Eigen::Isometry2f convertPose2D(const tf::StampedTransform& t) {
  double yaw,pitch,roll;
  tf::Matrix3x3 mat = t.getBasis();
  mat.getRPY(roll, pitch, yaw);
  Eigen::Isometry2f T;
  T.setIdentity();
  Eigen::Matrix2f R;
  R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  T.linear() = R;
  T.translation() = Eigen::Vector2f(t.getOrigin().x(), t.getOrigin().y());
  return T;
}
