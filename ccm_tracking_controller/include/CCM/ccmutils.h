#ifndef CCM_UTILS_H
#define CCM_UTILS_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <math.h>

namespace ccmutils
{
  void quat2rotM(const Eigen::Vector4d& q, Eigen::Matrix3d &R);
  void rotM2quat(Eigen::Vector4d& q, const Eigen::Matrix3d &R);
}


#endif /* CCM_UTILS_H */
