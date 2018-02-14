/**
 * \file utils.h
 * \copyright (c) 2018 Southwest Research Institute
 */
#ifndef VPR_TRACKING_UTILS_H
#define VPR_TRACKING_UTILS_H

#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>

namespace vpr_tracking
{

namespace utils
{
  geometry_msgs::TransformStamped poseToTransform(const geometry_msgs::PoseStamped &pose);

  /**
   * \brief Convert a pose to an Eigen transform. Templated so autodiff will work.
   */
  template <typename Scalar, int Mode>
  Eigen::Transform<Scalar, 3, Mode> poseToEigenTransform(const geometry_msgs::Pose &pose)
  {
    //explicitly convert double values to the Scalar autodiffable type
    Eigen::Translation<Scalar,3> trans(Scalar(pose.position.x), Scalar(pose.position.y), Scalar(pose.position.z));
    Eigen::Quaternion<Scalar> quat(
        Scalar(pose.orientation.w),
        Scalar(pose.orientation.x),
        Scalar(pose.orientation.y),
        Scalar(pose.orientation.z));
    return trans * quat;
  }

  template <typename Scalar, int Mode>
  Eigen::Transform<Scalar, 3, Mode> poseToEigenTransform(const geometry_msgs::PoseStamped &pose)
  {
    return poseToEigenTransform<Scalar, Mode>(pose.pose);
  }

  template <typename Scalar, int Mode>
  Eigen::Transform<Scalar, 3, Mode> poseToEigenTransform(const geometry_msgs::PoseWithCovariance &pose)
  {
    return poseToEigenTransform<Scalar, Mode>(pose.pose);
  }
}

}

#endif
