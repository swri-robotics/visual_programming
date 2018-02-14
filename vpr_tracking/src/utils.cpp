/**
 * \file utils.cpp
 * \copyright (c) 2018 Southwest Research Institute
 */
#include "vpr_tracking/utils.h"

namespace vpr_tracking
{
namespace utils
{

geometry_msgs::TransformStamped poseToTransform(const geometry_msgs::PoseStamped &pose)
{
  geometry_msgs::TransformStamped t;
  t.header = pose.header;
  t.transform.translation.x = pose.pose.position.x;
  t.transform.translation.y = pose.pose.position.y;
  t.transform.translation.z = pose.pose.position.z;
  t.transform.rotation = pose.pose.orientation;
  return t;
}

}
}
