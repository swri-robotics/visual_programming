/**
 * \file tool.cpp
 * \copyright (c) 2018 Southwest Research Institute
 */
#include "vpr_tracking/tool.h"
#include "vpr_tracking/utils.h"

#include <numeric>
#include <boost/format.hpp>

#include <urdf/model.h>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>

using EigenTf = Eigen::Isometry3d;

/**
 * \brief Computes the pose of a URDF link in coordinates of the URDF's root link
 * \throws std::runtime_error
 *
 * Follows the URDF tree up from the link to the root link and composes the transforms specified by all the joints in
 * between. Requires all joints between the two links to be fixed.
 */
geometry_msgs::PoseStamped poseInRootCoords(urdf::LinkConstSharedPtr link, std::unique_ptr<urdf::Model> &model)
{
  urdf::LinkConstSharedPtr root_link = model->getRoot();
  std::string leaf_link_name = link->name;
  std::vector<urdf::Pose> link_poses;

  while (link->name != root_link->name)
  {
    urdf::JointConstSharedPtr joint = link->parent_joint;
    if (joint->type != urdf::Joint::FIXED)
    {
      throw std::runtime_error(boost::str(
          boost::format("Joint '%1%' in tree from '%2%' to '%3%' not fixed; can't use as a reference pose") %
              joint->name %
              root_link->name %
              leaf_link_name));
    }
    link_poses.insert(link_poses.begin(), joint->parent_to_joint_origin_transform);
    link = model->getLink(joint->parent_link_name);
  }

  EigenTf root_t = EigenTf::Identity();
  for (auto & link_pose : link_poses)
  {
    geometry_msgs::Pose inc_pose;
    geometry_msgs::Quaternion &quat = inc_pose.orientation;
    inc_pose.position.x = link_pose.position.x;
    inc_pose.position.y = link_pose.position.y;
    inc_pose.position.z = link_pose.position.z;
    link_pose.rotation.getQuaternion(quat.x, quat.y, quat.z, quat.w);

    EigenTf inc_t;
    tf::poseMsgToEigen(inc_pose, inc_t);

    root_t = root_t * inc_t;
  }

  geometry_msgs::PoseStamped root_pose;
  root_pose.header.frame_id = root_link->name;
  tf::poseEigenToMsg(root_t, root_pose.pose);
  return root_pose;
}

/**
 * \brief Extracts all links from a URDF of form 'prefixID' as reference poses of the tool.
 *
 * ID must be parseable as an int and all the poses are stored in \param poses in coordinates of the root link of the
 * URDF.
 */
void extractMarkerPoses(
    std::unique_ptr<urdf::Model> &model,
    const std::string &prefix,
    std::map<int, geometry_msgs::PoseStamped> &poses)
{
  std::vector<urdf::LinkSharedPtr> links;
  model->getLinks(links);
  for (auto & link : links)
  {
    if (link->name.substr(0, prefix.length()) == prefix)
    {
      try
      {
        std::string id_str = link->name.substr(prefix.length()); //everything after the prefix
        if (std::any_of(id_str.begin(), id_str.end(), [](char c){return std::isdigit(c) == 0;}))
          throw std::runtime_error("ID from link '" + link->name + "' not an integer");
        int id = std::stoi(id_str);
        poses.insert(std::make_pair(id, poseInRootCoords(link, model)));
      }
      catch (std::runtime_error &ex)
      {
        ROS_WARN_STREAM(ex.what());
      }
    }
  }
}

vpr_tracking::Tool::Tool(const std::string &tool_description, const std::string &prefix) :
  tool_model_(new urdf::Model)
{
  if (! tool_model_->initString(tool_description))
    throw std::runtime_error("Failed to generate URDF model from tool description");

  extractMarkerPoses(tool_model_, prefix, marker_poses_);

  std::string report = std::accumulate(marker_poses_.begin(),marker_poses_.end(),std::string(""),[&](std::string s,const auto& kv) ->std::string
  {
    const geometry_msgs::PoseStamped& pose_st = kv.second;
    EigenTf pose;
    tf::poseMsgToEigen(pose_st.pose,pose);
    Eigen::Vector3d pos = pose.translation();
    Eigen::Vector3d rpy = pose.rotation().matrix().eulerAngles(0,1,2);
    s = s + std::string("\n") + boost::str(boost::format(
        "Marker %i pose relative to  '%s' : [x: %1.3f, y: %1.3f, z: %1.3f, rx: %1.3f, ry: %1.3f, rz: %1.3f]")
    % kv.first % pose_st.header.frame_id % pos.x() % pos.y() % pos.z() % rpy.x() % rpy.y() % rpy.z());
    return s;
  });

  ROS_DEBUG_STREAM(report);
}

//Explicit destructor is needed to have a std::unique_ptr of an incomplete type in the header
vpr_tracking::Tool::~Tool() = default;

bool vpr_tracking::Tool::markerPose(int marker_id, geometry_msgs::PoseStamped &pose) const
{
  if (marker_poses_.count(marker_id) == 0)
    return false;
  pose = marker_poses_.at(marker_id);
  return true;
}

bool vpr_tracking::Tool::hasMarker(int marker_id) const
{
  return marker_poses_.count(marker_id) > 0;
}

std::string vpr_tracking::Tool::getFrameID() const
{
  return tool_model_->getRoot()->name;
}
