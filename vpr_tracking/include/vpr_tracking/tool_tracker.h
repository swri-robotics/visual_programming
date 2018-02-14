/**
 * \file tool_tracker.h
 * \copyright (c) 2018 Southwest Research Institute
 */
#ifndef VPR_TRACKING_TOOL_TRACKER_H
#define VPR_TRACKING_TOOL_TRACKER_H

#include "vpr_tracking/utils.h"
#include "vpr_tracking/tool.h"
#include "vpr_tracking/ToolTrackerConfig.h"

#include <ros/ros.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <boost/optional.hpp>

namespace vpr_tracking
{

namespace tool_tracking
{

struct Marker
{
  std_msgs::Header header;
  int id;
  Eigen::Isometry3d pose;
};

/**
 * \brief A correspondence between a measured pose and its known relative geometry
 */
struct PoseCorrespondence
{
  ///A pose in the tool coordinate system
  Eigen::Isometry3d reference;
  ///The pose measured by the camera
  Eigen::Isometry3d measured;
};

typedef std::vector<PoseCorrespondence> PoseCorrespondences;

/**
 * \brief Apply basic filtering to remove bad markers from the current set
 */
std::vector<Marker> filterMarkers(const std::vector<Marker> &markers, const Tool &tool);

/**
 * \brief Compute a best-fit tool pose that explains the measured marker poses
 */
boost::optional<Eigen::Isometry3d> calculateToolPose(
    const PoseCorrespondences &pose_corrs,
    const ToolTrackerConfig &config);

/**
 * @brief Compute a best-fit tool pose that explains the measured marker poses
 * @return An optional tool pose which is empty if a good solution is not found
 */
boost::optional<Eigen::Isometry3d> calculateToolPose(
    const PoseCorrespondences &pose_corrs,
    const Eigen::Isometry3d &init_pose,
    const ToolTrackerConfig &config);

} //end tool_tracking

} //end namespace

#endif
