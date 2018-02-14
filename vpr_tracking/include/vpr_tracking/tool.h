/**
 * \file tool.h
 * \copyright (c) 2018 Southwest Research Institute
 */
#ifndef VPR_TRACKING_TOOL_H
#define VPR_TRACKING_TOOL_H

#include <map>
#include <geometry_msgs/PoseStamped.h>

namespace urdf
{
  class Model;
}

namespace vpr_tracking
{

class Tool
{
public:
  /**
   * \brief Constructs a Tool object from a URDF XML string.
   * \throws std::runtime_error
   *
   * All links in the URDF description matching a prefix value will be taken as markers for observation and their
   * coordinates in the tool geometry will be cached for use in the tracking loop.
   */
  Tool(const std::string &tool_description, const std::string &prefix);
  virtual ~Tool();

  /**
   * \brief Retrieves the pose of the marker on the tool with a given numeric ID.
   * \returns true if a marker with the given ID exists in the tool, false if not
   */
  bool markerPose(int marker_id, geometry_msgs::PoseStamped &pose) const;

  /**
   * @brief Checks if the marker ID is part of the tool
   * @returns True on success, false otherwise.
   */
  bool hasMarker(int marker_id) const;

  /**
   * \brief Returns the tool coordinate system, identified by the name of the root link in the URDF.
   */
  std::string getFrameID() const;

private:
  std::unique_ptr<urdf::Model> tool_model_;
  std::map<int, geometry_msgs::PoseStamped> marker_poses_;
};

}
#endif
