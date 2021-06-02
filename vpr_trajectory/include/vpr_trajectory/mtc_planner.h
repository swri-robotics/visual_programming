/*
 * mtc_planner.h
 *
 *  Created on: Jun 1, 2021
 *      Author: Jorge Nicho
 */

#ifndef INCLUDE_VPR_TRAJECTORY_MTC_PLANNER_H_
#define INCLUDE_VPR_TRAJECTORY_MTC_PLANNER_H_

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>

#include <tf2_ros/transform_listener.h>

#include <vpr_msgs/PlanTrajectories.h>

#include "vpr_trajectory/stages/plan_toolpath.h"

namespace vpr_trajectory {

class MTCPlanner {
public:
	MTCPlanner(moveit::core::RobotModelConstPtr robot_model);
	virtual ~MTCPlanner();

	/**
	 * @brief Plans linear trajectories each segment of waypoints provided in the request.
	* @return True if planning succeeded, false otherwise.
	*/
	bool plan(const vpr_msgs::PlanTrajectoriesRequest& req,moveit_msgs::DisplayTrajectory& solution) const;

protected:

	std::unique_ptr<moveit::task_constructor::stages::FixedState> createFixedStateStage(const moveit_msgs::RobotState& st_msg) const;

	std::unique_ptr<moveit::task_constructor::stages::MoveTo> createFreeSpacePlanStage(const std::string& stage_id,
	                                                                                   const std::string& group_name,
	                                                                                   const vpr_msgs::TrajectoryWaypoint& wp) const;

  std::unique_ptr<vpr_trajectory::stages::PlanToolpath> createToolpathPlanStage(const std::string& stage_id,
                                                                                     const std::string& group_name,
                                                                                     const vpr_msgs::PlanTrajectoriesRequest &properties,
                                                                                     const vpr_msgs::TrajectorySegment& segment) const;

  boost::optional<Eigen::Isometry3d> lookupTransform(std::string to_frame,  std::string from_frame) const;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

	moveit::core::RobotModelConstPtr robot_model_;
	std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> scene_monitor_;
};

} /* namespace vpr_trajectory */

#endif /* INCLUDE_VPR_TRAJECTORY_MTC_PLANNER_H_ */
