/*
 * trajectory_planner.h
 *
 *  Created on: Jul 27, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_TRAJECTORY_PLANNER_H_
#define INCLUDE_TRAJECTORY_PLANNER_H_

#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vpr_msgs/PlanTrajectories.h>
#include <tf2_ros/transform_listener.h>

namespace vpr_trajectory
{

struct JumpThreshold
{
  double revolute = 0.0;
  double prismatic = 0.0;
};

struct MaxEEFStep
{
  double translation = 0.0;
  double rotation = 0.0;
};

class TrajectoryPlanner
{
public:
  TrajectoryPlanner(moveit::core::RobotModelConstPtr robot_model);
  virtual ~TrajectoryPlanner();

  /**
   * @brief Plans linear trajectories each segment of waypoints provided in the request.
   * @return True if planning succeeded, false otherwise.
   */
  bool plan(vpr_msgs::PlanTrajectoriesRequest& req,moveit_msgs::DisplayTrajectory& traj) const;


protected:

  double computeCartesianPath(moveit::core::RobotStatePtr ref_state,const moveit::core::JointModelGroup* group,
                              std::vector<moveit::core::RobotStatePtr>& traj,
                              const moveit::core::LinkModel* link,
                              const Eigen::Affine3d& start_pose, const Eigen::Affine3d& target_pose,
                              const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                              const moveit::core::GroupStateValidityCallbackFn& validCallback = moveit::core::GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  double computeCartesianPath(moveit::core::RobotStatePtr ref_state,const moveit::core::JointModelGroup* group,
                              std::vector<moveit::core::RobotStatePtr>& traj,
                              const moveit::core::LinkModel* link, const EigenSTL::vector_Affine3d& waypoints,
                              const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                              const moveit::core::GroupStateValidityCallbackFn& validCallback = moveit::core::GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  boost::optional<moveit_msgs::RobotTrajectory> generateJointTrajectory(const moveit_msgs::RobotState& start_state_msg,
                                                                         const moveit_msgs::RobotState& goal_state_msg,
                                                                         moveit::planning_interface::MoveGroupInterface& move_group) const;

  double testAbsoluteJointSpaceJump(moveit::core::RobotStatePtr ref_state,
                                          const moveit::core::JointModelGroup* group,
                                          const std::vector<moveit::core::RobotStatePtr>& traj,
                                          double revolute_threshold, double prismatic_threshold) const;


  boost::optional<Eigen::Affine3d> lookupTransform(std::string to_frame,std::string from_frame) const;


  moveit::core::RobotModelConstPtr robot_model_;
  JumpThreshold state_precision_ = {.revolute = M_PI/40, .prismatic = 0.01};

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

};

} /* namespace vpr_trajectory */

#endif /* INCLUDE_TRAJECTORY_PLANNER_H_ */
