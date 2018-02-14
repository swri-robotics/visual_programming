/*
 * trajectory_planner.cpp
 *
 *  Created on: Jul 27, 2018
 *      Author: ros-industrial
 */

#include "vpr_trajectory/trajectory_planner.h"
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <boost/format.hpp>

static const double DEFAULT_MAX_VELOCITY_SCALING_FACTOR = 0.5;
static const double DEFAULT_MAX_ACCELERATION_SCALING_FACTOR = 0.5;

bool validateState(const moveit_msgs::RobotState& st_msg)
{
  return !(st_msg.joint_state.position.empty() or st_msg.joint_state.name.empty());
}

bool compareStates(const moveit::core::RobotState& rs1,const moveit::core::RobotState& rs2,
                   const moveit::core::JointModelGroup* group, double max_revolute_distance, double max_prismatic_distance)
{
  using namespace moveit::core;

  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for(const JointModel* jm : joints)
  {
    double max_allowed_dist = jm->getType() == JointModel::REVOLUTE ? max_revolute_distance : max_prismatic_distance;
    double d = std::abs(rs1.distance(rs2,jm));

    if(d > max_allowed_dist)
    {
      return false;
    }
  }
  return true;
}

namespace vpr_trajectory
{

TrajectoryPlanner::TrajectoryPlanner(moveit::core::RobotModelConstPtr robot_model):
    robot_model_(robot_model),
	tf_listener_(tf_buffer_)
{

}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

bool TrajectoryPlanner::plan(vpr_msgs::PlanTrajectoriesRequest& req,moveit_msgs::DisplayTrajectory& solution) const
{
  using namespace moveit::core;

  if(!robot_model_->hasJointModelGroup(req.group_name))
  {
    ROS_ERROR("Provided group name '%s' is not valid", req.group_name.c_str());
    return false;
  }

  // verifying all segments are non-empty
  for(vpr_msgs::TrajectorySegment& seg : req.trajectory_segments)
  {
    if(seg.waypoints.empty())
    {
      ROS_ERROR("One or more trajectory segments are empty");
      return false;
    }
  }

  // create move group
  moveit::planning_interface::MoveGroupInterface move_group(req.group_name);
  if(!move_group.startStateMonitor())
  {
    ROS_ERROR("Failed to start move_group state monitor");
    return false;
  }

  // planning first move from start state to start of trajectory
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model_));
  start_state->setToDefaultValues();

  if(!validateState(req.start_state))
  {
    ROS_ERROR("Empty or invalid start state found, can not plan");
    return false;
  }
  robot_state::robotStateMsgToRobotState(req.start_state,*start_state,true);
  moveit_msgs::RobotState goal_state_msg = req.trajectory_segments.front().waypoints.front().robot_state;

  if(!validateState(goal_state_msg))
  {
    ROS_ERROR("Empty or invalid goal state found, can not plan");
    return false;
  }
  boost::optional<moveit_msgs::RobotTrajectory> start_traj_opt = generateJointTrajectory(req.start_state,
                                                                                   goal_state_msg,
                                                                                   move_group);
  if(!start_traj_opt)
  {
    ROS_ERROR("Planning from start state to first trajectory point failed");
    return false;
  }

  // storing first trajectory
  solution.trajectory_start = req.start_state;
  solution.trajectory.push_back(static_cast<moveit_msgs::RobotTrajectory>(*start_traj_opt));

  // preparing planning configuration
  JumpThreshold jump_threshold = {.revolute = req.jump_threshold_revolute,.prismatic = req.jump_threshold_prismatic};
  MaxEEFStep eef_step = {.translation = req.max_eef_step_translation, .rotation = req.max_eef_step_rotation};

  auto validate_range = [](double min, double max, double val, double use_val) -> double{
    return (val < min || val > max ) ? use_val : val;
  };

  const double max_vel_scaling = validate_range(1e-3, 1.0,req.max_vel_scaling_factor, DEFAULT_MAX_VELOCITY_SCALING_FACTOR);
  const double max_acc_scaling = validate_range(1e-3, 1.0,req.max_acc_scaling_factor, DEFAULT_MAX_ACCELERATION_SCALING_FACTOR);

  ROS_INFO("Planning for %lu trajectory segments",req.trajectory_segments.size());
  std::string ref_link = "";
  boost::optional<Eigen::Affine3d> root_to_ref_transform;
  std::string model_frame = start_state->getRobotModel()->getModelFrame().c_str();
  for(std::size_t i = 0; i < req.trajectory_segments.size(); i++)
  {
    std::vector<vpr_msgs::TrajectoryWaypoint> waypoints = req.trajectory_segments[i].waypoints;
    if(waypoints.empty())
    {
      continue;
    }

    // checking reference frame
    if(ref_link != waypoints.front().tool_pose.header.frame_id)
    {
      root_to_ref_transform = boost::none;
      std::string ref_link = waypoints.front().tool_pose.header.frame_id;

      if(start_state->knowsFrameTransform(ref_link))
      {
        root_to_ref_transform = start_state->getFrameTransform(ref_link);
        ROS_INFO_COND(root_to_ref_transform.is_initialized(),"Grabbed transform from '%s' to '%s' using the RobotModel",
                      model_frame.c_str(),ref_link.c_str());
      }
      else
      {
        root_to_ref_transform = lookupTransform(start_state->getRobotModel()->getModelFrame(),ref_link);
        ROS_INFO_COND(root_to_ref_transform.is_initialized(),"Grabbed transform from '%s' to '%s' using TF",
                      model_frame.c_str(),ref_link.c_str());
      }
    }

    // check transform
    if(!root_to_ref_transform.is_initialized())
    {
      ROS_ERROR("Can not transform from model frame '%s' to frame '%s'",model_frame.c_str(),ref_link.c_str());
      return false;
    }

    // transforming all tool poses to root frame
    EigenSTL::vector_Affine3d tool_poses;
    std::transform(waypoints.begin(),waypoints.end(),std::back_inserter(tool_poses),[&](
        const vpr_msgs::TrajectoryWaypoint& wp)
    {
      Eigen::Affine3d tp;
      tf::poseMsgToEigen(wp.tool_pose.pose,tp);
      return root_to_ref_transform.get() * tp;
    });

    // computing path
    ROS_INFO_STREAM("Generating trajectories for tool link " << move_group.getEndEffectorLink());
    const moveit::core::LinkModel* tool_link = start_state->getLinkModel(move_group.getEndEffectorLink());
    std::vector<moveit::core::RobotStatePtr> traj;
    robot_state::robotStateMsgToRobotState(waypoints.front().robot_state,*start_state);
    const JointModelGroup* joint_model_group = start_state->getJointModelGroup(move_group.getName());
    double completion = computeCartesianPath(start_state, joint_model_group,traj,tool_link,tool_poses,
                         eef_step,jump_threshold);

    if(completion < 1.0)
    {
      ROS_ERROR("Unable to plan for trajectory segment '%lu'",i);
      return false;
    }

    moveit_msgs::RobotTrajectory rob_traj;
    rob_traj.joint_trajectory.joint_names = move_group.getJointNames();
    for(auto& rs : traj)
    {
      trajectory_msgs::JointTrajectoryPoint jp;
      jp.positions.resize(move_group.getJointNames().size());
      rs->copyJointGroupPositions(move_group.getName(),jp.positions);
      jp.effort.resize(jp.positions.size());
      jp.velocities.resize(jp.positions.size());
      jp.accelerations.resize(jp.positions.size());
      rob_traj.joint_trajectory.points.push_back(jp);
    }

    trajectory_processing::IterativeParabolicTimeParameterization time_data_filler;
    robot_trajectory::RobotTrajectory rob_traj_obj(start_state->getRobotModel(),move_group.getName());
    rob_traj_obj.setRobotTrajectoryMsg(*start_state,rob_traj.joint_trajectory);

    if(!time_data_filler.computeTimeStamps(rob_traj_obj,max_vel_scaling,max_acc_scaling))
    {
      ROS_ERROR("Time Parameterization failed");
      return false;
    }

    ROS_DEBUG("Time Parameterization complete");

    moveit_msgs::RobotTrajectory rob_traj_timed;
    rob_traj_obj.getRobotTrajectoryMsg(rob_traj_timed);


    // check to see if an intermediate trajectory that connects to the previous segment is needed
    if(i > 0)
    {
      // compare last state from previous with first from current
      RobotStatePtr r1(new RobotState(*start_state));
      RobotStatePtr r2(new RobotState(*start_state));

      r1->setJointGroupPositions(joint_model_group,solution.trajectory.back().joint_trajectory.points.back().positions);
      r2->setJointGroupPositions(joint_model_group,rob_traj_timed.joint_trajectory.points.front().positions);

      if(!compareStates(*r1,*r2,joint_model_group,state_precision_.revolute,state_precision_.prismatic))
      {
        std::string info_msg =  boost::str(
            boost::format("Planning joint trajectory between segments %1% and %2%") % int(i - 1) % int(i));
        ROS_INFO_STREAM(info_msg);

        // too far so create plan then
        moveit_msgs::RobotState rs_msg1, rs_msg2;
        robot_state::robotStateToRobotStateMsg(*r1,rs_msg1);
        robot_state::robotStateToRobotStateMsg(*r2,rs_msg2);
        boost::optional<moveit_msgs::RobotTrajectory> intermediate_traj = generateJointTrajectory(rs_msg1,rs_msg2,
                                                                                         move_group);
        if(!intermediate_traj.is_initialized())
        {
          std::string error_msg = boost::str(
              boost::format("Failed to plan intermediate trajectory between segments %1% and %2%") % int(i-1) % int(i));
          ROS_ERROR_STREAM(error_msg);
          return false;
        }
        solution.trajectory.push_back(static_cast<moveit_msgs::RobotTrajectory>(*intermediate_traj));
      }
    }
    solution.trajectory.push_back(rob_traj_timed);
  }

  return true;
}

boost::optional<moveit_msgs::RobotTrajectory> TrajectoryPlanner::generateJointTrajectory(
    const moveit_msgs::RobotState& start_state_msg,const moveit_msgs::RobotState& goal_state_msg,
    moveit::planning_interface::MoveGroupInterface& move_group) const
{
  //ROS_INFO_STREAM(start_state_msg);
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model_));
  robot_state::robotStateMsgToRobotState(start_state_msg, *start_state);
  move_group.setStartState(*start_state);
  move_group.setJointValueTarget(goal_state_msg.joint_state);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::MoveItErrorCodes error_code = move_group.plan(plan);
  return boost::make_optional(error_code.val == error_code.SUCCESS,plan.trajectory_);
}

double TrajectoryPlanner::computeCartesianPath(moveit::core::RobotStatePtr ref_state,
                            const moveit::core::JointModelGroup* group, std::vector<moveit::core::RobotStatePtr>& traj,
                            const moveit::core::LinkModel* link, const EigenSTL::vector_Affine3d& waypoints,
                            const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                            const moveit::core::GroupStateValidityCallbackFn& validCallback,
                            const kinematics::KinematicsQueryOptions& options) const
{
  using namespace moveit::core;
  static const JumpThreshold no_joint_space_jump_test;
  double percentage_solved = 0.0;

  if(waypoints.size() == 1)
  {
    //convert to geometry_msgs type to be kinetic and melodic compatible
    geometry_msgs::Pose gpose;
    tf::poseEigenToMsg(waypoints.front(), gpose);

    if(ref_state->setFromIK(group, gpose, link->getName(), 1, 0.0, validCallback, options))
    {
      ROS_WARN("A trajectory with one waypoint was received, returning single waypoint trajectory with IK solution");
      traj.push_back(RobotStatePtr(new RobotState(*ref_state)));
      percentage_solved = 1.0;
    }
    else
    {
      ROS_ERROR("IK failed on single waypoint trajectory");
      return 0.0;
    }
  }

  for (std::size_t i = 1; i < waypoints.size(); ++i)
  {
    // Don't test joint space jumps for every waypoint, test them later on the whole trajectory.
    std::vector<RobotStatePtr> waypoint_traj;
    double wp_percentage_solved = computeCartesianPath(ref_state,group, waypoint_traj, link, waypoints[i-1],waypoints[i],
                                                       max_step, no_joint_space_jump_test, validCallback, options);

    if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
    {
      percentage_solved = (double)(i + 1) / (double)waypoints.size();
      std::vector<RobotStatePtr>::iterator start = waypoint_traj.begin();
      if (i > 0 && !waypoint_traj.empty())
        std::advance(start, 1); // skip first point as it's equal to the last point from the previous segment
      traj.insert(traj.end(), start, waypoint_traj.end());
    }
    else
    {
      percentage_solved += wp_percentage_solved / (double)waypoints.size();
      std::vector<RobotStatePtr>::iterator start = waypoint_traj.begin();
      if (i > 0 && !waypoint_traj.empty())
        std::advance(start, 1);
      traj.insert(traj.end(), start, waypoint_traj.end());
      break;
    }
  }

  if(traj.size() > 1)
  {
    percentage_solved *= testAbsoluteJointSpaceJump(ref_state,group, traj,
                                                    jump_threshold.revolute, jump_threshold.prismatic);
  }



  return percentage_solved;
}

double TrajectoryPlanner::computeCartesianPath(moveit::core::RobotStatePtr ref_state,
                            const moveit::core::JointModelGroup* group,
                            std::vector<moveit::core::RobotStatePtr>& traj,
                            const moveit::core::LinkModel* link,
                            const Eigen::Affine3d& start_pose, const Eigen::Affine3d& target_pose,
                            const MaxEEFStep& max_step,
                            const JumpThreshold& jump_threshold,
                            const moveit::core::GroupStateValidityCallbackFn& validCallback,
                            const kinematics::KinematicsQueryOptions& options) const
{
  using namespace moveit::core;
  static const double MIN_STEPS_FOR_JUMP_THRESH = 3;

  const std::vector<const JointModel*>& cjnt = group->getContinuousJointModels();
  // make sure that continuous joints wrap
  for (std::size_t i = 0; i < cjnt.size(); ++i)
    ref_state->enforceBounds(cjnt[i]);

  Eigen::Quaterniond start_quaternion(start_pose.rotation());
  Eigen::Quaterniond target_quaternion(target_pose.rotation());

  if (max_step.translation <= 0.0 && max_step.rotation <= 0.0)
  {
    ROS_ERROR_NAMED("robot_state",
                    "Invalid MaxEEFStep passed into computeCartesianPath. Both the MaxEEFStep.rotation and "
                    "MaxEEFStep.translation components must be non-negative and at least one component must be "
                    "greater than zero");
    return 0.0;
  }

  double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  double translation_distance = (target_pose.translation() - start_pose.translation()).norm();

  // decide how many steps we will need for this trajectory
  std::size_t translation_steps = 0;
  if (max_step.translation > 0.0)
    translation_steps = floor(translation_distance / max_step.translation);

  std::size_t rotation_steps = 0;
  if (max_step.rotation > 0.0)
    rotation_steps = floor(rotation_distance / max_step.rotation);

  // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
  std::size_t steps = std::max(translation_steps, rotation_steps) + 1;
  if (steps < MIN_STEPS_FOR_JUMP_THRESH)
    steps = MIN_STEPS_FOR_JUMP_THRESH;

  traj.clear();
  traj.push_back(RobotStatePtr(new RobotState(*ref_state)));

  double last_valid_percentage = 0.0;
  ROS_DEBUG("Generating linear segment with %lu waypoints",steps);
  for (std::size_t i = 1; i <= steps; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * target_pose.translation() + (1 - percentage) * start_pose.translation();

    //convert to geometry_msgs type to be kinetic and melodic compatible
    geometry_msgs::Pose gpose;
    tf::poseEigenToMsg(pose, gpose);

    if (ref_state->setFromIK(group, gpose, link->getName(), 1, 0.0, validCallback, options))
    {
      traj.push_back(RobotStatePtr(new RobotState(*ref_state)));
    }
    else
    {
      ROS_ERROR("Failed to find IK at waypoint %lu",i);
      break;
    }

    last_valid_percentage = percentage;
  }

  if(!traj.empty())
  {
    last_valid_percentage *= testAbsoluteJointSpaceJump(ref_state,group, traj,
                                                        jump_threshold.revolute,
                                                        jump_threshold.prismatic);
  }

  return last_valid_percentage;
}

double TrajectoryPlanner::testAbsoluteJointSpaceJump(moveit::core::RobotStatePtr ref_state,
                                  const moveit::core::JointModelGroup* group,
                                  const std::vector<moveit::core::RobotStatePtr>& traj,
                                  double revolute_threshold, double prismatic_threshold) const
{

  using namespace moveit::core;

  struct LimitData
  {
    double limit;
    bool check;
  };
  LimitData data[2] = { { revolute_threshold, revolute_threshold > 0.0 },
                        { prismatic_threshold, prismatic_threshold > 0.0 } };
  bool still_valid = true;
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (std::size_t traj_ix = 0; traj_ix < traj.size() - 1; traj_ix++)
  {
    for (auto& joint : joints)
    {
      unsigned int type_index;
      switch (joint->getType())
      {
        case JointModel::REVOLUTE:
          type_index = 0;
          break;
        case JointModel::PRISMATIC:
          type_index = 1;
          break;
        default:
          ROS_WARN_NAMED("robot_state", "Joint %s has not supported type %s. \n"
                                        "testAbsoluteJointSpaceJump only supports prismatic and revolute joints.",
                         joint->getName().c_str(), joint->getTypeName().c_str());
          continue;
      }

      if (!data[type_index].check)
        continue;

      double distance = traj[traj_ix]->distance(*traj[traj_ix + 1], joint);
      if (distance > data[type_index].limit)
      {
        ROS_DEBUG_NAMED("robot_state", "Truncating Cartesian path due to detected jump of %.4f > %.4f in joint %s",
                        distance, data[type_index].limit, joint->getName().c_str());
        still_valid = false;
        break;
      }
    }

    if (!still_valid)
    {
      double percent_valid = (double)(traj_ix + 1) / (double)(traj.size());
      return percent_valid;;
    }
  }
  return 1.0;
}

boost::optional<Eigen::Affine3d> TrajectoryPlanner::lookupTransform(std::string to_frame,  std::string from_frame) const
{
	auto filter = [](const std::string& str) -> std::string
	{
	  std::string new_str;
	  if(str.front() == '/')
	  {
	    new_str.insert(new_str.end(),str.begin()+1,str.end());
	  }
	  else
	  {
	    new_str = str;
	  }
	  return new_str;
	};

	Eigen::Affine3d transform;
	try
	{
		geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform(filter(to_frame),
				filter(from_frame),ros::Time(0));
		tf::transformMsgToEigen(transform_msg.transform,transform);
	}
	catch(tf2::TransformException& e)
	{
		ROS_ERROR_STREAM(e.what());
		return boost::none;
	}

	return transform;
}

} /* namespace vpr_trajectory */
