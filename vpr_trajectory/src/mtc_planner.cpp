/*
 * mtc_planner.cpp
 *
 *  Created on: Jun 1, 2021
 *      Author: jnicho
 */

#include <tf2_eigen/tf2_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/task_constructor/task.h>

#include "vpr_trajectory/mtc_planner.h"

namespace mtc = moveit::task_constructor;

static const std::string ROBOT_DESCRIPTION= "robot_description";
static const double  DEFAULT_TIMEOUT = 30;

static std::shared_ptr<mtc::solvers::PipelinePlanner>  createFreeSpacePlanner()
{
  using namespace mtc;
  std::shared_ptr<mtc::solvers::PipelinePlanner> freespace_planner = std::make_shared<solvers::PipelinePlanner>();
  freespace_planner->setPlannerId("RRTConnectkConfigDefault");
  freespace_planner->setProperty("goal_joint_tolerance", 0.01);
  freespace_planner->setProperty("goal_position_tolerance", 0.005);
  freespace_planner->setProperty("goal_orientation_tolerance", 0.01);
  freespace_planner->setProperty("num_planning_attempts", 4);
  freespace_planner->setProperty("max_velocity_scaling_factor", 0.5);
  freespace_planner->setProperty("max_acceleration_scaling_factor", 0.5);
  return freespace_planner;
}

namespace vpr_trajectory {

MTCPlanner::MTCPlanner(moveit::core::RobotModelConstPtr robot_model):
  robot_model_(robot_model),
  tf_listener_(tf_buffer_)
{

  scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
  scene_monitor_->startStateMonitor();
  scene_monitor_->startSceneMonitor();
  if (!scene_monitor_->waitForCurrentRobotState(ros::Time::now(), 5.0))
  {
    throw std::runtime_error("Failed to get current robot state");
  }
  scene_monitor_->updateSceneWithCurrentState();
}

MTCPlanner::~MTCPlanner() {
}

bool MTCPlanner::plan(const vpr_msgs::PlanTrajectoriesRequest &req,
		moveit_msgs::DisplayTrajectory &solution_traj) const {

  using namespace mtc;

  // instantiating task
  TaskPtr task = std::make_shared<Task>("vpr_task", true, std::make_unique<SerialContainer>("planning_stages"));
  task->loadRobotModel();

  // getting start state stage
  StageUniquePtr start_st_stage = createFixedStateStage(req.start_state);
  task->stages()->insert(std::move(start_st_stage));

  // looping through each trajectory segment
  for(std::size_t i = 0; i < req.trajectory_segments.size(); i++)
  {
    const auto& segment = req.trajectory_segments[i];
    std::string stage_id = "segment_" + std::to_string(i+1);
    StageUniquePtr stage = nullptr;
    if(segment.waypoints.size() == 1) // handle single waypoint case
    {
      stage_id += "_fs";
      stage = createFreeSpacePlanStage(stage_id, req.group_name, segment.waypoints.front());
    }
    else if (segment.waypoints.size() > 1) // handle toolpath case (sequence of waypoints)
    {
      stage_id += "_tp";
      stage = createToolpathPlanStage(stage_id, req.group_name, req, segment);
    }
    else
    {
      continue;
      ROS_ERROR("Received invalid segment, skipping planning");
    }

    if(!stage)
    {
      return false;
    }
    task->stages()->insert(std::move(stage));
  }

  // now plan task
  task->enableIntrospection(true);
  bool success = task->plan();
  try
  {
    task->publishAllSolutions(false);
    ros::Duration(0.5).sleep();
    ROS_INFO("Published solutions");
  }
  catch (const InitStageException& ex)
  {
    ROS_ERROR("Failed to publish solutions %s", ex.what());
  }

  if(!success)
  {
    ROS_ERROR("Failed to plan task");
    return false;
  }

  // populating trajectory message from solutions
  for(const auto& s : task->solutions())
  {
    moveit_task_constructor_msgs::Solution sol_msg;
    s->fillMessage(sol_msg);
    solution_traj.trajectory_start = sol_msg.start_scene.robot_state;
    for(const auto sub_traj : sol_msg.sub_trajectory)
    {
      if(sub_traj.trajectory.joint_trajectory.points.empty())
      {
        continue;
      }

      solution_traj.trajectory.push_back(sub_traj.trajectory);
    }
    break;
  }

  return true;
}

std::unique_ptr<moveit::task_constructor::stages::MoveTo> MTCPlanner::createFreeSpacePlanStage(
    const std::string& stage_id,const std::string& group_name,const vpr_msgs::TrajectoryWaypoint& wp) const
{
  std::shared_ptr<mtc::solvers::PipelinePlanner> freespace_planner = createFreeSpacePlanner();

  std::unique_ptr<mtc::stages::MoveTo> planner_stage = std::make_unique<mtc::stages::MoveTo>(stage_id,
                                                                                             freespace_planner);

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  planner_stage->setGroup(group_name);
  planner_stage->setTimeout(DEFAULT_TIMEOUT);
  const moveit::core::LinkModel* tcp_link = jmg->getLinkModels().back();
  ROS_WARN("Freespace planning stage Using tcp link %s", tcp_link->getName().c_str());

  planner_stage->setIKFrame(tcp_link->getName());
  planner_stage->setGoal(wp.tool_pose);

  return planner_stage;
}

std::unique_ptr<vpr_trajectory::stages::PlanToolpath> MTCPlanner::createToolpathPlanStage(
    const std::string &stage_id, const std::string &group_name,
    const vpr_msgs::PlanTrajectoriesRequest &properties,
    const vpr_msgs::TrajectorySegment &segment) const
{
  using namespace vpr_trajectory::stages;
  using namespace moveit::core;

  std::unique_ptr<PlanToolpath> plan_toolpath_stage = std::make_unique<PlanToolpath>(stage_id,
                                                                                     group_name);

  // preparing planning configuration
  JumpThreshold jump_threshold = JumpThreshold{properties.jump_threshold_revolute, properties.jump_threshold_prismatic};
  MaxEEFStep eef_step = {.translation = properties.max_eef_step_translation, .rotation = properties.max_eef_step_rotation};

  plan_toolpath_stage->setProperty("jump_threshold", jump_threshold);
  plan_toolpath_stage->setProperty("eef_step", eef_step);

  boost::optional<Eigen::Isometry3d> model_to_ref_transform = boost::none;
  std::string model_frame = robot_model_->getModelFrame().c_str();
  std::string ref_link = segment.waypoints.front().tool_pose.header.frame_id;
  model_to_ref_transform = lookupTransform(model_frame,ref_link);
  if(!model_to_ref_transform)
  {
    ROS_ERROR("Failed to get transform from model to ref link %s", ref_link.c_str());
    return nullptr;
  }

  ROS_WARN_COND(model_to_ref_transform.is_initialized(),"Grabbed transform from '%s' to '%s' using TF",
                model_frame.c_str(),ref_link.c_str());

  // creating toolpath
  EigenSTL::vector_Isometry3d toolpath;
  const auto& waypoints = segment.waypoints;
  std::transform(waypoints.cbegin(), waypoints.cend(), std::back_inserter(toolpath),[&model_to_ref_transform](
      const vpr_msgs::TrajectoryWaypoint& wp){
    Eigen::Isometry3d waypoint;
    tf2::fromMsg(wp.tool_pose.pose, waypoint);
    return model_to_ref_transform.get() * waypoint;
  });

  plan_toolpath_stage->setToolpath(toolpath);
  return plan_toolpath_stage;
}

std::unique_ptr<mtc::stages::FixedState> MTCPlanner::createFixedStateStage(const moveit_msgs::RobotState& st_msg) const
{
  scene_monitor_->lockSceneRead();
  auto scene = planning_scene::PlanningScene::clone(scene_monitor_->getPlanningScene());
  if(st_msg.joint_state.position.empty()) // get current state from the monitor
  {
    scene->setCurrentState(scene_monitor_->getPlanningScene()->getCurrentState());
  }
  else
  {
    scene->setCurrentState(st_msg);
  }
  scene_monitor_->unlockSceneRead();

  // create Stage
  std::unique_ptr<mtc::stages::FixedState> fixed = std::make_unique<mtc::stages::FixedState>("start_state");
  fixed->setState(scene);
  return fixed;
}

boost::optional<Eigen::Isometry3d> MTCPlanner::lookupTransform(std::string to_frame,  std::string from_frame) const
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

  Eigen::Isometry3d transform;
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
