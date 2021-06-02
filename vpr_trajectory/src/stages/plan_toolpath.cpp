/*
 * plan_toolpath.cpp
 *
 *  Created on: Jun 1, 2021
 *      Author: Jorge Nicho
 */

#include <boost/format.hpp>

#include <moveit/task_constructor/properties.h>

#include <moveit/planning_scene/planning_scene.h>

#include <vpr_trajectory/stages/plan_toolpath.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace vpr_trajectory
{
namespace stages
{

PlanToolpath::PlanToolpath(const std::string& stage_id, const std::string& group_name):
    moveit::task_constructor::PropagatingEitherWay(stage_id)
{
  using namespace moveit::task_constructor;
  PropertyMap& props = properties();

  ROS_WARN("Created toolpath planner stage for group %s", group_name.c_str());

  props.declare<std::string>("group_name", group_name,"The kinematic group used for planning the motions");
  props.declare<EigenSTL::vector_Isometry3d>("toolpath", "The toolpath");

  moveit::core::MaxEEFStep eef_step = {.translation = 0.01, .rotation = 0.01};
  moveit::core::JumpThreshold jump_threshold(1.0, 1.0);

  props.declare<moveit::core::MaxEEFStep>("eef_step", eef_step, "step size between consecutive waypoints");
  props.declare<moveit::core::JumpThreshold>("jump_threshold", jump_threshold, "acceptable fraction of mean joint motion per step");

  props.declare<double>("max_velocity_scaling_factor", 0.5, "");
  props.declare<double>("max_acceleration_scaling_factor", 0.5, "");

}

PlanToolpath::~PlanToolpath()
{

}

void PlanToolpath::init(const moveit::core::RobotModelConstPtr &robot_model)
{
  using namespace moveit::task_constructor;

  PropagatingEitherWay::init(robot_model);

  // check the group name property
  PropertyMap& props = properties();
  const std::string group_name = props.get<std::string>("group_name");
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    throw std::runtime_error(boost::str(boost::format("Group name \"%s\" is invalid") % group_name));
  }
}

void PlanToolpath::computeForward(const moveit::task_constructor::InterfaceState &from)
{
  using namespace moveit::task_constructor;
  using namespace moveit::core;


  PropertyMap& props = properties();
  auto robot_model = from.scene()->getRobotModel();
  const std::string group_name = props.get<std::string>("group_name");
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);

  MaxEEFStep eef_step = props.get<MaxEEFStep>("eef_step");
  JumpThreshold jump_threshold = props.get<JumpThreshold>("jump_threshold");

  double max_velocity_scaling_factor = props.get<double>("max_velocity_scaling_factor");
  double max_acceleration_scaling_factor = props.get<double>("max_acceleration_scaling_factor");

  EigenSTL::vector_Isometry3d toolpath = props.get<EigenSTL::vector_Isometry3d>("toolpath");

  // planning scenes and interface states
  planning_scene::PlanningScenePtr start_scene = from.scene()->diff();
  planning_scene::PlanningScenePtr start_toolpath_scene = from.scene()->diff();

  // callback to check for collisions
  auto is_valid_cb = [&from, &start_scene](moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                         const double* joint_positions) {
    state->setJointGroupPositions(jmg, joint_positions);
    state->update();
    return !start_scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName());
  };

  // computing toolpath path first
  std::vector<moveit::core::RobotStatePtr> toolpath_trajectory_states;
  const moveit::core::LinkModel* tcp_link = jmg->getLinkModels().back();
  ROS_WARN("Plan Toolpath Stage using tcp link %s", tcp_link->getName().c_str());

  double achieved_fraction = start_scene->getCurrentStateNonConst().computeCartesianPath(
      jmg, toolpath_trajectory_states, tcp_link, toolpath, true, eef_step,jump_threshold ,
      is_valid_cb);

  if(achieved_fraction < 1.0)
  {
    ROS_ERROR("Failed to plan toolpath, only planned for %f of waypoints", achieved_fraction);
    silentFailure();
    return;
  }

  // creating toolpath trajectory object
  robot_trajectory::RobotTrajectoryPtr toolpath_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      robot_model, jmg);
  for (const auto& waypoint : toolpath_trajectory_states)
  {
    toolpath_trajectory->addSuffixWayPoint(waypoint, 0.0);
  }
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if(!timing.computeTimeStamps(*toolpath_trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor))
  {
    ROS_ERROR("Failed to compute time stamps for toolpath trajectory");
    silentFailure();
    return;
  }

  // setting last waypoint as final state
  planning_scene::PlanningScenePtr end_scene = start_scene->diff();
  end_scene->setCurrentState(toolpath_trajectory->getLastWayPoint());
  InterfaceState to = InterfaceState(end_scene);

  // sending solution forward
  sendForward(from, std::move(to), SubTrajectory(toolpath_trajectory));
}

void PlanToolpath::computeBackward(const moveit::task_constructor::InterfaceState &to)
{
  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " has not been implemented");
  silentFailure();
}

void PlanToolpath::setToolpath(const EigenSTL::vector_Isometry3d &toolpath)
{
  using namespace moveit::task_constructor;
  PropertyMap& props = properties();
  props.set<EigenSTL::vector_Isometry3d>("toolpath", toolpath);
}

} /* namespace stages */
} /* namespace vpr_trajectory */
