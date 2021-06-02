/*
 * trajectory_planner_node.cpp
 *
 *  Created on: Jul 27, 2018
 *      Author: ros-industrial
 */
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <vpr_msgs/PlanTrajectories.h>

#include "vpr_trajectory/mtc_planner.h"
#include "vpr_trajectory/trajectory_planner.h"

static const std::string PLAN_TRAJECTORIES_SERVICE = "plan_trajectories";

using namespace vpr_trajectory;
using namespace vpr_msgs;
using SrvCallback = std::function<bool (vpr_msgs::PlanTrajectoriesRequest& req, vpr_msgs::PlanTrajectoriesResponse& res)>;


template <class P>
bool planTrajectoryCallback(const P& planner,vpr_msgs::PlanTrajectoriesRequest& req,
                            vpr_msgs::PlanTrajectoriesResponse& res)
{
  res.success = planner.plan(req,res.robot_trajectories);
  return true;
}

//using PlannerType = vpr_trajectory::TrajectoryPlanner;
using PlannerType = vpr_trajectory::MTCPlanner;

int main(int argc, char** argv)
{
  using namespace std::placeholders;

  ros::init(argc,argv,"trajectory_planner");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;

  spinner.start();

  robot_model_loader::RobotModelLoader rmodel_loader("robot_description");
  moveit::core::RobotModelConstPtr robot_model = rmodel_loader.getModel();
  if(!robot_model)
  {
    ROS_ERROR("Unable to load robot model");
    return -1;
  }

  PlannerType task_planner(robot_model);

  // setting up service server
  SrvCallback srv_cb = std::bind(planTrajectoryCallback<PlannerType>,std::ref(task_planner),
                                 std::placeholders::_1,std::placeholders::_2);
  ros::ServiceServer planning_srv = nh.advertiseService<PlanTrajectoriesRequest,PlanTrajectoriesResponse>(
      PLAN_TRAJECTORIES_SERVICE,srv_cb);

  ros::waitForShutdown();

  return 0;
}


