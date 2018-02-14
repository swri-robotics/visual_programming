/*
 * execute_state.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: ros-industrial
 */

#include <vpr_core/states/execute_state.h>
#include "vpr_core/states/action_codes.h"
#include <vpr_msgs/VprErrorCode.h>
#include <std_srvs/Trigger.h>

static const std::string EXECUTE_PATH_SERVICE = "execute_path";
static const double SERVICE_WAIT_DURATION = 10.0;


namespace vpr_core
{
namespace states
{

ExecuteState::ExecuteState(ros::NodeHandle nh):
    ros_fsm_core::state_machine::State("Execute"),
    nh_(nh)
{

}

ExecuteState::~ExecuteState()
{

}

bool ExecuteState::init()
{
  execute_path_client_ = nh_.serviceClient<std_srvs::Trigger>(EXECUTE_PATH_SERVICE);
  if(!execute_path_client_.waitForExistence(ros::Duration(SERVICE_WAIT_DURATION)))
  {
    ROS_ERROR("Service %s was not found",execute_path_client_.getService().c_str());
    return false;
  }

  return true;
}

ros_fsm_core::state_machine::Response ExecuteState::enter(const boost::any& val)
{
  execution_timer_ = nh_.createTimer(ros::Duration(0.01),&ExecuteState::executionTimerCb,this,true,true);
  ROS_INFO("Called execute traj service");
  return true;
}

void ExecuteState::executionTimerCb(const ros::TimerEvent& evnt)
{
  using namespace vpr_core;
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;

  // call preview service
  std_srvs::Trigger srv;
  VprErrorCode err_code;
  if(!execute_path_client_.call(srv))
  {
    err_code.val = VprErrorCode::ROS_FAILURE;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
  }
  else
  {
    err_code.val = err_code.SUCCESS;
    ROS_INFO("Path Execution Done");
    notifyAction(Action(std::to_string(action_code::PROCEED_INTERNAL),err_code));
  }

  execution_timer_.stop();
  return;
}

} /* namespace states */
} /* namespace vpr_core */
