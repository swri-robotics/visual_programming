/*
 * move_home_state.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: ros
 */

#include <vpr_core/states/move_home_state.h>
#include <std_srvs/Trigger.h>
#include "vpr_core/states/action_codes.h"
#include <vpr_msgs/VprErrorCode.h>

static const std::string MOVE_HOME_SERVICE = "move_home";
static const double WAIT_SERVICE_PERIOD = 10.0;

namespace vpr_core
{
namespace states
{

MoveHomeState::MoveHomeState(ros::NodeHandle nh):
    ros_fsm_core::state_machine::State("MoveHome"),
    nh_(nh)
{

}

MoveHomeState::~MoveHomeState()
{

}

bool MoveHomeState::init()
{
  move_home_client_ = nh_.serviceClient<std_srvs::Trigger>(MOVE_HOME_SERVICE);
  if(!move_home_client_.waitForExistence(ros::Duration(WAIT_SERVICE_PERIOD)))
  {
    return false;
  }

  return true;
}

ros_fsm_core::state_machine::Response MoveHomeState::enter(const boost::any& val)
{
  using namespace vpr_core;
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;
  move_home_timer_ = nh_.createTimer(ros::Duration(0.1),[this](const ros::TimerEvent& evnt){

    VprErrorCode err_code;
    std_srvs::Trigger srv;
    if(!move_home_client_.call(srv) or !srv.response.success)
    {
      ROS_ERROR("Failed to call '%s' service",move_home_client_.getService().c_str());
      err_code.val = VprErrorCode::ROS_FAILURE;
      notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
      return;
    }

    notifyAction(Action(std::to_string(action_code::PROCEED_INTERNAL)));
    return;

  },true,true);

  return true;
}

} /* namespace states */
} /* namespace vpr_core */
