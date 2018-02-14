/*
 * faulted_state.cpp
 *
 *  Created on: May 29, 2018
 *      Author: ros-industrial
 */

#include "vpr_core/states/faulted_state.h"
#include "vpr_core/states/action_codes.h"
#include <vpr_msgs/VprErrorCode.h>
#include <vpr_msgs/ProcessAction.h>
#include <ros/service_client.h>

namespace vpr_core
{
namespace states
{

FaultedState::FaultedState(ros::NodeHandle nh):
         ros_fsm_core::state_machine::State("Faulted"),
         nh_(nh)
{

}

ros_fsm_core::state_machine::Response FaultedState::enter(const boost::any& val)
{
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;

  // unpacking error integer code
  VprErrorCode err_code;
  try
  {
    err_code = boost::any_cast<VprErrorCode>(val);
  }
  catch(boost::bad_any_cast& e)
  {
    ROS_ERROR("Couldn't to unpack error integer, unable to recover");
    return false;
  }

  switch(err_code.val)
  {
    case static_cast<int>(vpr_msgs::VprErrorCode::ROS_FAILURE):
    {
      ROS_ERROR("A ROS Failure was detected, check your nodes and reset the SM");
    }
    break;

    case static_cast<int>(vpr_msgs::VprErrorCode::INVALID_ACTION):
    {
      notifyAction(std::to_string(action_code::PROCEED_INTERNAL));
    }
    break;

    case static_cast<int>(vpr_msgs::VprErrorCode::BAD_DATA):
    {
      ROS_ERROR("BAD DATA error, unable to recover");
      return false;
    }
    break;

  }

  return true;
}


FaultedState::~FaultedState()
{

}

} /* namespace states */
} /* namespace vpr_core */
