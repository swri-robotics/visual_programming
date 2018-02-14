/*
 * bringup_state.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: ros-industrial
 */

#include <vpr_core/states/bringup_state.h>
#include "vpr_core/states/action_codes.h"

namespace vpr_core
{
namespace states
{



BringupState::BringupState(std::function<bool ()> bringup_cb):
  ros_fsm_core::state_machine::State("Bringup"),
  bringup_cb_(bringup_cb)
{

}

BringupState::~BringupState()
{

}

ros_fsm_core::state_machine::Response BringupState::enter(const boost::any& val)
{
  using namespace ros_fsm_core::state_machine;

  if(!bringup_cb_())
  {
    ROS_ERROR("Initialization failed");
    return false;
  }

  return notifyAction(Action(std::to_string(action_code::PROCEED_INTERNAL)));;
}

} /* namespace states */
} /* namespace vpr_core */
