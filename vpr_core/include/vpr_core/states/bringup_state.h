/*
 * bringup_state.h
 *
 *  Created on: Jun 1, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_BRINGUP_STATE_H_
#define INCLUDE_VPR_CORE_STATES_BRINGUP_STATE_H_

#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{

class BringupState: public ros_fsm_core::state_machine::State
{
public:
  BringupState(std::function<bool ()> bringup_cb);
  virtual ~BringupState();

  ros_fsm_core::state_machine::Response enter(const boost::any& val = boost::any()) override;

protected:

  std::function<bool ()> bringup_cb_;
};

} /* namespace states */
} /* namespace vpr_core */

#endif /* INCLUDE_VPR_CORE_STATES_BRINGUP_STATE_H_ */
