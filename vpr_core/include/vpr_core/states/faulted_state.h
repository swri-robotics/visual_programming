/*
 * faulted_state.h
 *
 *  Created on: May 29, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_FAULTED_STATE_H_
#define INCLUDE_VPR_CORE_STATES_FAULTED_STATE_H_

#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{

class FaultedState : public ros_fsm_core::state_machine::State
{
public:
  FaultedState(ros::NodeHandle nh);
  virtual ~FaultedState();

  ros_fsm_core::state_machine::Response enter(const boost::any& val = boost::any()) override;

  ros::NodeHandle nh_;
};

} /* namespace states */
} /* namespace vpr_core */

#endif /* INCLUDE_VPR_CORE_STATES_FAULTED_STATE_H_ */
