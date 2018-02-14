/*
 * tracking_state.h
 *
 *  Created on: May 17, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_TRACKING_STATE_H_
#define INCLUDE_VPR_CORE_STATES_TRACKING_STATE_H_

#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{

class TrackingState : public ros_fsm_core::state_machine::State
{
public:
  TrackingState(ros::NodeHandle nh);
  virtual ~TrackingState();

  ros::NodeHandle nh_;
};

} /* namespace states */
} /* namespace vpr_core */

#endif /* INCLUDE_VPR_CORE_STATES_TRACKING_STATE_H_ */
