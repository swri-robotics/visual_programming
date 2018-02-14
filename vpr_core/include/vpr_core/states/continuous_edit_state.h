/*
 * continuous_edit_state.h
 *
 *  Created on: Jul 31, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_CONTINUOUS_EDIT_STATE_H_
#define INCLUDE_VPR_CORE_STATES_CONTINUOUS_EDIT_STATE_H_

#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{

class ContinuousEditState: public ros_fsm_core::state_machine::State
{
public:
  ContinuousEditState(ros::NodeHandle nh);
  virtual ~ContinuousEditState();

  bool init() override;

  ros_fsm_core::state_machine::Response enter(const boost::any& val = boost::any()) override;
  ros_fsm_core::state_machine::Response execute(const ros_fsm_core::state_machine::Action& action) override;

protected:

  ros::ServiceClient edit_path_waypoints_client_;
  ros::NodeHandle nh_;
};

} /* namespace states */
} /* namespace vpr_core */

#endif /* INCLUDE_VPR_CORE_STATES_CONTINUOUS_EDIT_STATE_H_ */
