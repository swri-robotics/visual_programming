/*
 * execute_state.h
 *
 *  Created on: Aug 14, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_EXECUTE_STATE_H_
#define INCLUDE_VPR_CORE_STATES_EXECUTE_STATE_H_

#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{

class ExecuteState : public ros_fsm_core::state_machine::State
{
public:
  ExecuteState(ros::NodeHandle nh);
  virtual ~ExecuteState();

  bool init() override;
  ros_fsm_core::state_machine::Response enter(const boost::any& val = boost::any()) override;

protected:

  void executionTimerCb(const ros::TimerEvent& evnt);

  ros::NodeHandle nh_;
  ros::Timer execution_timer_;

  ros::ServiceClient execute_path_client_;
};

} /* namespace states */
} /* namespace vpr_core */

#endif /* INCLUDE_VPR_CORE_STATES_EXECUTE_STATE_H_ */
