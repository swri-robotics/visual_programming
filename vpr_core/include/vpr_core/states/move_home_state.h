/*
 * move_home_state.h
 *
 *  Created on: Aug 30, 2018
 *      Author: ros
 */

#ifndef INCLUDE_VPR_CORE_STATES_MOVE_HOME_STATE_H_
#define INCLUDE_VPR_CORE_STATES_MOVE_HOME_STATE_H_

#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{

class MoveHomeState : public ros_fsm_core::state_machine::State
{
public:
  MoveHomeState(ros::NodeHandle nh);
  virtual ~MoveHomeState();

  bool init() override;
  ros_fsm_core::state_machine::Response enter(const boost::any& val = boost::any()) override;

protected:

  ros::NodeHandle nh_;
  ros::ServiceClient move_home_client_;
  ros::Timer move_home_timer_;
};

} /* namespace states */
} /* namespace vpr_core */

#endif /* INCLUDE_VPR_CORE_STATES_MOVE_HOME_STATE_H_ */
