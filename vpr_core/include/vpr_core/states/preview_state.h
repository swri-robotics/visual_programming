/*
 * preview_state.h
 *
 *  Created on: Jul 17, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_PREVIEW_STATE_H_
#define INCLUDE_VPR_CORE_STATES_PREVIEW_STATE_H_

#include <ros/ros.h>
#include <ros_fsm_core/state_machine_definitions.h>

namespace vpr_core
{
namespace states
{


class PreviewState: public ros_fsm_core::state_machine::State
{
public:
  PreviewState(ros::NodeHandle nh);
  virtual ~PreviewState();

  bool init() override;
  ros_fsm_core::state_machine::Response enter(const boost::any& val = boost::any()) override;
  ros_fsm_core::state_machine::Response execute(const ros_fsm_core::state_machine::Action& action) override;
  void exit() override;

protected:

  void showPreview();
  void haltPreview();

  ros::NodeHandle nh_;
  ros::ServiceClient start_path_preview_client_;
  ros::ServiceClient stop_path_preview_client_;
  ros::Timer preview_timer_;

};

}
}

#endif /* INCLUDE_VPR_CORE_STATES_PREVIEW_STATE_H_ */
