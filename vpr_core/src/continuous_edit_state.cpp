/*
 * continuous_edit_state.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: ros-industrial
 */

#include "vpr_core/states/action_codes.h"
#include <vpr_msgs/VprErrorCode.h>
#include <vpr_msgs/ProcessAction.h>
#include <vpr_msgs/EditPathWaypoints.h>
#include <std_srvs/SetBool.h>
#include <ros/service_client.h>
#include <vpr_core/states/continuous_edit_state.h>

static const std::string EDIT_PATH_WAYPOINTS_SERVICE = "edit_path_waypoints";
static const double SERVICE_WAIT_DURATION = 5.0f;

namespace vpr_core
{
namespace states
{

ContinuousEditState::ContinuousEditState(ros::NodeHandle nh):
  ros_fsm_core::state_machine::State("ContinuousEdit"),
  nh_(nh)
{
  using namespace ros_fsm_core::state_machine;
  actions_ = {std::to_string(action_code::SAVE_SEGMENT), std::to_string(action_code::CANCEL_SEGMENT)};
}

ContinuousEditState::~ContinuousEditState()
{

}

bool ContinuousEditState::init()
{
  edit_path_waypoints_client_ = nh_.serviceClient<vpr_msgs::EditPathWaypoints>(EDIT_PATH_WAYPOINTS_SERVICE);
  if(!edit_path_waypoints_client_.waitForExistence(ros::Duration(SERVICE_WAIT_DURATION)))
  {
    ROS_ERROR("Service %s was not found",edit_path_waypoints_client_.getService().c_str());
    return false;
  }
  return true;
}

ros_fsm_core::state_machine::Response ContinuousEditState::enter(const boost::any& val)
{
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;
  typedef vpr_msgs::EditPathWaypoints::Request EditReq;

  // calling service
  EditReq req;
  EditPathWaypointsResponse res;
  int err_code;
  req.operation = req.CONTINUOUS_START;
  if(!edit_path_waypoints_client_.call(req,res))
  {
    err_code = VprErrorCode::ROS_FAILURE;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    return true;
  }

  if(!res.success)
  {
    err_code = VprErrorCode::FAILURE;
    ROS_ERROR_STREAM("Edit was rejected: "<<res.msg);
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
  }

  return true;
}

ros_fsm_core::state_machine::Response ContinuousEditState::execute(const ros_fsm_core::state_machine::Action& action)
{
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;
  typedef vpr_msgs::EditPathWaypoints::Request EditReq;

  static const std::map<std::string,int> action_mappings = {{std::to_string(action_code::SAVE_SEGMENT),EditReq::CONTINUOUS_STOP},
                                                            {std::to_string(action_code::CANCEL_SEGMENT),EditReq::CONTINUOUS_CANCEL}};

  if(action_mappings.count(action.id) <= 0)
  {
    ROS_ERROR("Invalid action %s requested on state %s",action.id.c_str(),getName().c_str());
    return false;
  }

  // calling service
  EditReq req;
  EditPathWaypointsResponse res;
  int err_code;
  req.operation = action_mappings.at(action.id);
  if(!edit_path_waypoints_client_.call(req,res))
  {
    err_code = VprErrorCode::ROS_FAILURE;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    return true;
  }

  if(!res.success)
  {
    err_code = VprErrorCode::FAILURE;
    ROS_ERROR_STREAM("Edit was rejected: "<<res.msg);
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
  }

  err_code = vpr_msgs::VprErrorCode::SUCCESS;
  notifyAction(Action(std::to_string(action_code::PROCEED_INTERNAL),err_code));

  return true;
}

} /* namespace states */
} /* namespace vpr_core */
