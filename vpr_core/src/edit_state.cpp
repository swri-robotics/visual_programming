/*
 * edit_state.cpp
 *
 *  Created on: May 17, 2018
 *      Author: ros-industrial
 */

#include <vpr_core/states/edit_state.h>
#include "vpr_core/states/action_codes.h"
#include <vpr_msgs/VprErrorCode.h>
#include <vpr_msgs/ProcessAction.h>
#include <vpr_msgs/EditPathWaypoints.h>
#include <std_srvs/SetBool.h>
#include <ros/service_client.h>

static const std::string EDIT_PATH_WAYPOINTS_SERVICE = "edit_path_waypoints";
static const std::string CONFIRM_EDIT_SERVICE = "confirm_edit";
static const double SERVICE_WAIT_DURATION = 5.0f;
static const int UNSUPPORTED_EDIT_ACTION = -99;

namespace vpr_core
{
namespace states
{

EditState::EditState(ros::NodeHandle nh):
     ros_fsm_core::state_machine::State("Edit"),
     nh_(nh)
{

}

EditState::~EditState()
{

}

bool EditState::init()
{
  edit_path_waypoints_client_ = nh_.serviceClient<vpr_msgs::EditPathWaypoints>(EDIT_PATH_WAYPOINTS_SERVICE);
  if(!edit_path_waypoints_client_.waitForExistence(ros::Duration(SERVICE_WAIT_DURATION)))
  {
    ROS_ERROR("Service %s was not found",edit_path_waypoints_client_.getService().c_str());
    return false;
  }

  return true;
}

ros_fsm_core::state_machine::Response EditState::enter(const boost::any& val)
{
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;
  typedef vpr_msgs::EditPathWaypoints::Request EditReq;
  static const std::map<std::string,int> action_mappings = {
    {std::to_string(action_code::ADD_WAYPOINT), EditReq::ADD},
    {std::to_string(action_code::CLEAR_WAYPOINTS), EditReq::CLEAR_ALL},
    {std::to_string(action_code::DELETE_WAYPOINT), EditReq::DELETE},
    {std::to_string(action_code::CHANGE_SNAP_MODE), EditReq::CHANGE_SNAP_MODE},
    {std::to_string(action_code::SELECT_WAYPOINT), UNSUPPORTED_EDIT_ACTION},
    {std::to_string(action_code::MODIFY_WAYPOINT), UNSUPPORTED_EDIT_ACTION}};

  int err_code;

  if(val.empty())
  {
    err_code = vpr_msgs::VprErrorCode::EMPTY_VALUE;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    return true;
  }

  if(val.type() != typeid(ProcessActionRequest))
  {
    err_code = vpr_msgs::VprErrorCode::BAD_DATA;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    return true;
  }

  ProcessActionRequest pa_req = boost::any_cast<ProcessActionRequest>(val);
  if(action_mappings.count(pa_req.action) <= 0)
  {
    err_code = VprErrorCode::INVALID_ACTION;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    return true;
  }

  if(action_mappings.at(pa_req.action) == UNSUPPORTED_EDIT_ACTION)
  {
    ROS_WARN("The %s edit action isn't supported yet, ignoring edit",pa_req.action.c_str());
    err_code = vpr_msgs::VprErrorCode::SUCCESS;
    notifyAction(Action(std::to_string(action_code::PROCEED_INTERNAL),err_code));
    return true;
  }

  // calling service
  EditReq req;
  EditPathWaypointsResponse res;
  req.operation = action_mappings.at(pa_req.action);
  if(!edit_path_waypoints_client_.call(req,res))
  {
    err_code = VprErrorCode::ROS_FAILURE;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    return true;
  }

  if(!res.success)
  {
    ROS_ERROR_STREAM("Edit was rejected: "<<res.msg);
  }

  // proceed
  err_code = vpr_msgs::VprErrorCode::SUCCESS;
  notifyAction(Action(std::to_string(action_code::PROCEED_INTERNAL),err_code));

  return true;
}




} /* namespace states */
} /* namespace vpr_core */
