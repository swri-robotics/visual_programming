/*
 * preview_state.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: ros-industrial
 */

#include <std_srvs/Trigger.h>
#include <vpr_core/states/preview_state.h>
#include "vpr_core/states/action_codes.h"
#include <vpr_msgs/VprErrorCode.h>

static const std::string START_PATH_PREVIEW_SERVICE = "start_path_preview";
static const std::string STOP_PATH_PREVIEW_SERVICE = "stop_path_preview";
static const double SERVICE_WAIT_DURATION = 10.0;

namespace vpr_core
{
namespace states
{

PreviewState::PreviewState(ros::NodeHandle nh):
         ros_fsm_core::state_machine::State("Preview"),
         nh_(nh)
{
  actions_ = {std::to_string(action_code::SHOW_PREVIEW)};
}

PreviewState::~PreviewState()
{

}

bool PreviewState::init()
{
  start_path_preview_client_ = nh_.serviceClient<std_srvs::Trigger>(START_PATH_PREVIEW_SERVICE);
  stop_path_preview_client_ = nh_.serviceClient<std_srvs::Trigger>(STOP_PATH_PREVIEW_SERVICE);

  // wait for servers
  std::vector<ros::ServiceClient*> clients = {&start_path_preview_client_, &stop_path_preview_client_};
  if(!std::all_of(clients.begin(),clients.end(),[&](ros::ServiceClient* client){
    if(!client->waitForExistence(ros::Duration(SERVICE_WAIT_DURATION)))
    {
      ROS_ERROR("Service %s was not found",client->getService().c_str());
      return false;
    }
    return true;
  }))
  {
    return false;
  }
  return true;
}

void PreviewState::exit()
{
  std_srvs::Trigger srv;
  stop_path_preview_client_.call(srv);
}

void PreviewState::showPreview()
{
  using namespace vpr_core;
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;

  preview_timer_ = nh_.createTimer(ros::Duration(0.01),[this](const ros::TimerEvent& evnt) -> void{
    // call preview service
    std_srvs::Trigger srv;
    VprErrorCode err_code;
    if(!start_path_preview_client_.call(srv) or !srv.response.success)
    {
      err_code.val = VprErrorCode::ROS_FAILURE;
      notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
    }
  }, true,true);
}

void PreviewState::haltPreview()
{
  using namespace vpr_core;
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;
  std_srvs::Trigger srv;
  if(!stop_path_preview_client_.call(srv))
  {
    VprErrorCode err_code;
    err_code.val = VprErrorCode::ROS_FAILURE;
    notifyAction(Action(std::to_string(action_code::FAILURE_DETECTED),err_code));
  }
}
ros_fsm_core::state_machine::Response PreviewState::enter(const boost::any& val)
{
  haltPreview();
  showPreview();
  return true;
}

ros_fsm_core::state_machine::Response PreviewState::execute(const ros_fsm_core::state_machine::Action& action)
{
  using namespace vpr_core;
  using namespace vpr_msgs;
  using namespace ros_fsm_core::state_machine;

  VprErrorCode err_code;
  if(std::find(actions_.begin(),actions_.end(),action.id) == actions_.end())
  {
    err_code.val = VprErrorCode::INVALID_ACTION;
    return false;
  }

  haltPreview();
  showPreview();

  return true;
}

}}
