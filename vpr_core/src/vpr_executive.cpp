/*
 * vpr_executive.cpp
 *
 *  Created on: May 16, 2018
 *      Author: ros-industrial
 */

#include <functional>
#include <algorithm>

#include "vpr_core/states/action_codes.h"
#include "vpr_core/states/move_home_state.h"
#include "vpr_core/states/tracking_state.h"
#include "vpr_core/states/faulted_state.h"
#include "vpr_core/states/edit_state.h"
#include "vpr_core/states/continuous_edit_state.h"
#include "vpr_core/states/bringup_state.h"
#include "vpr_core/states/preview_state.h"
#include "vpr_core/states/execute_state.h"

#include <std_msgs/String.h>
#include <vpr_msgs/ProcessAction.h>
#include <vpr_msgs/GetAvailableActions.h>

static const std::string GET_AVAILABLE_ACTIONS_SERVICE = "get_available_actions";
static const std::string PROCESS_ACTION_SERVICE = "process_action";
static const std::string CURRENT_STATE_TOPIC = "current_state";
static const double PUBLISH_TIMER_PERIOD = 0.1;

class VprStateMachine: public ros_fsm_core::state_machine::StateMachine
{
public:
  VprStateMachine()
  {

  }

  ~VprStateMachine()
  {
    publish_timer_.stop();
  }

  bool run()
  {
   if(!setup())
   {
     return false;
   }

   if(!start(start_state_->getName()))
   {
     return false;
   }

   publish_timer_.start();

   ros::waitForShutdown();
   return true;
  }

protected:

  bool setup()
  {
    using namespace ros_fsm_core::state_machine;
    using namespace vpr_core::states;

    // callbacks
    std::function<bool ()> bringup_cb = std::bind(&VprStateMachine::bringupCb,this);

    // create states
    StatePtr bringup_st = StatePtr(new BringupState(bringup_cb));
    StatePtr move_home_st = StatePtr(new MoveHomeState(nh_));
    StatePtr tracking_st = StatePtr(new TrackingState(nh_));
    StatePtr editing_st = StatePtr(new EditState(nh_));
    StatePtr continuous_edit_st = StatePtr(new ContinuousEditState(nh_));
    StatePtr preview_st = StatePtr(new PreviewState(nh_));
    StatePtr execute_st = StatePtr(new ExecuteState(nh_));
    StatePtr faulted_st = StatePtr(new FaultedState(nh_));
    states_ = {move_home_st, bringup_st, tracking_st, editing_st, preview_st, execute_st, faulted_st, continuous_edit_st};

    // add states to state machine
    for(auto st : states_)
    {
      addState(st);
    }

    // add transitions
    addTransition(bringup_st,Action(std::to_string(action_code::PROCEED_INTERNAL)),tracking_st);

    addTransition(tracking_st,Action(std::to_string(action_code::ADD_WAYPOINT)),editing_st);
    addTransition(tracking_st,Action(std::to_string(action_code::DELETE_WAYPOINT)),editing_st);
    addTransition(tracking_st,Action(std::to_string(action_code::MODIFY_WAYPOINT)),editing_st);
    addTransition(tracking_st,Action(std::to_string(action_code::CLEAR_WAYPOINTS)),editing_st);
    addTransition(tracking_st,Action(std::to_string(action_code::SELECT_WAYPOINT)),editing_st);
    addTransition(tracking_st,Action(std::to_string(action_code::CHANGE_SNAP_MODE)),editing_st);
    addTransition(tracking_st,Action(std::to_string(action_code::ADD_SEGMENT)),continuous_edit_st);
    addTransition(tracking_st,Action(std::to_string(action_code::FAILURE_DETECTED)),faulted_st);
    addTransition(tracking_st,Action(std::to_string(action_code::SHOW_PREVIEW)),preview_st);
    addTransition(tracking_st,Action(std::to_string(action_code::MOVE_HOME)),move_home_st);
    addTransition(tracking_st,Action(std::to_string(action_code::RESET)),bringup_st);

    addTransition(move_home_st,Action(std::to_string(action_code::PROCEED_INTERNAL)),tracking_st);
    addTransition(move_home_st,Action(std::to_string(action_code::FAILURE_DETECTED)),faulted_st);

    addTransition(editing_st,Action(std::to_string(action_code::PROCEED_INTERNAL)),tracking_st);
    addTransition(editing_st,Action(std::to_string(action_code::FAILURE_DETECTED)),faulted_st);

    addTransition(continuous_edit_st,Action(std::to_string(action_code::PROCEED_INTERNAL)),tracking_st);
    addTransition(continuous_edit_st,Action(std::to_string(action_code::FAILURE_DETECTED)),faulted_st);

    addTransition(preview_st,Action(std::to_string(action_code::EXIT_PREVIEW)),tracking_st);
    addTransition(preview_st,Action(std::to_string(action_code::EXECUTE)),execute_st);
    addTransition(preview_st,Action(std::to_string(action_code::FAILURE_DETECTED)),tracking_st);

    addTransition(execute_st,Action(std::to_string(action_code::PROCEED_INTERNAL)),tracking_st);
    addTransition(execute_st,Action(std::to_string(action_code::FAILURE_DETECTED)),faulted_st);

    addTransition(faulted_st,Action(std::to_string(action_code::RESET)),bringup_st);
    addTransition(faulted_st,Action(std::to_string(action_code::FAILURE_CORRECTED)),tracking_st);

    start_state_ = bringup_st;
    publish_timer_ = nh_.createTimer(ros::Duration(PUBLISH_TIMER_PERIOD),[this](const ros::TimerEvent& evnt){
      std_msgs::String msg;
      msg.data = this->getCurrentState();
      current_state_pub_.publish(msg);
    },false,false);

    return true;
  }

  bool bringupCb()
  {
    using namespace ros_fsm_core::state_machine;
    using namespace vpr_core::states;

    // initialize states
    if(!std::all_of(states_.begin(),states_.end(),[](const StatePtr& st){
      return st->init();
    }))
    {
      return false;
    }

    if(get_avail_actions_srv_.getService().empty())
    {
      get_avail_actions_srv_ = nh_.advertiseService(GET_AVAILABLE_ACTIONS_SERVICE,
                                                  &VprStateMachine::getAvailableActionsCallback,this);
    }

    if(process_action_srv_.getService().empty())
    {
      process_action_srv_ = nh_.advertiseService(PROCESS_ACTION_SERVICE,&VprStateMachine::processActionCallback,this);
    }

    current_state_pub_ = nh_.advertise<std_msgs::String>(CURRENT_STATE_TOPIC,1);

    return true;
  }

  bool getAvailableActionsCallback(vpr_msgs::GetAvailableActionsRequest& req, vpr_msgs::GetAvailableActionsResponse& res)
  {

    std::vector<std::string> actions;
    std::vector<std::string> sm_actions = this->getAvailableActions();

    // filter out internal actions
    std::for_each(sm_actions.begin(),sm_actions.end(),[&actions](const std::string& action_name){
      std::size_t pos = action_name.find(vpr_core::states::INTERNAL_ACTION_QUALIFIER);
      if(pos == std::string::npos)
      {
        actions.push_back(action_name);
      }
    });

    res.actions = actions;
    return true;
  }

  bool processActionCallback(vpr_msgs::ProcessActionRequest& req, vpr_msgs::ProcessActionResponse& res)
  {
    using namespace ros_fsm_core::state_machine;

    std::string action  = req.action;
    Action action_obj(req.action,boost::any(req));
    res.error.val = res.error.SUCCESS;

    if(isBusy())
    {
      res.error.val = res.error.BUSY;
      return true;
    }

    bool result = execute(action_obj);
    if(!result)
    {
      res.error.val = res.error.INVALID_ACTION;
      return true;
    }

    return true;
  }

  // ROS members
  ros::NodeHandle nh_;
  ros::ServiceServer get_avail_actions_srv_;
  ros::ServiceServer process_action_srv_;
  ros::Publisher current_state_pub_;
  ros::Timer publish_timer_;

  // sm members
  ros_fsm_core::state_machine::StatePtr start_state_;
  std::vector<ros_fsm_core::state_machine::StatePtr> states_;

};

int main(int argc,char** argv)
{
  ros::init(argc,argv,"vpr_executive");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  VprStateMachine sm;
  if(!sm.run())
  {
    return -1;
  }

  return 0;
}



