/*
 * state_machine_definitions.cpp
 *
 *  Created on: September 20, 2018
 *      Author: Jorge Nicho
 *       Email: jrgnichodevel@gmail.com
 */

#include "ros_fsm_core/state_machine_definitions.h"


namespace ros_fsm_core
{
namespace state_machine
{

const std::string PreviousState::PREVIOUS_STATE_NAME = "__SM_PREVIOUS__";

void StateTransitionTable::addTransition(const std::string& src_state,const Action& action, const std::string& dst_state)
{

  using namespace boost;
  if(state_actions_mappings_.count(src_state) <= 0)
  {
    state_actions_mappings_.insert(std::make_pair(src_state,ActionMap()));
  }

  ActionMap& action_state_map = state_actions_mappings_[src_state];
  if(action_state_map.count(action) > 0)
  {
    ROS_ERROR_STREAM(str(
        format("Transition from Source State %1% through action %2% already exists") % src_state % action.id));
    return;
  }

  action_state_map.insert(std::make_pair(action, dst_state));
}

bool StateTransitionTable::hasTransition(const std::string& src_state,const Action& action) const
{
  if(state_actions_mappings_.count(src_state) <= 0)
  {
    return false;
  }
  const ActionMap& action_state_map = state_actions_mappings_.at(src_state);

  if(action_state_map.count(action) <= 0)
  {
    return false;
  }
  return true;
}

std::vector<std::string> StateTransitionTable::getAvailableActions(const std::string& src_state) const
{
  std::vector<std::string> actions;
  if(state_actions_mappings_.count(src_state) > 0)
  {
    const ActionMap& action_state_map = state_actions_mappings_.at(src_state);

    for(ActionMap::const_iterator i = action_state_map.begin() ; i != action_state_map.end() ; i++)
    {
      actions.push_back(i->first.id);
    }
  }
  return std::move(actions);
}

std::string StateTransitionTable::getDestinationState(const std::string& src_state,const Action& action) const
{
  if(!hasTransition(src_state,action))
  {
    return "";
  }

  const ActionMap& action_state_map = state_actions_mappings_.at(src_state);
  return action_state_map.at(action);
}

StateMachine::StateMachine(int threads, double event_loop_period, int history_buffer_size):
  sm_nh_(),
  event_loop_period_(event_loop_period),
  spinner_(0),
  history_buffer_size_(history_buffer_size)
{
  sm_nh_.setCallbackQueue(&execution_queue_);
  spinner_ = ros::AsyncSpinner(threads,&execution_queue_);
}

StateMachine::~StateMachine()
{
  stop();
}

bool StateMachine::start(const std::string init_state)
{
  ROS_ASSERT(hasState(init_state));
  action_queue_.clear();

  // setting up timer
  auto timer_cb = [this](const ros::TimerEvent& evnt)
  {
    processQueuedActions();
  };
  process_queue_timer_ = sm_nh_.createTimer(ros::Duration(event_loop_period_),timer_cb);
  process_queue_timer_.start();

  // entering init state
  current_state_ = states_map_[init_state];
  if(!current_state_->enter())
  {
    return false;
  }

  is_busy_ = false;
  spinner_.start();

  return true;
}

void StateMachine::stop()
{
  process_queue_timer_.stop();
  spinner_.stop();
}

void StateMachine::addState(StatePtr st)
{
  using namespace boost;
  ROS_ASSERT_MSG(st,"Invalid State Pointer");

  if(states_map_.count(st->getName()) > 0 )
  {
    ROS_WARN_STREAM(str(format("Another '%1%' state entry already exists, it will be replaced") % st->getName()));
  }

  states_map_[st->getName()] = st;
  st->action_handler_callback_ = std::bind(&StateMachine::postAction,this,std::placeholders::_1);
}

bool StateMachine::hasState(const std::string& st_name) const
{
  return states_map_.count(st_name) > 0;
}

std::string StateMachine::getCurrentState() const
{
  std::lock_guard<std::mutex> lock(current_st_mutex_);
  return current_state_->getName();
}

std::vector<std::string> StateMachine::getAvailableActions() const
{
  std::vector<std::string> actions = transition_table_.getAvailableActions(getCurrentState());
  std::vector<std::string> state_actions = this->current_state_->getActions();
  actions.insert(actions.end(),state_actions.begin(),state_actions.end());
  return std::move(actions);
}

void StateMachine::addTransition(StatePtr src_state,const Action& action, StatePtr dst_state)
{
  using namespace boost;

  std::map<std::string,StatePtr> state_map = {{"Source", src_state}, {"Destination", dst_state}};
  for(const std::map<std::string,StatePtr>::value_type kv : state_map)
  {
    const StatePtr& st = kv.second;
    const std::string& role = kv.first;
    std::string msg = str(format("%1% State is an invalid pointer") % role);
    ROS_ASSERT_MSG(st,"%s",msg.c_str());

    if(st->getName() != PreviousState::PREVIOUS_STATE_NAME)
    {
      ROS_ASSERT_MSG(hasState(st->getName()),"%s",
                     boost::str(format("%1% State %2% has not been added") % role %st->getName()).c_str());
    }
  }

  transition_table_.addTransition(src_state->getName(),action,dst_state->getName());
}

Response StateMachine::execute(const Action& action,const boost::any& data)
{
  Action cloned_action = action;
  cloned_action.data = data;
  return execute(cloned_action);
}

Response StateMachine::execute(const Action& action)
{
  Response res = executeAction(action);

  // post next action if there's one
  boost::optional<Action> next_action = action.getFollowupAction();
  if(next_action)
  {
    postAction(*next_action);
  }

  return res;
}

void StateMachine::postAction(Action action)
{
  std::lock_guard<std::mutex> lock(action_queue_mutex_);
  action_queue_.push_back(action);
  ROS_DEBUG("Posted action %s",action.id.c_str());
}

bool StateMachine::isBusy() const
{
  std::lock_guard<std::mutex> lock(action_queue_mutex_);
  return !action_queue_.empty() || is_busy_;
}

bool StateMachine::wait(double timeout) const
{
  if(timeout <= 0)
  {
    timeout = std::numeric_limits<double>::infinity();
  }

  ros::Duration pause(0.05);
  ros::Time current_time = ros::Time::now();
  ros::Time prev_time = current_time;
  double time_elapsed = 0;
  while(time_elapsed < timeout)
  {
    pause.sleep();
    current_time = ros::Time::now();
    time_elapsed += (current_time - prev_time).toSec();
    prev_time = current_time;

    if(!isBusy())
    {
      return true;
    }

    if(!sm_nh_.ok())
    {
      return false;
    }
  }

  return false;
}

Response StateMachine::executeAction(const Action& action)
{
  class ScopeExit
  {
  public:
    ScopeExit(std::atomic<bool>* b):
      b_(b)
    {
      *b_ = true;
    }

    ~ScopeExit()
    {
      *(b_) = false;
    }

    std::atomic<bool>* b_;
  };

  if(!current_state_)
  {
    throw std::runtime_error("No valid current state, sm has not been properly initialized");
  }

  ROS_DEBUG("Executing action %s",action.id.c_str());

  ScopeExit scope_exit(&this->is_busy_);
  boost::any data = action.data;

  if(current_state_->hasAction(action))
  {
    Response res = current_state_->execute(action);
    if(!res)
    {
      std::string error_msg = boost::str(
          boost::format("State '%1%' execute failed") % current_state_->getName());
      throw std::runtime_error(error_msg);
    }
    return std::move(res);
  }

  if(!transition_table_.hasTransition(current_state_->getName(),action))
  {
    ROS_ERROR_STREAM(boost::str(
        boost::format("A transition from state '%1%' through action '%2%' is not valid") % current_state_->getName() % action.id));
    return Response(true,false);
  }

  // getting next state
  std::string st_name = transition_table_.getDestinationState(current_state_->getName(),action);
  if(st_name.empty())
  {
    std::string error_msg = boost::str(
        boost::format(
            "Failed to get the the destination state for the (state: %1%, action: %2%) pair") % current_state_->getName()
            % action.id);
    throw std::runtime_error(error_msg);
  }


  // check if requesting to return to the previous state
  bool backtracking = false;
  if(st_name == PreviousState::PREVIOUS_STATE_NAME)
  {
    // get previous state
    backtracking = true;
    if(boost::optional<std::string> prev_st_name = popPreviousState())
    {
      st_name = prev_st_name.get();
    }
    else
    {
      std::string error_msg = boost::str(
          boost::format("Returning to the previous state from state '%1%' through action '%2%' failed") %
          current_state_->getName() % action.id);
      ROS_ERROR_STREAM(error_msg);
      return Response(true,false,boost::none,error_msg);
    }
  }

  StatePtr next_state = states_map_[st_name];

  // do transition
  Response res;
  {
    std::lock_guard<std::mutex> lock(current_st_mutex_);
    current_state_->exit();
    res = next_state->enter(data);
    if(!res)
    {
      throw std::runtime_error(boost::str(boost::format("State Machine failed to enter state %1%") % next_state->getName()));
    }

    if(!backtracking)
    {
      pushPreviousState(current_state_->getName());
    }
    current_state_ = next_state;
  }

  return std::move(res);
}

void StateMachine::processQueuedActions()
{
  using namespace boost;
  std::lock_guard<std::mutex> lock(action_queue_mutex_);

  if(is_busy_)
  {
    ROS_DEBUG_STREAM_NAMED("SM",__func__<< " is busy");
    return;
  }

  if(action_queue_.empty())
  {
    return;
  }

  auto action  = action_queue_.front();
  action_queue_.pop_front();
  Response res = executeAction(action);

  return;
}

void StateMachine::pushPreviousState(const std::string& state_name)
{
  if(state_history_deque_.size() == history_buffer_size_)
  {
    state_history_deque_.pop_front();
  }
  state_history_deque_.push_back(state_name);
}

boost::optional<std::string> StateMachine::popPreviousState()
{
  if(state_history_deque_.empty())
  {
    return boost::none;
  }
  std::string st = state_history_deque_.back();
  state_history_deque_.pop_back();
  return st;
}

}
}
