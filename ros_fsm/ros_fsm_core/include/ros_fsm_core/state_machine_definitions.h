/*
 * state_machine_definitions.h
 *
 *  Created on: September 20, 2018
 *      Author: Jorge Nicho
 *       Email: jrgnichodevel@gmail.com
 */

#ifndef INCLUDE_ROS_FSM_CORE_STATE_MACHINE_DEFINITIONS_HPP_
#define INCLUDE_ROS_FSM_CORE_STATE_MACHINE_DEFINITIONS_HPP_

#include <string>
#include <functional>
#include <list>
#include <deque>
#include <ros/console.h>
#include <ros/assert.h>
#include <boost/format.hpp>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <tuple>
#include <mutex>
#include <atomic>

namespace ros_fsm_core
{
namespace state_machine
{

/***
 * @struct ros_fsm_core::state_machine::Action
 * @brief The Action structure that is used to transition between states
 */
struct Action
{
  /***
   * @brief Action constructor
   * @param name  The name of the action.  Used by the state machine to look up the transition
   * @param data  Any data that needs to be packed with the action for the next state to use.
   */
  Action(std::string id, boost::any data = boost::any(),boost::optional<Action> followup_action = boost::none):
    id(id),
    data(data),
    followup_action_ptr(nullptr)
  {
    if(followup_action)
    {
      followup_action_ptr.reset(new Action(*followup_action));
    }
  }

  ~Action()
  {

  }

  boost::optional<Action> getFollowupAction() const
  {
    boost::optional<Action> action = boost::none;
    if(followup_action_ptr)
    {
      action = Action(*followup_action_ptr);
    }
    return action;
  }

  /**
   * @brief allows using the action as a map key by using the id member
   * @param action
   * @return
   */
  bool operator< (const Action& action) const
  {
    return action.id < this->id;
  }

  std::string id;
  boost::any data;

private:
  std::shared_ptr<Action> followup_action_ptr;
};
typedef std::function<void (Action)> ActionHandlerCallback ;

/**
 * @struct ros_fsm_core::state_machine::Response
 * @brief Encapsulates the result of a state machine transaction.
 */
struct Response
{
  /**
   * @brief Constructor for the response object
   * @param status        Set to true if the state machine process proceeded as expected, false otherwise
   * @param transaction   Set to true if the expected transaction was completed, use false otherwise.
   * @param data          Optional data that was generated from the requested transaction.
   */
  Response(bool status = false, bool transaction = true, boost::any data = boost::none, std::string msg = ""):
    status(status),
    transaction(transaction),
    data(data),
    msg(msg)
  {

  }

  ~Response()
  {

  }

  Response& operator=(const bool& b)
  {
    this->status = b;
    this->transaction = b;
    return *this;
  }

  operator bool() const
  {
    return status;
  }

  bool status;
  bool transaction;
  boost::any data;
  std::string msg;
};

/***
 * @class ros_fsm_core::state_machine::State
 * @brief State class
 */
class State
{
public:

  /**
   * @brief State constructor
   * @param name      The name of the state
   * @param obj       An instance of the object to be managed by the state
   * @param callback  Used to notify the SM of an action that needs to be handled.  It can be used to
   *                  transition out of the state.
   */
  State(std::string name):
    name_(name)
  {

  }

  virtual ~State()
  {

  }

  std::string getName()
  {
    return name_;
  }

  /**
   * @brief Should be called once by client code to perform initialization tasks.
   * @return True if succeeded, false otherwise.
   */
  virtual bool init(){return true;}

  /**
   * @brief Called by the State Machine when the state is entered.
   * @param val  Data needed when the state is entered.
   * @return A Response object containing status and optional result data.
   */

  virtual Response enter(const boost::any& val = boost::any()){ return true;}

  /**
   * @brief Called by the State Machine when the state is exited.
   */
  virtual void exit() { }

  /**
   * @brief Executes the corresponding action if the state supports it.
   * @param action  The action to be executed.
   * @return  A Response object containing status and optional result data.
   */
  virtual Response execute(const Action& action)
  {
    ROS_WARN_STREAM(boost::str(boost::format("The State '%1%' %2% function is not implemented" ) % getName() % __func__));
    return true;
  }

  /**
   * @brief Checks whether or not the state can execute this action.
   * @param action   The action object.
   * @return  True if the action can be executed, false otherwise.
   */
  bool hasAction(const Action& action)
  {
    auto res = std::find(std::begin(actions_),std::end(actions_),action.id);
    return res != std::end(actions_);
  }

  /**
   * @brief Returns a list of actions allowed by this state.
   * @return A vector of action names.
   */
  const std::vector<std::string>& getActions()
  {
    return actions_;
  }

  friend class StateMachine;

protected:

  bool notifyAction(const Action& action)
  {
    if(action_handler_callback_)
    {
      action_handler_callback_(action);
    }
    else
    {
      ROS_ERROR("No action handler has been set, has this state been added to a state machine?");
      return false;
    }

    return true;
  }

  ActionHandlerCallback action_handler_callback_;
  std::vector<std::string> actions_;
  std::string name_;
};

typedef std::shared_ptr< State > StatePtr;

class StateTransitionTable
{

public:
  StateTransitionTable()
  {

  }

  ~StateTransitionTable()
  {

  }

  typedef std::map<Action,std::string> ActionMap;

  void addTransition(const std::string& src_state,const Action& action, const std::string& dst_state);

  bool hasTransition(const std::string& src_state,const Action& action) const;

  /**
   * @brief Gets the actions allowed by the state
   * @param src_state The name of the state
   * @return  An array of allowed action names.
   */
  std::vector<std::string> getAvailableActions(const std::string& src_state) const;

  std::string getDestinationState(const std::string& src_state,const Action& action) const;

  std::map<std::string, ActionMap > state_actions_mappings_; /** @brief <source_state, < action , destination state > > */
};

/**
 * @class ros_fsm_core::state_machine::PreviousState
 * @brief State Type used for the purposes of returning to a previous state
 */
class PreviousState: public State
{
public:

  virtual Response enter(const boost::any& val = boost::any()){ return true;}
private:
  static const std::string PREVIOUS_STATE_NAME;

  PreviousState():
    State(PREVIOUS_STATE_NAME)
  {

  }

  friend class StateMachine;
};

/***
 * @class ros_fsm_core::state_machine::StateMachine
 * @brief The state machine
 */
class StateMachine
{
public:


  StateMachine(int threads = 2, double event_loop_period = 0.1, int history_buffer_size = 100);

  virtual ~StateMachine();

  bool start(const std::string init_state);

  void stop();

  void addState(StatePtr st);

  bool hasState(const std::string& st_name) const;

  std::string getCurrentState() const;

  std::vector<std::string> getAvailableActions() const;

  StatePtr createPreviousState()
  {
    return StatePtr( new PreviousState());
  }


  /**
   * @brief adds a transition to the sm
   * @param src_state The source state
   * @param action    The action
   * @param dst_state The destination state
   * @param internal  True if this action is internal to the SM and isn't meant to be used by client code.
   */
  void addTransition(StatePtr src_state,const Action& action, StatePtr dst_state);

  /**
   * @brief executes the requested action. Throws a run_time exception when an unrecoverable error occurs.
   * @param action  The action
   * @param data    Any data needed to perform the action
   * @return  A Response object containing status and optional result data.
   */
  Response execute(const Action& action,const boost::any& data);

  /**
   * @brief executes the requested action. Throws a run_time exception when an unrecoverable error occurs.
   * @param action  The action
   * @return  A Response object containing status and optional result data.
   */
  Response execute(const Action& action);

  /**
   * @brief Adds an action to the execution queue.
   * @param action  The action to queue.
   */
  void postAction(Action action);

  /**
   * Checks if the SM is currently processing any queued actions
   * @return True when queued actions are being processed, false otherwise
   */
  bool isBusy() const;

  /**
   * @brief Pauses execution until all queued actions are processed or the timeout expires
   * @param timeout Time in seconds to wait, use -1 to wait indefinitely.
   * @return  True when there are not more queued actions to execute, false otherwise.
   */
  bool wait(double timeout = -1) const;

protected:

  /**
   * @brief Executes the exception.  Throws a run_time exception when an unrecoverable error occurs.
   * @param action
   * @return
   */
  Response executeAction(const Action& action);

  /***
   * @brief executes all the queued actions added through the action handler callback function.
   * Ideally this method should be call from its own execution event loop.  Not thread-safe.
   */
  void processQueuedActions();

  void pushPreviousState(const std::string& state_name);
  boost::optional<std::string> popPreviousState();

  std::list<Action> action_queue_;
  StateTransitionTable transition_table_;
  StatePtr current_state_;
  std::map<std::string,StatePtr > states_map_;
  ros::NodeHandle sm_nh_;
  ros::AsyncSpinner spinner_;
  ros::CallbackQueue execution_queue_;
  ros::Timer process_queue_timer_;
  double event_loop_period_;

  // thread safety
  mutable std::mutex current_st_mutex_;
  mutable std::mutex action_queue_mutex_;
  std::atomic<bool> is_busy_;
  std::deque<std::string> state_history_deque_;
  const int history_buffer_size_;
};


}
}



#endif /* INCLUDE_ROS_FSM_CORE_STATE_MACHINE_DEFINITIONS_HPP_ */
