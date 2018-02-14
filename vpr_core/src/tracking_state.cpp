/*
 * tracking_state.cpp
 *
 *  Created on: May 17, 2018
 *      Author: ros-industrial
 */

#include <vpr_core/states/tracking_state.h>

namespace vpr_core
{
namespace states
{

TrackingState::TrackingState(ros::NodeHandle nh):
    ros_fsm_core::state_machine::State("Tracking"),
    nh_(nh)
{

}

TrackingState::~TrackingState()
{

}

} /* namespace states */
} /* namespace vpr_core */
