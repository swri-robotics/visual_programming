/*
 * action_codes.cpp
 *
 *  Created on: Jun 4, 2018
 *      Author: ros-industrial
 */

#include "vpr_core/states/action_codes.h"

namespace std
{

  std::ostream& operator<<(std::ostream& os, vpr_core::states::action_code ac)
  {
    using namespace vpr_core::states;
    switch(ac)
    {
      case action_code::PROCEED_INTERNAL : os<< "PROCEED_" + INTERNAL_ACTION_QUALIFIER; break;
      case action_code::ADD_WAYPOINT : os<< "ADD_WAYPOINT"; break;
      case action_code::ADD_SEGMENT: os<< "ADD_SEGMENT"; break;
      case action_code::SAVE_SEGMENT : os<< "SAVE_SEGMENT"; break;
      case action_code::CANCEL_SEGMENT : os<< "CANCEL_SEGMENT"; break;
      case action_code::MODIFY_WAYPOINT : os<< "MODIFY_WAYPOINT"; break;
      case action_code::SELECT_WAYPOINT : os<< "SELECT_WAYPOINT"; break;
      case action_code::DELETE_WAYPOINT : os<< "DELETE_WAYPOINT"; break;
      case action_code::CHANGE_SNAP_MODE : os<< "CHANGE_SNAP_MODE"; break;
      case action_code::CLEAR_WAYPOINTS : os<< "CLEAR_WAYPOINTS"; break;
      case action_code::FAILURE_DETECTED : os<< "FAILURE_DETECTED"; break;
      case action_code::FAILURE_CORRECTED : os<< "FAILURE_CORRECTED"; break;
      case action_code::RESET : os<< "RESET"; break;
      case action_code::SHOW_PREVIEW : os<< "SHOW_PREVIEW"; break;
      case action_code::EXIT_PREVIEW : os<< "EXIT_PREVIEW"; break;
      case action_code::EXECUTE : os<< "EXECUTE"; break;
      case action_code::MOVE_HOME : os<< "MOVE_HOME"; break;
    }

    return os;
  }


  std::string to_string(vpr_core::states::action_code ac)
  {
    std::stringstream ss;
    ss<<ac;
    return ss.str();
  }
}
