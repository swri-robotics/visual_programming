/*
 * action_codes.hpp
 *
 *  Created on: May 16, 2018
 *      Author: ros-industrial
 */

#ifndef INCLUDE_VPR_CORE_STATES_ACTION_CODES_HPP_
#define INCLUDE_VPR_CORE_STATES_ACTION_CODES_HPP_

#include <iostream>
#include <sstream>

namespace vpr_core{

namespace states{

  static const std::string INTERNAL_ACTION_QUALIFIER = "INTERNAL";

  enum class action_code: int
  {
    PROCEED_INTERNAL = 1,
    ADD_WAYPOINT,
    ADD_SEGMENT,
    SAVE_SEGMENT,
    CANCEL_SEGMENT,
    MODIFY_WAYPOINT,
    CHANGE_SNAP_MODE,
    SELECT_WAYPOINT,
    DELETE_WAYPOINT,
    CLEAR_WAYPOINTS,
    FAILURE_DETECTED,
    FAILURE_CORRECTED,
    RESET,
    SHOW_PREVIEW,
    EXIT_PREVIEW,
    EXECUTE,
    MOVE_HOME,

  };
}
}

namespace std
{

  std::ostream& operator<<(std::ostream& os, vpr_core::states::action_code ac);
  std::string to_string(vpr_core::states::action_code ac);
}




#endif /* INCLUDE_VPR_CORE_STATES_ACTION_CODES_HPP_ */
