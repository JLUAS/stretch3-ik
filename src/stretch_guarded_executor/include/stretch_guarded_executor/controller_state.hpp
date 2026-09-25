#pragma once

#include <action_msgs/msg/goal_status_array.hpp>

namespace stretch_guarded_executor {
struct ControllerPreflight {
  bool action_server_available = false;
  bool status_message_received = false;
  bool active_goal_present = false;
  bool controller_idle = false;
};

// Availability and goal state are independent. Never expire a known active goal
// just because status messages stop; only a new status snapshot can clear it.
class ControllerState {
 public:
  void observe(const action_msgs::msg::GoalStatusArray& message) {
    status_message_received_ = true;
    active_goal_present_ = false;
    for (const auto& goal : message.status_list) {
      if (goal.status == goal.STATUS_ACCEPTED ||
          goal.status == goal.STATUS_EXECUTING ||
          goal.status == goal.STATUS_CANCELING)
        active_goal_present_ = true;
    }
  }

  ControllerPreflight preflight(bool server_available,
                                bool local_goal_pending) const {
    ControllerPreflight result;
    result.action_server_available = server_available;
    result.status_message_received = status_message_received_;
    result.active_goal_present = active_goal_present_ || local_goal_pending;
    result.controller_idle = server_available && !result.active_goal_present;
    return result;
  }

 private:
  bool status_message_received_ = false;
  bool active_goal_present_ = false;
};
}  // namespace stretch_guarded_executor
