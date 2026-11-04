# Status of a pick or place operation.

std_msgs/Header header

# Current state name (e.g., IDLE, OPEN_GRIPPER, PLAN_PREGRASP, etc.)
string state

# Human-readable status message
string message

# Whether the operation is still active
bool active

# Whether the operation succeeded (valid only when active=false)
bool success

# Target object ID being manipulated
string object_id

# Elapsed time in seconds since operation started
float64 elapsed_time
