# Current gripper state.

std_msgs/Header header

# Aperture in meters (0 = fully closed)
float64 aperture

# Effort (normalized, negative = squeezing)
float64 effort

# Whether object contact is detected
bool object_detected

# Whether the gripper is actively grasping (closed on object)
bool is_grasping

# Raw position value from driver [-100, 200]
float64 raw_position
