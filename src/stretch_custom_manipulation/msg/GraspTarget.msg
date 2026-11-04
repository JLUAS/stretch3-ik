# Grasp target produced by perception/grasp estimation.
# Used by the pick state machine to plan approach and grasp.

std_msgs/Header header

# Pre-grasp pose: collision-free approach position
geometry_msgs/PoseStamped pregrasp_pose

# Final grasp pose: TCP target when closing gripper
geometry_msgs/PoseStamped grasp_pose

# Direction of approach from pregrasp to grasp (unit vector in grasp frame)
geometry_msgs/Vector3 approach_vector

# Required gripper opening in meters (0 = closed, max ~0.13 m for Stretch)
float64 gripper_opening

# Grasp mode: 0=PINCH, 1=WRAP, 2=PUSH
uint8 grasp_mode
uint8 PINCH=0
uint8 WRAP=1 
uint8 PUSH=2

# Confidence [0.0, 1.0]
float64 confidence

# Object identifier (optional)
string object_id
