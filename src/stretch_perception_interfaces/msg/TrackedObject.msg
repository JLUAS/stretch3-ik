# Geometry of ONE CURRENT detection, measured only from visible depth points.
# Header stamp/frame comes from the ORIGINAL unrotated RGB optical camera.
std_msgs/Header header
bool has_instance_id
uint64 instance_id
bool confirmed
string product
# Object frame -> header.frame_id. Origin: visible 3D bounding-box center.
# Axes: upright calibrated camera X right, Y down, Z forward. Orientation is
# this fixed axis convention, NOT an estimated physical package orientation.
geometry_msgs/Pose pose
bool orientation_estimated
# Visible extents in the object axes above, in meters; hidden faces unknown.
geometry_msgs/Vector3 size
# size.x: extent along upright camera X. For gripper opening use the cloud's
# extent along the selected closing axis, plus clearance (grasp_planning.py).
float32 horizontal_width
uint32 visible_points
