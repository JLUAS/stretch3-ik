# Segmenter-independent object observation. Stamp: capture time of the color
# image; frame: frame of cloud, centroid and size.
std_msgs/Header header
string object_id
# Segmentation backend that produced the mask (e.g. sam2, fake).
string source
# Backend score in [0, 1].
float32 confidence
bool valid
# Why the observation is invalid; empty when valid.
string reason
# mono8 mask over the color image (255 = object); may be empty downstream.
sensor_msgs/Image mask
sensor_msgs/PointCloud2 cloud
geometry_msgs/Point centroid
# Axis-aligned extent in header.frame_id (m).
geometry_msgs/Vector3 size
