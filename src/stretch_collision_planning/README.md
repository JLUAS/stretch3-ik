# Overview

*stretch_collision_planning* holds the collision side of the arm motion: scenes the arm can collide with, a capsule model of the arm, state and edge validity checks, and an RRT-Connect planner over the five arm coordinates (lift, arm extension, wrist yaw/pitch/roll). The guarded executor links it as a library; `octomap_builder` is the only node.

## Library

| Header | Description |
|---|---|
| `primitive_environment.hpp` | Box scenes; `makeDeskBenchmark()` is the desk used by the kinematic simulation |
| `octomap_environment.hpp`, `octomap_scene_store.hpp` | OctoMap scene with freshness timeout, unknown-space policy and ignored capsules (the arm itself) |
| `robot_collision_model.hpp` | Conservative capsule model of the telescoping arm, wrist and gripper |
| `state_validity.hpp` | Joint limits plus robot-vs-scene checks for one state or an interpolated edge |
| `rrt_connect.hpp` | Seeded RRT-Connect with goal bias and shortcutting |

## octomap_builder

Builds an OctoMap from the head camera point cloud and publishes it for the executor and the grasp mission.

```bash
ros2 run stretch_collision_planning octomap_builder --ros-args -p map_frame:=odom
```

| Topic / service | Description |
|---|---|
| `/camera/depth/color/points` (in) | Input cloud (`input_cloud_topic`) |
| `/perception/octomap_binary` (out) | OctoMap for the executor |
| `/perception/filtered_points` (out) | Cloud after range, voxel and self filters, read by the grasp mission |
| `/perception/octomap_markers`, `/perception/scene_status` (out) | RViz voxels and a status line |
| `~/target_box` (in) | Box to keep out of the map: the object during a planned contact |
| `~/reset`, `~/capture_once` | Clear the map; clear it, capture the next valid cloud and freeze the map |

The self filter removes points inside the arm capsules and link boxes at the cloud's timestamp, so the arm never becomes its own obstacle. The map is cleared when odometry jumps (the driver resets it on every switch to trajectory mode).

## Tools

`planning_demo` plans one query on the desk scene; `cspace_slice` prints a 2D slice of the configuration space. Tests: `test_collision_validity`, `test_rrt_connect`, `test_octomap_environment`.
