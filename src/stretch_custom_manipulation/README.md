# Overview

*stretch_custom_manipulation* contains the unified bringup (launch file, simulation and real configuration, RViz layout), the shelf grasp mission, the perception nodes that feed it, the gripper bridge, and the end-to-end tests.

```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=true scenario:=fixture
```

`run_manipulation.sh` selects domain 20 (`sim:=true`, localhost-only) or 42 (`sim:=false`), restarts the ROS daemon, sources the workspace and forwards all other arguments to `bringup.launch.py`. On the real robot it adds `dry_run:=true` and `collision_environment:=octomap` unless they are given.

| Argument | Default | Description |
|---|---|---|
| `sim` | `true` | Simulation on domain 20, or the real driver on domain 42 |
| `scenario` | `small_house` (wrapper) | `fixture`: kinematic simulation with a primitive desk; `small_house`: Gazebo world from a separate workspace |
| `rviz` | `true` | Start RViz |
| `dry_run` | `false` (`true` on the robot via the wrapper) | Validate without sending trajectories |
| `execution_enabled` | `true` | `false` blocks validation and execution |
| `grasp_mission` | `true` | Start the grasp mission and perception (real robot only) |
| `use_slam`, `map` | `true`, empty | Live SLAM, or a saved map with `use_slam:=false map:=<yaml>` |
| `start_robot_driver`, `start_robot_camera`, `start_robot_lidar`, `start_nav2` | `true` | Skip a component only if it already runs elsewhere |
| `perception_python` | `~/venvs/stretch_perception/bin/python3` | Python environment from `scripts/install_perception.sh` |
| `perception_pose_source`, `perception_ocr`, `perception_viewer` | `tf`, `true`, `false` | Capture-time pose provider, background OCR and optional rqt viewer |

Configuration: `config/common.yaml`, `config/sim.yaml`, `config/real.yaml`, `config/perception.yaml`.

## Grasping nodes

| Executable | Description |
|---|---|
| `grasp_mission.py` | One mission at a time from an `ObjectObservation` to a verified pickup: readiness, grasp selection with IK and FK checks, base correction, pre-grasp, straight approach, close, carry, hold check. Action `~/grasp` (`GraspObject`), services `~/pick`, `~/stop`, `~/release`, `~/look`, `~/clear_fault` |
| `grasp_planning.py` | Side and top-down grasp candidates with an opening sized to the object and limited by its neighbors (library, no node) |
| `rgbd_perception` (`stretch_rgbd_perception`) | Continuous depth segmentation, persistent IDs, OCR, typed visible geometry, and click masks |
| `object_localizer.py` | Mask plus aligned depth to an object cloud in `odom` (Open3D outlier removal, largest cluster, gripper self-filter) |
| `gripper_bridge.py` | `GripperCommand` in meters of aperture on top of the driver; refuses while an arm trajectory runs |

Interfaces: `action/GraspObject.action`, `stretch_perception_interfaces/msg/ObjectObservation.msg` (plus `GraspTarget`, `GripperState`, `ManipulationStatus` used by the older state machines).

## Other scripts

`pick_state_machine.py`, `place_state_machine.py` and `autonomous_mission.py` are the earlier Gazebo-oriented pick, place and navigate-then-pick flows; `sim_gripper.py` and `camera_scan.py` support them. The real-robot path is `grasp_mission.py`.

## Tests

`test/` holds pytest suites for the mission, planner, perception chain and state machines, plus the one-click launch test. `test/sim/` runs the real executor, IK and gripper bridge against `mock_stretch_driver.py` and `mock_perception.py`; both refuse to start outside domain 20 or with a non-loopback interface. Run them through `scripts/sim_isolated.sh` (see the top-level README).

- [Grasping runbook](docs/GRASPING.md)
- [Segmentation interface](docs/SEGMENTATION_INTERFACE.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Real robot quickstart](docs/REAL_ROBOT_QUICKSTART.md)
- [Real robot test checklist](docs/REAL_ROBOT_TEST_CHECKLIST.md)
- [Gripper contract](docs/GRIPPER_CONTRACT.md)
