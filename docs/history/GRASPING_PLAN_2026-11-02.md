# Grasping implementation plan (Stretch 3)

> Dated working notes, kept as history. Paths such as `~/stretch3-ik/ws` and `robot_snapshot_2026-11-02` refer to the development machine at the time. The current behavior is in the package docs.

Source architecture: `stretch3_grasping_componentes.md` (SAM 2, RGB-D + Open3D,
OctoMap, grasp planner, IK + motion). This plan maps it onto what already runs
on the robot and lists what is new, what changes, and in what order.

## Status (2026-11-02, end of day)

Implemented and tested locally in `~/stretch3-ik/ws` (copy of the robot
workspace; not deployed, not committed). Runbook:
`ws/src/stretch_custom_manipulation/docs/GRASPING.md`.

| Phase | State |
|---|---|
| 0 Fixes | `unknown_is_collision: false`, stale dry-run test fixture fixed |
| 1 Speed | Hard caps raised, one minimum-time quintic per RRT edge with real velocities (no stop every 3 cm), real profile 0.10 m/s / 0.6 rad/s |
| 2 Map | OctoMap in `odom`, published at 5 Hz, capsule self-filter for the telescoping arm, target box for the planned contact, executor queries through `/odom`, dynamic obstacle stop, no arm motion while the base moves |
| 3 Gripper | `gripper_bridge.py` (GripperCommand on the driver action, refuses while the arm moves) |
| 4 IK | Grasp task (position + approach + finger axis) with optional base X; `ik_preview` `/ik_preview/grasp_goal` reports `base_dx` |
| 5 Perception | `sam2_segmenter.py` (click, SAM 2.1 tiny CPU, ~1.2 s on a laptop CPU), `object_localizer.py` (Open3D) |
| 6 Planner | `grasp_planning.py`: side/top candidates, adaptive opening limited by neighbors and 13 cm |
| 7 Mission | `grasp_mission.py`: head ahead while Nav2 drives, fixed right on arrival, base correction with Nav2, full pick, dry-run mode |
| 8 Tests | 229 tests pass; kinematic-sim smoke runs (pick DONE, dry run DONE, base correction requested) |

Pending on the robot (operator at the runstop): deploy, `install_sam2.sh`,
dry run, speed check, first grasps.

## Baseline: the robot workspace, not GitHub

`~/stretch_custom_manipulation_ws` on the robot (not under git) is well ahead of
this repository. A copy taken 2026-11-02 is in `../robot_snapshot_2026-11-02/`.
Extra on the robot:

- `stretch_collision_planning`: OctoMap environment, capsule robot model,
  state validity, RRT-Connect with shortcutting, `octomap_builder` node with
  link-box self-filter and `~/capture_once`, primitive desk environment.
- Executor: validates and executes collision-planned multi-point trajectories.
- `pick_state_machine.py`, `place_state_machine.py`, `GraspTarget`,
  `GripperState`, `ManipulationStatus` msgs, `config/perception.yaml`.
- `autonomous_mission.py`: target -> Nav2 approach (right side facing target)
  -> head aim -> OctoMap capture -> planned execution (Gazebo-oriented).

**Step 0 of everything:** commit the robot workspace into this repository so
the newest code has version control.

## Decisions (2026-11-02)

| Topic | Decision |
|---|---|
| Compute | Everything on the NUC (i5-1240P, 16 threads, 15 GB, no GPU) |
| Camera | Head D435i only |
| Object selection | Manual click on the color image, over remote desktop (VNC on :1) |
| Objects | Cans and medicine boxes |
| Navigation | Nav2 (`stretch_nav2`), maps already on the robot (see Open questions) |
| Cliff sensors | Stay off; secured area |
| Obstacle map | Persistent OctoMap built while navigating, updated at the grasp spot |
| Unknown space | `allow`: unseen voxels are free (operator commands safe trajectories) |
| Speed | Raise the executor caps |

## Measured robot interfaces (2026-11-02)

Full capture: `evidence/grasping/robot_interfaces_2026-11-02.txt`.

| Interface | Value |
|---|---|
| Color | `/camera/color/image_raw`, rgb8, 424x240, 15 Hz, `camera_color_optical_frame`, RELIABLE + TRANSIENT_LOCAL |
| Aligned depth | `/camera/aligned_depth_to_color/image_raw`, 16UC1 (mm), 424x240, 15 Hz, same frame |
| Intrinsics | fx 303.06, fy 303.07, cx 212.06, cy 122.79, no distortion |
| Cloud | `/camera/depth/color/points`, ~11 Hz, ~38k points, `camera_depth_optical_frame` |
| Lidar | `/scan` 7.9 Hz, `/scan_filtered` |
| Odom | `/odom` 30 Hz, `odom` -> `base_link`; **TF only when `broadcast_odom_tf=True`** (Nav2 launch sets it, plain driver launch does not) |
| Arm action | `/stretch_controller/follow_joint_trajectory` (only action on the driver) |
| Gripper | No `GripperCommand` server. In trajectory mode the driver's joint is `stretch_gripper` (finger radians) |
| Base in trajectory mode | Multi-DOF joint `position` (odom transform) |
| Mode services | `/switch_to_navigation_mode`, `/switch_to_trajectory_mode`, `/switch_to_position_mode` |
| Driver safeguard | `fail_if_motor_initial_point_is_not_trajectory_first_point=True` |
| Installed | `nav2_bringup`, `slam_toolbox`, `octomap_server`, `octomap_ros`, `realsense2_camera`, Open3D 0.18 (system Python), torch 2.4.0 (system Python) |
| Launch used | `stretch_core d435i_low_resolution.launch.py` |

## Gap analysis against the components doc

| Component | On the robot now | Gap |
|---|---|---|
| 1. SAM 2 | Nothing | Segmentation node |
| 2. RGB-D + Open3D | Raw cloud only | Mask -> object cloud -> OBB |
| 3. OctoMap | Snapshot map in `base_link`, unknown = collision on real | Persistent map in `odom`, unknown = free, monitoring during motion |
| 4. Grasp planner | `GraspTarget` msg only | Candidates from OBB |
| 5. IK + motion | IK (XYZ + local Z), RRT-Connect, executor, pick/place SMs | Approach-axis IK, speed caps, real gripper, base dx |

## The previous arm error

`config/real.yaml` sets `collision_planning.unknown_is_collision: true`. Every
voxel the camera has not observed counts as a collision, so RRT and validation
reject any path through unseen space (behind the arm, under the table edge,
outside the camera view). Fix: `false`, as `sim.yaml` already does.

## Phases

### Phase 0: Version control and quick fixes

1. Commit the robot workspace into this repo (new branch), keeping GitHub-only
   files (LICENSE, README, evidence).
2. `real.yaml`: `unknown_is_collision: false`.
3. Choose or build the Nav2 map for the deployment area.

**Done when:** repo matches the robot, build and existing tests pass.

### Phase 1: Faster execution

The caps are hardcoded in `Config::validate()` (`delta_cap` 3 cm / 0.15 rad,
`velocity_cap` 0.02 m/s / 0.1 rad/s, `min_duration >= 3 s`); YAML alone cannot
raise them.

- Raise the code caps; compute each segment's duration as the minimum quintic
  time: `T = max(1.875*|d|/v_max, sqrt(5.774*|d|/a_max), T_min)`.
- Final targets, reached in 3 steps on hardware with tracking error measured at
  each: lift/arm 0.10 m/s, 0.15 m/s^2; wrists 0.6 rad/s, 1.5 rad/s^2;
  0.30 m / 1.5 rad per segment; `T_min` 0.5 s.
- Planned (RRT) trajectories get the same time-parameterization.
- Widen tracking envelopes from measured lag.

**Done when:** guard unit tests and sim launch test pass at the new caps.

### Phase 2: Persistent OctoMap and dynamic monitoring

- `octomap_builder` with `map_frame: odom` (Nav2 launch provides the odom TF);
  `reset_map_each_cloud: false`; continuous insertion instead of capture-only.
- Executor collision check: transform robot capsules from `base_link` into the
  map frame at query time (one rigid transform) instead of requiring the map in
  `base_link`.
- Head scan while navigating: port `camera_scan.py` from Gazebo joints to the
  driver's `joint_head_pan` / `joint_head_tilt` via the trajectory action (the
  driver accepts head goals in navigation mode).
- Dynamic monitor: during execution re-check the remaining trajectory against
  each map update; a new occupied voxel calls `/guarded_executor/stop`. The
  head looks at the workspace during the move. Padding covers reaction
  distance (expected 0.3-0.5 s; measured in Phase 7).

### Phase 3: Real gripper

`pick_state_machine.py` calls `/stretch_gripper_controller/gripper_action`
(`GripperCommand`), which only `sim_gripper.py` provides. Add a small bridge
that serves that action on the real robot and sends `stretch_gripper`
trajectory goals, reporting position from `/stretch/joint_states`. The pick
state machine stays unchanged. Grasp check: fingers stop short of closed.

### Phase 4: IK grasp mode and base dx

- IK mode: position + approach axis (local X), roll set analytically from the
  finger axis, base X as a virtual prismatic joint.
- Base dx executed by Nav2 (`DriveOnHeading` / `BackUp`) before the arm moves,
  or by the driver's `position` multi-DOF joint in trajectory mode if Nav2 is
  not precise enough at a few cm (test both).
- Executor refuses arm motion while `/odom` twist is non-zero.
- Tests in the style of `test_ik.cpp`.

### Phase 5: Perception (new package `stretch_grasp_perception`)

- **Segmentation node:** color image + click from `rqt_image_view` (opened in
  the VNC desktop) -> SAM 2.1 `hiera_tiny` on CPU -> `mono8` mask. SAM 2 needs
  torch >= 2.5.1 and the system has 2.4.0, so it lives in its own venv; Phase 5
  starts with a CPU benchmark (accept <= 3 s per click at 424x240; fallback
  MobileSAM behind the same node).
- **Object cloud node:** mask + aligned depth + intrinsics -> Open3D (system
  0.18): drop zero depth, outlier removal, RANSAC table plane, largest cluster,
  transform to `base_link` and `odom` with TF2 at the image stamp; publish
  object cloud and OBB.
- Tests on rosbags recorded with the topics above.

### Phase 6: Grasp planner -> `GraspTarget`

- Candidates from the OBB. Cans: side wrap first, top-down second. Medicine
  boxes: top-down across the narrow side first, side second.
- Filter by aperture, grasp-mode IK (pre-grasp, approach line, retreat), base
  dx bound, OctoMap clearance with the object's padded OBB ignored (planned
  contact).
- Publish the existing `GraspTarget` message; `pick_state_machine.py` consumes it.

### Phase 7: Mission integration and commissioning

- Adapt `autonomous_mission.py` for the real robot: Nav2 to zone with head scan
  -> look at area -> click -> segment -> plan -> base dx -> pick SM -> retreat.
- Unit tests per module, sim end-to-end with a fixed object fixture, bag tests
  for perception, real-robot checklist: speed ramp, STOP latency at speed,
  empty-gripper approach, then cans and boxes.

## Out of scope for v1

MoveIt, text prompts / semantic detector, SAM 2 video tracking, gripper camera.

## Risks

- SAM 2 CPU latency and NUC load next to Nav2 + camera + OctoMap.
- Cliff sensors off: Nav2 relies on lidar only; safe only in the secured area.
- Faster arm: tracking envelopes and STOP latency must be re-measured.
- OctoMap in `odom` drifts over long drives; re-scan the grasp area on arrival.
- Smooth cans can slip: check aperture after closing and after lifting.
- The robot workspace has no version control until Phase 0 step 1.

## Open questions

- Which map is the deployment area: `~/maps/lab.yaml`, `pasillo_lab.yaml`,
  `puzzlebot_lab.yaml`, `~/stretch_user/maps/map.yaml`, `nav2_demo_map.yaml`,
  or a new one?
- Commit the robot workspace to `main` or to a new branch?
