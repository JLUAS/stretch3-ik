# Shelf grasping on the real Stretch 3

Click an object in the head camera image, and the robot plans and executes a
side (or top-down) grasp with an opening sized to the object. Everything runs
on the robot's NUC.

```
D435i RGB-D --click--> rgbd_perception --mask--> object_localizer --odom cloud-->
grasp_mission --grasp goal--> ik_preview --> guarded_executor (RRT on OctoMap)
                  |--> gripper_bridge (GripperCommand -> driver)
                  `--> driver translate_mobile_base (small base correction, lidar-guarded)
```

## What each piece does

| Node | Role |
|---|---|
| `octomap_builder` | Persistent OctoMap in `odom`, filled while driving, updated at the shelf. Removes the robot (arm capsules + link boxes) and the target object (`~/target_box`) from the map |
| `rgbd_perception` | Continuous geometry detection, persistent IDs, OCR and `/perception/objects`; clicks produce `/perception/object_mask` and overlay |
| `object_localizer.py` | Mask + aligned depth -> object cloud in `odom` (Open3D outliers + largest cluster) |
| `grasp_mission.py` | Head control, grasp planning (`grasp_planning.py`), base correction, pick sequence |
| `ik_preview` | Grasp goals on `/ik_preview/grasp_goal`: position + approach + finger axis. Reports `base_dx` when the fixed base cannot reach |
| `guarded_executor` | Guards, RRT on the OctoMap, faster minimum-time trajectories, stops if a new obstacle appears on the remaining path, refuses to move while the base moves |
| `gripper_bridge.py` | `/stretch_gripper_controller/gripper_action` in meters of aperture, on the driver's trajectory action |

## One-time setup on the robot

1. Deploy and build (from the development machine):
   `ROBOT=<user>@<robot-ip> BASELINE=<local copy of the robot workspace> ./scripts/deploy_to_robot.sh`
   It replaces only this repository's packages in `~/ament_ws/src` (and
   `~/ament_ws/scripts`), aborts if the robot copies changed since
   `BASELINE`, and leaves a backup tarball plus the old copies outside the
   workspace.
2. On the robot: `./scripts/install_perception.sh` (Open3D, OpenCV, SciPy and
   RapidOCR in `~/venvs/stretch_perception`). No SAM checkpoint is used.
3. Rebuild all packages: `ObjectObservation` now belongs to
   `stretch_perception_interfaces`; all publishers/subscribers must use this type.

## Map of the current surroundings

With the robot standing where the work will start (nothing else running):

```bash
~/ament_ws/scripts/grasping_quickmap.sh        # ~65 s, saves ~/maps/grasping_area
```

It maps what the lidar sees from that spot without driving. The map origin
is the robot's pose there, so the default AMCL start pose (0, 0, 0) matches
as long as the robot starts from the same spot.

## Run

```bash
cd ~/ament_ws
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=false \
  use_slam:=false map:=$HOME/maps/grasping_area.yaml dry_run:=true
```

`scripts/grasping_dryrun.sh` and `scripts/grasping_run.sh` wrap these two
commands and keep each run's output in `~/grasping_logs/`. They need
`HELLO_FLEET_ID` set (the robot serial) when started over SSH.

`dry_run:=true` plans and validates but sends no arm motion. Use
`dry_run:=false` once the dry run looks right. The robot must be homed and
the runstop released.

Pass `perception_viewer:=true` to open rqt on `/perception/preview` (upright).
Set rqt rotation to **0 degrees** and the click topic to
`/perception/preview_mouse_left`. Alternatively use the native
`/camera/color/image_raw` and `/camera/color/image_raw_mouse_left`.
See [the perception package](../../stretch_rgbd_perception/README.md) for geometry
frames, dimensions, configuration and perception-only testing.

## Operator workflow

1. AMCL starts at `initial_x/initial_y/initial_yaw` (default: map origin).
   If the robot is elsewhere, click **2D Pose Estimate** in RViz and drag
   where it really is, or launch with its pose, e.g. `initial_x:=1.2`.
2. Park the robot with the object on its right, about 0.55-0.9 m from the
   base center (closer: `object too close`, farther: `object too far`). Nav2 is
   optional: aim the head however you like (Nav2 arrival or
   `ros2 service call /grasp_mission/look std_srvs/srv/Trigger` turns it right).
3. Click the object in `rqt_image_view`. With `auto_start` (default) the click
   starts the mission as soon as a collision-free grasp is planned; status says
   `PLANNED: N grasps ...; starting click-...`, or `PLAN FAILED`/`NOT STARTED`
   with the reason. Nothing moves in those two cases. Live options:
   `ros2 param set /grasp_mission grasp_mode top|side|auto`,
   `fully_open true|false`, `auto_start false` (then use step 4).
4. `ros2 service call /grasp_mission/pick std_srvs/srv/Trigger` answers at
   once: `started mission_id=pick-...` or the reason it did not start
   (`NOT_READY: missing: ...`, `BUSY`, `STALE_OBSERVATION`, ...). Submit once;
   a refused pick never moved anything. The action
   `/grasp_mission/grasp` (`stretch_custom_manipulation/action/GraspObject`)
   does the same with feedback and a result.
5. Follow `ros2 topic echo /grasp_mission/status` (stages) or
   `/grasp_mission/events` (JSON). The mission ends once: `DONE` (pickup
   verified), `FAILED: <id> at <stage>: <CODE>: <reason>` or `CANCELED`;
   `/grasp_mission/result` holds the last result as JSON.
6. `ros2 service call /grasp_mission/release std_srvs/srv/Trigger` opens the
   gripper. `/grasp_mission/stop` stops arm, base correction and the mission.
   `STOP_UNCONFIRMED` blocks new missions: check the robot, then
   `ros2 service call /grasp_mission/clear_fault std_srvs/srv/Trigger`.

## Pick sequence

1. Plan side grasps (straight along -y, 0/-0.25 rad pitch) and a
   top-down grasp, rejecting any whose gripper body or fingers hit scene
   points. Opening = object width + 1.5 cm per side, limited by the gap to
   neighbors and by 13 cm.
2. If the arm cannot reach from where the base stands, drive the base along
   X by the `base_dx` the IK reports (driver `translate_mobile_base` in
   position mode, stopped if the lidar sees something in the base path), up
   to 3 times, max 0.5 m.
3. Each IK answer is checked against the plan with forward kinematics
   (approach within 0.05 rad, no turn between pre-grasp and grasp, fingers
   still fit); ik_preview's approach tolerance is 0.05 rad.
4. Fingers closed for the transit -> pre-grasp (10 cm before the object)
   -> open -> optional fresh observation (`reobserve_at_pregrasp`, off by
   default) -> remove the object from the map -> straight approach in 5 cm
   steps -> close (squeeze) -> verify the fingers moved and stopped on the
   object -> lift 5 cm -> retract the arm -> carry pose -> squeeze again to
   check the object did not slip.
5. Before every arm motion the mission switches to trajectory mode itself and
   waits for the arm to stand still; any failure after motion started stops
   the executor and waits for the stop to be confirmed.

## Tuning (grasp_mission parameters)

| Parameter | Default | Meaning |
|---|---|---|
| `search_pan`, `search_tilt` | -1.571, -0.45 | Head pose for the right side (shelf at ~1 m) |
| `navigation_pan`, `navigation_tilt` | 0.0, -0.6 | Head pose while Nav2 drives |
| `pregrasp_offset` | 0.10 | Pre-grasp distance before the grasp (m) |
| `close_aperture` | -0.02 | Squeeze target when closing (m) |
| `min_held_aperture` | 0.01 | Below this after closing: nothing grasped |
| `max_object_size` | 0.35 | Larger selections are rejected (shelf, wall, leaked mask) |
| `max_base_correction` | 0.5 | Largest base correction before asking to re-navigate (m) |
| `max_approach_error`, `max_approach_change` | 0.05, 0.03 | FK-achieved approach vs plan, and turn between pre-grasp and grasp (rad) |
| `approach_step` | 0.05 | Longest single step of the final approach (m) |
| `reobserve_at_pregrasp` | false | Fresh observation before the approach. Off: at the pre-grasp the gripper hides the object from the head camera |
| `max_target_shift` | 0.03 | Larger shift at re-observation aborts (m) |
| `max_observation_age` | 120 | Older clicks are refused (s) |

Arm speed lives in `config/real.yaml` (`max_velocity`, `max_acceleration`):
0.10 m/s lift/arm, 0.6 rad/s wrist. If tracking aborts on the robot, halve
both. Code hard caps: 0.12 m/s, 0.8 rad/s.

## First run on the robot (operator at the runstop)

1. Dry run: Nav2 goal, click a can, `pick`. Status goes through `MOVING:`
   steps with `DRY RUN:` lines for head and gripper; nothing on the arm moves.
   A needed base correction ends the dry run with its distance.
2. `dry_run:=false`, empty shelf spot: click a can, `pick`, watch the
   pre-grasp move. Stop with the runstop on anything unexpected.
3. Dynamic stop: during an approach, put a hand in the path at a safe
   distance; the executor reports `DYNAMIC_OBSTACLE` and stops.
4. Cans, then medicine boxes, then neighbors close by.

## Known limits

- One head camera: space hidden behind the arm or the object stays unknown
  and is treated as free (`unknown_is_collision: false`).
- The base correction uses odometry; the object is carried into the moved
  `base_link` frame before the next plan (the driver resets odometry on every
  switch to trajectory mode).
- No place action yet: `release` only opens the gripper.
