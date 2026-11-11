# Semantic grasp in simulation: implementation plan

Goal for Thursday 2026-11-12: in a simulated house, the user types "grab the red mug in the kitchen". Stretch finds the mug, drives to a base pose that puts it within arm reach, re-detects it, and grasps it with the existing pipeline.

## Status at end of Tue 2026-11-10

Works in the MuJoCo house, verified live on the red mug: text parsing, keyframe memory lookup (mug within ~3 cm), base pose with the object on the arm side, Nav2 across rooms and doorways, head aim, live OWLv2 detection, the click handoff to perception, object localization, grasp planning (side grasp, base corrections).

Not working yet: no object has been grasped. The last run reached MOVE_PREGRASP and the executor aborted with "mobile base moving or odometry stale".

## Plan for Wed 2026-11-11 (finish the sim)

| # | Task | Est. | Done when |
|---|---|---|---|
| 1 | Put the launch helpers in the repo (`scripts/sim_house/`: driver, stack, semantic node, goto, look, click, say). The session copies were in a scratchpad and are gone. | 0.5 h | One command starts each piece. |
| 2 | Sim driver: keep odometry and TF publishing while a trajectory runs (action server in its own callback group with a multithreaded executor). Cause of the last abort. | 1 h | Pre-grasp move completes. |
| 3 | Sim driver: base `move_by` returns only after the base has stopped (min duration from distance, then N still checks). Today the mission over/undershoots by 4-5 cm. | 1 h | One base correction converges. |
| 4 | First grasp of the red mug by click: approach, close, lift, verify. Tune only through `mujoco.yaml` (MuJoCo friction, finger insert, opening). | 2-3 h | `grasp_mission` returns SUCCEEDED, mug lifted. |
| 5 | Restart the semantic node with the new code, drive the tour once, `/semantic/save`. | 0.5 h | `~/.ros/semantic_memory.npz` exists; later restarts skip the tour. |
| 6 | Full text command for the red mug, then the other five objects (bottle, can, box on the low coffee table, sponge under the shelf board, cup on the bedroom desk). Fix per-object issues (height, overhead board, head tilt). | 3 h | At least 4 of 6 objects picked from a text command. |

Cut line for Wednesday: if task 4 is not done by early afternoon, Thursday's evaluation is limited to stages up to GRASP (locate, navigate, detect, plan) and the paper reports grasp execution as future work.

## Plan for Thu 2026-11-12 (test and write up)

| # | Task | Est. |
|---|---|---|
| 7 | Trial runner: driver restart per trial (resets robot and objects), random start room, command per object; logs stage reached, failure code, time. | 1.5 h |
| 8 | Run about 20 trials (~6 min each at 0.56x real time). Start before lunch. | 2-3 h of machine time |
| 9 | Results table per stage, and one demo video of a full text-to-grasp run. | 1.5 h |
| 10 | Limitations for the paper: map from scene geometry, perfect odometry (no localization test), doorways widened to 1.4 m, simple primitive objects, sim-only perception thresholds, sim driver patches. | 0.5 h |

## What is missing, in one line each

- Driver: odometry stalls during trajectories; base moves report done too early.
- Grasp execution in MuJoCo (approach, close, lift) never reached.
- Persistent keyframe memory and a saved tour.
- Objects other than the red mug never tried.
- Trial runner, trials, table, video.

## Decisions

| Decision | Choice | Reason |
|---|---|---|
| Simulator | MuJoCo, Hello Robot's `stretch_mujoco_driver` (built in `~/cross_embodiment_sim/simulation/stretch3/ros_ws`), with a house scene of our own (`worlds/mujoco_house/house.xml`) | Same ROS interface as the real driver: `/stretch_controller/follow_joint_trajectory`, mode services, `/camera/color/image_raw`, `/camera/depth/color/points`, `/scan_filtered`. Real contact physics for the gripper. Chosen 2026-11-10 after Phase 0 found the local Gazebo small house incomplete (no trajectory action, mode services or Nav2 files, about 0.5x real time; the full `stretch3-sim-to-real-phase1` workspace is not on this laptop). RoboCasa is not installed and wants 16-32 GB RAM. |
| Text to object | OWLv2 (open vocabulary, `transformers`) in `~/venvs/stretch_perception` (torch 2.9 + CUDA already there) | The query is free text, so a fixed-class detector is not enough. Runs on the RTX 4060. |
| Handoff to grasping | Detector publishes the box center as the same `geometry_msgs/Point` a click sends on `/perception/segment_prompt` | `grasp_mission` and perception stay unchanged. |
| Semantic memory | Keyframes (RGB, depth, camera pose in `map`) saved while driving. A query runs OWLv2 with the user's text over the saved keyframes and keeps the best hit. | No fixed vocabulary to define in advance, and little code. Room names come from a small YAML of room rectangles in `map`. |
| Grasp success | MuJoCo contact physics, no attach shortcut. A grasp counts only if the object lifts with the gripper. | Honest number for the paper. |
| Location | New package `src/stretch_semantic_grasp` in this repo. The house scene goes in `stretch_custom_manipulation/worlds/mujoco_house/`. The driver gets one new parameter, `scene_xml_path`. | Keeps the tested grasp packages untouched. |

## Pipeline

```
text ──> parse ──> memory lookup (OWLv2 over keyframes) ──> object xyz in map
     ──> pre-grasp base pose ──> Nav2 NavigateToPose ──> aim head at object
     ──> live OWLv2 detection ──> Point on /perception/segment_prompt
     ──> grasp_mission (existing: base corrections, IK, guarded executor, gripper)
     ──> lift check ──> report
```

Pre-grasp base pose: the Stretch arm extends along base_link -y. Sample base yaws around the object. For each, put the base so the object sits at the arm's comfortable reach on the -y side. Reject poses whose footprint hits the Nav2 global costmap. Keep the free pose closest to the robot. `grasp_mission` then does the fine alignment it already does.

## Phases

### Phase 0: house scene + sim baseline (Tue 11-10, ~4 h)
- `scene_xml_path` parameter on `stretch_mujoco_driver`.
- House scene: three rooms (kitchen, living room, bedroom) with walls and doorways, kitchen counter, table, shelf, coffee table, nightstand. 6-8 free-body objects with distinct color and shape (mug, can, bottle, box, ball, cup), sized for the gripper.
- New bringup `scenario:=mujoco`: our stack against the driver. Wall time (the driver stamps everything with wall time), `/scan` remapped to `/scan_filtered`.
- Map: generated from the MJCF geometry (`worlds/mujoco_house/make_map.py`), Nav2 with AMCL, like the robot. slam_toolbox was dropped 2026-11-10: under this CPU load its map->odom lagged the scans and Humble's planner_server died on an uncaught TF extrapolation.
- `grasp_mission` is disabled when `sim:=true` (`grasp_mission = not sim and ...`). Enable it for `mujoco`.
- Exit: cameras, depth cloud, scan and Nav2 up; real-time factor measured.

### Phase 1: click-to-grasp in the house (Tue 11-10 evening, ~3 h)
- Exit: click-to-grasp lifts an object in the house 3 times out of 3 at one spot.

### Phase 2: text to grasp at a fixed spot (Wed 11-11 morning, ~3 h)
- `detector_node`: `/semantic/query` (String) to OWLv2 on the latest head image, publishes the box center Point and a debug image with the box.
- Text parsing: strip the verb ("grab", "pick up", "bring me"), split off "in the <room>". Regex only.
- Exit: "grab the red mug" with the robot already in front of it ends in a grasp.

### Phase 3: navigate to the object (Wed 11-11 afternoon, ~4 h)
- `pregrasp_pose.py` (pure function plus unit tests) and `semantic_grasp` orchestrator node: state machine QUERY, LOCATE, NAVIGATE, LOOK, DETECT, GRASP, VERIFY, with a reason on every failure.
- Head aim at the object's map position after Nav2 arrives.
- Exit: object position given by hand, robot starts in another room, drives over and grasps.

### Phase 4: semantic memory (Thu 11-12 morning, ~3 h)
- `memory_node`: drive a fixed waypoint tour of the house (head panned at each stop) and save keyframes. Query runs OWLv2 over them, projects the best box through depth to `map`, and optionally filters by room.
- Exit: full text to grasp from any room, no positions given.

### Phase 5: evaluation for the paper (Thu 11-12 afternoon)
- Trial runner: N queries over objects and start rooms, logging stage reached, failure reason, time, and nav/base error.
- Outputs: success table per stage (locate, navigate, detect, grasp, lift) and one demo video.

## Cut line if time runs short
1. Phase 4 becomes a hand-written object list (the paper can still show text, navigation and grasp).
2. The panel text box becomes a `ros2 topic pub` command.
3. Phase 5 drops to 20 trials.

## Known risks
- OWLv2 on MuJoCo renders: flat-colored primitives may score low. Mitigation: distinct colors and shapes, and queries that name color and shape.
- 14 GB RAM: MuJoCo with cameras, Nav2, OctoMap and OWLv2 together. Do not run cross_embodiment_sim benchmarks at the same time.
- The house is our own simple scene, less realistic than the AWS house. The paper should say so.
- Real-robot issues stay open (perception accepts about 40 % of clicks, diagnostics, calibration). Nothing here gets deployed to the robot.
