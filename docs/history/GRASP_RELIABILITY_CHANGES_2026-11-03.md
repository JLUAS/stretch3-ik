# CHANGES: grasp mission reliability (2026-11-03)

> Dated working notes, kept as history. Paths such as `~/stretch3-ik/ws` and the baseline tarballs refer to the development machine at the time; `TEST_RESULTS.md` was not published. The current behavior is in the package docs.

Scope: `~/stretch3-ik/ws` only. Nothing was deployed, nothing was committed,
and nothing touched the robot. Baseline for every diff:
`~/stretch3-ik/baseline_ws_src_2026-11-03.tgz` (ws/src + ws/scripts before
any edit). See GRASP_RELIABILITY_AUDIT_2026-11-03.md for the evidence behind each change and
TEST_RESULTS.md for what was measured.

## Rollback

```bash
cd ~/stretch3-ik/ws
rm -rf src scripts && tar xzf ../baseline_ws_src_2026-11-03.tgz
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stretch_custom_kinematics stretch_custom_manipulation
```

Per file: every change below is confined to the file named. The executor,
collision planning, guards, runstop handling, joint limits and the gripper
bridge are unchanged.

## 1. `grasp_mission.py`: one request, one mission, one result

| Change | Why (AUDIT ref) | Safety effect |
|---|---|---|
| `GraspObject` action at `/grasp_mission/grasp` (goal: `mission_id`, `target_id`; feedback: stage; result: success, `error_code`, `failed_stage`, message, held aperture). `~/pick` stays and returns `started mission_id=...` or the rejection code at once; its final result goes to `~/result` (latched JSON). | R1 | None on motion. |
| Readiness before a mission claims the arm: observation (present, valid, confidence, age, base not moved since capture), joint states, homed and not runstopped, ik_preview and executor status fresh, executor idle, executor services, driver action, gripper bridge, mode topic, camera info, TF. Fails with `NOT_READY` / `TARGET_UNKNOWN` / `STALE_OBSERVATION` / `PERCEPTION_INVALID` before any motion. | R1, R6 | Fewer motions started on a robot that is not ready. |
| Exactly one job owns the arm (`self.active`): missions, `~/look`, `~/release` and the Nav2-arrival head move. Distinct goal while busy: `BUSY`. Same `mission_id` while running: `DUPLICATE_ACTIVE`. Same ID after it finished: the stored result, no motion. | R5 | Removes the head-thread race with a running mission. |
| `prepare_arm()` before every executor call: switch to trajectory mode itself, wait until `/mode` reports it, then wait until the arm joints are still for 0.5 s (bounded, `ARM_NOT_SETTLED`). | R2 | The executor now plans synchronously; its result comes back in the service response. |
| Executor result correlated by `REQUEST_ID` (newer than the one before the call): any `ABORTED`/`DISARMED` for this request fails the stage at once with the executor's reason, mapped to `COLLISION`, `CONTROLLER_REJECTED`, `IK_FAILED` or `MOTION_ABORTED`. No more waiting for a state the executor never enters. | R2 | Failures surface in < 1 s instead of after the 60 s timeout. |
| IK answers matched by the goal's header stamp echoed by ik_preview; `stale state` answers are ignored, not taken as "unreachable". | R4 | No stale `base_dx` can drive the base. |
| Any failure after motion may have started: cancel the mission's own driver/gripper goals, call the executor STOP, and wait up to 3 s for `pending_goal=0`, `stop_unconfirmed=0`, not `EXECUTING`. If not confirmed: result `STOP_UNCONFIRMED` and a latched fault that blocks new missions until `~/clear_fault` (which refuses while anything is active). | R3 | Fail closed; never retries after an unconfirmed stop. |
| FK check of every IK answer for the grasp: achieved approach within 0.05 rad of the plan, finger axis within 0.10 rad, approach turns < 0.03 rad between pre-grasp and grasp, and the planner's finger/body clearance re-run at the achieved orientation. Rejected candidates are logged with residuals (`GRASP_GEOMETRY`). | G1 | Rejects grasps the IK accepted inside its 0.12 rad tolerance. |
| Final approach as a straight line in steps of at most 5 cm, each a full guarded executor motion that must use the DIRECT (straight) path; the achieved approach is checked per step. | G1, Phase 3 | Same guards as before, more checkpoints. |
| Fresh observation at pre-grasp: prompts the segmenter at the projected object centroid, waits up to 12 s for a newer observation; aborts with `LOST_TARGET` (none, invalid, low confidence) or `TARGET_MOVED` (> 3 cm); re-plans the same grasp and moves to a refined pre-grasp when the grasp moved > 5 mm. | Phase 3 | No approach without a current view of the target. |
| Closure check: closing must reduce the aperture by >= 5 mm (`GRIPPER_NOT_CLOSED`), fingers must stop >= 1 cm apart (`NOTHING_GRASPED`, then back off), and again after lift and retreat (`OBJECT_SLIPPED`). | R7 | A close command alone is never treated as a grasp. |
| Dry run never reports success (`DRY_RUN`). | R7 | |
| Object cloud converted to `base_link` with TF at the capture stamp, not the latest TF. | G3 | |
| Bounded recovery: a synchronous executor rejection (nothing moved) is retried once after 1 s, with mode and settle re-verified. Collision, IK, motion aborts and gripper faults are never retried. Base corrections stay bounded at 3. | Phase 1 | No generic mission retry. |
| Structured events: JSON lines on `~/events` and in the log with `mission_id`, `stage`, `event`, `t_mono`, `t_ros`, elapsed, stage durations, target/achieved poses (frame `base_link`), executor responses, error codes. | Phase 1.5 | |
| Debug markers on `~/debug`: object frame, planned grasp and pre-grasp frames, FK-achieved TCP frame, approach line. Display added to `rviz/bringup.rviz`. | Phase 2 | |

New parameters (defaults): `max_observation_age` 120 s, `min_confidence` 0.5,
`max_approach_error` 0.05 rad, `max_approach_change` 0.03 rad,
`max_finger_error` 0.10 rad, `approach_step` 0.05 m, `reobserve_at_pregrasp`
true, `reobserve_timeout` 12 s, `max_target_shift` 0.03 m, `replan_threshold`
0.005 m, `settle_timeout` 4 s, `min_close_travel` 0.005 m,
`camera_info_topic` `/camera/color/camera_info`.

## 2. `ik_preview.cpp`

- Status adds `goal_stamp_ns` (header stamp of the external goal being
  answered; 0 for the interactive marker) and `solved_pose` (FK of the
  solution, `x y z qx qy qz qw`, only on success). Additive: the executor
  parses only the keys it parsed before.
- The stale-state path resets `base_dx` to NaN instead of reporting the last
  solve's correction.

## 3. `bringup.launch.py` (real robot section)

- ik_preview `grasp_approach_tolerance: 0.05` (was the 0.12 default). In
  the sweep it changes nothing for straight side grasps (all already within
  0.046 rad) and stops top-down solutions up to 0.099 rad off the planned
  approach from being reported as success. Side grasps needing a base
  correction: 20 of 32 at 0.12, 22 of 32 at 0.05. Revert: delete that line.

## 4. `grasp_planning.py`

- `plan_grasps` default `yaws=(0.0,)` (was `(0.0, 0.2, -0.2)`): with the base
  fixed, a yawed approach cannot be a straight line, and the real IK never
  solved one (0 of 128, AUDIT G1). Pitch -0.25
  and top-down candidates are unchanged.
- New `fits(obj, scene, rot)`: the existing candidate check for an explicit
  orientation. New `outside_gripper(points, grasp_center)`: oriented box
  around fingers and wrist.

## 5. Perception boundary

- `msg/ObjectObservation.msg` (new): header (capture stamp, frame),
  `object_id`, `source`, `confidence`, `valid`, `reason`, mask, cloud,
  centroid, size.
- `sam2_segmenter.py`: SAM 2 moved into `Sam2Backend.segment(rgb, (u, v)) ->
  (mask, score)`; parameter `backend` (`module:Class`, default
  `sam2_segmenter:Sam2Backend`) selects the implementation. Also listens on
  `/perception/segment_prompt`. Publishes `ObjectObservation` on
  `/perception/segmentation` (an invalid one with the reason on failure);
  `/perception/object_mask` and the overlay are unchanged. Mask selection
  (`pick_mask`) and model loading are unchanged.
- `object_localizer.py`: consumes `/perception/segmentation`, publishes
  `ObjectObservation` on `/perception/object_observation` (always: valid, or
  invalid with the reason) and still `/perception/object_cloud` for RViz.
  The 10 cm sphere self-filter around three gripper links is replaced by the
  gripper/wrist box (AUDIT G2). A missing `link_grasp_center` TF now fails
  the observation instead of skipping the filter.
- `CMakeLists.txt`/`package.xml`: generate the new msg and action
  (`sensor_msgs`, `action_msgs`), register the two new pytest files.

Deployment note (not done): the SAM 2 virtualenv Python must import
`stretch_custom_manipulation.msg` (it already imports rclpy from the ROS
install, so sourcing the rebuilt install should be enough; unverified on
the robot).

## 6. Test tooling (new, test-only)

- `ws/scripts/sim_isolated.sh`: runs a command in a private user+network+mount
  namespace with only loopback, private `/dev/shm` and `/sys`, domain 20,
  localhost-only; refuses if a non-loopback link, a route to 192.168.12.3,
  or a shared `/dev/shm` is visible.
- `test/sim/`: `mock_stretch_driver.py`, `mock_perception.py` (both refuse to
  start outside domain 20 / with a non-loopback interface),
  `grasp_sim.launch.py` (refuses the same), `run_trials.py`,
  `action_checks.py`, `ik_sweep.py`, `run_sim.sh`.
- `test/test_grasp_mission.py`, `test/test_perception_chain.py`.

## Not changed

Guarded executor, guards, collision planning, OctoMap builder, runstop and
stop handling, joint limits, gripper bridge, IK solver, pick/place state
machines, autonomous_mission (Gazebo), Nav2 configuration. No semantic
navigation work.
