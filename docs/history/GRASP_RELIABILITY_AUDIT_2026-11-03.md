# AUDIT: grasp mission reliability, Stretch 3 (2026-11-03)

> Dated working notes, kept as history. Paths such as `~/stretch3-ik/ws` and the baseline tarballs refer to the development machine at the time; `TEST_RESULTS.md` was not published. The current behavior is in the package docs.

Source of truth: the local workspace `~/stretch3-ik/ws` (copy of the robot
workspace taken 2026-11-02 plus the grasping work of that day). Baseline of
this audit: `~/stretch3-ik/baseline_ws_src_2026-11-03.tgz`. Line numbers
below refer to that baseline. No robot, robot log, live topic or bag was
accessed. There are no mission logs or bags in the local tree: the run
scripts write `~/grasping_logs/` on the robot, which was off limits. So the
root causes below come from code reading plus reproduction against the real
executor/IK/gripper-bridge binaries in an isolated simulation. Which
transient fires on the robot itself is not observed (see "What is not
known").

## 1. Where things are

| What | Where |
|---|---|
| Mission entry (real robot) | `grasp_mission.py`: `~/pick` Trigger (L148) -> `start()` (L215) -> thread `run(pick)` |
| Other entry points (not used by `grasping_run.sh`) | `pick_state_machine.py` (sim gripper only, `gripper_enabled`), `autonomous_mission.py` (Gazebo/Nav2) |
| Segmentation | `sam2_segmenter.py`: click -> SAM 2.1 tiny CPU -> mono8 mask |
| 3D reconstruction | `object_localizer.py`: mask + aligned 16UC1 depth + TF at depth stamp -> cloud in `odom` |
| Occupancy | `stretch_collision_planning/octomap_builder` (odom, 5 Hz, clears on odom jumps) |
| Grasp planner | `grasp_planning.py` `plan_grasps()`: side (yaw 0, +-0.2; pitch 0, -0.25) and top-down candidates |
| IK | `ik_preview` (`/ik_preview/grasp_goal` topic in, `/ik_preview/status` out), 5-DoF, grasp task = XYZ + approach (local X) + finger axis (local Y) |
| Motion | `guarded_executor` `move_to_preview` Trigger -> RRT-Connect on the scene -> quintic timing -> driver `FollowJointTrajectory` |
| Gripper | `gripper_bridge.py`: `GripperCommand` -> driver `gripper_aperture` in position mode |
| Final grasp pose | `grasp_planning._candidate()` (center, aperture) -> `grasp_mission.pick()` -> `move(c['grasp'])` |

## 2. Data and control flow (baseline)

```
operator click (rqt_image_view Point)
  -> sam2_segmenter: mask (color stamp)                        [SAM 2, ~1-3 s]
  -> object_localizer: depth frame nearest stamp (<= 0.1 s), deproject,
     TF odom<-camera @stamp, drop points within 10 cm of 3 gripper links,
     voxel/outlier/DBSCAN -> /perception/object_cloud (odom)
  -> grasp_mission.on_object: odom->base_link with the LATEST TF, plan()
     -> "PLANNED: N grasps"
operator: ros2 service call /grasp_mission/pick  -> {success: true, "started"}
  thread pick():
    head check (driver action) -> choose(): plan + solve() per candidate
        solve(): publish PoseStamped, poll /ik_preview/status for an answer
                 whose target_xyz equals the goal (no request id)
        base correction (position mode, translate_mobile_base) <= 3x
    trajectory_mode() (driver switch, odom reset, sleep 1 s)
    move(pregrasp): solve -> move_to_preview Trigger -> poll executor status
    grip(aperture): bridge switches driver to POSITION mode
    target box -> sleep 0.6 -> move(grasp)   <- executor sees position mode:
          answers "switching to trajectory mode; planning follows", switches,
          re-plans 0.5 s later inside its timer (executor_node.cpp L830-852)
    grip(squeeze) -> held >= 1 cm ? -> move(lift) -> move(retreat)
    grip(squeeze) again -> "DONE" | raise -> "FAILED: <text>" on ~/status
```

State as the baseline sees it:

```
IDLE --pick--> started (Trigger returns success here)
  started --any exception--> FAILED (status text only; nothing stopped)
  started --> MOVING:pre-grasp --> MOVING:approach --> MOVING:lift
          --> MOVING:retreat --> DONE
```

When the baseline considers each milestone reached:

| Milestone | Baseline criterion |
|---|---|
| accepted | `~/pick` returns `success=true, "started"` before anything was checked |
| started | thread spawned |
| motion completed | executor status `COMPLETE` + `sequence_active=0`, only if `ARMED/READY/EXECUTING` or `pending_goal=1` was seen first |
| gripper closed | bridge result `SUCCEEDED` (position read after 0.3 s) |
| object verified | fingers stopped >= 1 cm after closing, and again after retreat |

## 3. Root causes: missions that need re-submission

**R1. The request reports success before anything happened, and failures
are only status text.** `start()` returns `success=True, 'started'`
(L215-226) without checking any dependency; `run()` turns every error into
`say('FAILED: ...')` (L228-234). A caller of `~/pick` cannot tell a mission
that failed in 1 s from one that is running. The operator's only signal is
"nothing moved", and the only remedy is to submit again. That is the
mechanism behind "submitted 3-4 times": every transient below becomes a
silent failure plus a re-submission.

**R2. The approach, lift and back-off motions run through a hidden
asynchronous path whose failures the mission ignores.** The gripper bridge
switches the driver to position mode for every gripper goal
(`gripper_bridge.py` L108-110). The next `move()` therefore hits the
executor while it is not in trajectory mode. The executor answers the
Trigger with `success=true, "switching to trajectory mode; planning
follows"` (executor_node.cpp L1431-1452) without arming, switches, and
re-plans in its timer 0.5 s later (L830-852). A failure there only logs
`Planning after mode switch failed` and sets `ABORTED`. `move()` ignores
`ABORTED` unless it saw `ARMED/READY/EXECUTING` first (L344-359), which
never happens on this path. It then waits the full `move_timeout` (60 s)
and reports `approach timed out`, losing the real reason. Reproduced:
TEST_RESULTS.md F1.

**R3. Failures leave the robot mid-sequence.** On any exception, `run()`
neither stops the executor nor cancels driver or gripper goals. After the
60 s timeout of R2 the executor or the driver can still be busy. The next
submission can be refused (`executor unavailable or motion already
active`) or start from an unknown mode. Nothing confirms a stop.

**R4. IK answers are matched by target position only.** `solve()` accepts
any status whose `target_xyz` equals the goal (L309-327). `choose()` solves
the pre-grasp, then `move()` publishes the same pose again (after the mode
switch), and the periodic status from the earlier solve, computed with the
old seed, is accepted at once. In the stale-state branch ik_preview
publishes `success=false` for the new target while keeping the previous
`base_dx` (ik_preview.cpp L290-299). `choose()` would read that as "drive
the base by the old dx".

**R5. The Nav2-arrival head move is not covered by the busy flag.**
`on_nav()` starts `safe_head` in its own thread (L194-204) outside
`running`. A pick submitted while the head turns sends a second goal to the
same driver action, and the driver serves one goal at a time.

**R6. Readiness is discovered mid-mission.** Executor, gripper bridge,
driver action, homed and runstop are only checked when first used, after
"started" was returned.

**R7. Success without proof in two places.** If the fingers do not close at
all (servo fault, gripper stuck), `grip(squeeze)` returns the open aperture,
which passes `held >= min_held` (L540) and the final re-squeeze. The mission
reports `DONE: object held`. Reproduced: F2. Dry run returns
`max(aperture, min_held)` (L296), so a dry run also ends in `DONE: object
held`.

## 4. Root causes: final approach / grasp misses

Transformation chain checked: pixel/mask -> aligned depth (16UC1, mm ->
/1000, verified) -> intrinsics of `aligned_depth_to_color/camera_info`
(same as color, correct) -> optical frame (x right, y down, z forward,
verified) -> TF `odom <- camera` at the depth stamp (direction verified) ->
`base_link` -> grasp frame (X approach, Y fingers, matches
`link_grasp_center` in the URDF: fingertip origins at x = -0.015 m, y =
+-0.021 m closed, +-0.054 m open) -> IK -> FK. Numbers in TEST_RESULTS.md
P1-P4.

**G1. The planned approach is not always what the arm executes.** With the
base fixed, the TCP x coordinate depends only on the wrist yaw (lift moves
z, the telescoping arm moves y). Measured with the real ik_preview over 32
synthetic cans x 7 candidates (TEST_RESULTS.md P3):
- yawed side grasps (yaw +-0.2, `grasp_planning.py` L88) never have a
  fixed-base IK solution (0 of 128 at either tolerance). They cannot be
  straight-line approaches (pre-grasp and grasp differ in x by
  0.10 x sin 0.2 = 2 cm). All they do is cost IK round trips and request
  base corrections that do not converge (the pre-grasp stays off);
- straight side grasps are executed within 0.046 rad of the planned
  approach even at the 0.12 rad tolerance (median 0.028 rad, the IK stops
  at the first iterate inside tolerance). Not a cause of misses on its own;
- top-down grasps: the 0.12 rad tolerance accepted achieved approaches up
  to 0.099 rad off. Pre-grasp and grasp solutions turned by up to 0.045 rad
  between each other (3 of 8 solvable top-down cases violate the new
  checks at 0.12, 2 of 8 at 0.05).
The planner's clearance was checked on the planned frame only, not the
achieved one. A 0.1 rad turn inside the old tolerance sweeps a finger into
a divider 6.5 cm from the can axis
(`test_rejects_unfit_orientations_and_openings`).

**G2. A re-click after a miss localizes the object several cm off.** The
localizer drops every point within 10 cm of `link_grasp_center`,
`link_gripper_s3_body` and `link_wrist_yaw` (`object_localizer.py` L79-87).
After a miss the mission backs off to the pre-grasp, 10 cm from the grasp
center, which puts the near face of a can inside that sphere. Ray-cast
replay: 280 of 1044 can points survive and the centroid moves 40.1 mm away
from the robot (`test_gripper_envelope_filter_keeps_the_object_at_pregrasp`).
The next grasp is then planned about 4 cm too deep. This makes retries after
a miss systematically worse.

**G3. Smaller items:**
- `on_object` converts the cloud with the latest TF, not the capture-stamp
  TF (L186). It is harmless while the base stands still, wrong by the full
  odometry offset if a trajectory-mode switch (odometry reset) falls between
  capture and arrival.
- Visible-surface bias, measured on a ray-cast 66 mm can at 0.75 m: width
  seen 54.7 mm (-11.3 mm; 2 px mask erosion plus grazing edges); grasp
  center 6.0 mm short of the axis along the approach, 3.8 mm vertical,
  0.6 mm lateral. The opening (width + 3 cm) still clears the true diameter
  by 9 mm per side, not the 15 mm intended.
- Gripper aperture is the hello_helpers nominal mapping (`2 x 0.171 x
  finger_rad`; command via robotis counts, 9 cm at 70). Physical aperture
  versus command is uncalibrated. Not verifiable offline.
- D405 wrist camera is in the URDF and unused; no calibration data locally.

## 5. Reproductions (isolated simulation, details in TEST_RESULTS.md)

Filled from the runs: see TEST_RESULTS.md section 4.

## 6. What is not known

- Which transient trips R2 on the robot. Candidates that the executor
  rejects at that moment (preflight in executor_node.cpp L1475-1484):
  diagnostics ERROR/STALE, `base_still` (odometry twist after the mode
  switch), `start_drift` > 1 mm from the IK seed while the arm settles, and
  scene freshness after the OctoMap is cleared on the odometry reset. The
  harness reproduces the mechanism with an injected diagnostics transient.
  A 4 mm / 0.8 s settle after each mode switch alone did not trigger it in
  simulation.
- Real arm settle profile after a mode switch, real OctoMap rebuild time
  after an odometry reset, physical aperture calibration, head camera
  extrinsic error, contact behavior. All need the robot.
