# Real Robot Validation Checklist

This checklist contains the hardware-only verification items that must be executed on the physical Stretch robot during commissioning.

Keep the runstop button within reach for every step. The workspace around the arm must be free of people and obstacles: there is no collision planning.

## Pre-Flight Verification

- [ ] Ensure `stretch_driver` is running and publishing `/stretch/joint_states`, `/tf`, `/robot_description`, `/is_homed`, `/is_runstopped`, `/mode`, `/is_streaming_position` and `/diagnostics` on domain 42.
- [ ] Ensure the robot is homed, runstop is released, and the driver parameter `fail_if_motor_initial_point_is_not_trajectory_first_point` is `true`.
- [ ] Ensure the hardware limits in `src/stretch_custom_kinematics/config/hardware_limits.csv` accurately reflect this specific robot's calibration.
- [ ] Confirm the Pimu cliff range calibration is represented by `cliff_range_min: -100.0` and `cliff_range_max: 200.0` in `src/stretch_custom_manipulation/config/real.yaml`.
- [ ] Confirm the Pimu temperature diagnostics are active and the robot is not currently overheating (execution blocks above 50.0 C).

## 1. Dry-Run Commissioning (Safe Startup)

Launch with dry-run enabled and command validation available:
```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=true execution_enabled:=true
```

- [ ] **RViz Load**: Verify the robot model correctly matches the physical pose of the real robot.
- [ ] **Ghost Robot**: Verify the ghost (preview) robot appears at the measured state.
- [ ] **IK Target Manipulation**: Drag the `target_tcp` marker around the workspace. Verify the ghost robot follows it and the IK Preview Status panel reports `success` for reachable targets (XYZ plus TCP local-Z direction; spin about Z is free).
- [ ] **Dry-Run Validation**: Click **Move to goal** (panel button or marker right-click menu). Verify the robot does not move, the executor reports a validated dry-run plan, and a report is written to `/tmp/stretch_guarded_executor_report.json`.

## 2. 5-DoF Trajectory Execution (Live Hardware)

Stop the previous launch completely and restart with physical execution enabled:
```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=false execution_enabled:=true
```

- [ ] **Startup State**: Verify the executor status reports `DISARMED` and nothing moves until a click.
- [ ] **Lift-Only Move**: Move the marker purely vertically by a few centimeters and click **Move to goal** once. Verify a smooth lift motion in bounded segments and that the executor disarms when the goal is reached.
- [ ] **Arm Extension Mapping**: Move the marker outward and click **Move to goal**. Verify all 4 telescope stages extend uniformly and tracking errors remain within the `tracking_error` envelopes.
- [ ] **Wrist Yaw/Pitch/Roll**: Rotate the marker using its ring controls and click **Move to goal**. Verify the physical wrist matches the commanded TCP direction.
- [ ] **Double Click**: While a move is executing, click **Move to goal** again. Verify the second request is rejected.
- [ ] **Unreachable Goal**: Drag the marker out of reach. Verify the panel reports a failure and **Move to goal** is rejected.
- [ ] **Tracking Error Interrupt**: While a trajectory is executing, lightly restrict one of the joints manually to induce tracking error. Verify the executor immediately aborts, disarms, and stops the trajectory.
- [ ] **Manual Stop**: Start a multi-segment move. Immediately click **DISARM / STOP**. Verify the trajectory and all remaining segments are canceled.

## 3. Diagnostic Protections

- [ ] **Cliff Detection**: `real.yaml` downgrades the `stretch_driver: Pimu/Cliff` ERROR to WARN only while the reported cliff range is within `[cliff_range_min, cliff_range_max]`, because base motion is prohibited. Verify the downgrade is logged as a WARN, and that a cliff ERROR outside that range blocks **Move to goal**.
- [ ] **Bounded Temperature**: If the driver issues a Pimu temperature ERROR, the diagnostic policy must downgrade it to a WARN at or below `50.0 C`. Above `50.0 C`, verify execution remains blocked.
- [ ] **Other Errors**: Verify any other diagnostic ERROR or STALE status blocks **Move to goal**.
