# Real Robot Validation Checklist

This checklist contains the hardware-only verification items that must be executed on the physical Stretch robot during commissioning.

## Pre-Flight Verification

- [ ] Ensure `stretch_driver` is running and publishing `/stretch/joint_states`, `/tf`, `/robot_description`, and `/diagnostics` on Domain 42.
- [ ] Ensure the hardware limits in `src/stretch_custom_kinematics/config/hardware_limits.csv` accurately reflect this specific robot's calibration.
- [ ] Confirm the Pimu cliff range calibration is represented by `cliff_range_min: -100.0` and `cliff_range_max: 200.0` in `src/stretch_custom_manipulation/config/real.yaml`.
- [ ] Confirm the Pimu temperature diagnostics are active and the robot is not currently overheating (execution blocks above 50.0 C).

## 1. Dry-Run Commissioning (Safe Startup)

Launch with dry-run enabled and command validation available:
```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=true execution_enabled:=true
```

- [ ] **RViz Load**: Verify the white robot model correctly matches the physical pose of the real robot.
- [ ] **Ghost Robot**: Verify the blue ghost robot appears at the initial state.
- [ ] **IK Target Manipulation**: Move the interactive marker around the workspace. Verify the ghost robot smoothly follows the 6-DoF target.
- [ ] **Validation Service**: Right-click the marker and select "Validate Preview". Verify that the node output reports successful validation and generates a dry-run report in `/tmp/stretch_guarded_executor_report.json`.

## 2. 5-DoF Trajectory Execution (Live Hardware)

Kill the previous launch and restart with execution enabled:
```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=false execution_enabled:=true dry_run:=false
```

- [ ] **Arm Executor**: Right-click the marker and select "ARM Executor". Ensure the status string indicates `ARMED`.
- [ ] **Lift-Only Test**: Move the marker purely vertically. Right-click, "Validate Preview", then "Execute Preview". Verify the robot executes a smooth quintic lift.
- [ ] **Arm Extension Mapping**: The logical `arm_extension` must map correctly to the 4 physical telescope stages. Move the marker outward. Validate and Execute. Verify all 4 telescope stages extend uniformly and tracking errors remain within `tracking_error` envelopes.
- [ ] **Wrist Yaw/Pitch/Roll**: Command a wrist rotation using the ring controls on the marker. Validate and Execute. Verify the physical wrist matches the commanded rotation.
- [ ] **Tracking Error Interrupt**: While a trajectory is executing, lightly restrict one of the joints manually to induce tracking error. Verify the executor immediately aborts, disarms, and stops the trajectory.
- [ ] **Manual Stop**: Initiate a long trajectory. Immediately right-click and select "DISARM / STOP". Verify the trajectory cancels abruptly.

## 3. Diagnostic Protections

- [ ] **Cliff Detection**: Pick up the robot or trigger a cliff diagnostic. Verify that while the cliff error is active, the executor refuses to `ARM`.
- [ ] **Bounded Temperature**: If the driver issues a Pimu temperature ERROR, the diagnostic policy must downgrade it to a WARN at or below `50.0 C`. Above `50.0 C`, verify execution remains blocked.
