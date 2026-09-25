# Stretch Custom Manipulation Architecture

This meta-package unifies the manipulation stack for the Stretch robot, providing a robust, bounded 5-DoF guarded execution pipeline with offline simulation capabilities.

## Package Structure

The workspace consists of 5 packages:

1. **stretch_custom_manipulation**: The meta bringup package. Contains unified launch files (`sim:=true/false`), configuration for execution limits, unified RViz configuration, and documentation. No C++ code.
2. **stretch_robot_interface**: A robust, zero-dependency abstract logical representation of the Stretch physical state. Normalizes measurements, filters noise, checks freshness and establishes homing/runstop flags.
3. **stretch_custom_kinematics**: Provides Inverse Kinematics and an interactive RViz ghost robot (preview) that accepts TCP requests and solves the 5-DoF constraints (`x, y, z` plus target TCP Z-axis direction).
4. **stretch_guarded_executor**: The core safety state machine. Intercepts the preview target, validates it against per-joint acceleration and velocity bounds, tracks it against execution envelopes, and generates the multi-joint quintic splines for the Stretch driver.
5. **stretch_lift_rviz_control**: Legacy UI panel specifically for absolute, explicit lift actuation testing.

## Execution Data Flow

1. **Measurement**: `/stretch/joint_states` and `/robot_description` are ingested by `ik_preview` and `guarded_executor`.
2. **Preview**: The RViz operator moves the Interactive Marker (`/ik_preview/target`). The IK solver outputs the closest valid joint solution and publishes a ghost robot.
3. **Execution Controls**: The panel's `Move to goal` button calls
   `/guarded_executor/move_to_preview`. The executor captures the valid IK target,
   prepares trajectory mode when needed, and automatically validates and executes
   bounded five-axis segments from fresh measured state. Duplicate requests are
   rejected. `DISARM / STOP` cancels motion and all remaining segments.
4. **Hardware Validation**: The executor dynamically validates incoming diagnostics (e.g. downgrading the Pimu/Temperature error when the measured temperature is at or below the configured 50.0 C ceiling) and fail-closes on any unknown errors.
5. **Spline Output**: The `Execute` command generates a `/stretch_controller/follow_joint_trajectory` command mapping the logical `arm_extension` into the 4 uniform telescope stages (`joint_arm_l3`..`l0`).

## Safety Principles

- **Fail-Closed**: Any absent prerequisite, stale state, unapproved diagnostic, or envelope violation abruptly disarms the executor.
- **Explicit click to move**: Unified bringup enables execution in both modes;
  startup remains disarmed. `dry_run:=true` is an optional inspection mode.
  `sim:=true` provides local kinematic trajectory execution on domain 20;
  `sim:=false` uses the existing driver on domain 42. Simulation bypasses cannot
  be used outside domain 20 with localhost-only DDS.
- **Physical Constraints**: Quintic trajectories respect maximum velocity and acceleration limits on a per-joint basis.
- **Immutable Approval**: Each segment is validated against its measured seed;
  drift invalidates that approval. The full target is fixed at the initial click.
  All five measured joint endpoints must satisfy tolerance before proceeding.
- **Scope**: XYZ plus TCP local-Z direction, fixed base, no collision planning.
  Simulation validates ROS transport and kinematics, not physical dynamics.
