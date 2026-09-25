# 1-Click IK Goal

The driver must be started on the robot, in ROS domain 42, publishing
`/stretch/joint_states`, `/robot_description`, `/is_homed`, `/is_runstopped`,
`/mode`, `/is_streaming_position` and `/diagnostics`. Full homing is required,
runstop released, and the driver parameter
`fail_if_motor_initial_point_is_not_trajectory_first_point=true`.

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
source install/setup.bash
ros2 launch stretch_custom_manipulation bringup.launch.py sim:=false
```

Move the TCP marker in RViz to a valid solution and click **Move to goal**.
The executor requests trajectory mode when needed, captures the solution and
executes coordinated segments of lift, arm and the three wrist axes.
It checks the measured state in each segment and disarms upon completion.
**DISARM / STOP** cancels the movement and all remaining segments.
The panel shows the rejection reasons and the execution status.

For local kinematic simulation use the same command with `sim:=true`.
The simulation starts its own controller in domain 20 with local DDS and uses
wall time. It does not simulate dynamics, contacts or obstacles.

The unified launch allows execution in both modes but does not start movement
by itself. For inspection without sending trajectories add `dry_run:=true`;
to prevent execution, use `execution_enabled:=false`.

The IK resolves XYZ plus the local Z direction of the TCP, with free rotation around Z.
It does not move the base, gripper or head. It does not include collision planning:
the joint path must be free of obstacles.

The limits of each segment are 1 cm for lift/arm and 0.05 rad for the wrist
(slightly smaller in simulation), with a duration of five seconds.
Segments are automatically repeated until the goal is reached, preserving limits
of velocity/acceleration and tracking checks on all five axes.
Loss of valid states, feedback or diagnostics aborts the sequence.

The automated test checks the five axes, final TCP, double click, unreachable
goal and STOP. It does not replace physical validation with the connected robot.
