# 1-Click IK Goal

The unified bringup starts the official driver, D435i, RPLidar, live SLAM,
Nav2, manipulation nodes and RViz on the robot in ROS domain 42. The driver publishes
`/stretch/joint_states`, `/robot_description`, `/is_homed`, `/is_runstopped`,
`/mode`, `/is_streaming_position` and `/diagnostics`. Full homing is required,
runstop released, and the driver parameter
`fail_if_motor_initial_point_is_not_trajectory_first_point=true`.

On the real robot the wrapper defaults to `dry_run:=true` and
`collision_environment:=octomap`. The first startup must remain motion-free:

```bash
cd ~/ament_ws
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false
```

The wrapper sources `${STRETCH_ROS_WS:-$HOME/ament_ws}` before the custom
workspace. Use `start_robot_driver:=false` or `start_robot_camera:=false` only
when that component is already running elsewhere; duplicate hardware drivers
must not be launched.

The default is `use_slam:=true`, so no saved map is required. RViz opens with
the Nav2 panel, initial-pose and goal tools, costmaps, footprint, LiDAR, RGB-D,
OctoMap and manipulation markers. Use `rviz:=false` only for a headless SSH
session. For a saved map, pass
`use_slam:=false map:=/absolute/path/to/map.yaml`.

```bash
cd ~/ament_ws
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=false execution_enabled:=true
```

Move the TCP marker in RViz to a valid solution and click **Move to goal**.
The executor requests trajectory mode when needed, captures the solution and
executes coordinated segments of lift, arm and the three wrist axes.
It checks the measured state in each segment and disarms upon completion.
**DISARM / STOP** cancels the movement and all remaining segments.
The panel shows the rejection reasons and the execution status.

For the local kinematic simulation use:

```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=true scenario:=fixture
```

`scenario:=fixture` starts its own controller in domain 20 with local DDS and
a primitive desk as the collision scene. It does not simulate dynamics or
contacts. Without `scenario`, `sim:=true` selects `small_house`, a Gazebo
world that needs a separate simulation workspace
(`STRETCH_SMALL_HOUSE_WS`).

The wrapper selects the correct ROS domain, resets the ROS daemon, sources the
workspace and starts the unified bringup. For real-robot inspection without
sending trajectories, use `sim:=false dry_run:=true execution_enabled:=true`.
Setting `execution_enabled:=false` blocks validation/execution rather than
simulating a move.

The IK resolves XYZ plus the local Z direction of the TCP, with free rotation
around Z. It does not move the base, gripper or head. The executor plans the
joint path with RRT-Connect against the collision scene (primitive desk in
simulation, live OctoMap on the robot) and times it as minimum-time quintics
within the `max_velocity`, `max_acceleration` and `max_delta` limits of
`config/real.yaml`. Loss of valid states, feedback or diagnostics aborts the
motion.

The automated test checks the five axes, final TCP, double click, unreachable
goal and STOP. It does not replace physical validation with the connected robot.
