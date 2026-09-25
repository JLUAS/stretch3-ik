# Stretch custom manipulation foundation

## 5-DOF IK: One-click in RViz

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch stretch_custom_manipulation bringup.launch.py sim:=true
```

For the real robot simply change to `sim:=false`. Move and orient the
`target_tcp` marker until a valid IK is obtained and click **Move to goal** once.
The click captures the goal and automatically executes the necessary segments.
**DISARM / STOP** cancels the movement and any pending segments.
Dragging the marker does not move the robot; clicks are rejected during execution.

The axes are lift, full arm extension, and wrist yaw/pitch/roll. The IK
controls XYZ and the local Z direction of the TCP; the rotation around Z remains free.
The base remains fixed. An unreachable goal does not execute the previous ghost.

The simulation is **kinematic**, with trajectories, feedback, and cancellation,
wall time, domain 20, and local DDS; it does not simulate dynamics or contacts.
The real mode uses domain 42 and requires the driver to be running on the robot with
`/stretch/joint_states`, URDF, and homed/runstop states available. The button
requests trajectory mode when needed. It does not perform homing or release runstop.
The driver must have `fail_if_motor_initial_point_is_not_trajectory_first_point=true`.
There is no collision planning: the path must be free of obstacles.

The unified launch enables execution and disables dry-run by default in both
modes; it does not send movement until the button is clicked. For inspection without movement
add `dry_run:=true`; to block execution, `execution_enabled:=false`.
The old commissioning launches keep their conservative values.

The integration test moves the five axes, verifies the measured TCP, and tests
the rejection of double clicks, unreachable goals, and STOP:

```bash
ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=1 colcon test --packages-select stretch_custom_manipulation --event-handlers console_direct+
colcon test-result --verbose
```

The following sections document components and previous tests. Their default
values correspond to the commissioning launches, not to the bringup above.

ROS 2 Humble, C++17, colcon. Core packages `stretch_robot_interface` and
`stretch_custom_kinematics` remain independent of hardware execution. The third
package, `stretch_guarded_executor`, adds explicitly enabled, armed, bounded
lift-only execution and a transport-free dry run. No MoveIt, planning or mapping.

## Guarded executor

[Implementation, test evidence, live blockers and manual 1 cm lift procedure](docs/GUARDED_EXECUTOR.md).
The default launch is DISARMED, dry run, execution disabled. Real tests require trajectory mode. The lift-only profile downgrades the exact
allowlisted cliff ERROR to a logged WARN because base commands are prohibited;
all other ERRORs and STALE conditions remain blocking. No physical motion was performed.

## Offline IK and RViz preview

[IK task, limits, preview launch and failure semantics](docs/IK_AND_PREVIEW.md)
provide the second milestone. [Measured results](docs/IK_VALIDATION_RESULTS.md)
include 1,000 random FK→IK→FK cases and isolated preview validation on domains
20 and 42. The task controls XYZ plus the TCP local-Z direction; spin about Z
is intentionally unconstrained.

After building and sourcing the workspace:

```bash
source scripts/use_sim.sh
ros2 launch stretch_custom_kinematics ik_preview.launch.py offline:=true
```

For read-only real-state seeding, use a separate terminal:

```bash
source scripts/use_real.sh
ros2 launch stretch_custom_kinematics ik_preview.launch.py
```

Preview joint state and TF remain under `/ik_preview`; the real driver is only
read. The offline seed fixture refuses to run outside domain 20.

## Build and test

Use a fresh terminal to avoid unrelated workspace overlays:

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
source scripts/use_sim.sh
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Dependencies are declared in the package manifests. On a new Humble machine,
resolve them with `rosdep install --from-paths src --ignore-src -r -y` before building.
Unit tests use captured URDF fixtures; the runtime always requires live
`/robot_description`. Mesh packages are unnecessary for FK.

## Real robot: domain 42, read only

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
source install/setup.bash
source scripts/use_real.sh
ros2 run stretch_robot_interface interface_validator
```

In another terminal with the same setup:

```bash
ros2 run stretch_custom_kinematics fk_validator
```

This consumes canonical `/stretch/joint_states` and compares FK against interpolated
live TF at the driver timestamp. Republisher timing and encoder quantization can
exceed the default 0.001 rad diagnostic threshold; this is a cross-stream check,
not a pure test of the FK equations. For the strict numerical FK test, use the
same input and timestamp as the robot's TF publisher:

```bash
ros2 run stretch_custom_kinematics fk_validator --ros-args \
  -p joint_state_topic:=/joint_states \
  -p require_exact_tf_sample:=true \
  -p position_tolerance_m:=0.00000001 \
  -p orientation_tolerance_rad:=0.00000001
```

The override is a validation experiment and does not change the canonical robot
state contract. Optional `-p report_csv:=/absolute/path/results.csv` records every
successful comparison. Ctrl-C stops either validator. They never start drivers.

## Simulation/local: domain 20

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
source install/setup.bash
source scripts/use_sim.sh
ros2 run stretch_robot_interface interface_validator --ros-args -p use_sim_time:=true
ros2 run stretch_custom_kinematics fk_validator --ros-args -p use_sim_time:=true
```

Run each continuously running command in its own configured terminal. A simulator
must already publish the interface contract and `/clock`. For a wall-clock local
fixture omit `use_sim_time`. If the simulator publishes measured state only at
`/joint_states`, select `-p joint_state_topic:=/joint_states` on both nodes. This
workspace does not launch a simulator or invent homed/runstop flags.

## API and parameters

`stretch_robot_interface::RobotModel::decode(JointState)` returns named logical
state. `InterfaceNode::state()` adds freshness and readable homed/runstop flags;
`ready()` also requires valid URDF, recent `/tf` and received `/tf_static`.
Use a single-threaded executor with `InterfaceNode`.

`stretch_custom_kinematics::ForwardKinematics(model).computeTcpPose(state)` returns
`geometry_msgs::msg::Pose` in `base_frame`; `computeTcpTransform` returns an Eigen
isometry. The input must have `valid=true`; FK enforces finite values and model
limits. Timestamp/freshness policy belongs to the interface, not pure FK.

| Parameter | Default | Purpose |
|---|---|---|
| `joint_state_topic` | `/stretch/joint_states` | Measured input, configurable for adapters/validation |
| `base_frame` | `base_link` | URDF ancestor frame |
| `tcp_frame` | `link_grasp_center` | URDF-defined fixed grasp center |
| `state_timeout` | `1.0` seconds | Receipt/header/status freshness |
| `use_sim_time` | `false` | ROS clock selection |
| `position_tolerance_m` | `0.001` | FK report threshold |
| `orientation_tolerance_rad` | `0.001` | FK report threshold |
| `require_exact_tf_sample` | `false` | Strict test: require every moving chain transform at the state timestamp |
| `report_csv` | empty | Optional comparison output file |

The last four parameters belong only to `fk_validator`. It buffers state for
0.2 seconds for TF delivery and drops comparisons older than 1 second. Missing
TF produces an explicit warning and no numerical result. Streaming diagnostic
nodes do not return a failing process exit code on a threshold failure; inspect
the report/CSV. Startup WAIT is expected during DDS discovery.

## Evidence and documentation

- [ROS interface contract](docs/ROS_INTERFACE_CONTRACT.md)
- [Kinematic model and numerical validation](docs/KINEMATIC_MODEL.md)
- [Simulation/real architecture](docs/SIM_TO_REAL_ARCHITECTURE.md)
- [Raw interface capture](docs/evidence/interface_inspection.json)

To repeat the read-only topic inspection:

```bash
python3 scripts/inspect_interface.py --output /tmp/stretch-interface-capture --seconds 20
```

No credentials are required by, or stored in, this workspace.
# stretch3-ik
