# stretch3-ik

### Overview

This repository holds ROS 2 Humble packages for guarded 5-DoF inverse kinematics
on the Stretch 3 mobile manipulator from Hello Robot Inc. Drag a TCP marker in
RViz, check the IK ghost, and click **Move to goal** once: the robot moves its
lift, arm and three wrist axes to the target in small, validated segments.

The IK controls TCP position (XYZ) and the TCP local-Z direction; rotation about
Z is left free. The base, head and gripper never move. There is no MoveIt, motion
planning, mapping or collision checking: the path must be free of obstacles.

> **Note:** This is an independent project. It is not developed, endorsed or
> supported by Hello Robot Inc. Use it on real hardware at your own risk, with
> the runstop button within reach.

### Packages

| Resource | Description |
|---|---|
| [stretch\_robot\_interface](src/stretch_robot_interface/) | Read-only robot state: URDF model, joint decoding, freshness and homed/runstop flags |
| [stretch\_custom\_kinematics](src/stretch_custom_kinematics/) | URDF-derived FK, 5-DoF IK, isolated RViz IK preview and status panel |
| [stretch\_guarded\_executor](src/stretch_guarded_executor/) | Explicitly armed, bounded trajectory execution with dry run and diagnostic policy |
| [stretch\_lift\_rviz\_control](src/stretch_lift_rviz_control/) | Legacy lift-only RViz marker and panel used for commissioning |
| [stretch\_custom\_manipulation](src/stretch_custom_manipulation/) | Unified bringup (`sim:=true/false`), configuration, RViz layout and launch test |

`stretch_robot_interface` and `stretch_custom_kinematics` never command hardware.
Only `stretch_guarded_executor` can send trajectories, and only after an explicit
request.

### Requirements

- Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- For the real robot: a Stretch 3 running `stretch_driver`, reachable on ROS domain 42
- Optional: `pip3 install hello-robot-stretch-urdf` to show robot meshes in RViz (FK and IK do not need them)

### Installation

The repository is itself a colcon workspace:

```bash
git clone https://github.com/JLUAS/stretch3-ik.git ~/stretch3-ik
cd ~/stretch3-ik
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Use a fresh terminal to avoid unrelated workspace overlays.

### One-click 5-DoF IK in RViz

#### Simulation

```bash
cd ~/stretch3-ik
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=true
```

The wrapper selects domain 20 for simulation or domain 42 for the real robot,
stops a stale ROS daemon, sources ROS and the workspace, and then starts the
unified bringup. Do not invoke `bringup.launch.py` directly for the normal
workflow.

The simulation is **kinematic**, with trajectories, feedback, and cancellation,
wall time, domain 20, and local DDS; it does not simulate dynamics or contacts.

#### Real robot

The real mode uses domain 42 and requires the driver to be running on the robot
with `/stretch/joint_states`, URDF, and homed/runstop states available. The robot
must be homed with runstop released; this workspace does not perform homing or
release runstop. The driver must have
`fail_if_motor_initial_point_is_not_trajectory_first_point=true`.

For real-robot inspection without physical movement, use:

```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=true execution_enabled:=true
```

After inspection, stop that process completely and explicitly enable physical
execution with:

```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=false execution_enabled:=true
```

See the [real robot quickstart](src/stretch_custom_manipulation/docs/REAL_ROBOT_QUICKSTART.md)
and the [commissioning checklist](src/stretch_custom_manipulation/docs/REAL_ROBOT_TEST_CHECKLIST.md)
before the first physical run.

#### Usage

Move and orient the `target_tcp` marker until a valid IK is obtained and click
**Move to goal** once. The click captures the goal and automatically executes
the necessary segments, requesting trajectory mode when needed.
**DISARM / STOP** cancels the movement and any pending segments.
Dragging the marker does not move the robot; clicks are rejected during
execution. An unreachable goal does not execute the previous ghost.

The wrapper forwards additional launch arguments unchanged. The bringup enables
execution and disables dry-run by default, but it does not send movement until
the button is clicked. Use the explicit real-robot commands above so the active
mode is always visible in terminal history. Setting `execution_enabled:=false`
blocks validation/execution rather than simulating a move.
The older commissioning launches described below keep their conservative
defaults (DISARMED, dry run, execution disabled).

### Testing

Run all unit, transport and launch tests on the isolated simulation domain:

```bash
cd ~/stretch3-ik
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/use_sim.sh
ROS_LOCALHOST_ONLY=1 colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The integration test in `stretch_custom_manipulation` moves the five axes,
verifies the measured TCP, and tests the rejection of double clicks, unreachable
goals, and STOP. To run only that test:

```bash
ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=1 colcon test --packages-select stretch_custom_manipulation --event-handlers console_direct+
colcon test-result --verbose
```

Unit tests use captured URDF fixtures; the runtime always requires live
`/robot_description`. Mesh packages are unnecessary for FK.

### Components and earlier milestones

The following sections document components and previous tests. Their default
values correspond to the commissioning launches, not to the bringup above.

#### Guarded executor

[Implementation, test evidence, live blockers and manual 1 cm lift procedure](docs/GUARDED_EXECUTOR.md).
The default launch is DISARMED, dry run, execution disabled. Real tests require
trajectory mode. The lift-only profile downgrades the exact allowlisted cliff
ERROR to a logged WARN because base commands are prohibited; all other ERRORs
and STALE conditions remain blocking.

#### Offline IK and RViz preview

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

#### Read-only validators: real robot, domain 42

```bash
cd ~/stretch3-ik
source /opt/ros/humble/setup.bash
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

#### Read-only validators: simulation/local, domain 20

```bash
cd ~/stretch3-ik
source /opt/ros/humble/setup.bash
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

### API and parameters

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

### Documentation

- [Architecture](src/stretch_custom_manipulation/docs/ARCHITECTURE.md)
- [Gripper contract](src/stretch_custom_manipulation/docs/GRIPPER_CONTRACT.md)
- [ROS interface contract](docs/ROS_INTERFACE_CONTRACT.md)
- [Kinematic model and numerical validation](docs/KINEMATIC_MODEL.md)
- [Simulation/real architecture](docs/SIM_TO_REAL_ARCHITECTURE.md)
- [Raw interface capture](docs/evidence/interface_inspection.json)

To repeat the read-only topic inspection:

```bash
python3 scripts/inspect_interface.py --output /tmp/stretch-interface-capture --seconds 20
```

No credentials are required by, or stored in, this workspace.

### Licenses

For license details for this repository, see [LICENSE](LICENSE) and
[THIRD\_PARTY\_LICENSES.md](THIRD_PARTY_LICENSES.md). A summary of the licenses follows:

| Resource | License |
|---|---|
| All packages, scripts and documentation | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0) |
| Captured Stretch URDF files (`*.urdf`) from Hello Robot's `stretch_description` | [BSD 3-Clause Clear License](https://choosealicense.com/licenses/bsd-3-clause-clear/) |
