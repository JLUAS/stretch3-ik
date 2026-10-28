# Overview

*stretch_robot_interface* provides a read-only view of the Stretch 3 robot state. It parses the live URDF, decodes `/stretch/joint_states` into named logical joints (lift, arm extension, wrist yaw/pitch/roll) and tracks freshness and homed/runstop flags. It never commands hardware.

## interface_validator

Runs the interface node against the live graph and reports whether the inputs of the [ROS interface contract](../../docs/ROS_INTERFACE_CONTRACT.md) are present and fresh. Startup WAIT is expected during DDS discovery.

```bash
source scripts/use_real.sh
ros2 run stretch_robot_interface interface_validator
```

## Library

`RobotModel::decode(JointState)` returns named logical state. `InterfaceNode::state()` adds freshness and homed/runstop flags; `ready()` also requires a valid URDF, recent `/tf` and received `/tf_static`. Use a single-threaded executor with `InterfaceNode`. See the [top-level README](../../README.md#api-and-parameters) for parameters.
