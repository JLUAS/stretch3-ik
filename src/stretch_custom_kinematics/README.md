# Overview

*stretch_custom_kinematics* provides URDF-derived forward kinematics, a 5-DoF inverse kinematics solver and an isolated RViz IK preview for Stretch 3. The IK controls TCP position (XYZ) and the TCP local-Z direction over lift, arm extension and wrist yaw/pitch/roll; rotation about Z is free. Nothing in this package commands hardware.

## Nodes

| Executable | Description |
|---|---|
| `ik_preview` | Interactive `target_tcp` marker and ghost robot. Publishes preview joint state, TF and status under `/ik_preview` only. |
| `fk_validator` | Compares FK against live TF and reports position/orientation error. |
| `ik_benchmark` | Offline FK→IK→FK benchmark over random reachable samples. |
| `offline_seed.py` | Domain-20-only mock robot state and trajectory controller for simulation and tests. |

The `IkStatusPanel` RViz plugin shows the IK result and provides the **Move to goal** and **DISARM / STOP** buttons, which call the guarded executor.

## Launch

```bash
source scripts/use_sim.sh
ros2 launch stretch_custom_kinematics ik_preview.launch.py offline:=true
```

Arguments: `offline`, `rviz`, `use_sim_time`, `joint_state_topic`, `mesh_package_path`. `offline:=true` is allowed only on `ROS_DOMAIN_ID=20`.

See [IK and preview](../../docs/IK_AND_PREVIEW.md), [measured results](../../docs/IK_VALIDATION_RESULTS.md) and the [kinematic model](../../docs/KINEMATIC_MODEL.md).
