# Gripper Contract

The custom manipulation stack explicitly excludes the gripper from the 5-DoF
kinematic chain (`lift`, `arm_extension`, `wrist_yaw`, `wrist_pitch`,
`wrist_roll`).

## Why the Gripper is Excluded

The current Stretch URDF and driver model the gripper command values in
arbitrary units (e.g., `[-100, 200]`), which do not map linearly to radians or
meters like the other joints. Including the gripper in the uniform quintic
trajectory generation and tracking error bounds would result in safety
violations and unexpected behavior.

## Gripper Interface Architecture

The gripper operates through a standard `control_msgs/action/GripperCommand`
action interface.  This is the same interface provided by the Stretch driver on
the real robot.

### Simulation

In `sim:=true` mode, the `sim_gripper` node provides:

- `GripperCommand` action server at `/stretch_gripper_controller/gripper_action`
- Simulated gripper joint state on `/gripper/joint_states`
- Gripper state on `/gripper/state` (`GripperState` message)
- Convenience services: `/gripper/open_full`, `/gripper/close_full`
- Simulated object detection based on effort/position thresholds

The simulated gripper is restricted to domain 20 with localhost-only DDS,
matching the existing simulation isolation boundary.

### Real Robot

In `sim:=false` mode, no `sim_gripper` node is started. The Stretch driver
provides the `GripperCommand` action server at the same topic. The pick/place
state machines use identical code paths in both modes.

### Aperture Mapping

The logical aperture is in meters: `[0.0, 0.13]` where 0 is fully closed and
0.13 is fully open. The mapping to Stretch raw values `[-100, 200]`:

```text
raw = raw_min + (aperture / max_aperture) * (raw_max - raw_min)
aperture = max_aperture * (raw - raw_min) / (raw_max - raw_min)
```

### Custom Messages

The `GraspTarget` message encapsulates everything needed for a pick operation:

| Field | Type | Description |
|---|---|---|
| `pregrasp_pose` | `PoseStamped` | Collision-free approach position |
| `grasp_pose` | `PoseStamped` | TCP target when closing gripper |
| `approach_vector` | `Vector3` | Direction from pregrasp to grasp |
| `gripper_opening` | `float64` | Required aperture in meters |
| `grasp_mode` | `uint8` | PINCH=0, WRAP=1, PUSH=2 |
| `confidence` | `float64` | Grasp confidence [0, 1] |
| `object_id` | `string` | Object identifier |

The `GripperState` message provides:

| Field | Type | Description |
|---|---|---|
| `aperture` | `float64` | Current opening in meters |
| `effort` | `float64` | Normalized effort (negative = squeezing) |
| `object_detected` | `bool` | Whether contact is detected |
| `is_grasping` | `bool` | Closed on object |
| `raw_position` | `float64` | Raw driver value |

### Integration with Pick/Place

The pick state machine uses the gripper action client independently from the
5-DoF arm executor. The sequence is:

1. Open gripper to safe aperture (gripper action)
2. Plan and execute pregrasp pose (IK + guarded executor)
3. Plan and execute grasp approach (IK + guarded executor)
4. Close gripper to target aperture (gripper action)
5. Verify grasp via effort/position feedback
6. Retract (IK + guarded executor)

Do not attempt to add `joint_gripper_finger_left` to the guarded executor's
tracking array. The gripper geometry should be considered for collision checking
when the aperture changes significantly during a pick/place operation.
