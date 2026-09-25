# ROS interface contract

Observed locally on domain 42 on 2026-09-23 (Monterrey), 2026-09-24 UTC.
Evidence is captured from the active robot, not inferred from another workspace.
Inspection did not start/restart any driver or camera and issued no motion commands.

## Topic publishers, types and QoS

| Topic | Exact message type | Publisher | Reliability / durability | Observed Hz or behavior |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | /joint_state_publisher | reliable / volatile | 30.06 |
| `/stretch/joint_states` | `sensor_msgs/msg/JointState` | /stretch_driver | reliable / volatile | 30.05 |
| `/robot_description` | `std_msgs/msg/String` | /robot_state_publisher | reliable / transient local | latched, one model |
| `/tf` | `tf2_msgs/msg/TFMessage` | /robot_state_publisher | reliable / volatile | 29.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | /robot_state_publisher, /camera | reliable / transient local | latched per publisher |
| `/joint_limits` | `sensor_msgs/msg/JointState` | /stretch_driver | reliable / volatile | on getter request |
| `/is_homed` | `std_msgs/msg/Bool` | /stretch_driver | reliable / volatile | 30.06 |
| `/is_runstopped` | `std_msgs/msg/Bool` | /stretch_driver | reliable / volatile | 30.10 |
| `/mode` | `std_msgs/msg/String` | /stretch_driver | reliable / volatile | 30.10 |
| `/tool` | `std_msgs/msg/String` | /stretch_driver | reliable / volatile | 30.10 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | /stretch_driver | reliable / volatile | 1.00 |

Rates are arrival estimates over 15 seconds on Wi-Fi, not guaranteed controller
rates. Both latched `/tf_static` messages were received during discovery; their
arrival spacing is not a meaningful update frequency. All observed publisher
endpoints report automatic liveliness and infinite deadline, lifespan and
liveliness lease. The DDS graph reports history UNKNOWN and depth 0; these are
**unavailable endpoint metadata**, not proof of an actual zero-depth history.
No exact publisher queue depth is claimed. Full returned QoS is in
[evidence/interface_inspection.json](evidence/interface_inspection.json).

Subscribers use best-effort volatile sensor QoS for live state/status/TF, which
matches the observed reliable publishers. The model and static-TF subscriptions
request reliable transient-local durability to receive retained data.

## Canonical source and republisher distinction

**Use `/stretch/joint_states` for measured robot state.** Its publisher is
`/stretch_driver`, and it includes the logical alias `wrist_extension` as well as
the four physical arm stages. It has no wheel joint positions in the captured
message.

`/joint_states` is published by `/joint_state_publisher`. Its live parameter dump
sets `source_list: [/stretch/joint_states]`, `rate: 30.0`,
`publish_default_positions: true`, `use_mimic_tags: true`,
`use_smallest_joint_limits: true`, and `use_sim_time: false`.
It adds default `joint_right_wheel` and `joint_left_wheel`, drops the non-URDF
`wrist_extension` alias, reorders joints and assigns publication timestamps.
It can keep publishing cached/default state if the measured input disappears,
so its timestamp alone is not a trustworthy measure of hardware freshness.
`/robot_state_publisher` subscribes to this processed `/joint_states` stream.
The two streams must not be treated as interchangeable.

[Parameter dump](evidence/joint_state_publisher.yaml) and both complete example
JointState messages are retained in the interface capture.

## Manipulation joint mapping

| Logical coordinate | Physical URDF/JointState name | Unit | URDF lower, upper |
|---|---|---|---|
| lift | `joint_lift` | m | 0.0, 1.1 |
| arm_extension / 4 | `joint_arm_l3` | m | 0.0, 0.13 |
| arm_extension / 4 | `joint_arm_l2` | m | 0.0, 0.13 |
| arm_extension / 4 | `joint_arm_l1` | m | 0.0, 0.13 |
| arm_extension / 4 | `joint_arm_l0` | m | 0.0, 0.13 |
| wrist_yaw | `joint_wrist_yaw` | rad | -1.75, 4.0 |
| wrist_pitch | `joint_wrist_pitch` | rad | -1.57, 0.56 |
| wrist_roll | `joint_wrist_roll` | rad | -3.14, 3.14 |
| outside five-DOF state | `joint_gripper_finger_right` | rad | 0.0, 0.0 |
| outside five-DOF state | `joint_gripper_finger_left` | rad | 0.0, 0.0 |

`joint_arm_l4` is fixed; it is not an additional extension coordinate. Physical
stage order from proximal to distal is l3 → l2 → l1 → l0. No telescope joint has
a URDF mimic relationship. The parser uses names, never array indices.

`joint_gripper_s3_body`, `joint_gripper_fingertip_left`,
`joint_gripper_fingertip_right`, `joint_grasp_center`,
`joint_wrist_yaw_bottom` and `joint_wrist_quick_connect` are fixed joints.
The two finger joints are revolute and have no mimic tags in this model.
Their live values are equal but nonzero, while their URDF limits are both zero;
those limits are unsuitable for gripper execution. Fingers are tracked in the
raw evidence and intentionally excluded from the five-DOF TCP chain.
`gripper_aperture` appears only in the hardware limit response here.
Head pan/tilt and wheel joints are outside the manipulation API.

The mapping is:

```
arm_extension = joint_arm_l3 + joint_arm_l2 + joint_arm_l1 + joint_arm_l0
joint_arm_lN = arm_extension / 4       # inverse representation for this model
wrist_extension == arm_extension      # measured consistency check, not a URDF joint
```

Across 330 live driver samples, maximum alias-versus-sum residual was 0.0 m and maximum inter-stage spread was 0.0 m. See [telescope observations](evidence/telescope_mapping.json).

A local vendor source checkout also implements four `pos_out / 4` stages in
`stretch_core/stretch_core/stretch_driver.py`; it is corroborating source only,
not a claim that the robot runs the same Git revision. The conclusion rests on
the live URDF, observed samples and graph configuration. Upstream reference:
[Hello Robot Humble driver](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/stretch_driver.py).

## Status and limits

Observed `/is_homed=true`, `/is_runstopped=false`, `/mode="position"` and
`/tool="eoa_wrist_dw3_tool_sg3"`. These are a snapshot, not persisted safety state.
The interface keeps status receipt times and requires both status channels to
be fresh. Readable false homed or true runstop remains valid data, never execution
authorization.

`/joint_limits` has no periodic or retained sample. The read-only
`/get_joint_states` (`std_srvs/srv/Trigger`) getter returned success and caused the
limit publication. In this driver-specific message, `position` means lower limit
and `velocity` means upper limit; neither is a current position or velocity.
The validator never calls this getter and has no service clients.

| Hardware limit name | Lower | Upper |
|---|---:|---:|
| `joint_wrist_pitch` | -1.57079632679 | 0.450990351638 |
| `joint_arm` | 0 | 0.516755465151 |
| `joint_lift` | 0 | 1.09943164589 |
| `joint_head_tilt` | -2.04019444789 | 0.509281621578 |
| `joint_head_pan` | -4.00675781796 | 1.81316529128 |
| `joint_wrist_roll` | -2.91149553541 | 2.91763145856 |
| `joint_wrist_yaw` | -1.26106003937 | 4.50095529512 |
| `gripper_aperture` | -0.128525493799 | 0.27303628847 |
| `joint_gripper_finger_left` | -0.375805537423 | 0.798351720674 |
| `joint_gripper_finger_right` | -0.375805537423 | 0.798351720674 |

These hardware bounds differ from the URDF. The current pure model validates
**URDF bounds**, not authorization to use the entire hardware range. Future
execution would need an explicit intersection of model/hardware bounds and
safety policy; none exists in this milestone.

## Diagnostic finding

The captured diagnostics report `stretch_driver: Pimu/Cliff` at ERROR level:
`cliff_range=83.83697509765625 | out of range [-50,20]`. Lift, arm, wrist and
gripper device diagnostics report present/calibrated or OK. This passive
kinematics validator does not certify robot health, clear faults or authorize
movement. The cliff finding remains unresolved and must be investigated before
any future execution milestone. See the full diagnostic sample in the capture.

## Validation behavior

`interface_validator` checks valid model receipt, required named joints, finite
values, duplicate/malformed arrays, physical and logical model limits, receipt
and header freshness, status availability/freshness, fresh dynamic TF and received
static TF. A rejected latest message invalidates state instead of retaining an
older valid sample. Default freshness is 1 second; header times more than 50 ms
in the future are rejected. Clock synchronization is required for domain 42.
FK validation additionally requires a connected base-to-TCP TF chain at the
chosen timestamp. Model updates invalidate the current state.

Real interface validation passed; the report is in
[evidence/interface_validation.log](evidence/interface_validation.log).
Data are exposed through a C++ library and are not republished as another state
topic. There is no command interface.
