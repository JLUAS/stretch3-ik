# Kinematic model and validation

This is the preserved milestone-1 FK report. The subsequent IK implementation
and measurements are documented in [IK and preview](IK_AND_PREVIEW.md).

## Model provenance and TCP

The runtime model comes from the robot's retained `/robot_description`
(`std_msgs/msg/String`) publication. It includes the current robot's calibrated
origins, not a manually measured replacement. The captured fixture is
[evidence/robot.urdf](evidence/robot.urdf); identical test fixtures have SHA-256
`463caa1c76bb7395964356145dac5377689846928522fe35721913b782ec0bfa`.
Test fixtures are never silently substituted for a missing live model.

Chosen reference: **`base_link`**. Chosen TCP: **`link_grasp_center`**.
The latter exists in both the captured URDF and live TF. It is the model's nominal
grasp center fixed to `link_gripper_s3_body` by `joint_grasp_center`, with origin
`xyz=(0,0,0.23)` m and
`rpy=(-1.5707963267949,-1.5707963267948,0)` rad. This definition is independent of
finger opening and fits the requested five-DOF state. It is not a measured
contact point or a claim of physical calibration accuracy.

The base-to-TCP chain is:

| Joint | Type | Child link |
|---|---|---|
| `joint_mast` | fixed | `link_mast` |
| `joint_lift` | prismatic | `link_lift` |
| `joint_arm_l4` | fixed | `link_arm_l4` |
| `joint_arm_l3` | prismatic | `link_arm_l3` |
| `joint_arm_l2` | prismatic | `link_arm_l2` |
| `joint_arm_l1` | prismatic | `link_arm_l1` |
| `joint_arm_l0` | prismatic | `link_arm_l0` |
| `joint_wrist_yaw` | revolute | `link_wrist_yaw` |
| `joint_wrist_yaw_bottom` | fixed | `link_wrist_yaw_bottom` |
| `joint_wrist_pitch` | revolute | `link_wrist_pitch` |
| `joint_wrist_roll` | revolute | `link_wrist_roll` |
| `joint_gripper_s3_body` | fixed | `link_gripper_s3_body` |
| `joint_grasp_center` | fixed | `link_grasp_center` |

## Logical state and telescope

`q = [lift, arm_extension, wrist_yaw, wrist_pitch, wrist_roll]` uses meters for
lift/extension and radians for wrist angles. The C++ `RobotState` also includes
the source timestamp, homed/runstop flags and data validity.

The URDF telescope has four independent prismatic joints (l3, l2, l1, l0), each
with local axis `(0,0,1)` and limits `[0,0.13]` m; **none has a mimic tag**.
`joint_arm_l4` is fixed. Live driver data verify that each stage is
`wrist_extension/4` and their sum is `wrist_extension`.

Decoding sums the four stage positions. The optional `wrist_extension` alias
must agree within 1 micrometer if present; it never replaces missing stages.
Expanding a logical state assigns `arm_extension/4` to each stage. All physical
limits are checked on measured input and on expanded output. Therefore the
current logical model permits `[0,0.52]` m, which differs from hardware limits.
See the interface contract for both sets of limits.

The model constructor verifies unit axes, correct joint types, complete named
chain, finite ordered limits, valid origins, contiguous telescope stages and
parallel codirectional stage axes after applying URDF origins. With no rotation
between telescope stages, their translations sum; unequal valid input stage
values have the same TCP pose as equal redistribution with the same sum.
Fixed offsets and small calibrated rotations remain in the transform product.
Nonparallel or mimic telescope representations are explicitly unsupported and
rejected, rather than approximated or interpreted by a runtime-specific branch.

## FK formulation

For every joint from base to TCP:

```
T_base_tcp(q) = product_j [ T_origin(j) * T_motion(j, physical(q)) ]
T_origin     = translation(URDF.xyz) * rotation(URDF.quaternion)
T_fixed      = identity
T_prismatic  = translation(axis * position)
T_revolute   = rotation_about(axis, angle)
```

The origin precedes motion because the URDF axis is expressed in the joint
frame. All fixed transforms are included. Eigen evaluates the product in double
precision. `computeTcpTransform` returns an `Eigen::Isometry3d`;
`computeTcpPose` returns a `geometry_msgs::msg::Pose` in the configured base frame.

The pure function rejects invalid logical state and nonfinite/out-of-limit
values. It does not independently read ROS topics or infer freshness. Consumers
must obtain current data through `InterfaceNode::state()` or provide a valid
offline test configuration. URDF parsing and mapping live in the interface
library; the FK library depends on it. Neither depends on MoveIt or on a runtime
selection environment variable.

## Read-only live validation

The local validators ran on domain 42 while the existing driver and camera kept
running. The interface reported PASS for model, required joints, fresh state,
TF, static TF, and readable fresh homed/runstop channels. No command publisher,
action client, hardware service client or TF broadcaster exists in these packages.

The pose errors are:

```
position_m = norm(p_custom - p_tf)
position_mm = 1000 * position_m
orientation_rad = shortest quaternion angular distance(R_custom, R_tf)
orientation_deg = orientation_rad * 180 / pi
```

TF lookups use the source timestamp, never an unlabelled latest transform. The
validator buffers input briefly for TF arrival and performs nonblocking lookups.
Two distinct experiments are necessary because `/joint_states` retimestamps
cached driver samples and robot_state_publisher can skip input messages:

1. **Strict FK/model equivalence:** use `/joint_states` and
   `require_exact_tf_sample=true`. Require each moving chain transform to have
   actually been observed at that exact timestamp. Otherwise count the sample as
   skipped; do not label interpolated TF as numerical ground truth. The canonical
   robot state contract remains `/stretch/joint_states`.
2. **Canonical stream cross-check:** use `/stretch/joint_states` and TF at its
   timestamp, allowing interpolation. This includes encoder quantization,
   retimestamping and republisher lag, so it is not an isolated FK arithmetic test.

| Experiment | Compared | Threshold failures | Maximum position (m / mm) | Maximum orientation (rad / degrees) |
|---|---:|---:|---|---|
| Strict exact-sample FK | 836 | 0 | 8.7090720536e-16 / 8.7090720536e-13 | 1.98912806972e-15 / 1.13968643306e-13 |
| Canonical stream vs interpolated TF | 886 | 111 | 0.000416838015291 / 0.416838015291 | 0.00209989019406 / 0.120314845561 |

The strict run passed all 836 comparisons at tolerances **1e-8 m and 1e-8 rad**.
Six input samples lacked an exact observed TF sample and were explicitly skipped;
there were zero stale comparison drops and zero TF lookup failures. Residuals
are at double-precision rounding scale. This validates the URDF-based mapping
and FK against the live TF implementation at the stationary observed configuration.
It does **not** establish physical TCP calibration accuracy or full-workspace
robot accuracy, since both calculations use the same model and no motion was
commanded.

The canonical cross-check used the unchanged default thresholds, **0.001 m and
0.001 rad**. It had 111 orientation-threshold failures out of 886 comparisons.
Those failures remain reported, not hidden by loosening the threshold. Small
wrist encoder steps and retimestamping account for the cross-stream residual;
the exact-sample experiment independently isolates and verifies the FK equations.
For pure numerical regression, use the strict command in the README. Do not use
these diagnostics as an execution interlock or claim canonical/TF timestamps
represent identical measurements.

Raw CSVs, logs and machine-readable maxima/RMS values are in
[evidence/validation_summary.json](evidence/validation_summary.json),
[evidence/fk_tf_input.csv](evidence/fk_tf_input.csv),
[evidence/fk_tf_input.log](evidence/fk_tf_input.log),
[evidence/fk_canonical.csv](evidence/fk_canonical.csv), and
[evidence/fk_canonical.log](evidence/fk_canonical.log).

## Tests and build results

Both packages build successfully in Humble with C++17, Eigen3, urdf, rclcpp,
sensor_msgs, geometry_msgs, tf2 and tf2_ros. No compiler warnings appeared in the
final build. The 19 GoogleTest cases pass: 11 interface tests and 8 kinematics
tests. `colcon test-result` reports **21 tests, 0 errors, 0 failures, 0 skipped**
because its summary includes two CTest wrapper results as well as the 19 cases.

Coverage includes order-independent named mapping, telescope sum/expansion,
inconsistent aliases, missing physical joints, mismatched/duplicate/nonfinite
input, bounds and physical-stage limits, stale receipt/header and future stamp
handling, malformed/missing-chain URDFs, rejected mimic/nonparallel models,
three fixed numeric FK configurations, lift translation, invalid FK inputs,
pose/error conversion and a 27-configuration logical→physical→logical grid.

The three numeric golden configurations were independently composed with Python
NumPy/SciPy from the captured URDF and then frozen in C++ (not calculated by the
implementation under test). They include zero, nominal and nonzero wrist/arm
poses. Tests need neither robot connectivity nor mesh assets.

`test/round_trip.hpp` defines a future inverse-solver callback and FK residual
checker. Its current test deliberately supplies an incorrect result and verifies
that the harness detects the residual. There is no IK implementation or claim
that IK round trips already work.

## Remaining limitations

- Simulation contract parity and multi-configuration simulated TF validation
  remain unverified; no simulator was launched for this milestone.
- Canonical state and TF are asynchronous representations; their timing residual
  remains visible as described above.
- Hardware and URDF joint limits differ; the gripper URDF limits are degenerate.
- The live cliff diagnostic is ERROR and remains unresolved (interface contract).
- Only the verified independent, parallel four-stage model is supported; other
  models require explicit mapping support and regression tests.

These do not block the completed read-only foundation/FK milestone. Address the
simulation contract and offline IK tests next; investigate robot health and
execution-specific limits before any later, separately authorized motion work.
