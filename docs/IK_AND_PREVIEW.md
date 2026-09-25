# Offline IK and RViz preview

This milestone adds C++17 inverse kinematics and a read-only RViz preview to the
existing `stretch_custom_kinematics` package. There are still only two packages.
No trajectory action client, hardware command publisher, controller launcher or
physical execution exists.

## Five task-space constraints

The task is **three Cartesian coordinates plus two direction coordinates**:

1. TCP x in `base_link`.
2. TCP y in `base_link`.
3. TCP z in `base_link`.
4–5. The unit direction of the TCP's **local Z axis**, expressed in `base_link`.

The last two are represented as a three-component unit-vector difference on
S², which has only two independent degrees of freedom. Local Z is a transverse
TCP axis in this URDF. It is deliberately not the gripper approach/local X axis:
that approach direction is parallel to the wrist-roll axis and would discard
roll's effect, making position plus approach direction rank-deficient here.

The residual is `[p_target - p_fk; 0.2*(z_target - z_fk)]`. The 0.2 factor balances
meters and unit-direction error in the numerical objective; acceptance uses
unweighted errors. Success requires position error ≤ **1e-6 m** and Z-direction
angular error ≤ **1e-5 rad**. Rotation about the target Z axis is **unconstrained**.
The full target quaternion is not a sixth equality constraint.

Both orientation measures are always reported:

- **Direction error:** `atan2(norm(z_fk × z_target), dot(z_fk,z_target))`; controlled.
- **Full orientation error:** shortest quaternion angular distance; diagnostic,
  including the free spin. It can be large on a successful five-constraint solve.

The marker has translation and rotation controls for a full pose. Changing only
free Z spin need not move the ghost. Tests explicitly verify this behavior.

## Solver API and current-state seed

```cpp
InverseKinematics ik(model, JointBounds::load(hardware_limits_file));
IkResult result = ik.solve(target_pose, current_logical_state, state_age_seconds);
```

The first numerical attempt always starts at `current_logical_state`. The RViz
node reads this from `InterfaceNode::state()` for each target change. It never
substitutes the previous ghost solution for the measured seed. Status includes
`seed_topic`, `seed_joint_state` and the source timestamp for inspection.

The solver uses finite-difference Jacobians, damped least squares, bounded steps,
projection onto joint limits and backtracking. If the current-state attempt
stalls, deterministic Halton-distributed bounded restarts search alternate
branches. Defaults allow 160 iterations per attempt and 64 attempts. Iteration
counts are totals across all attempts. The solver is local and numerical; the
finite restart budget is not a global completeness guarantee.

Input freshness is checked before solving. The ROS interface also independently
checks message receipt age, source timestamp and status freshness. A stale or
invalid seed is not converted into a valid pose. Preview solves are serialized
in a single-threaded executor and marker updates are coalesced at 10 Hz.

## Hardware joint limits

`config/hardware_limits.csv` stores the exact values from the previously validated
read-only `/get_joint_states` response in `docs/evidence/joint_limits.json`.
`joint_arm` in that driver response maps to logical `arm_extension`. The solver
intersects these bounds with those derived from the current URDF. No manually
measured geometry or separate real/simulation solver is used.

| Coordinate | Effective lower | Effective upper | Unit |
|---|---:|---:|---|
| lift | 0 | 1.0994316458886308 | m |
| arm_extension | 0 | 0.5167554651511435 | m |
| wrist_yaw | -1.261060039374321 | 4.0 | rad |
| wrist_pitch | -1.57 | 0.45099035163837853 | rad |
| wrist_roll | -2.9114955354069467 | 2.9176314585584895 | rad |

For example hardware permits yaw above 4 rad but the validated URDF does not,
so the upper intersection remains 4 rad. Every seed and returned solution must
satisfy these bounds. A missing, malformed or empty-intersection limit profile
fails explicitly. The captured profile belongs to the inspected Stretch 3 with
`eoa_wrist_dw3_tool_sg3`; revalidate it if hardware/tool calibration changes.

## Failure reasons

| Reason | Meaning |
|---|---|
| `unreachable target` | Position exceeds a conservative reach sphere derived from URDF origin lengths and prismatic bounds; this is a geometric certificate. |
| `joint limit` | Seed violates effective bounds, or the best exhausted bounded search is active at a bound. The latter reports a constrained numerical failure, not a global infeasibility proof. |
| `convergence failure` | Search budget is exhausted without a valid task solution and without a bound-active best candidate. |
| `invalid target` | Nonfinite position, invalid rotation/quaternion, or an unsupported preview target frame. |
| `stale state` | Seed is invalid, too old, too far in the future, or unavailable/freshness-invalid in the ROS interface. |

All failures set `success=false` and invalidate the candidate solution. Preview
retains the last successful ghost and explicitly labels it LAST VALID; it never
presents the failed candidate as solved. Unreachable configurations inside the
conservative sphere can produce a limit or convergence failure rather than a
false proof of unreachability.

## Launch on domain 20

From a fresh terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
source install/setup.bash
source scripts/use_sim.sh
ros2 launch stretch_custom_kinematics ik_preview.launch.py offline:=true
```

This starts a fixed, explicitly labelled offline current-state fixture under
`/ik_input`, its robot_state_publisher, the preview and RViz. The fixture uses a
captured URDF and wall-clock timestamps; it does not simulate dynamics. It is
restricted to **domain 20**, including a check inside the fixture executable.
No fixture messages are published at global `/joint_states` or `/tf`.

For an already running compatible simulator, use `offline:=false` and select its
measured stream and clock, for example:

```bash
ros2 launch stretch_custom_kinematics ik_preview.launch.py \
  offline:=false joint_state_topic:=/joint_states use_sim_time:=true
```

The simulator must supply model, status and measured state as specified in the
interface contract. The fixed fixture does not establish external simulator parity.

## Launch on domain 42: read-only real seed

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
source install/setup.bash
source scripts/use_real.sh
ros2 launch stretch_custom_kinematics ik_preview.launch.py
```

The default `offline:=false` reads the live robot model and canonical
`/stretch/joint_states`. It launches only the preview and RViz. It never starts,
stops or reconfigures robot drivers. `offline:=true` is rejected on domain 42.
The environment scripts still only export domain/runtime variables.

Optional launch arguments: `rviz:=false` for automated tests,
`mesh_package_path:=/path/to/stretch_urdf/SE3` for vendor mesh assets. Launch
automatically detects an installed Python `stretch_urdf` SE3 asset directory;
otherwise it leaves original package URIs for normal ament resource resolution.
On this machine all 61 referenced assets resolved from the installed vendor
package. Only mesh resource URIs are adjusted for visualization; FK/IK geometry
comes from the unchanged current URDF. No assets from other local workspaces are
required when the vendor Python package or `stretch_description` is available.

## Preview isolation and UI

Every preview link frame is prefixed `ik_preview/`. The fixed RViz frame is
`ik_preview/base_link`. Preview publishers are:

- `/ik_preview/joint_states`
- `/ik_preview/tf` and `/ik_preview/tf_static`
- `/ik_preview/robot_description`
- `/ik_preview/target_pose` and `/ik_preview/solved_pose`
- `/ik_preview/markers`, `/ik_preview/status`, `/ik_preview/target/update`

Interactive feedback is accepted only at `/ik_preview/target/feedback`. The
node requires namespace `/ik_preview` and validates resolved publisher names
before sending messages, rejecting remaps outside that namespace. RViz listens
to the isolated preview TF topics, not the robot TF tree. Its status panel only
subscribes to preview data. ROS infrastructure topics `/rosout` and
`/parameter_events` remain normal logging/parameter events.

Drag the target's translation arrows or rotation rings with RViz's Interact
tool. The path is marker feedback → current-state-seeded IK → bounded ghost
joints → FK solved pose → private TF/robot model/TCP triads and status. The target
triad is translucent and the solved triad is opaque. The panel displays success
or failure, target and solved positions, position/direction/full-orientation
errors, solve time, iterations/attempts and the five-joint solution. It scrolls
when needed. On failure the last valid ghost is clearly labelled.

There are no trajectory action clients, command publishers or hardware service
clients in the preview node. Runtime graph audit evidence lists both preview
publishers and the unchanged robot-topic publishers. Namespace isolation here
is a programming invariant, not a DDS security boundary.

## Reproduce numerical and integration validation

```bash
colcon test
colcon test-result --verbose
ros2 run stretch_custom_kinematics ik_benchmark \
  src/stretch_custom_kinematics/test/robot.urdf \
  src/stretch_custom_kinematics/config/hardware_limits.csv \
  /tmp/reachable_1000.csv 1000
```

The benchmark generates 1,000 uniform random configurations within the effective
bounds (MT19937 seed 20260924), creates targets with FK and solves from the same
captured measured current-state seed. It does **not** seed IK with each known
answer. CSV includes target and solved joints, residuals, timing, iterations,
attempts and failure reasons. Timings cover complete solves including restarts.
Error statistics describe successful solves; timing/iteration statistics include
all attempts and all cases. The executable returns nonzero if any case fails.

With either preview running, the following sends only private marker feedback:

```bash
python3 scripts/validate_ik_preview.py --output /tmp/preview_validation.json
```

For domain-20 fixture stale/recovery validation, add `--fixture-pid PID` using the
PID of this launch's `offline_seed.py`. The test checks its command identity and
domain before pausing that process, and resumes it in a finally block. This is
never used with the real driver. Unit tests separately exercise all five failure
reasons, explicit ±10 m unreachable poses, limit handling, free-spin semantics
and FK→IK→FK for 100 additional deterministic random configurations.

Numerical results and domain-20/domain-42 integration results are recorded in
[IK validation results](IK_VALIDATION_RESULTS.md). No physical movement was
requested or commanded during validation.
