# Guarded lift execution milestone

Implemented in `src/stretch_guarded_executor`. Hardware execution defaults disabled;
startup is always DISARMED. IK mathematics and the preview node are unchanged.
Only `joint_lift` can appear in an outgoing trajectory. Arm and wrist execution is
rejected, including otherwise valid IK solutions that change those coordinates.

## Interface discovered on domain 42

Live ROS discovery found one `control_msgs/action/FollowJointTrajectory` server at
`/stretch_controller/follow_joint_trajectory`, provided by `/stretch_driver`.
There were zero action clients before this work. This is the Stretch driver's
action server, not an assumed ros2_control controller. See
[evidence](evidence/executor/action_info.txt) and
[driver graph](evidence/executor/driver_interfaces.txt).

Relevant services actually advertised:

| Interface | Type | Executor use |
|---|---|---|
| `/stop_the_robot` | `std_srvs/srv/Trigger` | Supplemental STOP in physical mode |
| `/runstop` | `std_srvs/srv/SetBool` | Assert `true` if cancellation is unconfirmed; never clear automatically |
| `/switch_to_trajectory_mode` | `std_srvs/srv/Trigger` | Operator setup only; executor never calls it |
| `/switch_to_position_mode` | `std_srvs/srv/Trigger` | Not called |
| `/stretch_driver/get_parameters` | `rcl_interfaces/srv/GetParameters` | Read-only initial-point safeguard verification |

The Humble vendor [action implementation](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/joint_trajectory_server.py)
and [trajectory components](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/trajectory_components.py)
were inspected alongside local vendor source and previously captured robot source.
In position mode, points are processed as position commands without respecting
trajectory timing. The executor therefore requires fresh `/mode=trajectory`.
Trajectory mode requires a zero header stamp and at least two points. Its lift
component consumes positions and velocities; acceleration fields are not used.
The implementation supports logical `wrist_extension`, but this milestone sends
only `joint_lift`.

Live reads during the final dry run confirmed
`fail_if_motor_initial_point_is_not_trajectory_first_point=true`. The executor
polls this parameter once per second and fails closed when it cannot verify it.
This prevents the driver from automatically moving to a differing first point.
An earlier discovery query returned an empty values list; the executor's dedicated
single-parameter query succeeded. SSH batch authentication was unavailable, so
installed robot source hashes were not reverified this session. No goal acceptance,
physical interpolation, tracking, or cancellation behavior has been tested on hardware.

## State machine and APIs

All services are under `/guarded_executor`; status is `/guarded_executor/status`.
Configuration is immutable after startup. Use exactly one executor instance.

| Service | Type | Behavior |
|---|---|---|
| `arm` | `std_srvs/srv/SetBool` | `true`: explicit arm; `false`: disarm, cancel if active |
| `disarm` | `std_srvs/srv/Trigger` | Clear approval; abort/cancel active motion |
| `use_ik_preview` | `std_srvs/srv/Trigger` | Select the existing preview candidate |
| `stage_lift_test` | `std_srvs/srv/Trigger` | Stage configured relative lift, default +0.01 m; does not execute |
| `validate_preview` | `std_srvs/srv/Trigger` | Check target and freeze approval, entering READY |
| `execute_preview` | `std_srvs/srv/Trigger` | Recheck everything, then execute the approved lift-only trajectory |
| `stop` | `std_srvs/srv/Trigger` | Immediate local abort plus physical cancel/STOP when enabled; independent of IK |

`DISARMED → ARMED → READY → EXECUTING → COMPLETE` is the success path.
Execution failures and STOP enter ABORTED. Both COMPLETE and ABORTED have
`armed=false`; a new explicit arm and validation are required. Terminal state is
retained for inspection rather than immediately overwriting it with DISARMED.
A changed/failed preview or expired approval invalidates READY and requires a
new validation. Failed validation sends nothing. A failure after execution begins
automatically disarms. Arming is refused while a previous goal or STOP is unresolved.

The preview's `/ik_preview/joint_states` alone is insufficient: it retains the last
valid ghost on IK failure. The executor pairs it with successful, fresh
`/ik_preview/status`, the matching logical solution and the original measured seed.
An interactive marker only updates a candidate; it never invokes execute.

## Controller-idle preflight

Server availability is independent of receiving a previous action status message.
The old `action_seen && idle` condition incorrectly rejected a reachable, idle
server that had never published a `GoalStatusArray`.

| Report field | Meaning |
|---|---|
| `action_server_available` | Physical ActionClient reports server ready; dry run discovers the driver's typed send_goal/get_result/cancel_goal services and status/feedback publishers without creating a client |
| `status_message_received` | At least one action status snapshot has been received; informational, not an idle prerequisite |
| `active_goal_present` | An ACCEPTED, EXECUTING or CANCELING goal is known, or a local goal is pending/unresolved |
| `controller_idle` | Server available and no active/pending goal |

These fields are included in `/guarded_executor/status` and dry-run JSON reports.
A reachable server with no status history is idle. An empty array or a snapshot
containing only SUCCEEDED/ABORTED/CANCELED goals is idle. Any active goal blocks
approval and is checked again before execution. A known active goal is never
forgotten because messages stop or the server disconnects. Local pending goals
block even if an empty/terminal snapshot arrives. Server loss rejects preflight
and aborts execution; cancellation and unresolved-stop protection are unchanged.
This is a preflight snapshot, not arbitration against other command clients.

## Diagnostic relevance policy

ERROR and STALE remain blocking by default. The immutable real-test profile sets:

```yaml
base_motion_enabled: false
diagnostic_error_to_warn_allowlist: ["stretch_driver: Pimu/Cliff"]
```

Only an exact-name `stretch_driver: Pimu/Cliff` **ERROR** is classified locally as
WARN, and only with base motion disabled. The executable always prohibits base
commands and refuses startup if `base_motion_enabled=true`; the policy classifier
also rejects the cliff exception for a base-enabled profile. This parameter never
grants permission to move the base. Other command clients must remain idle as
before; this package does not arbitrate external clients.

The allowlist defaults empty when the profile is not loaded. Entries must have a
reviewed diagnostic/subsystem mapping; the only supported entry is the cliff
diagnostic. Unknown, wildcard, lift, and controller entries are rejected at startup.
No prefix matching or message-text matching is used. A matching name cannot
downgrade STALE, and the independent 2.5 s diagnostics-stream timeout is unchanged.
Empty diagnostic arrays still block. Lift/controller faults, runstop, homing,
joint-state freshness and executor tracking guards are never bypassed.

Each downgrade emits a WARN log containing the exact name, original ERROR level,
original message and policy reason (base motion disabled in the lift-only profile).
The warning also remains visible in `/guarded_executor/status`. Incoming diagnostic
messages are not modified or republished; the cliff fault is not declared fixed.
All trajectory bounds, speed/acceleration caps and tracking thresholds are unchanged.

## Guards and trajectory

Before validation and again immediately before sending, require:

- Explicit `execution_enabled=true`, explicit arm, and a valid unexpired candidate.
- Canonical `/stretch/joint_states`, required joints, finite values, fresh measured
  state and homed/runstop flags (0.5 s), homed=true, runstop=false.
- Live URDF and validated hardware CSV limits intersected with URDF limits.
  An alternate limits file can only tighten the captured hardware limits.
- Fresh trajectory mode, streaming position disabled, idle action status, fresh
  diagnostics without blocking ERROR/STALE after the above relevance policy,
  and the initial-point safeguard verified true.
- Target inside bounds; deltas no greater than `[.01,.01,.05,.05,.05]` in logical
  order `[lift, arm_extension, wrist_yaw, wrist_pitch, wrist_roll]`.
- **Lift-only:** candidate arm/wrist changes relative to its seed are rejected.
  Passive-axis encoder drift may be at most `[.001,.005,.005,.005]` for arm/wrists;
  no commands are sent for those axes.
- Duration 3–10 s and computed lift velocity/acceleration below 0.005 m/s and
  0.005 m/s². Default duration is 5 s. Nonzero motion is required.
- Approval expires after 15 s. Start drift after approval is at most
  `[.001,.001,.005,.005,.005]`. A manually staged relative lift remains exactly
  +0.01 m from the latest measured start within this drift tolerance; a preview
  candidate retains its absolute approved lift target.

With `u=t/T`, the reference is
`q(t)=q_start+(q_target-q_start)*(10u³-15u⁴+6u⁵)`.
It has zero endpoint velocity and acceleration. Sample every 0.1 s with position,
velocity and acceleration: a 5 s lift has 51 points. Stretch interpolates the
position/velocity samples as cubic segments. There is no position step.
For +1 cm in 5 s the analytic peaks are 0.00375 m/s and approximately
0.0023094 m/s², below the enforced caps.

A 20 ms timer monitors all five measured coordinates against the reference.
Default tracking thresholds are `[.003,.003,.02,.02,.02]`. Stale state, unhoming,
runstop, unsafe mode/streaming/diagnostics, missing safeguard, excessive tracking
error, controller rejection/failure/disappearance, goal acknowledgement timeout
(1 s), feedback timeout (1 s), or execution timeout (`T+2 s`) abort and disarm.
Completion additionally requires a successful action result, elapsed trajectory
time, and measured lift endpoint error ≤0.001 m.

STOP requests action cancellation and `/stop_the_robot`. An unacknowledged goal
immediately triggers runstop protection; otherwise unconfirmed cancellation after
0.5 s triggers `/runstop=true`. Late accepted goals are canceled too. A STOP service
acknowledgement alone is not considered action cancellation. SIGINT/SIGTERM request
STOP for active execution and allow one second for callbacks. Never automatically
clear runstop or rearm.

The software cannot guarantee stopping after process kill, host failure, or loss
of the robot network. Keep the physical runstop available for a supervised test.
This package has no collision checking or external command arbitration; other
robot clients must remain idle. Those limitations are not addressed by passing
kinematics or dry-run tests.

## Tests and actual real dry run

The diagnostic-policy update passed **52 guard unit tests and the ROS dry-run
transport test**, with zero failures. See the [test summary](evidence/executor/diagnostic_policy_test_summary.txt),
[transport evidence](evidence/executor/diagnostic_policy_dry_run.json), and
[warning logs](evidence/executor/diagnostic_policy_dry_run.log).
It is tested offline on localhost-only domain 20. It
adds unit cases for cliff ERROR with base disabled/enabled, strict defaults,
unknown/lift/controller faults, STALE status/stream, exact matching and preservation
of runstop/homing/freshness/tracking guards. The transport test loads the real-test
YAML, validates with a persistent cliff ERROR, checks the required warning log,
rejects a base-enabled startup, and verifies other faults and stream timeout still
block. No hardware execution or new real-robot test was performed for this update.

The following records describe the **original milestone before this policy change**; the
historical cliff ERROR now qualifies for the explicit lift-only profile exception.

See [offline results](evidence/executor/offline_dry_run.json) and
[real results](evidence/executor/real_dry_run.json).

- Workspace test summary: **75 reported tests, zero errors/failures/skips**.
- 40 C++ guard unit tests passed: transitions, all prerequisite failures, bounds,
  delta caps, passive-axis rejection, staleness, candidate changes, immutable
  approval, duration/speed, monitoring, timeout, endpoint, disarm and STOP.
- Domain-20 transport test completed full dry-run validation and execution for
  both manual lift and IK preview; verified failed/changed preview and stale state
  cannot reuse approval. It is registered as `test_dry_transport` in colcon tests.
  Fixtures use localhost-only domain 20 and are never constructed on domain 42.
- Real domain-42 dry run verified default-disabled rejection, explicit arm, staging,
  validation rejection, execute rejection, local STOP/disarm, and namespaced status.
  It created a 51-point inspection trajectory with `approved=false`.
- Runtime graph inspection found **no hardware action clients, STOP/runstop clients,
  command publishers, or real state/TF publishers** in dry run. Its only external
  service client was the read-only driver parameter query.
- Real blockers: `/mode=position` and diagnostic ERROR
  `stretch_driver: Pimu/Cliff` (66.20 in the final capture, allowed range [-50,20]).
  Homed=true, runstop=false, state fresh, initial-point guard verified true.
  The full live approval path remains blocked; it was not bypassed or simulated.

No physical goals or control-service requests were sent during development.
The live test's STOP service was the executor's dry-run local STOP, with no hardware
clients present. Driver mode and runstop were not changed.

## Build and repeat dry run

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
colcon build --packages-up-to stretch_guarded_executor --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select stretch_guarded_executor --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=42 python3 scripts/test_guarded_executor_dry.py --live \
  --output docs/evidence/executor/real_dry_run.json
```

Or launch interactively (does not arm):

```bash
source scripts/use_real.sh
ros2 launch stretch_guarded_executor real_test.launch.py \
  dry_run:=true execution_enabled:=true
```

`execution_enabled=true` permits guard validation in dry run but does not create
hardware clients. `dry_run` cannot be changed while running. Default launch with
no arguments is dry run with execution disabled.

## Manually authorized +1 cm lift test — NOT run during this milestone

Blocking conditions must first be resolved by the operator. Under the explicit
lift-only profile, a fresh allowlisted cliff ERROR is a logged warning; any other
blocking diagnostic, stale stream, or non-trajectory mode still prevents approval.
Stop the dry-run executor before starting the physical one. Keep other command
clients and gamepad operation idle.
The following is the exact lift-only procedure, not a claim of hardware readiness.

Once the executor is running DISARMED with physical execution enabled, one
explicit operator command performs fresh arm → stage → validate → execute:

```bash
~/stretch_custom_manipulation_ws/scripts/execute_guarded_lift_test.sh --execute
```

The wrapper sources Humble and this workspace, selects domain 42, and verifies
that exactly one executor is configured for physical +0.01 m / 5 s lift testing.
It never retries a motion request. It waits for COMPLETE/ABORTED and requests
disarm/cancel if its own test fails or is interrupted. With `--check` instead of
`--execute`, it only arms/stages/validates, then disarms without sending a goal.
Fresh approval is obtained inside the command, preserving the 15 s expiry rather
than leaving an old approval armed while waiting for the operator.

To start the executor if it is not already running, use terminal A:

```bash
source /opt/ros/humble/setup.bash
cd ~/stretch_custom_manipulation_ws
source install/setup.bash
source scripts/use_real.sh
ros2 launch stretch_guarded_executor real_test.launch.py \
  dry_run:=false execution_enabled:=true
```

This starts DISARMED and sends no goal. In terminal B, source the same environment.
Only after all blocking conditions have been resolved, explicitly select trajectory
mode if it is not already selected:

```bash
ros2 service call /switch_to_trajectory_mode std_srvs/srv/Trigger '{}'
```

This is the driver's inspected mode-configuration service, not a trajectory
command. The executor never calls it automatically. Confirm success, then run
these separately; **stop on any `success=false`**. Complete staging through execute
within 15 s, or restage/revalidate:

```bash
ros2 service call /guarded_executor/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /guarded_executor/stage_lift_test std_srvs/srv/Trigger '{}'
ros2 service call /guarded_executor/validate_preview std_srvs/srv/Trigger '{}'
```

After inspecting READY approval, the following single command explicitly requests
**one +0.01 m lift trajectory over 5 seconds**. It is the only command in this
procedure that requests motion, and it rechecks all guards:

```bash
ros2 service call /guarded_executor/execute_preview std_srvs/srv/Trigger '{}'
```

Monitor `/guarded_executor/status`. The result must become COMPLETE with armed=0;
ABORTED also disarms. For independent software STOP:

```bash
ros2 service call /guarded_executor/stop std_srvs/srv/Trigger '{}'
```

If software STOP cannot be confirmed, use the physical runstop. Do not proceed to
arm/wrist execution; the next milestone is this supervised lift-only hardware
acceptance test, recording measured tracking, terminal error and actual stop/cancel
behavior after the present blockers are resolved.
