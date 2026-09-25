# Simulation to real architecture

The foundation below records milestone 1. Milestone 2 adds offline IK and an
isolated RViz preview; see [IK and preview](IK_AND_PREVIEW.md) for the current
implementation and commands. Hardware execution remains absent.

| Runtime | ROS_DOMAIN_ID | STRETCH_RUNTIME | Clock |
|---|---:|---|---|
| Local/simulation | 20 | `sim` | Set `use_sim_time=true` only with simulator `/clock` |
| Real Stretch 3 | 42 | `real` | ROS system time; host clocks must agree |

Source `scripts/use_sim.sh` or `scripts/use_real.sh`. Each script contains only
two environment-variable exports, launches nothing and does not source another
workspace. Source ROS and the colcon overlay separately. Separate terminals and
DDS domains prevent accidental topic mixing; these are not a security boundary.

The dependency flow is:

1. Existing runtime supplies measured joints, a matching URDF, TF and status.
2. `stretch_robot_interface` validates the contract and exposes logical state.
3. `stretch_custom_kinematics` evaluates that state against the runtime URDF.

There are no `STRETCH_RUNTIME` conditionals in either core package. Geometry,
calibration, origins, axes, transforms and limits come from the received model.
The only model-specific contract is the verified joint naming and four-stage,
parallel, independent telescope representation. Unknown/mimic/nonparallel chain
representations fail explicitly; they never silently select guessed kinematics.
Future support for another representation belongs in a tested general model
mapping or boundary adapter and must be shared by both runtimes.

## Current interface differences

The real system has been inspected and validated. Its canonical measured stream
is `/stretch/joint_states`; `/joint_states` is a retimestamped, default-completed
stream used by robot_state_publisher. The calibrated URDF has four independent
arm stages, no mimic tags, and `link_grasp_center`.

No simulator was started or modified in this milestone, so simulation topic,
URDF and status equivalence is **not yet verified**. A simulator must provide:

- The same five logical coordinates represented by the supported URDF chain.
- Actual joint values with source timestamps and a compatible `joint_state_topic`.
- Matching `/robot_description`, `/tf` and `/tf_static` QoS.
- Explicit readable `/is_homed` and `/is_runstopped` status; no fabricated defaults.
- `/clock` if simulated time is enabled.

Name/topic adapters and simulator-owned status belong at the runtime boundary.
A missing interface remains invalid. Real homed/runstop states are tracked
separately from data validity: a readable `homed=false` or `runstopped=true` still
allows passive FK diagnostics. Data validity is never permission to execute.

## Scope and next milestone

Only the state foundation and forward kinematics exist. There is no physical
movement, command publisher, trajectory client, service that controls hardware,
IK, planner, OctoMap or collision world. The driver and camera were left running.
Robot access used local read-only ROS subscriptions, parameter reads, and the
read-only `/get_joint_states` limit getter; SSH access was not needed.

Next recommended milestone: establish domain-20 contract parity and run the same
strict FK/TF regression across a simulated configuration grid, then implement
standalone, limit-aware numerical IK with FK→IK→FK tests. Include unreachable
and underactuated orientation targets and singularity handling. Keep all hardware
execution excluded. The five-DOF arm cannot realize every six-DOF pose.


## Guarded execution milestone

`stretch_guarded_executor` is the only package with optional hardware action/STOP
clients. Core interface/kinematics and preview remain command-free. Dry run creates
no such clients; physical mode requires domain 42 and is restricted to lift-only
execution through explicit arm, validate and execute services. Domain 20 hosts
localhost-only transport fixtures. See [guarded execution](GUARDED_EXECUTOR.md) for
guards, current real blockers and the manual test procedure.
