# IK validation results

Validation completed on 2026-09-23 local time / 2026-09-24 UTC. All robot access
was read-only input inspection; interactive outputs stayed in the preview namespace.

## Random reachable benchmark

Uniform configurations inside the hardware/URDF limit intersection were generated
with MT19937 seed 20260924. Targets came from FK. Each solve began with the same
captured current-state seed, not the target's generating joints. Bounded fallback
restarts were included in the reported timing and total iteration count.

**1,000 / 1,000 successes (100%).**

| Metric | Mean | Maximum |
|---|---:|---:|
| Position error (m) | 1.11725495695e-07 | 9.94371134511e-07 |
| Constrained Z-direction error (rad) | 2.98097672086e-07 | 7.64977153801e-06 |
| Full orientation error (rad; free Z spin included) | 0.0652155105073 | 2.13626313753 |
| Solve time (ms) | 4.375595853 | 120.008443 |
| Total iterations | 257.484 | 6891 |
| Attempts | 2.523 | 44 |

Position errors in millimeters: mean **0.000111725496**, maximum **0.000994371135**.
Full orientation error in degrees: mean **3.73657351**, maximum **122.398862**.

The full orientation residual is deliberately reported even though it is not
an acceptance constraint. All five controlled constraints met their tolerances;
rotation about target Z is free. This benchmark does not demonstrate six-DOF
pose matching or global solver completeness. Timing is measured wall time on
this workstation and will vary with system load.

Raw evidence: [per-case CSV](evidence/ik/reachable_1000.csv),
[summary JSON](evidence/ik/benchmark_summary.json),
[benchmark output](evidence/ik/benchmark.log).

## Automated tests

29 GoogleTest cases pass: 11 interface, 8 FK/framework and 10 new IK cases.
`colcon test-result` reports 32 test results including three CTest wrapper
results, with zero failures, errors or skips. Tests cover every requested failure
reason, effective hardware limits, explicitly unreachable poses in positive and
negative directions, free-spin semantics and 100 additional random FK→IK→FK
cases. This is separate from the 1,000-case benchmark.

[Build log](evidence/ik/build.log), [test result](evidence/ik/test_result.log),
and [full unit-test output](evidence/ik/unit_tests.log).

## Domain-20 integration

Using the isolated fixed `/ik_input` fixture and RViz:

- A +0.04 m target-Z change through actual interactive-marker feedback solves.
- Preview ghost TF agrees with the solved FK TCP within 1e-8 m.
- An explicitly unreachable +10 m X target reports `unreachable target` and
  preserves the last successful ghost.
- An incorrect target frame reports `invalid target`.
- Pausing only the offline seed fixture causes `stale state`; resuming it recovers.
- No preview/input publishers exist on global `/joint_states`, `/tf`,
  `/tf_static`, `/joint_pose_cmd` or `/stretch/cmd_vel`.

Evidence: [domain-20 integration](evidence/ik/preview_domain20_validation.json)
and [RViz screenshot](evidence/ik/rviz_domain20.png).

## Domain-42 read-only integration

The same preview consumed current `/stretch/joint_states` and live
`/robot_description`. The recorded seed differs from the offline benchmark seed,
confirming current measured input was used. Reachable feedback, explicit
unreachable feedback, invalid-frame rejection and ghost TF/FK agreement passed.
No real source was paused, reconfigured or commanded.

The namespace audit lists preview outputs solely under `/ik_preview`, apart from
normal ROS logging/parameter infrastructure. The global TF publishers remain
robot_state_publisher/camera, and global JointState remains joint_state_publisher.
DDS temporarily reported an unknown node name for one canonical driver endpoint;
that is recorded verbatim rather than replaced with an assumed name. The preview
node's own complete publisher list independently confirms namespace isolation.
Its node-info report has no action clients or hardware service clients.

Evidence: [domain-42 integration](evidence/ik/preview_domain42_validation.json),
[node interfaces](evidence/ik/preview_domain42_node_info.txt),
[RViz screenshot](evidence/ik/rviz_domain42.png).

## Direct RViz interaction

Beyond synthetic feedback testing, the actual RViz Interact tool was used to drag
the target's `move_2` handle. `/ik_preview/rviz` emitted the recorded mouse-down,
pose-update and mouse-up events. Target Z moved from approximately 0.692815 m to
0.728871 m. The preview solved successfully and the displayed solved TCP matched
the dragged target. Only the ghost changed; no robot command was published.

[RViz feedback events](evidence/ik/gui_drag.json) and
[post-drag solver/target/solved state](evidence/ik/gui_drag_result.json).

## Namespace guard checks

All four negative launch tests were rejected before preview output: missing
preview namespace, TF remapped to global `/tf`, interactive-marker update remapped
to global `/tf`, and attempting to launch the offline fixture on domain 42.
[Guard test evidence](evidence/ik/isolation_guard_tests.json).

## Scope boundary

Offline IK and isolated RViz visualization are validated. No motion planning,
collision checking, trajectory execution or robot movement command was added.
A compatible external simulator has not been exercised; domain-20 validation used
the included fixed fixture. General local IK can still return a convergence or
limit failure on other reachable targets, especially near singularities, despite
the successful measured sample set.
