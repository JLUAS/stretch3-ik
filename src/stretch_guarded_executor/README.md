# Overview

*stretch_guarded_executor* is the only package in this repository that can send trajectories to the Stretch driver. It starts DISARMED, validates every segment against measured state, hardware limits, per-joint step/velocity/acceleration envelopes and driver diagnostics, and fails closed on stale or missing inputs. Base motion is refused.

## guarded_executor

Must run in the `/guarded_executor` namespace. All parameters are read-only after startup.

```bash
source scripts/use_real.sh
ros2 launch stretch_guarded_executor real_test.launch.py
```

`real_test.launch.py` defaults to `dry_run:=true execution_enabled:=false`. In dry run no hardware action or stop clients are created.

### Services

| Service | Type | Description |
|---|---|---|
| `move_to_preview` | `std_srvs/Trigger` | Capture the current valid IK target and execute it in bounded segments |
| `stop` | `std_srvs/Trigger` | Cancel motion and remaining segments, then disarm |
| `disarm` | `std_srvs/Trigger` | Disarm and abort active motion |
| `arm` | `std_srvs/SetBool` | Arm (`true`) or disarm (`false`) |
| `use_ik_preview`, `validate_preview`, `execute_preview`, `stage_lift_test` | `std_srvs/Trigger` | Step-by-step commissioning flow |

### Action servers

| Action | Type | Description |
|---|---|---|
| `execute_lift_target` | `stretch_guarded_executor/action/ExecuteLiftTarget` | Segmented absolute lift move |

### Topics

| Topic | Type | Notes |
|---|---|---|
| `status` | `std_msgs/String` | Executor state and reasons (latched) |
| `lift_execution_status` | `std_msgs/String` | Lift action progress (latched) |

See the [guarded executor documentation](../../docs/GUARDED_EXECUTOR.md) for parameters, diagnostic policy and test evidence.
