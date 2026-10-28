# Overview

*stretch_custom_manipulation* contains the unified bringup for one-click 5-DoF IK: launch file, simulation/real configuration, RViz layout and the end-to-end launch test.

```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=true
```

`run_manipulation.sh` selects domain 20 (`sim:=true`) or 42 (`sim:=false`), restarts the ROS daemon, sources the workspace and forwards all other arguments to `bringup.launch.py`.

| Argument | Default | Description |
|---|---|---|
| `sim` | `true` | Kinematic simulation on domain 20, or the real driver on domain 42 |
| `rviz` | `true` | Start RViz |
| `dry_run` | `false` | Validate without sending trajectories |
| `execution_enabled` | `true` | `false` blocks validation and execution |

Configuration: `config/common.yaml`, `config/sim.yaml`, `config/real.yaml`.

- [Architecture](docs/ARCHITECTURE.md)
- [Real robot quickstart](docs/REAL_ROBOT_QUICKSTART.md)
- [Real robot test checklist](docs/REAL_ROBOT_TEST_CHECKLIST.md)
- [Gripper contract](docs/GRIPPER_CONTRACT.md)
