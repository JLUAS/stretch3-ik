# Overview

*stretch_lift_rviz_control* is the legacy lift-only commissioning UI: a vertical-only RViz marker (`lift_marker`) and the `LiftControlPanel` RViz plugin, which sends the selected lift target to the guarded executor's `execute_lift_target` action.

```bash
ros2 launch stretch_lift_rviz_control real_lift_control.launch.py
```

`real_lift_control.launch.py` starts the guarded executor and `lift_marker`, and defaults to `dry_run:=true execution_enabled:=false rviz:=false`. `rviz_only.launch.py` starts only the RViz frontend for a remote machine on domain 42. For normal use, prefer the 5-DoF bringup in [stretch_custom_manipulation](../stretch_custom_manipulation/).
