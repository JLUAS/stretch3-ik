# Validation ? 4 November 2026

Tested on Stretch `stretch-se3-3075`, Ubuntu 22.04, ROS2 Humble, CPU inference.
The integration was built in a separate validation workspace; existing driver,
IK and executor installations were used as an underlay and were not replaced.

* Three affected packages built successfully: `stretch_perception_interfaces`,
  `stretch_rgbd_perception`, `stretch_custom_manipulation`.
* 49 native tests and six ROS adapter tests passed through `colcon test`.
* 47 grasp-planner, mission, perception-chain, localizer and launch compatibility
  tests passed through pytest, including capture-header isolation and inclusion
  of the new node with `start_camera=false`.
* Python syntax and the local package dependency graph were checked. No package
  cycle exists. Current launch/docs/code no longer depend on SAM or Torch.

Live perception-only launch: D435i at 848x480/30 Hz, internal clockwise
rotation, board pose, OCR and rqt enabled. No driver, executor, grasp mission or
physical motion was launched for validation. The attached desktop tmux session
is `rgbd_ros:perception`. Its current command uses the isolated validation
workspace and the existing verified perception interpreter.

Live native-color prompts produced explicit invalid answers and a valid mask.
Checks covered source capture stamps, mask/header equality, mono8 encoding,
848x480 dimensions, binary pixel values and an empty downstream cloud. The
live typed geometry stream contained six current objects, finite positive
camera depths, unit quaternions, metric widths consistent with `size.x`, and
`orientation_estimated=false`. Observed widths were approximately 51?74 mm.
Five products were recognized; one remained unknown.

A separate 15-second metrics-only sample (168 processed frames) measured:

| Measurement | Result |
| --- | ---: |
| Median rolling throughput | 11.28 FPS |
| Minimum rolling throughput in sample | 10.76 FPS |
| Median adapter processing | 65.99 ms |
| Median source-image age at processing start | 54.30 ms |
| Object count | 6 |
| OCR errors | 0 |

The interface probe's extra reliable full-resolution RGB subscription reduced
throughput to about 8.5 FPS in one run. It is additional observer load and is
not included in the normal-load sample above. Tests do not establish a general
throughput guarantee with Nav2, OctoMap and other robot workloads running.

Live capture-time TF mode and physical grasp execution remain to be tested in
the complete stack. Synthetic tests cover TF direction, capture timestamps and
all four image rotations. Visible extents are not hidden-shape reconstruction;
object axes are calibrated camera axes, not estimated package orientation.
