# RGB-D perception

One `rgbd_perception` node owns detection, persistence and a bounded OCR worker.
The `rgbd` library is ordinary Python: the node and native `pipeline.py` import
the identical classes. The library source came from the validated native
`rgbd_sandbox` pipeline, including its ID-continuity fix. No model-based
segmentation, SAM or Torch is used. Native tests are included in `tests/`.

## Install and build

From the workspace root on Ubuntu 22.04 / ROS2 Humble:

```bash
./scripts/install_perception.sh
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash  # Robot driver/camera dependencies, if needed
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Python dependencies are in `requirements.txt` and `requirements-ocr.txt`.
`requirements-live.txt` adds pyrealsense2 only for the optional native camera
frontend. ROS acquisition uses the existing RealSense ROS driver.
Set `STRETCH_PERCEPTION_PYTHON` or the `python_executable` standalone launch
argument to use a different compatible interpreter. The default environment is
`~/venvs/stretch_perception/bin/python3`. Do not use `env -u PYTHONPATH` for ROS:
the sourced overlays supply the message modules.

## Launch

Perception only, camera and optional rqt, with **no motion or grasp nodes**:

```bash
ros2 launch stretch_rgbd_perception perception.launch.py pose_source:=board
```

This board mode matches the verified native setup. Use `start_camera:=false`
when the camera already runs; there must only be one camera owner. The serial
argument is optional and device selection is restricted to D435. Raw rectified
depth and color run at 848x480/30 Hz; aligned depth stays available downstream.

Unified real bringup automatically includes this node with TF pose and reuses
its camera. Existing dry-run defaults remain unchanged:

```bash
./src/stretch_custom_manipulation/scripts/run_manipulation.sh \
  sim:=false dry_run:=true perception_viewer:=true
```

`perception_pose_source:=tf|board|none`, `perception_ocr:=true|false`,
`perception_python:=<interpreter>` configure the integration. TF mode requires
capture-time `RGB optical <- odom`; no latest-pose substitution is made.
Missing pose still permits current detections and camera geometry, but IDs
cannot be confirmed and OCR labels cannot be retained without a reference.

In rqt select `/perception/preview`, set display rotation **0 degrees** and
enable clicks on `/perception/preview_mouse_left`. Saved rqt settings may retain
an old rotation/custom click topic: check both. The preview is upright, while
its stamp/frame identify the original color capture. Do not use its rotated
pixels for optical projection outside the dedicated click adapter.

## Data contract

`/perception/objects`: `stretch_perception_interfaces/msg/TrackedObjectArray`.
It contains **current detections only**, each with:

* Original RGB capture header, in the **unrotated RGB optical camera frame**.
* `has_instance_id`, persistent `instance_id`, `confirmed`, OCR `product`.
* `pose`: **object -> camera** transform. Apply `p_camera = R*p_object + t`.
* `size`: visible 3D bounding extents in object-frame meters.
* `horizontal_width = size.x`, plus `visible_points` and `orientation_estimated`.

Object axes follow the internally upright calibrated camera: X right, Y down,
Z forward. The origin is the visible bounding-box center. The rotation maps
these axes back into the original optical camera, so it includes the configured
image-axis rotation. **It is not estimated package orientation**:
`orientation_estimated` is false. Hidden surfaces are not reconstructed and
visible dimensions can underestimate the complete object.

This horizontal width is useful geometry, not an unconditional jaw command.
The existing planner computes width along each candidate's **finger-closing
axis** from the filtered cloud, adds clearance and checks the gripper limit.
Its pre-grasp poses, self-filter and collision checks remain authoritative.

`/perception/tracks`: JSON `std_msgs/String` with reference-frame track memory,
missing durations, labels, preview boxes, frame age and stage timings.
`/diagnostics`: ROS diagnostic status, FPS, skew, input age, pose/OCR state.
`/perception/preview`: upright annotations.
`/perception/reset` (`std_srvs/Trigger`): clears memories while retaining IDs.

## Click-to-grasp compatibility

`/perception/segment_prompt` and `<image_topic>_mouse_left` bind to the latest
native color image received before the prompt. The preview click topic binds
to the preview's source frame and inverts its pixel rotation. Every received
request is answered, including queue overflow, missing/stale images, failures,
and clicks on unknown depth. A frozen historical frame is independently
segmented if it is no longer cached; the live tracker is never rewound.

`/perception/segmentation`: `stretch_perception_interfaces/ObjectObservation`.
The fields and topic semantics of the old interface are preserved. This is a
**message-package migration**: rebuild the localizer, mission and any external
clients and update their imports from `stretch_custom_manipulation.msg` to
`stretch_perception_interfaces.msg`. Mixing old/new wire types is unsupported.

The output header and `mono8` mask copy the original color capture header;
mask dimensions match that color image. Cloud/centroid/size are left empty for
`object_localizer`. The localizer still matches aligned depth, applies TF,
removes the gripper envelope/outliers and produces the cloud in `odom` on
`/perception/object_observation`. The grasp mission consumes that unchanged
cloud contract. Answer IDs are unique per request; persistent IDs are separate.
Optional mask/overlay topics retain their previous names.

Mask projection uses measured depth ownership with no convex filling of grasp
masks. Confidence is a binary geometry acceptance flag (`1` valid, `0` invalid),
not a calibrated probability or OCR score. A hole under a click may be invalid.
The detector assumes one support plane and package-sized objects (20 cm visible
footprint by default); it does not replace arbitrary-scene segmentation.

## Parameters and bounded execution

`config/perception.yaml` defines the parameters. Transport, rotation, pose source
and OCR enablement require restart. Exposed segmentation, feature count,
persistence, pose-filter window and OCR refresh parameters are live adjustable.
For example: `ros2 param set /rgbd_perception appearance.features 120`.
Input buffers hold four color and eight depth frames. Only the newest complete
pair is processed; pair skew <=15 ms. The prompt queue is bounded at eight;
source captures older than 0.5 seconds are rejected. OCR has one outstanding
job and no backlog. Pipeline ownership and memory updates are serial.

## Tests and native execution

```bash
colcon test --packages-select stretch_rgbd_perception
colcon test-result --verbose
cd src/stretch_rgbd_perception
python -m unittest discover -s tests -t . -v
python pipeline.py /path/to/recording.db3 --start-frame 137
```

After sourcing the workspace, run `test/test_node.py` on an unused ROS domain.
`test/live_probe.py` sends valid/invalid segmentation prompts only and checks
the source headers and masks; do not run it alongside an armed grasp mission,
which can act on observations. Native live use requires `requirements-live.txt`.
Physical grasp execution is not validated merely by passing perception tests.

See [VALIDATION.md](VALIDATION.md) for the robot checks, measured performance
and remaining live TF/grasp validation boundary.
