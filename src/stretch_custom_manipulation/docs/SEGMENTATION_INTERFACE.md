# Segmentation and object geometry interface

The current segmenter is the single `rgbd_perception` node in
`stretch_rgbd_perception`. It performs depth geometry segmentation, appearance
association, persistent instance tracking and asynchronous product OCR.

The complete input/output, frame, width, prompt and parameter contracts are in
[the perception package README](../../stretch_rgbd_perception/README.md).

The mission's request/response topics are unchanged. Native pixel prompts on
`/perception/segment_prompt` and `/camera/color/image_raw_mouse_left` produce
one `/perception/segmentation` answer, copying the source color header and
dimensions. The upright preview's click topic is `/perception/preview_mouse_left`.
The answer type is now `stretch_perception_interfaces/msg/ObjectObservation`;
all clients must rebuild/update imports. `object_localizer.py` fills its cloud
from matching aligned depth and publishes `/perception/object_observation`.

Current camera-relative 3D poses, visible extents, horizontal widths, IDs and
product labels are on `/perception/objects`, using
`stretch_perception_interfaces/msg/TrackedObjectArray`. Pose rotation specifies
upright camera-aligned axes, not inferred physical package orientation.
Grasp aperture remains the measured cloud extent along the selected finger
axis plus clearance; camera-horizontal width is not substituted for it.
