# Vision segmentation: inputs and outputs

This is the contract of the segmentation node in the Stretch 3 grasping stack. The current node runs SAM 2 (`sam2_segmenter.py`), but any node that reads the same inputs and publishes the same outputs can replace it. Everything after the mask (depth, 3D points, grasp planning) already runs downstream, so the segmenter only has to answer one question: **which pixels of this color image are the object at this pixel?**

```
/camera/color/image_raw ──┐
                          ├──> [ segmenter ] ──> /perception/segmentation  (ObjectObservation)
prompt pixel (u, v) ──────┘                 └──> /perception/object_mask, /perception/segmentation_overlay (optional, for the operator)
```

## Inputs

### 1. Color image

| | |
|---|---|
| Topic | `/camera/color/image_raw` (ROS parameter `image_topic`) |
| Type | `sensor_msgs/msg/Image` |
| Source | Head RealSense D435i, color stream |
| Encoding | `rgb8` or `bgr8` |

Keep the latest frame. A prompt is always answered on the most recent image received before it.

### 2. Prompt: the pixel to segment

| | |
|---|---|
| Topics | `/camera/color/image_raw_mouse_left` (operator clicks in rqt_image_view) and `/perception/segment_prompt` (the grasp mission asks for a fresh mask) |
| Type | `geometry_msgs/msg/Point` |
| Fields | `x` = column `u`, `y` = row `v`, in pixels of the color image. `z` is unused. |

Both topics mean the same thing: "segment the object under this pixel". The first topic name is `<image topic>_mouse_left`, so it changes if `image_topic` changes.

## Output

### `/perception/segmentation` (required)

Type: `stretch_custom_manipulation/msg/ObjectObservation`

```
std_msgs/Header header
string object_id
string source
float32 confidence
bool valid
string reason
sensor_msgs/Image mask
sensor_msgs/PointCloud2 cloud
geometry_msgs/Point centroid
geometry_msgs/Vector3 size
```

What the segmenter fills:

| Field | Value |
|---|---|
| `header` | **Copy the header of the color image the mask was computed on, unchanged.** The stamp is how the next node finds the matching depth frame (it must be within 0.1 s). Do not use the current time. |
| `object_id` | Any unique string per answer. Current node: `obj-<sec>.<nanosec>` of the image stamp. |
| `source` | Name of your algorithm, e.g. `sam2`. Only used for logs. |
| `confidence` | Score in [0, 1]. The grasp mission refuses targets below **0.5** (parameter `min_confidence` of `grasp_mission`). |
| `valid` | `true` if a mask was found, `false` otherwise. |
| `reason` | Empty when valid. When invalid, one short line on why (e.g. `no object mask at the click`). It shows up in the mission logs. |
| `mask` | `sensor_msgs/Image`, encoding `mono8`, **same width and height as the color image**, `255` = object, `0` = background, `step = width`. Its header is the same color image header. Leave it empty when `valid` is false. |
| `cloud`, `centroid`, `size` | **Leave empty.** The localizer node fills them from the aligned depth. |

### `/perception/object_mask` and `/perception/segmentation_overlay` (optional)

For the operator only, nothing downstream depends on them.

- `/perception/object_mask`: the same `mono8` mask as above, as a plain `sensor_msgs/Image`.
- `/perception/segmentation_overlay`: `rgb8` copy of the color image with the mask tinted green.

## Rules the rest of the system relies on

1. **One answer per prompt, always.** If the click is outside the image, nothing is found, or the model crashes, publish `valid: false` with a `reason`. The mission waits for an answer and otherwise fails on a timeout.
2. **Answer within about 10 seconds.** The localizer keeps roughly 10 s of depth frames to match against the mask stamp. After that the matching depth is gone and the observation fails. The mission's own wait (`reobserve_timeout`) is 12 s. SAM 2 currently takes about 3 s on the robot's CPU.
3. **Mask the whole object at the click**, not a part of it (for example the label on a bottle). The current node takes the largest candidate mask that contains the clicked pixel, covers less than 40% of the image, and scores close to the best one.
4. **The mask must contain the clicked pixel's object only.** Background or a neighbor leaking into the mask becomes 3D points that change the grasp.
5. While one prompt is being processed, the current node drops new ones (it logs a warning). Queueing them instead is fine too, as long as rule 1 holds.

## What happens downstream (for context, nothing to build)

`object_localizer.py` takes the mask, reads the aligned depth (`/camera/aligned_depth_to_color/image_raw`) closest to the mask stamp, turns the masked pixels into 3D points in `odom`, cleans them, and republishes the full `ObjectObservation` on `/perception/object_observation`. The grasp mission only reads that topic.

## Two ways to plug in a new algorithm

**A. Replace the whole node.** Publish and subscribe exactly as above. In `bringup.launch.py` the segmenter is started as `sam2_segmenter.py` (node name `sam2_segmenter`), so the new executable goes there.

**B. Keep the node, swap only the model.** The node loads its model from the `backend` parameter, in the form `module:Class` (default `sam2_segmenter:Sam2Backend`). The class needs:

```python
class MyBackend:
    name = 'my_algo'                  # goes into ObjectObservation.source

    def __init__(self, node):         # rclpy node, for declare_parameter / logging
        ...

    def segment(self, rgb, click):
        # rgb:   numpy uint8 array, shape (H, W, 3), RGB order
        # click: (u, v) ints, column and row
        # return (mask, score): mask is a bool array (H, W), or None if nothing found; score in [0, 1]
        ...
```

With B, the node handles all the ROS side (topics, headers, invalid answers, overlay), so the backend code never touches ROS messages.
