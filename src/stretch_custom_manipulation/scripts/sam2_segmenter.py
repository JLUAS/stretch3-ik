#!/usr/bin/env python3
"""Click or prompt -> object mask on the head D435i color image.

Click the object in rqt_image_view (topic /camera/color/image_raw, enable
"publish click location"); rqt_image_view publishes the pixel as a plain
geometry_msgs/Point on `<image topic>_mouse_left`. The grasp mission asks for
a fresh mask the same way on /perception/segment_prompt.

The model sits behind `segment(rgb, (u, v)) -> (mask or None, score)`; the
`backend` parameter (`module:Class`) picks it, SAM 2 by default. Downstream
nodes only see ObjectObservation on /perception/segmentation (mask, score,
validity) and never a backend type. Also publishes the mono8 mask and an
overlay for the operator. SAM 2 runs on CPU (Stretch NUC); start the node
with the Python of the SAM 2 virtualenv.
"""
import importlib
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from stretch_custom_manipulation.msg import ObjectObservation


def to_rgb(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    rgb = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return rgb[:, :, ::-1].copy() if msg.encoding == 'bgr8' else rgb.copy()


def image_msg(header, array, encoding):
    msg = Image()
    msg.header = header
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.step = msg.width * (3 if encoding == 'rgb8' else 1)
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


def pick_mask(masks, scores, click, max_fraction=0.4, score_window=0.15):
    """Whole object at the click: SAM returns part, sub-part and whole masks,
    and a part (a logo) can score highest. Take the largest mask that contains
    the click, is not most of the image, and scores near the best."""
    u, v = click
    ok = [i for i in range(len(masks))
          if masks[i][v, u] and masks[i].mean() < max_fraction]
    if not ok:
        return None
    best = max(scores[i] for i in ok)
    near = [i for i in ok if scores[i] >= best - score_window]
    return masks[max(near, key=lambda i: masks[i].sum())].astype(bool)


class Sam2Backend:
    name = 'sam2'

    def __init__(self, node):
        checkpoint = node.declare_parameter('checkpoint', '').value
        config = node.declare_parameter('model_config', 'configs/sam2.1/sam2.1_hiera_t.yaml').value
        threads = node.declare_parameter('torch_threads', 4).value
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        torch.set_num_threads(threads)  # leave cores for Nav2 and the camera
        if checkpoint:
            self.predictor = SAM2ImagePredictor(build_sam2(config, checkpoint, device='cpu'))
        else:
            self.predictor = SAM2ImagePredictor.from_pretrained(
                'facebook/sam2.1-hiera-tiny', device='cpu')

    def segment(self, rgb, click):
        u, v = click
        self.predictor.set_image(rgb)
        masks, scores, _ = self.predictor.predict(
            point_coords=np.array([[u, v]], np.float32),
            point_labels=np.array([1], np.int32), multimask_output=True)
        mask = pick_mask(masks, scores, (u, v))
        if mask is None:
            return None, 0.0
        best = max(float(s) for m, s in zip(masks, scores) if np.array_equal(m.astype(bool), mask))
        return mask, best


def load_backend(spec, node):
    module, cls = spec.split(':')
    return getattr(importlib.import_module(module), cls)(node)


class Sam2Segmenter(Node):
    def __init__(self):
        super().__init__('sam2_segmenter')
        image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
        spec = self.declare_parameter('backend', 'sam2_segmenter:Sam2Backend').value
        self.backend = load_backend(spec, self)
        self.image = None
        self.busy = threading.Lock()
        self.create_subscription(Image, image_topic, self.on_image, 1)
        self.create_subscription(Point, image_topic + '_mouse_left', self.on_click, 1)
        self.create_subscription(Point, '/perception/segment_prompt', self.on_click, 1)
        self.obs_pub = self.create_publisher(ObjectObservation, '/perception/segmentation', 1)
        self.mask_pub = self.create_publisher(Image, '/perception/object_mask', 1)
        self.overlay_pub = self.create_publisher(Image, '/perception/segmentation_overlay', 1)
        self.get_logger().info(f'{self.backend.name} ready; click objects on {image_topic}')

    def on_image(self, msg):
        self.image = msg

    def on_click(self, click):
        image = self.image
        if image is None:
            self.get_logger().warn('Click ignored: no color image yet')
            return
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn('Click ignored: previous segmentation running')
            return
        threading.Thread(target=self.segment, args=(image, click), daemon=True).start()

    def publish(self, image, mask, score, reason=''):
        stamp = image.header.stamp
        obs = ObjectObservation(header=image.header, source=self.backend.name,
                                object_id=f'obj-{stamp.sec}.{stamp.nanosec:09d}',
                                confidence=float(score), valid=mask is not None, reason=reason)
        if mask is not None:
            obs.mask = image_msg(image.header, mask.astype(np.uint8) * 255, 'mono8')
            self.mask_pub.publish(obs.mask)
        self.obs_pub.publish(obs)

    def segment(self, image, click):
        try:
            rgb = to_rgb(image)
            u = int(round(click.x))
            v = int(round(click.y))
            if not (0 <= u < image.width and 0 <= v < image.height):
                self.publish(image, None, 0.0, f'click ({u}, {v}) outside the image')
                return
            start = time.monotonic()
            mask, score = self.backend.segment(rgb, (u, v))
            elapsed = time.monotonic() - start
            if mask is None:
                self.publish(image, None, 0.0, 'no object mask at the click')
                return
            self.get_logger().info(
                f'Mask {int(mask.sum())} px at ({u}, {v}), score {score:.2f}, in {elapsed:.2f} s')
            self.publish(image, mask, score)
            overlay = rgb.copy()
            overlay[mask] = (0.5 * overlay[mask] + [0, 127, 0]).astype(np.uint8)
            self.overlay_pub.publish(image_msg(image.header, overlay, 'rgb8'))
        except Exception as e:  # keep the node alive for the next click
            self.get_logger().error(f'Segmentation failed: {e}')
            self.publish(image, None, 0.0, f'segmentation failed: {e}')
        finally:
            self.busy.release()


def main():
    rclpy.init()
    node = Sam2Segmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
