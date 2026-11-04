"""One serial owner of the shared pipeline; bounded ROS input and OCR work."""
from collections import deque
from array import array as byte_array
from copy import deepcopy
from dataclasses import asdict, replace
import json
from time import monotonic, perf_counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import Buffer, TransformListener, TransformException
from stretch_perception_interfaces.msg import ObjectObservation, TrackedObject, TrackedObjectArray
from scipy.spatial.transform import Rotation
from .geometry import measure_object

from rgbd.types import Calibration, Pose, rotate_calibration
from rgbd.detection.pipeline import DetectionPipeline
from rgbd.detection.tabletop import TabletopSegmenter, Parameters as SegmentationParameters
from rgbd.pose.board import BoardPoseEstimator
from rgbd.pose.filter import PoseFilter
from rgbd.classification.worker import ObjectClassifier
from rgbd.demos.detection import draw
from .conversions import (stamp_ns, frame_from_messages, camera_from_info,
                          transform_matrix, optical_rotation, native_labels,
                          select_mask, preview_to_native)


def image_message(header, array, encoding):
    array = np.ascontiguousarray(array)
    msg = Image(header=deepcopy(header), height=array.shape[0], width=array.shape[1],
                encoding=encoding, is_bigendian=0, step=array.strides[0])
    # ROS's bytes setter validates every element in Python. array('B') takes
    # the generated message's fast path and avoids a full-frame Python loop.
    msg.data = byte_array('B', array.tobytes())
    return msg


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('rgbd_perception')
        def startup(name, value):
            return self.declare_parameter(name, value, ParameterDescriptor(read_only=True)).value
        self.image_topic = startup('image_topic', '/camera/color/image_raw')
        self.depth_topic = startup('depth_topic', '/camera/depth/image_rect_raw')
        self.rotations = startup('rotations', 1)
        self.pose_source = startup('pose_source', 'board')
        self.reference = startup('reference_frame', 'odom')
        self.max_skew = startup('max_pair_skew', .015)
        self.max_age = startup('max_frame_age', .5)
        self.ocr_enabled = startup('ocr_enabled', True)
        if self.rotations not in (0, 1, 2, 3) or self.pose_source not in ('board', 'tf', 'none'):
            raise ValueError('rotations must be 0..3; pose_source must be board, tf or none')
        if self.max_skew <= 0 or self.max_age <= 0:
            raise ValueError('frame age and pair skew must be positive')
        self.settings = {
            'segmentation.max_depth': 1.1, 'segmentation.plane_min_inliers': 300,
            'segmentation.max_object_footprint': .20,
            'appearance.features': 120, 'persistence.probation_period': 1.,
            'persistence.forget_after': 30., 'persistence.match_distance': .12,
            'persistence.continuity_distance': .06,
            'persistence.reappearance_penalty': .3, 'pose_filter.window': 3,
            'ocr.refresh_seconds': 5.}
        for name, value in self.settings.items():
            self.declare_parameter(name, value)
        validation = self.validate_parameters([])
        if not validation.successful:
            raise ValueError(validation.reason)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.color = None
        self.colors = deque(maxlen=4)
        self.depths = deque(maxlen=8)
        self.info = {}
        self.pending = deque()
        self.cache = deque(maxlen=4)
        self.pipeline = self.calibration = self.estimator = None
        self.smoother = PoseFilter()
        self.classifier = ObjectClassifier() if self.ocr_enabled else None
        self.previous_stamp = None
        self.sequence = self.processed = self.skipped = 0
        self.last_received = monotonic()
        self.finished = deque(maxlen=100)
        self.state = {'status': 'waiting for images/calibration', 'objects': 0}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self.on_color, qos)
        self.create_subscription(Image, self.depth_topic, self.depths.append, qos)
        for key, topic in [('color', startup('color_info_topic', '/camera/color/camera_info')),
                           ('depth', startup('depth_info_topic', '/camera/depth/camera_info'))]:
            self.create_subscription(CameraInfo, topic, lambda m, k=key: self.on_info(k, m), qos)
        self.create_subscription(Point, self.image_topic + '_mouse_left', self.on_prompt, 10)
        self.create_subscription(Point, '/perception/segment_prompt', self.on_prompt, 10)
        self.create_subscription(Point, '/perception/preview_mouse_left', self.on_preview_prompt, 10)
        self.observation_pub = self.create_publisher(ObjectObservation, '/perception/segmentation', 10)
        self.mask_pub = self.create_publisher(Image, '/perception/object_mask', 1)
        self.overlay_pub = self.create_publisher(Image, '/perception/segmentation_overlay', 1)
        self.preview_pub = self.create_publisher(Image, '/perception/preview', 1)
        self.tracks_pub = self.create_publisher(String, '/perception/tracks', 1)
        self.objects_pub = self.create_publisher(TrackedObjectArray, '/perception/objects', 1)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.create_service(Trigger, '/perception/reset', self.reset)
        self.create_timer(1 / 30, self.tick)
        self.create_timer(1., self.diagnostics)
        cv2.setNumThreads(1)
        self.get_logger().info(f'Using shared rgbd library; pose={self.pose_source}, OCR={self.ocr_enabled}')

    def validate_parameters(self, changes):
        values = {n: self.get_parameter(n).value for n in self.settings}
        values.update({p.name: p.value for p in changes if p.name in values})
        try:
            for n, v in values.items():
                if not np.isfinite(v) or v < 0:
                    raise ValueError(f'{n} must be finite and nonnegative')
                if n not in ('persistence.probation_period', 'persistence.reappearance_penalty', 'ocr.refresh_seconds') and v == 0:
                    raise ValueError(f'{n} must be positive')
            SegmentationParameters(max_depth=values['segmentation.max_depth'],
                plane_min_inliers=values['segmentation.plane_min_inliers'],
                max_object_footprint=values['segmentation.max_object_footprint']).validate()
            return SetParametersResult(successful=True)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def on_info(self, key, msg):
        previous = self.info.get(key)
        self.info[key] = msg
        if previous is not None and (previous.width, previous.height, list(previous.k), list(previous.d), previous.header.frame_id) != (
                msg.width, msg.height, list(msg.k), list(msg.d), msg.header.frame_id):
            self.clear('calibration changed')
            self.pipeline = None

    def on_color(self, msg):
        if self.color is not None and stamp_ns(msg) < stamp_ns(self.color):
            self.clear('capture clock moved backward')
        if self.color is not None and self.previous_stamp != stamp_ns(self.color):
            self.skipped += 1
        self.color = msg
        self.colors.append(msg)
        self.last_received = monotonic()

    def depth_for(self, color):
        if not self.depths:
            return None
        depth = min(self.depths, key=lambda d: abs(stamp_ns(d) - stamp_ns(color)))
        return depth if abs(stamp_ns(depth) - stamp_ns(color)) <= self.max_skew * 1e9 else None

    def initialize(self):
        if self.pipeline is not None:
            return True
        if not all(k in self.info for k in ('color', 'depth')):
            return False
        color, depth = self.info['color'], self.info['depth']
        extrinsic = np.eye(4)
        if color.header.frame_id != depth.header.frame_id:
            extrinsic = transform_matrix(self.tf.lookup_transform(
                color.header.frame_id, depth.header.frame_id, Time()).transform)
        self.calibration = rotate_calibration(Calibration(camera_from_info(color),
            camera_from_info(depth), extrinsic), self.rotations)
        self.pipeline = DetectionPipeline(self.calibration)
        self.estimator = BoardPoseEstimator(self.calibration.color)
        return True

    def settings_update(self):
        groups = {'segmentation': self.pipeline.segmenter.parameters,
                  'appearance': self.pipeline.extractor.parameters,
                  'persistence': self.pipeline.persistor.parameters,
                  'pose_filter': self.smoother.parameters}
        if self.classifier:
            groups['ocr'] = self.classifier.parameters
        for name in self.settings:
            group, field = name.split('.')
            if group in groups:
                setattr(groups[group], field, self.get_parameter(name).value)

    def pose(self, frame, color, smooth=True):
        if self.pose_source == 'board':
            pose = self.estimator.estimate(frame)
        elif self.pose_source == 'tf':
            try:
                matrix = transform_matrix(self.tf.lookup_transform(color.header.frame_id,
                    self.reference, Time.from_msg(color.header.stamp)).transform)
                pose = Pose(frame.timestamp_ns, optical_rotation(self.rotations) @ matrix, self.reference)
            except TransformException:
                pose = None
        else:
            pose = None
        return self.smoother.update(pose) if smooth else pose

    def enqueue(self, msg, color):
        self.sequence += 1
        request = (self.sequence, color, (msg.x, msg.y), monotonic())
        if color is None or len(self.pending) >= 8:
            self.answer(request, None, 'no color image yet' if color is None else 'prompt queue full')
        else:
            self.pending.append(request)

    def on_prompt(self, msg):
        self.enqueue(msg, self.color)

    def on_preview_prompt(self, msg):
        color = self.cache[-1][0] if self.cache else None
        if color is not None:
            u, v = preview_to_native(msg.x, msg.y, color.width, color.height, self.rotations)
            msg = Point(x=float(u), y=float(v))
        self.enqueue(msg, color)

    def answer(self, request, mask, reason):
        sequence, color, click, received = request
        header = Header() if color is None else deepcopy(color.header)
        obs = ObjectObservation(header=header, object_id=f'obj-{header.stamp.sec}.{header.stamp.nanosec:09d}-{sequence}',
            source='rgbd_geometry', confidence=1. if mask is not None else 0., valid=mask is not None, reason=reason)
        # 1.0 is a binary geometry acceptance flag, NOT a probability or OCR score.
        if mask is not None:
            obs.mask = image_message(header, mask.astype(np.uint8) * 255, 'mono8')
            self.mask_pub.publish(obs.mask)
            from .conversions import image_array
            rgb = image_array(color)
            rgb[mask] = (rgb[mask].astype(float) * .5 + [0, 127, 0]).astype(np.uint8)
            self.overlay_pub.publish(image_message(header, rgb, 'rgb8'))
        self.observation_pub.publish(obs)
        self.get_logger().info(f'Prompt {sequence}: {"valid" if obs.valid else reason}')

    def handle_prompt(self):
        request = self.pending[0]
        sequence, color, click, received = request
        try:
            cached = None
            if self.cache and stamp_ns(self.cache[-1][0]) == stamp_ns(color):
                frame, segmentation = self.preview_result
                cached = native_labels(frame, segmentation, self.calibration, self.rotations)
            if cached is None:
                depth = self.depth_for(color)
                if depth is None or not self.initialize():
                    if monotonic() - received < .3:
                        return
                    raise ValueError('matching depth/calibration unavailable')
                self.settings_update()
                frame = frame_from_messages(color, depth, self.rotations)
                # Historical prompts never rewind the live tracker or its filter.
                segmenter = TabletopSegmenter(self.calibration, replace(self.pipeline.segmenter.parameters))
                segmentation = segmenter.segment(frame.depth, self.pose(frame, color, smooth=False))
                if segmentation.error:
                    raise ValueError(segmentation.error)
                cached = native_labels(frame, segmentation, self.calibration, self.rotations)
            age = (self.get_clock().now().nanoseconds - stamp_ns(color)) / 1e9 - (monotonic() - received)
            if age > self.max_age or age < -.1:
                raise ValueError('prompt image is stale or camera clock is incompatible')
            mask, reason = select_mask(cached, click)
            self.answer(request, mask, reason)
        except Exception as exc:
            self.answer(request, None, str(exc))
        self.pending.popleft()

    def tick(self):
        if self.pending:
            self.handle_prompt()
            return
        # Pick the newest COMPLETE pair, retaining a few color headers while
        # depth arrives. Never wait on an unmatched newest color and discard
        # an already complete pair behind it.
        color = next((c for c in reversed(self.colors)
            if (self.previous_stamp is None or stamp_ns(c) > self.previous_stamp)
            and self.depth_for(c) is not None), None)
        if color is None:
            return
        depth = self.depth_for(color)
        if depth is None:
            self.state['status'] = 'waiting for matching depth'
            return
        started = perf_counter()
        try:
            if not self.initialize():
                return
            self.settings_update()
            age = (self.get_clock().now().nanoseconds - stamp_ns(color)) / 1e9
            if age > self.max_age or age < -.1:
                self.state['status'] = 'stale frame or incompatible camera clock'
                return
            frame = frame_from_messages(color, depth, self.rotations)
            result = self.pipeline.process(frame, self.pose(frame, color))
            self.previous_stamp = frame.timestamp_ns
            labels = None
            if self.classifier:
                self.classifier.poll(result.tracks)
                self.pipeline.persistor.freeze_templates(self.classifier.memory.conflicts)
                self.classifier.submit(frame, result.tracks, result.observations, self.pipeline.persistor.elapsed_seconds)
                labels = self.classifier.memory.labels
            self.publish_objects(color.header, frame, result, labels)
            # Build native RGB masks only for prompted frames, not every live frame.
            preview = draw(frame, result.observations, tracks=result.tracks, labels=labels)
            now = monotonic()
            self.finished.append(now)
            fps = (len(self.finished)-1)/(self.finished[-1]-self.finished[0]) if len(self.finished)>1 else 0.
            self.state = dict(status=result.status, pose_ok=result.pose is not None,
                objects=len(result.segmentation.detections), fps=round(fps, 2),
                input_age_ms=round(age*1000, 2), pair_skew_ms=abs(stamp_ns(depth)-stamp_ns(color))/1e6,
                processing_ms=round((perf_counter()-started)*1000, 2), timings_ms=result.timings_ms,
                skipped=self.skipped, stamp_ns=frame.timestamp_ns, reference_frame='' if result.pose is None else result.pose.reference_frame,
                ocr_state='off' if self.classifier is None else self.classifier.state,
                ocr_error='' if self.classifier is None else self.classifier.error,
                tracks=[dict(id=t.id, confirmed=t.confirmed, visible=t.visible, detection_index=t.detection_index,
                    position=t.position.tolist(), missing_seconds=t.missing_seconds,
                    preview_box=None if t.detection_index is None else result.observations[t.detection_index].box,
                    label='' if not labels or t.id not in labels else labels[t.id].label) for t in result.tracks])
            text = f'{fps:.1f} FPS | {result.status} | OCR {self.state["ocr_state"]}'
            cv2.putText(preview, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 0), 3)
            cv2.putText(preview, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
            self.preview_pub.publish(image_message(color.header, cv2.cvtColor(preview, cv2.COLOR_BGR2RGB), 'rgb8'))
            self.state['processing_ms'] = round((perf_counter()-started)*1000, 2)
            # Store source frame + segmentation; avoid full RGB projection per frame.
            self.cache.clear()
            self.cache.append((color, None))
            self.preview_result = (frame, result.segmentation)
            self.tracks_pub.publish(String(data=json.dumps(self.state)))
            self.processed += 1
        except TransformException:
            self.state['status'] = 'waiting for depth-to-color TF'
        except Exception as exc:
            self.state['status'] = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(self.state['status'])

    def publish_objects(self, header, frame, result, labels):
        message = TrackedObjectArray(header=deepcopy(header))
        owners = {t.detection_index: t for t in result.tracks if t.visible}
        for index, detection in enumerate(result.segmentation.detections):
            measured = measure_object(detection, frame, self.calibration, self.rotations)
            if measured is None:
                continue
            obj = TrackedObject(header=deepcopy(header), orientation_estimated=False,
                horizontal_width=measured.horizontal_width, visible_points=measured.visible_points)
            owner = owners.get(index)
            if owner is not None:
                obj.has_instance_id, obj.instance_id, obj.confirmed = True, owner.id, owner.confirmed
                if labels and owner.id in labels:
                    obj.product = labels[owner.id].label
            pose = measured.camera_from_object
            obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = pose[:3, 3].tolist()
            q = Rotation.from_matrix(pose[:3, :3]).as_quat().tolist()
            obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w = q
            obj.size.x, obj.size.y, obj.size.z = measured.size.tolist()
            message.objects.append(obj)
        self.objects_pub.publish(message)

    def clear(self, reason):
        while self.pending:
            self.answer(self.pending.popleft(), None, reason)
        self.cache.clear()
        self.colors.clear()
        self.depths.clear()
        self.smoother.reset()
        if self.pipeline:
            self.pipeline.reset()
        if self.classifier:
            self.classifier.clear()
        self.previous_stamp = None
        self.finished.clear()

    def reset(self, request, response):
        self.clear('perception reset')
        response.success, response.message = True, 'Perception memory cleared; ID counter preserved'
        return response

    def diagnostics(self):
        state = dict(self.state)
        if monotonic()-self.last_received > 1.:
            state['status'] = 'camera color stream unavailable'
        good = (state.get('pose_ok', False) and not state.get('ocr_error')
                and state['status'] in ('tracked', 'acquired'))
        diagnostic = DiagnosticStatus(name='rgbd_perception', hardware_id='head_camera',
            level=DiagnosticStatus.OK if good else DiagnosticStatus.WARN,
            message=state['status'], values=[KeyValue(key=k, value=json.dumps(v)) for k,v in state.items() if k != 'tracks'])
        self.diagnostic_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[diagnostic]))
        self.get_logger().info(f'{state["status"]} | {state.get("fps", 0):.1f} FPS | {state.get("objects", 0)} objects | OCR {state.get("ocr_state", "loading")}')


def main():
    rclpy.init()
    node = None
    try:
        node = PerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.classifier:
                node.classifier.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
