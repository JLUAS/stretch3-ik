"""Run after sourcing ROS and the built package; no camera or robot required."""
from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import Header
from rgbd_perception.node import PerceptionNode, image_message


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'ocr_enabled:=false'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = PerceptionNode()
        self.answers = []
        self.node.observation_pub = NS(publish=self.answers.append)
        self.node.mask_pub = self.node.overlay_pub = NS(publish=lambda m: None)

    def tearDown(self):
        self.node.destroy_node()

    def color(self):
        header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id='camera_color_optical_frame')
        return image_message(header, np.zeros((6,8,3),np.uint8),'rgb8')

    def test_no_image_and_full_queue_each_produce_invalid_answer(self):
        self.node.on_prompt(Point(x=1.,y=1.))
        self.assertFalse(self.answers[-1].valid)
        self.node.color = self.color()
        for _ in range(9):self.node.on_prompt(Point(x=1.,y=1.))
        self.assertEqual(len(self.node.pending),8)
        self.assertEqual(len(self.answers),2)
        self.assertIn('queue full',self.answers[-1].reason)
        self.node.clear('reset')
        self.assertEqual(len(self.answers),10)
        self.assertEqual(len({a.object_id for a in self.answers}),10)

    def test_prompt_freezes_image_and_preserves_headers_mask_and_empty_cloud(self):
        color=self.color();self.node.color=color
        self.node.on_prompt(Point(x=3.,y=2.))
        self.node.color=self.color()
        self.node.cache.append((color,None));self.node.preview_result=(None,None)
        labels=np.full((6,8),-1);labels[1:4,2:5]=0;labels[1:4,6:8]=1
        with patch('rgbd_perception.node.native_labels',return_value=labels):
            self.node.handle_prompt()
        answer=self.answers[-1]
        self.assertTrue(answer.valid)
        self.assertEqual(answer.header,color.header)
        self.assertEqual(answer.mask.header,color.header)
        self.assertEqual((answer.mask.width,answer.mask.height,answer.mask.step),(8,6,8))
        self.assertEqual(answer.mask.encoding,'mono8')
        np.testing.assert_array_equal(np.frombuffer(answer.mask.data,np.uint8).reshape(6,8),(labels==0)*255)
        self.assertEqual(answer.cloud.width,0)
        self.assertEqual(answer.centroid.x,0.)
        self.assertEqual(len(self.node.pending),0)

    def test_invalid_prompt_and_processing_exception_always_answer(self):
        color=self.color();self.node.color=color
        self.node.cache.append((color,None));self.node.preview_result=(None,None)
        self.node.on_prompt(Point(x=-1.,y=0.))
        with patch('rgbd_perception.node.native_labels',return_value=np.zeros((6,8),int)):
            self.node.handle_prompt()
        self.assertFalse(self.answers[-1].valid)
        self.assertEqual(len(self.answers[-1].mask.data),0)
        self.node.on_prompt(Point(x=1.,y=1.))
        with patch('rgbd_perception.node.native_labels',side_effect=RuntimeError('test failure')):
            self.node.handle_prompt()
        self.assertEqual(self.answers[-1].reason,'test failure')

    def test_parameters_are_validated_and_transport_settings_read_only(self):
        self.assertFalse(self.node.set_parameters([Parameter('appearance.features',value=0)])[0].successful)
        self.assertFalse(self.node.set_parameters([Parameter('rotations',value=2)])[0].successful)
        self.assertTrue(self.node.set_parameters([Parameter('appearance.features',value=100)])[0].successful)

    def test_tf_pose_uses_capture_time_and_rotated_rgb_optical_axes(self):
        color=self.color();self.node.pose_source='tf'
        transform=TransformStamped()
        transform.transform.rotation.w=1.
        transform.transform.translation.x=2.
        with patch.object(self.node.tf,'lookup_transform',return_value=transform) as lookup:
            pose=self.node.pose(NS(timestamp_ns=123),color,smooth=False)
        np.testing.assert_allclose(pose.reference_to_camera[:3,3],[0,2,0],atol=1e-12)
        self.assertEqual(pose.reference_frame,'odom')
        self.assertEqual(lookup.call_args.args[0:2],(color.header.frame_id,'odom'))
        self.assertEqual(lookup.call_args.args[2].nanoseconds,
            color.header.stamp.sec*10**9+color.header.stamp.nanosec)

    def test_typed_geometry_has_camera_frame_units_and_no_inferred_orientation(self):
        from rgbd.types import Camera, Calibration, Frame, Detection
        from rgbd_perception.conversions import optical_rotation
        color=self.color();self.node.rotations=0
        camera=Camera(8,6,np.array([[100.,0,3.],[0,100.,2.],[0,0,1.]]),np.zeros(5),'plumb_bob')
        self.node.calibration=Calibration(camera,camera,np.eye(4))
        mask=np.zeros((6,8),bool);mask[1:5,2:7]=True
        frame=Frame(1,np.zeros((6,8,3),np.uint8),np.ones((6,8),np.float32))
        detection=Detection(mask,np.zeros(3),np.zeros(3))
        messages=[];self.node.objects_pub=NS(publish=messages.append)
        # No reference pose/track: current measured camera geometry is still available.
        result=NS(tracks=[],segmentation=NS(detections=[detection]))
        self.node.publish_objects(color.header,frame,result,None)
        obj=messages[-1].objects[0]
        self.assertEqual(messages[-1].header,color.header)
        self.assertEqual(obj.header,color.header)
        self.assertFalse(obj.has_instance_id);self.assertFalse(obj.orientation_estimated)
        self.assertAlmostEqual(obj.horizontal_width,.04,places=6)
        self.assertAlmostEqual(obj.pose.position.z,1.)
        self.assertAlmostEqual(obj.pose.orientation.w,1.)
        self.assertEqual(obj.visible_points,20)
        self.assertEqual(obj.size.z,0.)


if __name__=='__main__':unittest.main()
