import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from stretch_perception_interfaces.msg import ObjectObservation
from std_msgs.msg import Header

path=Path(__file__).parents[1]/'scripts/object_localizer.py'
sys.path.insert(0,str(path.parent))
spec=importlib.util.spec_from_file_location('object_localizer',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def test_localization_does_not_mutate_the_source_color_header():
    seg=ObjectObservation(header=Header(frame_id='camera_color_optical_frame'),valid=True)
    seg.header.stamp.sec=12
    seg.mask.header=Header(frame_id='camera_color_optical_frame')
    outputs=[]
    node=NS(frame='odom',localize=lambda mask,obs:'',obs_pub=NS(publish=outputs.append))
    module.ObjectLocalizer.on_segmentation(node,seg)
    assert outputs[0].header.frame_id=='odom'
    assert seg.header.frame_id=='camera_color_optical_frame'
    assert seg.mask.header.frame_id=='camera_color_optical_frame'
    assert outputs[0].header.stamp==seg.header.stamp


def test_dotted_rgbd_geometry_mask_keeps_its_points():
    # The rgbd_geometry mask is a lattice of depth-owned pixels (1 px gaps),
    # as captured on the robot: two erosion steps used to leave nothing.
    import numpy as np
    from sensor_msgs.msg import Image
    h, w = 120, 160
    mask = np.zeros((h, w), np.uint8)
    mask[40:80:2, 60:100] = 255
    mask[40:80, 60:100:3] = 0
    depth = np.full((h, w), 450, np.uint16)
    stamp = Header(frame_id='camera_color_optical_frame')
    stamp.stamp.sec = 5
    msg = Image(header=stamp, height=h, width=w, encoding='mono8', step=w, data=mask.tobytes())
    dimg = Image(header=stamp, height=h, width=w, encoding='16UC1', step=2 * w, data=depth.tobytes())
    node = NS(info=NS(k=[600.0, 0, 80, 0, 600.0, 60, 0, 0, 1]), depths=[dimg], max_skew=0.1,
              frame='odom', tf=NS(lookup_transform=lambda *a, **k: (_ for _ in ()).throw(
                  module.TransformException('stop after the depth step'))))
    for source, expect_points in (('rgbd_geometry', True), ('sam2', False)):
        reason = module.ObjectLocalizer.localize(node, msg, ObjectObservation(source=source))
        assert ('valid depth points' not in reason) == expect_points, (source, reason)
