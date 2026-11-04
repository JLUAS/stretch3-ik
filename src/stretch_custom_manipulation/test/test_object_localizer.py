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
