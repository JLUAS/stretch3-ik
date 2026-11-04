"""Exercise the live segmenter interface only; never invoke a grasp or motion."""
import json
import time
import statistics
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from stretch_perception_interfaces.msg import ObjectObservation, TrackedObjectArray


def main():
    rclpy.init();node=rclpy.create_node('rgbd_interface_probe')
    rows=[];answers=[];stamps=set();geometry=[]
    node.create_subscription(String,'/perception/tracks',lambda m:rows.append(json.loads(m.data)),1)
    node.create_subscription(Image,'/camera/color/image_raw',lambda m:stamps.add((m.header.stamp.sec,m.header.stamp.nanosec)),QoSProfile(depth=60))
    node.create_subscription(ObjectObservation,'/perception/segmentation',answers.append,10)
    node.create_subscription(TrackedObjectArray,'/perception/objects',geometry.append,1)
    pub=node.create_publisher(Point,'/perception/segment_prompt',10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def prompt(x,y):
        count=len(answers);started=time.monotonic();pub.publish(Point(x=float(x),y=float(y)))
        while len(answers)==count and time.monotonic()-started<5:rclpy.spin_once(node,timeout_sec=.05)
        assert len(answers)==count+1,'prompt did not receive exactly one answer'
        answer=answers[-1]
        print('PROMPT',x,y,'VALID',answer.valid,'REASON',answer.reason,'SECONDS',round(time.monotonic()-started,3),flush=True)
        return answer
    try:
        spin(6);assert rows,'no pipeline output'
        invalid=prompt(-1,-1);assert not invalid.valid and invalid.reason and not invalid.mask.data
        # Native color width=848,height=480; preview is its clockwise rotation.
        # Use native prompt coordinates, translating the rotated preview boxes.
        valid=None
        for track in rows[-1]['tracks']:
            box=track.get('preview_box')
            if not box:continue
            x0,y0,x1,y1=box
            from rgbd_perception.conversions import preview_to_native
            for fraction in (.5,.35,.65):
                x,y=preview_to_native(x0+(x1-x0)*fraction,(y0+y1)/2,848,480,1)
                answer=prompt(x,y)
                if answer.valid:valid=answer;break
            if valid is not None:break
        assert valid is not None,'no valid mask at tested object interiors'
        assert valid.mask.encoding=='mono8' and valid.mask.step==848
        assert (valid.mask.width,valid.mask.height)==(848,480)
        assert valid.header==valid.mask.header
        spin(.5)  # Image and observation are different DDS streams; drain both.
        assert (valid.header.stamp.sec,valid.header.stamp.nanosec) in stamps
        pixels=np.frombuffer(valid.mask.data,np.uint8)
        assert set(np.unique(pixels))=={0,255} and not valid.cloud.data
        spin(4)
        print('MEDIAN FPS',statistics.median(r['fps'] for r in rows[-50:]),
              'MEDIAN PROCESSING MS',statistics.median(r['processing_ms'] for r in rows[-50:]),
              'MEDIAN INPUT AGE MS',statistics.median(r['input_age_ms'] for r in rows[-50:]),flush=True)
        print('LAST STATE',json.dumps(rows[-1]),flush=True)
        assert geometry and geometry[-1].objects,'no typed camera-relative geometry'
        for obj in geometry[-1].objects:
            assert obj.header==geometry[-1].header
            assert obj.header.frame_id=='camera_color_optical_frame'
            assert obj.visible_points>=3 and not obj.orientation_estimated
            assert 0<obj.horizontal_width<.3 and abs(obj.horizontal_width-obj.size.x)<1e-6
            q=obj.pose.orientation
            assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-6
            assert np.isfinite([obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]).all()
            assert obj.pose.position.z>0
        print('OBJECT WIDTHS M',[(o.instance_id,round(o.horizontal_width,4),o.product) for o in geometry[-1].objects],flush=True)
        print('LIVE INTERFACE AND GEOMETRY PASS',flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
