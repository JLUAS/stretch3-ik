import importlib.util
import math
from pathlib import Path
from geometry_msgs.msg import PoseStamped

path = Path(__file__).parents[1] / 'scripts' / 'pick_state_machine.py'
spec = importlib.util.spec_from_file_location('pick_state_machine', path)
sm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sm)

def test_compute_pregrasp_pose():
    target = PoseStamped()
    target.pose.position.x = 1.0
    target.pose.position.y = 0.0
    target.pose.position.z = 1.0
    # Identity orientation: the TCP approach vector is its X-axis.
    target.pose.orientation.w = 1.0
    
    offset = 0.1
    pregrasp = sm.compute_pregrasp_pose(target, offset)
    
    assert math.isclose(pregrasp.pose.position.x, 0.9)
    assert math.isclose(pregrasp.pose.position.y, 0.0)
    assert math.isclose(pregrasp.pose.position.z, 1.0)

def test_compute_retract_pose():
    target = PoseStamped()
    target.pose.position.z = 1.0
    retract_height = 0.2
    
    retract = sm.compute_retract_pose(target, retract_height)
    assert math.isclose(retract.pose.position.z, 1.2)
