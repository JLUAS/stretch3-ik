import importlib.util
import math
from pathlib import Path
from geometry_msgs.msg import PoseStamped

path = Path(__file__).parents[1] / 'scripts' / 'place_state_machine.py'
spec = importlib.util.spec_from_file_location('place_state_machine', path)
sm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sm)

def test_compute_retract_pose():
    target = PoseStamped()
    target.pose.position.z = 1.0
    retract_height = 0.3
    
    retract = sm.compute_retract_pose(target, retract_height)
    assert math.isclose(retract.pose.position.z, 1.3)
