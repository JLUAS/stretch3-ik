import unittest
from types import SimpleNamespace as NS
import numpy as np
from rgbd.types import Camera, Calibration, Frame, Detection, rotate_calibration
from rgbd_perception.geometry import measure_object
from rgbd_perception.conversions import optical_rotation

class ObjectGeometryTests(unittest.TestCase):
    def test_object_transform_and_width_for_every_internal_rotation(self):
        camera=Camera(8,6,np.array([[100.,0,3.],[0,100.,2.],[0,0,1.]]),np.zeros(5),'plumb_bob')
        native_mask=np.zeros((6,8),bool);native_mask[1:5,2:7]=True
        depth=np.ones((6,8),np.float32)
        # An off-center RGB extrinsic catches wrong translation/rotation order.
        extrinsic=np.eye(4);extrinsic[:3,3]=[.012,-.007,.02]
        for turns in range(4):
            calibration=rotate_calibration(Calibration(camera,camera,extrinsic),turns)
            frame=Frame(7,np.zeros((calibration.color.height,calibration.color.width,3),np.uint8),
                        np.ascontiguousarray(np.rot90(depth,-turns)))
            detection=Detection(np.rot90(native_mask,-turns),np.zeros(3),np.zeros(3))
            value=measure_object(detection,frame,calibration,turns)
            # Native center X=.01, Y=.005 before the depth->RGB translation.
            np.testing.assert_allclose(value.camera_from_object[:3,3],[.022,-.002,1.02],atol=1e-7)
            np.testing.assert_allclose(value.camera_from_object[:3,:3],optical_rotation(turns)[:3,:3].T)
            expected=[.04,.03,0.] if turns%2==0 else [.03,.04,0.]
            np.testing.assert_allclose(value.size,expected,atol=1e-7)
            self.assertAlmostEqual(value.horizontal_width,expected[0],places=6)
            self.assertEqual(value.visible_points,20)

    def test_invalid_depth_is_excluded_and_no_geometry_is_invented(self):
        camera=Camera(2,2,np.eye(3),np.zeros(5),'plumb_bob')
        calibration=Calibration(camera,camera,np.eye(4))
        frame=Frame(1,np.zeros((2,2,3),np.uint8),np.array([[0,np.nan],[1,1]],np.float32))
        detection=Detection(np.ones((2,2),bool),np.zeros(3),np.zeros(3))
        self.assertIsNone(measure_object(detection,frame,calibration,0))

if __name__=='__main__':unittest.main()
