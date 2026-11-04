"""Measured visible object frames; no inferred hidden shape or physical rotation."""
from dataclasses import dataclass
import numpy as np
from .conversions import optical_rotation

@dataclass
class Geometry:
    camera_from_object: np.ndarray
    size: np.ndarray
    visible_points: int

    @property
    def horizontal_width(self):
        return float(self.size[0])

def measure_object(detection, frame, calibration, rotations):
    y,x=np.where(detection.mask)
    z=frame.depth[y,x]
    good=np.isfinite(z)&(z>0)
    x,y,z=x[good],y[good],z[good]
    if len(z)<3:
        return None
    k=calibration.depth.K
    points=np.column_stack(((x-k[0,2])*z/k[0,0],(y-k[1,2])*z/k[1,1],z))
    t=calibration.depth_to_color
    points=np.einsum('ij,kj->ki',t[:3,:3],points)+t[:3,3]
    points=points[np.isfinite(points).all(axis=1)&(points[:,2]>0)]
    if len(points)<3:
        return None
    lo,hi=points.min(axis=0),points.max(axis=0)
    native_from_upright=optical_rotation(rotations).T
    pose=native_from_upright.copy()
    pose[:3,3]=native_from_upright[:3,:3]@((lo+hi)/2)
    return Geometry(pose,hi-lo,len(points))
