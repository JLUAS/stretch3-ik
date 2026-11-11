"""Occupancy map of house.xml straight from its geometry (no SLAM run needed).

Run with the stretch_mujoco venv:
  ~/cross_embodiment_sim/external/stretch_mujoco/.venv/bin/python make_map.py <house.xml symlink in models/> <out dir>
Writes map.pgm + map.yaml; map frame == MuJoCo world frame.
Every static box (world body) that spans the lidar/base height band is an obstacle.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

RES, BAND = 0.05, (0.05, 0.4)


def main(xml, out):
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    boxes = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX or m.geom_contype[g] == 0:
            continue
        c, h = d.geom_xpos[g], m.geom_size[g]
        assert np.allclose(d.geom_xmat[g].reshape(3, 3), np.eye(3)), 'axis-aligned boxes only'
        if c[2] - h[2] < BAND[1] and c[2] + h[2] > BAND[0]:
            boxes.append((c[:2] - h[:2], c[:2] + h[:2]))
    lo = np.min([b[0] for b in boxes], axis=0) - 0.5
    hi = np.max([b[1] for b in boxes], axis=0) + 0.5
    w, h = np.ceil((hi - lo) / RES).astype(int)
    occ = np.zeros((h, w), bool)
    for a, b in boxes:
        i0, j0 = np.floor((a - lo) / RES).astype(int)
        i1, j1 = np.ceil((b - lo) / RES).astype(int)
        occ[j0:j1, i0:i1] = True
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]  # pgm row 0 is the top (max y)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / 'map.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode() + img.tobytes())
    (out / 'map.yaml').write_text(
        f'image: map.pgm\nresolution: {RES}\norigin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    base = d.body('base_link').xpos
    print(f'{len(boxes)} obstacles, {w}x{h} cells, robot base_link at ({base[0]:.3f}, {base[1]:.3f})')


if __name__ == '__main__':
    main(*sys.argv[1:3])
