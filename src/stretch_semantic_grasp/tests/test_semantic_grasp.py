import math

import numpy as np
import pytest

from semantic_grasp.geometry import backproject, box_depth, upright, upright_to_native
from semantic_grasp.memory import Keyframe, Memory, locate
from semantic_grasp.pregrasp import base_for, pregrasp_poses
from semantic_grasp.text import parse_command

ROOMS = ('kitchen', 'living room', 'bedroom')


@pytest.mark.parametrize('text,phrase,room', [
    ('Grab the red mug in the kitchen', 'a red mug', 'kitchen'),
    ('pick up the green bottle.', 'a green bottle', None),
    ('bring me the cup on the table', 'a cup on the table', None),
    ('please fetch a purple cup from the bedroom', 'a purple cup', 'bedroom'),
    ('blue box in the living room', 'a blue box', 'living room'),
])
def test_parse(text, phrase, room):
    q = parse_command(text, ROOMS)
    assert (q.phrase, q.room) == (phrase, room)


def test_parse_empty():
    with pytest.raises(ValueError):
        parse_command('grab the', ROOMS)


def test_upright_round_trip():
    img = np.arange(4 * 6).reshape(4, 6)
    up = upright(img)
    for r in range(4):
        for c in range(6):
            uu, uv = np.argwhere(up == img[r, c])[0][::-1]   # upright (col, row)
            assert upright_to_native(uu, uv, 4) == (c, r)


def test_backproject_and_depth():
    k = [[100, 0, 50], [0, 100, 40], [0, 0, 1]]
    assert np.allclose(backproject(150, 40, 2.0, k), [2.0, 0.0, 2.0])
    d = np.zeros((10, 10)); d[3:7, 3:7] = 1.5
    assert box_depth(d, (2, 2, 8, 8)) == 1.5
    assert box_depth(np.zeros((10, 10)), (2, 2, 8, 8)) is None


def test_base_for_puts_object_on_the_right():
    for yaw in np.linspace(-3, 3, 7):
        x, y, _ = base_for((1.0, 2.0), yaw, 0.65, along=0.0)
        # object in base frame
        dx, dy = 1.0 - x, 2.0 - y
        bx = math.cos(yaw) * dx + math.sin(yaw) * dy
        by = -math.sin(yaw) * dx + math.cos(yaw) * dy
        assert bx == pytest.approx(0.0, abs=1e-9) and by == pytest.approx(-0.65)


def test_pregrasp_faces_counter_edge():
    # Counter fills x > 3.35; object 0.2 m past the edge. The best base must
    # stand off the counter with the arm pointing +x (yaw +90 deg).
    occupied = lambda x, y: x > 3.35
    free = lambda x, y, yaw: x < 3.35 - 0.25
    poses = pregrasp_poses((3.55, 0.0), (0.0, 0.0), free, occupied)
    x, y, yaw = poses[0]
    assert yaw == pytest.approx(math.pi / 2, abs=math.radians(10))
    assert abs(y) < 0.1
    # The stowed arm must not start inside the counter: edge >= ARM_CLEAR away.
    assert 3.35 - x >= 0.65 - 0.02


def test_pregrasp_none_when_blocked():
    assert pregrasp_poses((0, 0), (1, 1), lambda *a: False, lambda *a: True) == []


class FakeDetector:
    def __init__(self, box_up, score=0.9):
        self.box_up, self.score = box_up, score

    def detect(self, rgb, phrase, threshold):
        return [(self.score, self.box_up)] if 'mug' in phrase else []


def test_memory_locates_in_map_and_filters_room():
    h, w = 40, 60
    depth = np.full((h, w), 2.0)
    k = np.array([[50., 0, 30], [0, 50., 20], [0, 0, 1]])
    # Camera 1 m above map origin, optical z along map x.
    t = np.eye(4); t[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]; t[2, 3] = 1.0
    f = Keyframe(np.zeros((h, w, 3), np.uint8), depth, k, t)
    # Native centre pixel (30, 20) is upright (h-1-20, 30) = (19, 30).
    box_up = (17, 28, 21, 32)
    xyz, box = locate(f, box_up)
    assert box[0] < 30 < box[2] and box[1] < 20 < box[3]
    assert np.allclose(xyz, [2.0, 0.0, 1.0], atol=0.05)
    m = Memory(FakeDetector(box_up)); m.add(f)
    assert len(m.query('a red mug')) == 1
    assert m.query('a red mug', room=((-1, 1), (-1, 1))) == []
    assert m.query('a blue box') == []


def test_memory_save_load_round_trip(tmp_path):
    f = Keyframe(np.zeros((4, 6, 3), np.uint8), np.ones((4, 6)), np.eye(3), np.eye(4))
    m = Memory(None); m.add(f); m.add(f)
    m.save(tmp_path / 'mem.npz')
    m2 = Memory(None); m2.load(tmp_path / 'mem.npz')
    assert len(m2.frames) == 2 and np.array_equal(m2.frames[1].depth, f.depth)
