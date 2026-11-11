"""Base pose that puts an object within side reach of the Stretch arm.

The arm extends along base_link -y, so the base parks parallel to the
furniture edge with the object on its right. grasp_mission then does the
fine correction along x and the arm motion.
"""
import math

# base_link -> link_grasp_center: y -0.515 at 0.10 m extension (measured in sim),
# so the reachable lateral span is about 0.42 .. 0.93 m; aim for the middle.
LATERALS = (0.65, 0.60, 0.70, 0.55, 0.75, 0.80, 0.85)
# The stowed gripper sticks out ~0.52 m to the right (0.10 m extension): no
# furniture may be closer than this along the arm line, or the planner starts
# in collision (seen 2026-11-10: counter face 0.49 m away).
ARM_CLEAR = 0.65
ALONG = -0.02          # grasp center x in base_link
YAW_STEP = math.radians(10)


def base_for(obj_xy, yaw, lateral, along=ALONG):
    """Base pose with the object at (along, -lateral) in base_link."""
    s, c = math.sin(yaw), math.cos(yaw)
    # object = base + along*(c, s) + lateral*(s, -c)
    return (obj_xy[0] - along * c - lateral * s, obj_xy[1] - along * s + lateral * c, yaw)


def occupied_length(a, b, occupied, step=0.02):
    n = max(1, int(math.dist(a, b) / step))
    return sum(occupied(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
               for i in range(n + 1)) * math.dist(a, b) / (n + 1)


def arm_line_clear(base_xy, obj_xy, occupied, clear=ARM_CLEAR, step=0.02):
    d = math.dist(base_xy, obj_xy)
    n = int(min(clear, d) / step)
    return not any(occupied(base_xy[0] + (obj_xy[0] - base_xy[0]) * i * step / d,
                            base_xy[1] + (obj_xy[1] - base_xy[1]) * i * step / d)
                   for i in range(n + 1))


def pregrasp_poses(obj_xy, robot_xy, base_free, occupied, travel_weight=0.1):
    """Candidate (x, y, yaw) best first.
    base_free(x, y, yaw): the footprint fits there (costmap check).
    occupied(x, y): a map cell is an obstacle; none may lie within ARM_CLEAR
    of the base along the arm line.
    Cost: arm length over furniture (short = straight at the edge, close to
    it) plus a little travel distance. Lateral distances are tried
    nearest-to-comfortable first; the first free one per yaw is kept."""
    out = []
    for k in range(int(round(2 * math.pi / YAW_STEP))):
        yaw = -math.pi + k * YAW_STEP
        for lateral in LATERALS:
            x, y, _ = base_for(obj_xy, yaw, lateral)
            if base_free(x, y, yaw) and arm_line_clear((x, y), obj_xy, occupied):
                cost = (occupied_length((x, y), obj_xy, occupied)
                        + abs(lateral - LATERALS[0])
                        + travel_weight * math.dist((x, y), robot_xy))
                out.append((cost, (x, y, yaw)))
                break
    return [p for _, p in sorted(out)]
