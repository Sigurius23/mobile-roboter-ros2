"""'Where are we' map: the path of one lidar scan through the ICP node, with the current task lit up.

Layout (three rows):
    lidar scan  →  scanCallback  →  TASK 1
                   odom / IMU   ↗      ↓
                                     TASK 2  →  TASK 3
                                                  ↓
                                 robot pose  ←  Kalman filter
"""
import numpy as np

import manimgx as m

from common import AMBER, FG, MUTED, PANEL, T

# key: (title, plain-language subtitle, centre)
BOXES = {
    "scan": ("lidar scan", "a ring of distances", (-5.0, 1.6)),
    "callback": ("scanCallback", "runs for every scan", (-1.3, 1.6)),
    "task1": ("TASK 1  getTransform", "sensor frame → robot frame", (2.6, 1.6)),
    "odomimu": ("odom / IMU callbacks", "wheels and gyro", (-1.3, -0.35)),
    "task2": ("TASK 2  initial guess", "where ICP starts looking", (2.6, -0.35)),
    "task3": ("TASK 3  ICP loop", "how did the robot move?", (5.35, -2.3)),
    "filter": ("Kalman filter", "combines all sensors", (1.9, -2.3)),
    "pose": ("robot pose", "TF odom → base_footprint", (-1.6, -2.3)),
}


def _box(key, lit):
    title, sub, (x, y) = BOXES[key]
    is_task = key.startswith("task")
    color = AMBER if lit else (FG if is_task else MUTED)
    t = T(title, 21, color, weight=m.BOLD if is_task else m.NORMAL)
    s = T(sub, 17, color if lit else MUTED)
    g = m.VGroup(t, s).arrange(m.DOWN, buff=0.08)
    w = max(2.6, g.width + 0.35)
    r = m.RoundedRectangle(width=w, height=0.95, corner_radius=0.12, color=color, stroke_width=3 if lit else 1.5,
                           fill_color=PANEL, fill_opacity=1)
    box = m.VGroup(r, g.move_to(r))
    box.move_to([x, y, 0])
    return box


def _arrow(points, color=MUTED):
    """A polyline with an arrow tip at its last point."""
    pts = [np.array(p, dtype=float) for p in points]
    head = m.Arrow(pts[-2], pts[-1], buff=0, color=color, stroke_width=2.5, max_tip_length_to_length_ratio=0.5,
                   max_stroke_width_to_length_ratio=20)
    if len(pts) == 2:
        return head
    body = m.VMobject(color=color, stroke_width=2.5)
    body.set_points_as_corners(pts[:-1])
    return m.VGroup(body, head)


def pipeline(lit=None):
    """The map as a VGroup; .boxes[key] gives each box."""
    b = {k: _box(k, k == lit) for k in BOXES}
    R, L, U, D = (lambda k: b[k].get_right()), (lambda k: b[k].get_left()), (lambda k: b[k].get_top()), (lambda k: b[k].get_bottom())
    g = 0.06
    arrows = m.VGroup(
        _arrow([R("scan") + [g, 0, 0], L("callback") - [g, 0, 0]]),
        _arrow([R("callback") + [g, 0, 0], L("task1") - [g, 0, 0]]),
        _arrow([D("task1") - [0, g, 0], U("task2") + [0, g, 0]]),
        # TASK 2 → TASK 3: right, then down
        _arrow([R("task2") + [g, 0, 0], [b["task3"].get_x(), b["task2"].get_y(), 0], U("task3") + [0, g, 0]]),
        _arrow([L("task3") - [g, 0, 0], R("filter") + [g, 0, 0]]),
        _arrow([L("filter") - [g, 0, 0], R("pose") + [g, 0, 0]]),
        # odom / IMU: into TASK 1 (they convert their vectors too) and into the filter
        _arrow([R("odomimu") + [g, 0.2, 0], [b["task1"].get_left()[0] - 0.35, b["odomimu"].get_y() + 0.2, 0],
                [b["task1"].get_left()[0] - 0.35, b["task1"].get_bottom()[1] + 0.05, 0],
                [b["task1"].get_left()[0] - 0.05, b["task1"].get_bottom()[1] + 0.05, 0]]),
        _arrow([D("odomimu") - [0, g, 0], [b["odomimu"].get_x(), -1.3, 0], [b["filter"].get_x() - 0.6, -1.3, 0],
                [b["filter"].get_x() - 0.6, b["filter"].get_top()[1] + g, 0]]),
    )
    group = m.VGroup(arrows, *b.values())
    group.boxes = b
    return group
