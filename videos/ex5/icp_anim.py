"""Draw and animate the ICP iterations of a real scan pair (data/icp_demo.json)."""
import manimgx as m

from common import AMBER, MUTED, TEAL, VIOLET, T, dots, to_scene


class View:
    """Where a part of the scan pair is drawn: metres around `center`, `scale` units per metre,
    at `origin` on screen, clipped to +-half_w / +-half_h screen units."""

    def __init__(self, scale, center=(0.0, 0.0), origin=(0.0, 0.0), half_w=6.5, half_h=3.4, radius=0.03):
        self.scale, self.center, self.origin = scale, center, origin
        self.half_w, self.half_h, self.radius = half_w, half_h, radius

    def inside(self, xy):
        p = to_scene([xy], self.scale, self.center, self.origin)[0]
        return abs(p[0] - self.origin[0]) < self.half_w and abs(p[1] - self.origin[1]) < self.half_h

    def dots(self, xy, color, opacity=1.0):
        return dots([p for p in xy if self.inside(p)], color, self.scale, self.center, self.origin,
                    radius=self.radius, opacity=opacity)

    def point(self, xy):
        return to_scene([xy], self.scale, self.center, self.origin)[0]

    def frame(self, color=MUTED):
        return m.Rectangle(width=2 * self.half_w, height=2 * self.half_h, color=color, stroke_width=1.5) \
            .move_to([self.origin[0], self.origin[1], 0])


def moving_dots(view, pts_now, keep_idx, color=VIOLET):
    """Dots for the source points whose index is in keep_idx (so every step draws the same points)."""
    return m.VGroup(*[m.Dot(view.point(pts_now[i]), radius=view.radius, color=color) for i in keep_idx])


def source_indices(view, steps):
    """Source points that stay inside the view in the first and the last step."""
    first, last = steps[0]["points"], steps[-1]["points"]
    return [i for i in range(len(first)) if view.inside(first[i]) and view.inside(last[i])]


def pair_lines(view, step, every=1):
    lines = []
    for k, (s, t) in enumerate(zip(step["pairs_src"], step["pairs_tgt"])):
        if k % every or not (view.inside(s) and view.inside(t)):
            continue
        lines.append(m.Line(view.point(s), view.point(t), color=AMBER, stroke_width=1.6, stroke_opacity=0.85))
    return m.VGroup(*lines)


def counter(text, pos):
    return T(text, 24, MUTED).move_to(pos)


def step_to(view, steps, i, keep_idx, color=VIOLET):
    return moving_dots(view, steps[i]["points"], keep_idx, color)
