"""Shared look and helpers for the Exercise 5 videos (manimgx).

Data comes from data/icp_demo.json (real scans of dlg_feldtage_24:cut, see README.md),
code excerpts from the fixed node in ros2_ws/src/icp_odom/src/icp_node.cpp.
"""
import json
from pathlib import Path

import manimgx as m

HERE = Path(__file__).parent
NODE_CPP = HERE.parent.parent / "ros2_ws" / "src" / "icp_odom" / "src" / "icp_node.cpp"
SRC_LINES = NODE_CPP.read_text(encoding="utf-8").splitlines()
DEMO = json.loads((HERE / "data" / "icp_demo.json").read_text())

# palette (dark): same roles as the study guide
BG = "#0f151b"
FG = "#e2e9ee"
MUTED = "#94a4b1"
AMBER = "#ff8b4f"   # ours / highlight
TEAL = "#52c6de"    # topics, the new scan
VIOLET = "#b9a2fb"  # the old scan
GREEN = "#7bd389"
RED = "#ff6b6b"
PANEL = "#1a242d"

m.config.background_color = BG

SANS = "sans-serif"


def T(text, size=30, color=FG, weight=m.NORMAL, **kw):
    """Plain text in Noto Sans."""
    return m.Text(text, font=SANS, font_size=size, color=color, weight=weight, **kw)


def lines_block(lines, size=28, color=FG, buff=0.22):
    """Several left-aligned text lines."""
    return m.VGroup(*[T(s, size, color) for s in lines]).arrange(m.DOWN, aligned_edge=m.LEFT, buff=buff)


def header(title, kicker=None):
    """Section title in the top-left corner."""
    t = T(title, 40, FG, weight=m.BOLD)
    g = m.VGroup(t)
    if kicker:
        k = T(kicker, 22, AMBER)
        g = m.VGroup(k, t).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.12)
    return g.to_corner(m.UL, buff=0.45)


MAX_CHARS = 92


def _short(line):
    """Lines too long to read on screen end in '…' (only long log calls and comments are this long)."""
    if len(line) <= MAX_CHARS:
        return line
    cut = line[:MAX_CHARS - 1].rstrip()
    if "//" not in cut and cut.count('"') % 2 == 1:
        cut += '"'  # close a string literal that was cut, or the highlighter colours the rest as a string
    return cut + " …"


def code(first, last, size=17, lines=None, language="cpp"):
    """Lines first..last of the fixed icp_node.cpp, numbered like the file (or `lines` shown in their place)."""
    if lines is None:
        lines = [_short(s) for s in SRC_LINES[first - 1:last]]
    text = "\n".join(lines)
    c = m.Code(code_string=text, language=language, line_numbers_from=first,
               paragraph_config={"font_size": size, "line_spacing": 0.45},
               background_config={"stroke_color": "#33414d"})
    c.first_line = first
    if language == "python":  # the vim style draws Python strings dark purple: make the code light, numbers amber
        for row, line in zip(c.code_lines, lines):
            if len(row):
                row.set_color(FG)
    recolor_comments(c, lines)
    return c


COMMENT = "#9fb3c2"


def recolor_comments(c, lines):
    """The 'vim' style draws comments dark blue on black; make them readable.
    Each row of c.code_lines holds the line's glyphs without spaces."""
    for row, line in zip(c.code_lines, lines):
        k = line.find("//")
        if k < 0 and line.lstrip().startswith("#"):
            k = line.find("#")
        elif k < 0 and "  # " in line:
            k = line.find("  # ") + 2
        if k < 0 or len(row) == 0:
            continue
        before = len(line[:k].replace(" ", "").replace("\t", ""))
        glyphs = list(row)
        for g in glyphs[before:]:
            g.set_color(COMMENT)


def mark(c, first, last, color=AMBER):
    """A rectangle around file lines first..last of a code block."""
    rows = [c.code_lines[n - c.first_line] for n in range(first, last + 1)
            if SRC_LINES[n - 1].strip() and n - c.first_line < len(c.code_lines)]
    group = m.VGroup(*[r for r in rows if len(r) > 0])
    return m.SurroundingRectangle(group, color=color, buff=0.035, stroke_width=2.5, corner_radius=0.04)


def card(text_lines, width=None, color=PANEL, size=26, title=None, title_color=AMBER):
    """A filled panel with text."""
    body = lines_block(text_lines, size=size)
    parts = [body]
    if title:
        parts.insert(0, T(title, size + 2, title_color, weight=m.BOLD))
    g = m.VGroup(*parts).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.25)
    box = m.SurroundingRectangle(g, buff=0.35, color=color, fill_color=color, fill_opacity=1,
                                 corner_radius=0.15, stroke_width=0)
    if width:
        box.stretch_to_fit_width(width)
    return m.VGroup(box, g)


def qa_card(question):
    """Testat question shown while the voice asks and answers it."""
    q = T("Q", 30, BG, weight=m.BOLD)
    qb = m.SurroundingRectangle(q, buff=0.14, color=AMBER, fill_color=AMBER, fill_opacity=1, corner_radius=0.08)
    head = m.VGroup(qb, q)
    text = T(question, 30, FG, weight=m.BOLD)
    row = m.VGroup(head, text).arrange(m.RIGHT, buff=0.3)
    box = m.SurroundingRectangle(row, buff=0.35, color=PANEL, fill_color=PANEL, fill_opacity=1,
                                 corner_radius=0.15, stroke_width=0)
    return m.VGroup(box, row)


# ── scans ──────────────────────────────────────────────────────────────────────────────


def to_scene(xy, scale, center=(0.0, 0.0), origin=(0.0, 0.0)):
    """Metres (robot frame, x forward) to scene coordinates: forward = up on screen."""
    out = []
    for x, y in xy:
        out.append([origin[0] - (y - center[1]) * scale, origin[1] + (x - center[0]) * scale, 0.0])
    return out


def dots(xy, color, scale, center=(0.0, 0.0), origin=(0.0, 0.0), radius=0.028, opacity=1.0, limit=None):
    pts = to_scene(xy, scale, center, origin)
    if limit:
        pts = [p for p in pts if abs(p[0] - origin[0]) < limit[0] and abs(p[1] - origin[1]) < limit[1]]
    return m.VGroup(*[m.Dot(p, radius=radius, color=color, fill_opacity=opacity) for p in pts])


def robot(origin=(0.0, 0.0), size=0.3, color=AMBER):
    """A small triangle pointing up (the robot's forward)."""
    tri = m.Triangle(color=color, fill_color=color, fill_opacity=1).scale(size / 2)
    tri.move_to([origin[0], origin[1], 0])
    return tri
