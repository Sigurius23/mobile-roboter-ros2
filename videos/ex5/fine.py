"""Line-by-line code walkthrough: light up one line, dim the rest, label parts of the line.

A Walk holds a code block on screen. walk.step(...) moves the light to some lines, underlines
substrings of those lines (tokens) and labels each one below the code, while the voice speaks.
"""
import manimgx as m

from common import AMBER, BG, FG, GREEN, MUTED, PANEL, SRC_LINES, TEAL, T, _short, code

DIM = 0.28


class Walk:
    def __init__(self, scene, first, last, size=24, top=2.5, max_w=13.4, max_h=3.5, max_chars=78,
                 src=None, language="cpp"):
        self.scene, self.first, self.last = scene, first, last
        raw = (src or SRC_LINES)[first - 1:last]
        # drop the indentation all shown lines share, then shorten what is still too long
        indent = min((len(s) - len(s.lstrip()) for s in raw if s.strip()), default=0)
        self.lines = []
        for s in raw:
            s = s[indent:]
            if len(s) > max_chars:
                cut = s[:max_chars - 1].rstrip()
                if "//" not in cut and cut.count('"') % 2 == 1:
                    cut += '"'
                s = cut + " …"
            self.lines.append(s)
        c = code(first, last, size, lines=self.lines, language=language)
        if c.width > max_w:
            c.scale_to_fit_width(max_w)
        if c.height > max_h:
            c.scale_to_fit_height(max_h)
        c.move_to([0, top - c.height / 2, 0])
        self.c = c
        self.bar = None
        self.marks = m.VGroup()
        self.labels = m.VGroup()
        self.note = None

    # ── geometry ──────────────────────────────────────────────────────────────────────
    def row(self, n):
        return self.c.code_lines[n - self.first]

    def number(self, n):
        return self.c.line_numbers[n - self.first]

    def token(self, n, text, occurrence=0):
        """The glyphs of `text` in file line n (its occurrence-th appearance)."""
        line = self.lines[n - self.first]
        pos = -1
        for _ in range(occurrence + 1):
            pos = line.find(text, pos + 1)
        if pos < 0:
            raise ValueError(f"{text!r} not in line {n}: {line!r}")
        start = len(line[:pos].replace(" ", ""))
        count = len(text.replace(" ", ""))
        row = self.row(n)
        return m.VGroup(*list(row)[start:start + count])

    def bar_for(self, lines):
        rows = [self.row(n) for n in lines if len(self.row(n)) > 0]
        nums = [self.number(n) for n in lines]
        grp = m.VGroup(*rows, *nums)
        left = self.c.get_left()[0] + 0.12
        right = self.c.get_right()[0] - 0.12
        h = grp.height + 0.1
        r = m.Rectangle(width=right - left, height=h, stroke_width=0, fill_color=AMBER, fill_opacity=0.16)
        r.move_to([(left + right) / 2, grp.get_center()[1], 0])
        return r

    # ── animation ─────────────────────────────────────────────────────────────────────
    def show(self, script=None, *extra):
        if script:
            self.scene.say(script, m.AnimationGroup(m.FadeIn(self.c), *extra))
        else:
            self.scene.play(m.FadeIn(self.c), *extra, run_time=0.6)

    def _dim_anims(self, lines):
        on = set(lines)
        anims = []
        for n in range(self.first, self.last + 1):
            o = 1.0 if (not on or n in on) else DIM
            row = self.row(n)
            if len(row):
                anims.append(row.animate.set_opacity(o))
            anims.append(self.number(n).animate.set_opacity(o))
        return anims

    def _labels(self, tokens):
        """tokens: list of (line, text, label) or (line, text, label, occurrence)."""
        marks, labels, wires = m.VGroup(), m.VGroup(), m.VGroup()
        y_label = self.c.get_bottom()[1] - 0.55
        items = []
        for t in tokens:
            n, text, label = t[0], t[1], t[2]
            occ = t[3] if len(t) > 3 else 0
            g = self.token(n, text, occ)
            u = m.Line(g.get_corner(m.DL) + m.DOWN * 0.05, g.get_corner(m.DR) + m.DOWN * 0.05, color=AMBER, stroke_width=3)
            lab = T(label, 22, AMBER)
            items.append((g.get_center()[0], u, lab))
        in_order = [(u, lab) for _, u, lab in items]  # the order the tokens were given (= the order they are spoken)
        items.sort(key=lambda it: it[0])
        gap = 0.3
        total = sum(lab.width for _, _, lab in items) + gap * (len(items) - 1)
        if total > 13.4:  # too many labels for one row: shrink them all
            for _, _, lab in items:
                lab.scale(13.4 / total)
        # centred under their tokens where possible: push right to avoid overlap, then back left from the edge
        xs = []
        x_prev = -7.0
        for x, _, lab in items:
            x_c = max(x, x_prev + lab.width / 2 + gap)
            xs.append(x_c)
            x_prev = x_c + lab.width / 2
        x_next = 7.0
        for i in range(len(items) - 1, -1, -1):
            w = items[i][2].width
            xs[i] = min(xs[i], x_next - w / 2 - gap)
            x_next = xs[i] - w / 2
        placed = []
        for (x, u, lab), x_c in zip(items, xs):
            lab.move_to([x_c, y_label, 0])
            placed.append((u, lab))
        wire_of = {}
        for u, lab in placed:
            marks.add(u)
            labels.add(lab)
            wire = m.Line(u.get_center(), lab.get_top() + m.UP * 0.05, color=AMBER, stroke_width=1.2, stroke_opacity=0.7)
            wires.add(wire)
            wire_of[id(lab)] = wire
        per_token = [(u, lab, wire_of[id(lab)]) for u, lab in in_order]
        return marks, m.VGroup(labels, wires), per_token

    @staticmethod
    def _section(tag_text, color, rows):
        tag = T(tag_text, 20, BG, weight=m.BOLD)
        tag_box = m.SurroundingRectangle(tag, buff=0.1, color=color, fill_color=color, fill_opacity=1, corner_radius=0.06)
        tag_g = m.VGroup(tag_box, tag)
        body = m.VGroup(*rows).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.12)
        return tag_g, body

    def why_panel(self, why, instead=None, cpp=None):
        """Box at the bottom: C++ syntax notes (teal), the reason for the choice (WHY, green), and the alternative."""
        sections = []
        if cpp:
            sections.append(self._section("C++", TEAL, [T(s, 22, FG) for s in cpp]))
        if why:
            rows = [T(s, 22, FG) for s in why] + [T("instead: " + s, 22, MUTED) for s in (instead or [])]
            sections.append(self._section("WHY", GREEN, rows))
        tag_w = max(t.width for t, _ in sections)
        lines = []
        for tag_g, body in sections:
            body.next_to(tag_g, m.RIGHT, buff=0.3 + tag_w - tag_g.width, aligned_edge=m.UP)
            lines.append(m.VGroup(tag_g, body))
        g = m.VGroup(*lines).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.22)
        if g.width > 13.2:
            g.scale_to_fit_width(13.2)
        g.to_edge(m.DOWN, buff=0.3).to_edge(m.LEFT, buff=0.5)
        bg = m.SurroundingRectangle(g, buff=0.18, color=PANEL, fill_color=PANEL, fill_opacity=0.96, corner_radius=0.1, stroke_width=0)
        panel = m.VGroup(bg, g)
        panel.sections = lines  # in the order cpp, why
        return panel

    def step(self, lines, script, tokens=(), note=None, extra=(), why=None, instead=None, cpp=None):
        """Light up `lines`, then show each token's label, the C++ box and the WHY box, each when it is spoken.

        `script` marks the moments with brackets, in this order:
          [words that move the highlight to the line]  [words for token 1] … [words for token n]
          [words for the C++ box] (if cpp)  [words for the WHY box] (if why)
        Each animation starts on its words and lasts as long as they take, so keep each bracket short.
        """
        if isinstance(lines, int):
            lines = [lines]
        need = 1 + len(tokens) + (1 if cpp else 0) + (1 if why else 0) + len(extra)
        have = script.count("[")
        if have != need:
            raise ValueError(f"step {lines}: {have} brackets in the script, need {need} "
                             f"(highlight + {len(tokens)} tokens{' + C++' if cpp else ''}{' + WHY' if why else ''}"
                             f"{' + extras' if extra else ''}): {script[:60]!r}")
        old = [x for x in (self.marks, self.labels) if len(x)]
        if self.note is not None:
            old.append(self.note)
        new_bar = self.bar_for(lines)
        first = m.AnimationGroup(*[m.FadeOut(x) for x in old], *self._dim_anims(lines),
                                 m.ReplacementTransform(self.bar, new_bar) if self.bar is not None else m.FadeIn(new_bar),
                                 run_time=0.6)
        marks, labels, per_token = self._labels(tokens) if tokens else (m.VGroup(), m.VGroup(), [])
        token_anims = [m.AnimationGroup(m.Create(u), m.Create(w), m.FadeIn(lab), run_time=0.6) for u, lab, w in per_token]
        panel_anims, new_note = [], None
        if why or cpp:
            new_note = self.why_panel(why, instead, cpp)
            bg, secs = new_note[0], new_note.sections
            for i, sec in enumerate(secs):
                parts = [m.FadeIn(bg)] if i == 0 else []
                panel_anims.append(m.AnimationGroup(*parts, m.FadeIn(sec, shift=m.UP * 0.1), run_time=0.6))
        elif note:
            new_note = T(note, 22, FG)
            new_note.move_to([0, -3.55, 0])
            if new_note.width > 13:
                new_note.scale_to_fit_width(13)
            token_anims.append(m.FadeIn(new_note, run_time=0.6))
            if script.count("[") == need:  # a plain note has no bracket of its own: show it with the last token
                token_anims[-2:] = [m.AnimationGroup(*token_anims[-2:])] if len(token_anims) > 1 else token_anims
        self.scene.say(script, first, *token_anims, *panel_anims, *extra)
        self.bar, self.marks, self.labels, self.note = new_bar, marks, labels, new_note

    def done(self):
        objs = [self.c] + [x for x in (self.bar, self.marks, self.labels, self.note) if x is not None and (not hasattr(x, "__len__") or len(x))]
        self.scene.play(*[m.FadeOut(x) for x in objs], run_time=0.4)
        self.bar, self.marks, self.labels, self.note = None, m.VGroup(), m.VGroup(), None
