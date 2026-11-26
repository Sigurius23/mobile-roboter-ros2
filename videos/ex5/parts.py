"""Shared scene helpers for the four-part line-by-line walkthrough (ex5_walk.py).

Narration uses brackets to time animations: each [bracketed span] starts one animation
(see fine.Walk.step). Helpers here check the bracket count so words and pictures stay in sync.
"""
import manimgx as m

from common import AMBER, BG, FG, GREEN, MUTED, PANEL, T, header, lines_block, qa_card
from context import pipeline
from phononvoice import phonon_voice


def _need(script, n, what):
    have = script.count("[")
    if have != n:
        raise ValueError(f"{what}: {have} brackets in the script, need {n}: {script[:60]!r}")


class Part(m.Scene):
    voice = staticmethod(phonon_voice)

    # ── framing ────────────────────────────────────────────────────────────────────────
    def clear(self, run_time=0.4):
        if self.mobjects:
            self.play(*[m.FadeOut(x) for x in self.mobjects], run_time=run_time)

    def opening(self, number, title, contents, script):
        """Title card of one part. `script` has one bracket per entry of `contents`, plus one for the title."""
        _need(script, 1 + len(contents), "opening")
        k = T(f"MOBILE ROBOTS · EXERCISE 5 · PART {number} OF 4", 24, AMBER)
        t = T(title, 54, FG, weight=m.BOLD)
        head = m.VGroup(k, t).arrange(m.DOWN, buff=0.3).move_to([0, 1.8, 0])
        items = lines_block(contents, 28, MUTED, buff=0.22).next_to(head, m.DOWN, buff=0.8)
        self.say(script, m.FadeIn(head, shift=m.UP * 0.2, run_time=0.8),
                 *[m.FadeIn(x, shift=m.RIGHT * 0.15, run_time=0.5) for x in items])
        self.clear()

    def section(self, title, kicker):
        """A header for a new section; returns it so it can be faded out later."""
        h = header(title, kicker)
        self.play(m.FadeIn(h), run_time=0.4)
        return h

    def where(self, lit, lines, callers=None):
        """The map of the node with task `lit` lit up. The first bracket in `lines` pulses the lit
        box, the second one shows the `callers` note (where in the file it happens)."""
        _need(" ".join(lines), 1 + (1 if callers else 0), f"where({lit})")
        h = header("Where are we?", "the path of one lidar scan through the node")
        mp = pipeline(lit)
        self.play(m.FadeIn(h), m.FadeIn(mp), run_time=0.6)
        note = lines_block(callers, 20, MUTED, buff=0.1).move_to([-5.2, -1.3, 0]) if callers else None
        queue = [m.Indicate(mp.boxes[lit], color=AMBER, scale_factor=1.08)]
        if note is not None:
            queue.append(m.FadeIn(note, run_time=0.6))
        for line in lines:
            k = line.count("[")
            anims, queue = queue[:k], queue[k:]
            self.say(line, *anims)
        self.play(*[m.FadeOut(x) for x in (h, mp, note) if x is not None], run_time=0.5)

    def tour(self, keys, lines):
        """The whole map, nothing lit; each bracket pulses the next box in `keys`."""
        _need(" ".join(lines), len(keys), "tour")
        h = header("The whole node in one picture", "the path of one lidar scan")
        mp = pipeline(None)
        self.play(m.FadeIn(h), m.FadeIn(mp), run_time=0.6)
        queue = [m.Indicate(mp.boxes[k], color=AMBER, scale_factor=1.08) for k in keys]
        for line in lines:
            n = line.count("[")
            anims, queue = queue[:n], queue[n:]
            self.say(line, *anims)
        self.play(m.FadeOut(h), m.FadeOut(mp), run_time=0.5)

    def qa(self, question, answer_lines, answer, spoken=None):
        """A Testat question; `answer` has one bracket per line in `answer_lines`."""
        _need(answer, len(answer_lines), f"qa {question!r}")
        self.clear()
        q = qa_card(question).move_to([0, 2.0, 0])
        if q.width > 13:
            q.scale_to_fit_width(13)
        a = lines_block(answer_lines, size=27, color=FG, buff=0.28)
        if a.width > 12.4:
            a.scale_to_fit_width(12.4)
        a.next_to(q, m.DOWN, buff=0.7).align_to(q, m.LEFT).shift(m.RIGHT * 0.4)
        self.say("Question: " + (spoken or question), m.FadeIn(q, shift=m.DOWN * 0.2, run_time=0.6))
        self.say(answer, *[m.FadeIn(x, shift=m.RIGHT * 0.15, run_time=0.5) for x in a])
        self.wait(0.6)
        self.clear()

    def plain(self, title, kicker, lines, script):
        """A short plain-language card before the code: one bracket per line."""
        _need(script, len(lines), f"plain {title!r}")
        h = header(title, kicker)
        self.play(m.FadeIn(h), run_time=0.4)
        body = lines_block(lines, 30, FG, buff=0.4).move_to([0, -0.2, 0])
        if body.width > 12.6:
            body.scale_to_fit_width(12.6)
        self.say(script, *[m.FadeIn(x, shift=m.UP * 0.1, run_time=0.5) for x in body])
        self.play(m.FadeOut(h), m.FadeOut(body), run_time=0.4)

    def closing(self, lines, script):
        _need(script, len(lines), "closing")
        self.clear()
        body = lines_block(lines, 34, FG, buff=0.35)
        self.say(script, *[m.FadeIn(x, run_time=0.5) for x in body])
        self.wait(0.8)
