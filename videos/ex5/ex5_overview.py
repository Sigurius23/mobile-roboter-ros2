"""Exercise 5 in five minutes: lidar odometry with ICP (overview video).

Render:  manimgx render ex5_overview.py Ex5Overview
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import manimgx as m

from common import (AMBER, BG, DEMO, FG, GREEN, MUTED, PANEL, RED, TEAL, VIOLET, T, card, dots, header,
                    lines_block, to_scene)
from icp_anim import View, counter, pair_lines, source_indices, step_to
from winvoice import windows_voice


def box(label, sub, color, width=3.6):
    t = T(label, 28, color, weight=m.BOLD)
    s = T(sub, 20, MUTED)
    g = m.VGroup(t, s).arrange(m.DOWN, buff=0.1)
    r = m.RoundedRectangle(width=width, height=1.15, corner_radius=0.15, color=color, stroke_width=2.5,
                           fill_color=PANEL, fill_opacity=1)
    return m.VGroup(r, g.move_to(r))


def polyline(points, color, width=4):
    v = m.VMobject(color=color, stroke_width=width)
    v.set_points_as_corners(points)
    return v


class Ex5Overview(m.Scene):
    voice = staticmethod(windows_voice)

    def clear(self, run_time=0.6):
        if self.mobjects:
            self.play(*[m.FadeOut(x) for x in self.mobjects], run_time=run_time)

    def construct(self) -> None:
        self.title()
        self.big_picture()
        self.slice()
        self.task1()
        self.icp_loop()
        self.task2()
        self.fusion()
        self.results()
        self.fixes()

    # ── 0 ──────────────────────────────────────────────────────────────────────────────
    def title(self):
        k = T("MOBILE ROBOTS · EXERCISE 5", 24, AMBER)
        t = T("Lidar odometry with ICP", 60, FG, weight=m.BOLD)
        s = T("the idea, the three tasks, and what the real bag showed", 28, MUTED)
        g = m.VGroup(k, t, s).arrange(m.DOWN, buff=0.35)
        self.say("Exercise five adds a third sensor to our Kalman filter: the lidar. "
                 "In about five minutes we go through the idea, the three tasks, "
                 "and what we found when we tested it on the real bag file.",
                 m.LaggedStart(m.FadeIn(k), m.Write(t), m.FadeIn(s), lag_ratio=0.4))
        self.clear()

    # ── 1 ──────────────────────────────────────────────────────────────────────────────
    def big_picture(self):
        h = header("The big picture")
        self.play(m.FadeIn(h))
        odom = box("/odom", "wheels: v, ω", AMBER).move_to([-4.3, 1.6, 0])
        imu = box("/imu/data_raw", "gyro: ω", TEAL).move_to([-4.3, 0.0, 0])
        scan = box("/ouster/scan", "lidar → ICP", VIOLET).move_to([-4.3, -1.6, 0])
        kf = box("Kalman filter", "x, y, θ, v, ω, a", FG, width=3.4).move_to([0.6, 0, 0])
        tf = box("TF", "odom → base_footprint", GREEN, width=3.6).move_to([4.8, 0, 0])
        arrows = m.VGroup(*[m.Arrow(b.get_right(), kf.get_left(), buff=0.15, color=MUTED, stroke_width=3)
                            for b in (odom, imu, scan)])
        out = m.Arrow(kf.get_right(), tf.get_left(), buff=0.15, color=MUTED, stroke_width=3)
        self.say("In exercises two and four, the robot only felt its own body: [the wheels] and [the gyroscope]. "
                 "That is like walking with your eyes closed and counting steps.",
                 m.FadeIn(odom, shift=m.RIGHT * 0.3), m.FadeIn(imu, shift=m.RIGHT * 0.3))
        self.say("Exercise five opens the eyes. [The lidar] compares the scan it sees now with the scan from a moment ago, "
                 "and works out how the robot must have moved to make them line up. That is ICP, iterative closest point.",
                 m.FadeIn(scan, shift=m.RIGHT * 0.3))
        self.say("That motion goes into [the same Kalman filter] as a third measurement, and the filter [publishes the pose] as a transform.",
                 m.AnimationGroup(m.Create(arrows), m.FadeIn(kf)), m.AnimationGroup(m.Create(out), m.FadeIn(tf)))
        self.clear()

    # ── 2 ──────────────────────────────────────────────────────────────────────────────
    def slice(self):
        h = header("From 3D to 2D", "given: pointcloud_to_laserscan")
        self.play(m.FadeIn(h))
        # side view, k units per metre, ground at y = g0
        k, g0 = 0.72, -3.4
        lx = -5.6

        def hy(metres):  # height above ground -> screen y
            return g0 + metres * k

        ground = m.Line([-6.6, g0, 0], [3.2, g0, 0], color=MUTED, stroke_width=2)
        body = m.Rectangle(width=0.9, height=0.32, color=AMBER, fill_color=AMBER, fill_opacity=0.9).move_to([lx, hy(0.25), 0])
        lidar = m.Dot([lx, hy(0.54), 0], radius=0.08, color=TEAL)
        band = m.Rectangle(width=8.0, height=4.0 * k, color=TEAL, stroke_width=0, fill_color=TEAL, fill_opacity=0.12) \
            .move_to([-2.6, hy(4.54), 0])
        band_top = m.DashedLine([-6.6, hy(6.54), 0], [1.4, hy(6.54), 0], color=TEAL)
        band_bot = m.DashedLine([-6.6, hy(2.54), 0], [1.4, hy(2.54), 0], color=TEAL)
        lab = lines_block(["kept: 2 m to 6 m above the lidar", "= 2.5 m to 6.5 m above ground"], 20, TEAL)
        lab.move_to([-0.9, hy(4.54), 0])
        building = m.Rectangle(width=1.0, height=6.2 * k, color=FG, stroke_width=2).move_to([2.4, hy(3.1), 0])
        px = -2.6
        person = m.VGroup(m.Circle(radius=0.12, color=MUTED).move_to([px, hy(1.6), 0]),
                          m.Line([px, hy(1.45), 0], [px, hy(0.8), 0], color=MUTED),
                          m.Line([px, hy(0.8), 0], [px - 0.2, g0, 0], color=MUTED),
                          m.Line([px, hy(0.8), 0], [px + 0.2, g0, 0], color=MUTED))
        rays = m.VGroup(*[m.Line(lidar.get_center(), [-3.1, hy(0.54 + dy), 0], color=TEAL, stroke_width=1.2, stroke_opacity=0.6)
                          for dy in (-0.4, 1.0, 2.2, 3.3, 4.4, 5.4)])
        self.add(ground)
        self.say("The filter is two dimensional, but the Ouster lidar sees in three dimensions. "
                 "So [a given node, point cloud to laser scan,] cuts a slice out of the cloud.",
                 m.AnimationGroup(m.FadeIn(body), m.FadeIn(lidar), m.Create(rays)))
        self.say("It keeps only points [between two and six metres above the lidar]. "
                 "The lidar sits about half a metre above the ground, so the slice starts at about two and a half metres.",
                 m.AnimationGroup(m.FadeIn(band), m.Create(band_top), m.Create(band_bot), m.FadeIn(lab)))
        self.say("Down here, [people and vehicles move]. Up there, [facades, roofs and tree tops] stay put, so they make better landmarks.",
                 m.FadeIn(person), m.Create(building))
        side = m.VGroup(*[x for x in self.mobjects if x is not h])
        # top view of a real scan
        src = DEMO["pairs"]["straight"]["src"]
        top = dots(src, TEAL, 0.075, origin=(3.6, -0.4), radius=0.03,
                   limit=(3.2, 3.3))
        me = m.Triangle(color=AMBER, fill_color=AMBER, fill_opacity=1).scale(0.12).move_to([3.6, -0.4, 0])
        cap = T("one real scan from the bag, seen from above", 20, MUTED).move_to([3.6, -3.55, 0])
        self.play(side.animate.scale(0.62).move_to([-3.6, -0.6, 0]), run_time=1)
        self.say("For every half degree of direction it keeps [the closest point]. "
                 "That is about seven hundred rays, and after empty directions are dropped, "
                 "about two hundred and fifty points per scan in our bag.",
                 m.AnimationGroup(m.FadeIn(me), m.LaggedStart(*[m.FadeIn(d) for d in top], lag_ratio=0.004), m.FadeIn(cap)))
        self.clear()

    # ── 3 ──────────────────────────────────────────────────────────────────────────────
    def task1(self):
        h = header("Task 1: getTransform", "TASK 1")
        self.play(m.FadeIn(h))
        src = DEMO["pairs"]["straight"]["src"]
        off = DEMO["lidar_offset"]
        # points back in the lidar frame: the lidar is turned by 180 degrees
        in_lidar = [[-(x - off[0]), -(y - off[1])] for x, y in src]
        left = dots(in_lidar, VIOLET, 0.065, origin=(-3.4, -0.6), radius=0.028, limit=(2.9, 2.6))
        right = dots(src, TEAL, 0.065, origin=(3.4, -0.6), radius=0.028, limit=(2.9, 2.6))
        lt = T("as the lidar reports it (os_lidar)", 22, VIOLET).move_to([-3.4, 2.35, 0])
        rt = T("in the robot frame (base_footprint)", 22, TEAL).move_to([3.4, 2.35, 0])
        rl = m.Triangle(color=AMBER, fill_color=AMBER, fill_opacity=1).scale(0.12).move_to([-3.4, -0.6, 0])
        rr = m.Triangle(color=AMBER, fill_color=AMBER, fill_opacity=1).scale(0.12).move_to([3.4, -0.6, 0])
        self.say("Every sensor reports in its own coordinate frame. On this robot, the lidar frame is "
                 "[turned by one hundred and eighty degrees] and sits fifty four centimetres up.",
                 m.AnimationGroup(m.FadeIn(lt), m.FadeIn(rl), m.LaggedStart(*[m.FadeIn(d) for d in left], lag_ratio=0.004)))
        self.say("getTransform asks TF for that mounting, and every scan is [converted into the robot frame] before ICP sees it.",
                 m.AnimationGroup(m.FadeIn(rt), m.FadeIn(rr), m.TransformFromCopy(left, right)))
        bad = T("without it:  −0.68 m  (drives backwards)", 28, RED).move_to([0, -3.3, 0])
        good = T("with it:  +0.68 m  (wheels: +0.63 m)", 28, GREEN).move_to([0, -3.3, 0])
        self.say("Without it, our real scan pair gives [minus zero point six eight metres]: the robot would seem to drive backwards. "
                 "With it, [plus zero point six eight], close to the wheels.",
                 m.FadeIn(bad), m.ReplacementTransform(bad, good))
        self.clear()

    # ── 4 ──────────────────────────────────────────────────────────────────────────────
    def icp_loop(self):
        h = header("Task 3: the ICP loop", "TASK 3 · real scans, 0.5 s apart")
        self.play(m.FadeIn(h))
        pair = DEMO["pairs"]["straight"]
        steps = pair["steps"]
        view = View(scale=0.95, center=(-14.0, 12.0), origin=(-2.2, 0.35), half_w=4.2, half_h=1.95, radius=0.05)
        keep = source_indices(view, steps)
        tgt = view.dots(pair["tgt"], TEAL)
        src = step_to(view, steps, 0, keep)
        frame = view.frame()
        note = T("zoomed into one corner of the field: 8 m × 5 m", 20, MUTED).next_to(frame, m.DOWN, buff=0.15)
        legend = m.VGroup(m.VGroup(m.Dot(color=VIOLET), T("old scan (source)", 22, VIOLET)).arrange(m.RIGHT, buff=0.15),
                          m.VGroup(m.Dot(color=TEAL), T("new scan (target)", 22, TEAL)).arrange(m.RIGHT, buff=0.15)) \
            .arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.2).move_to([4.6, 2.2, 0])
        self.add(frame)
        self.say("Here are two real scans, half a second apart, zoomed into one corner of the field. "
                 "[Violet] is the old scan, [teal] the new one.",
                 m.AnimationGroup(m.FadeIn(src), m.FadeIn(legend[0]), m.FadeIn(note)),
                 m.AnimationGroup(m.FadeIn(tgt), m.FadeIn(legend[1])))
        lines = pair_lines(view, steps[0])
        s1 = T("1  pair each old point with its nearest new point (kd-tree, < 1 m)", 21, FG)
        s2 = T("2–4  centroids → H = Σ (s − s̄)(t − t̄)ᵀ → SVD:  R = V·Uᵀ,  t = t̄ − R·s̄", 21, FG)
        s3 = T("5–6  move the old points by R, t and add the step to the total", 21, FG)
        s4 = T("7  stop when the step is tiny (0.1 mm, 0.0001 rad), at most 20×", 21, FG)
        steps_txt = m.VGroup(s1, s2, s3, s4).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.16)
        steps_txt.move_to([0, -2.85, 0]).to_edge(m.LEFT, buff=0.6)
        self.play(m.FadeOut(note), run_time=0.3)
        self.say("Step one: every old point is paired with [its nearest new point], found with a k d tree. "
                 "Pairs more than one metre apart are dropped.",
                 m.AnimationGroup(m.Create(lines), m.FadeIn(s1)))
        self.say("Steps two to four: the centre of each group, a two by two cross covariance matrix, and its singular value "
                 "decomposition give [the best rotation and shift] in one go, without any searching.",
                 m.FadeIn(s2))
        it = counter("iteration 1", [4.6, 1.1, 0])
        self.say("Steps five and six move the old points by that rotation and shift, [like this], and add the step to the total.",
                 m.AnimationGroup(m.Transform(src, step_to(view, steps, 1, keep)), m.FadeOut(lines), m.FadeIn(s3), m.FadeIn(it)))
        rest = []
        for i in range(1, len(steps) - 1):
            ln = pair_lines(view, steps[i])
            rest.append(m.Succession(m.Create(ln, run_time=0.35),
                                     m.AnimationGroup(m.Transform(src, step_to(view, steps, i + 1, keep)), m.FadeOut(ln),
                                                      m.Transform(it, counter(f"iteration {i + 1}", [4.6, 1.1, 0])), run_time=0.6)))
        self.say("Then it pairs again and repeats. [After eight iterations], the step is below a tenth of a millimetre, and ICP stops.",
                 m.AnimationGroup(m.Succession(*rest), m.FadeIn(s4)))
        dx = pair["delta"][0]
        res = card([f"robot: {dx:+.2f} m forward", f"wheels: {pair['truth'][0]:+.2f} m",
                    "points move opposite", "→ result inverted"],
                   size=19, title="result").move_to([4.9, -0.75, 0])
        self.say("The result: the robot moved [zero point six eight metres forward], and the wheels say zero point six three. "
                 "One detail: ICP moves the points, and points move opposite to the robot, so the result is inverted at the end.",
                 m.FadeIn(res))
        self.clear()

    # ── 5 ──────────────────────────────────────────────────────────────────────────────
    def task2(self):
        h = header("Task 2: the initial guess", "TASK 2 · real scans while turning")
        self.play(m.FadeIn(h))
        pair = DEMO["pairs"]["turn"]
        view = View(scale=0.085, center=(5.0, 0.0), origin=(-2.0, -0.6), half_w=3.6, half_h=3.0, radius=0.032)
        tgt = view.dots(pair["tgt"], TEAL)
        z_steps = pair["steps"]
        keep = list(range(len(z_steps[0]["points"])))
        keep = [i for i in keep if view.inside(z_steps[0]["points"][i])]
        src = m.VGroup(*[m.Dot(view.point(z_steps[0]["points"][i]), radius=view.radius, color=VIOLET) for i in keep])
        guess = m.MathTex(r"\text{guess} = (v\,\Delta t,\; 0,\; \omega\,\Delta t)", font_size=40, color=AMBER).move_to([4.2, 2.2, 0])
        self.say("Task two is the initial guess. ICP only finds the nearest good alignment, so where it starts matters. "
                 "Here the robot [turns on the spot, almost fifty degrees] in half a second.",
                 m.AnimationGroup(m.FadeIn(tgt), m.FadeIn(src)))

        def run(steps, color):
            anims = []
            for i in range(1, len(steps)):
                pts = steps[i]["points"]
                # points thrown outside the view fade out instead of covering the header
                nxt = m.VGroup(*[m.Dot(view.point(pts[j]), radius=view.radius, color=color,
                                       fill_opacity=1.0 if view.inside(pts[j]) else 0.0) for j in keep])
                anims.append(m.Transform(src, nxt, run_time=0.45))
            return m.Succession(*anims)

        fail = card(["start at zero:", f"only {9} point pairs,", "result: 11 m sideways — nonsense"], size=22,
                    title="stuck", title_color=RED).move_to([4.2, 0.0, 0])
        self.say("Started at zero, [ICP gets stuck]: only nine point pairs are left, and the result is eleven metres sideways.",
                 m.AnimationGroup(run(z_steps, RED), m.FadeIn(fail)))
        g_steps = pair["guess_steps"]
        restart = m.VGroup(*[m.Dot(view.point(g_steps[0]["points"][j]), radius=view.radius, color=VIOLET) for j in keep])
        ok = card([f"{pair['guess_inliers']} point pairs", f"turn {pair['guess_delta'][2]:+.3f} rad",
                   f"wheels {pair['truth'][2]:+.3f} rad"], size=22, title="converges", title_color=GREEN).move_to([4.2, -1.6, 0])
        self.say("Started at [the filter's guess], speed times d t forward, nothing sideways, and turn rate times d t, "
                 "[it converges]: minus zero point eight four radians, exactly what the wheels say.",
                 m.AnimationGroup(m.FadeIn(guess), m.FadeOut(fail), m.Transform(src, restart)),
                 m.AnimationGroup(run(g_steps, GREEN), m.FadeIn(ok)))
        self.clear()

    # ── 6 ──────────────────────────────────────────────────────────────────────────────
    def fusion(self):
        h = header("ICP into the Kalman filter")
        self.play(m.FadeIn(h))
        z = m.MathTex(r"z = \left[\frac{\Delta x}{\Delta t},\ \frac{\Delta\theta}{\Delta t}\right]", font_size=50).move_to([-3.4, 1.6, 0])
        new = m.MathTex(r"\sigma_v = \frac{\mathrm{rms}}{\Delta t} \qquad \sigma_\omega = \frac{\mathrm{rms}}{\bar r\,\Delta t}",
                        font_size=44, color=GREEN).move_to([-3.0, -0.2, 0])
        newl = T("fixed: leftover distance per scan", 22, GREEN).next_to(new, m.DOWN, buff=0.2)
        old = m.MathTex(r"\sigma_v = \frac{\mathrm{rms}}{\sqrt{N}\,\Delta t} \qquad \sigma_\omega = \frac{\mathrm{rms}}{\sqrt{N}\,\bar r}",
                        font_size=44, color=RED).move_to([3.6, -0.2, 0])
        oldl = T("original", 22, RED).next_to(old, m.DOWN, buff=0.2)
        ez = m.MathTex(r"E_z = \mathrm{diag}(\sigma_v^2,\ \sigma_\omega^2)\,/\,\mathrm{icp\_gain}", font_size=40).move_to([3.4, 1.6, 0])
        nums = T("on the bag: rms ≈ 0.165 m,  N ≈ 248 points,  r̄ ≈ 22 m", 24, MUTED).move_to([0, -2.4, 0])
        self.say("ICP becomes a speed measurement: [the shift divided by the time between the scans], the same kind of measurement as the wheels.",
                 m.Write(z))
        self.say("How much the filter trusts it comes from [the leftover distance] between matched points after the alignment. "
                 "In our bag that is about sixteen centimetres.",
                 m.AnimationGroup(m.Write(new), m.FadeIn(newl), m.FadeIn(ez), m.FadeIn(nums)))
        cross = m.Cross(old, stroke_color=RED, stroke_width=5)
        self.say("The original version also divided by [the square root of the number of points], as if every point were an independent witness. "
                 "They are not. Neighbouring points lie on the same wall and make the same mistake, so ICP was trusted about sixteen times too much.",
                 m.AnimationGroup(m.Write(old), m.FadeIn(oldl)))
        self.say("It also never divided the turn rate error by d t, so ICP [overruled the gyroscope completely].",
                 m.Create(cross))
        self.clear()

    # ── 7 ──────────────────────────────────────────────────────────────────────────────
    def results(self):
        h = header("What the real bag shows")
        self.play(m.FadeIn(h))
        s, ox, oy = 0.36, -5.6, 1.0

        def scr(x, y):
            return [ox - y * s, oy + x * s, 0]

        wheels = polyline([scr(p[1], p[2]) for p in DEMO["path_wheels"]], AMBER)
        icp = polyline([scr(p[1], p[2]) for p in DEMO["path_icp"]], VIOLET)
        start = m.Dot(scr(0, 0), color=FG)
        wl = T("wheels: 26.3 m down the field", 22, AMBER)
        wl.move_to(wheels.get_end() + m.UP * 0.35).align_to(wheels.get_end(), m.RIGHT)
        il = T("ICP alone: 22.3 m", 22, VIOLET)
        il.move_to(icp.get_end() + m.DOWN * 0.75).align_to(icp.get_end(), m.RIGHT)
        self.add(start)
        self.say("Here is the whole forty second drive, from the robot's own start. [The wheels] end twenty six metres down the field. "
                 "[ICP alone] ends at twenty two: it measures too little forward motion, and it even creeps while the robot stands still.",
                 m.AnimationGroup(m.Create(wheels), m.FadeIn(wl)), m.AnimationGroup(m.Create(icp), m.FadeIn(il)))
        why = card(["point-to-point ICP slides along walls", "few rigid features 2–6 m up on an open field",
                    "far points are sparse: 0.5° at 20 m ≈ 17 cm apart"], size=22, title="why ICP looks like this").move_to([-2.8, -2.2, 0])
        self.say("Why? [Point to point ICP slides along walls, the slice above two metres has few features on an open field, and far points are sparse.] "
                 "That is the answer to the question on the exercise sheet.",
                 m.FadeIn(why))
        rows = [("mode", "original", "fixed"),
                ("no ICP", "−78.6°", "−90.0°"),
                ("ICP + filter", "−23.5 m, −85.3°", "−26.3 m, −88.0°")]
        cols_x = [1.7, 3.9, 6.0]  # centres of the three columns
        tbl = m.VGroup()
        for r, row in enumerate(rows):
            for j, c in enumerate(row):
                cell = T(c, 21, MUTED if r == 0 else (GREEN if j == 2 else FG))
                cell.move_to([cols_x[j], -1.6 - r * 0.5, 0])
                tbl.add(cell)
        self.say("With the fixes, [the filter keeps the wheels' distance, and its heading follows the gyroscope]: minus eighty eight degrees. "
                 "Without ICP it now ends at minus ninety, like exercise four.",
                 m.FadeIn(tbl))
        self.clear()

    # ── 8 ──────────────────────────────────────────────────────────────────────────────
    def fixes(self):
        h = header("Fixed after testing on the bag")
        self.play(m.FadeIn(h))
        b1 = T("1   the node froze when the bag looped → reset when time jumps back > 1 s", 26, FG)
        b2 = T("2   ICP noise: no √N, and σω divided by Δt", 26, FG)
        b3 = T("3   /ouster/imu drifted 14° → /imu/data_raw, gyro only", 26, FG)
        b4 = T("the TASK code itself (getTransform, guess, ICP loop) was correct", 26, GREEN)
        g = m.VGroup(b1, b2, b3, b4).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.45).move_to([0, 0.1, 0])
        self.say("Three things were fixed after testing: [the node froze when the bag looped], [the ICP noise formula], "
                 "and [the IMU topic], whose gyroscope drifted fourteen degrees in forty seconds.",
                 m.FadeIn(b1, shift=m.RIGHT * 0.2), m.FadeIn(b2, shift=m.RIGHT * 0.2), m.FadeIn(b3, shift=m.RIGHT * 0.2))
        self.say("The tasks themselves, [getTransform, the initial guess and the ICP loop, were correct]. "
                 "The long walkthrough video goes through every line of the code, and the questions you might get in the Testat.",
                 m.FadeIn(b4))
        self.wait(1)
        self.clear()
