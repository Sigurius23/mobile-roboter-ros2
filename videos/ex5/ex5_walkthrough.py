"""Exercise 5 code walkthrough: every block of icp_node.cpp, the choices behind it, and
the questions you might get in the Testat (long video).

Render:  manimgx render ex5_walkthrough.py Ex5Walkthrough
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import manimgx as m

from common import (AMBER, DEMO, FG, GREEN, MUTED, PANEL, RED, TEAL, VIOLET, T, card, code, header, lines_block,
                    mark, qa_card)
from icp_anim import View, counter, pair_lines, source_indices, step_to
from winvoice import windows_voice

LAUNCH = (Path(__file__).parent.parent.parent / "ros2_ws" / "src" / "icp_odom" / "launch" / "icp_odom.launch.py") \
    .read_text(encoding="utf-8").splitlines()


CODE_TOP = 2.45


def notes(c, lines, size=22, color=FG):
    """Short notes in two columns under the code block c; their order is the order of `lines`."""
    half = (len(lines) + 1) // 2
    cols = [lines_block(lines[:half], size=size, color=color, buff=0.16)]
    if lines[half:]:
        cols.append(lines_block(lines[half:], size=size, color=color, buff=0.16))
    g = m.VGroup(*cols).arrange(m.RIGHT, buff=0.9, aligned_edge=m.UP)
    if g.width > 13.0:
        g.scale_to_fit_width(13.0)
    g.next_to(c, m.DOWN, buff=0.3).set_x(0)
    if g.get_bottom()[1] < -3.85:
        g.scale(max(0.6, (c.get_bottom()[1] - 0.3 + 3.85) / g.height)).next_to(c, m.DOWN, buff=0.3).set_x(0)
    # flat list in the original order, so n[i] is lines[i]
    flat = m.VGroup(*[row for col in cols for row in col])
    return flat


def place_code(c, max_w=13.2, max_h=4.4):
    if c.width > max_w:
        c.scale_to_fit_width(max_w)
    if c.height > max_h:
        c.scale_to_fit_height(max_h)
    c.move_to([0, CODE_TOP - c.height / 2, 0])
    return c


class Ex5Walkthrough(m.Scene):
    voice = staticmethod(windows_voice)

    # ── helpers ────────────────────────────────────────────────────────────────────────
    def clear(self, run_time=0.5):
        if self.mobjects:
            self.play(*[m.FadeOut(x) for x in self.mobjects], run_time=run_time)

    def chapter(self, number, title, kicker=None):
        self.clear()
        n = T(f"{number:02d}", 30, AMBER)
        t = T(title, 50, FG, weight=m.BOLD)
        g = m.VGroup(n, t).arrange(m.DOWN, buff=0.25)
        self.play(m.FadeIn(g, shift=m.UP * 0.2), run_time=0.7)
        self.wait(0.6)
        self.play(m.FadeOut(g), run_time=0.4)
        h = header(title, kicker or f"{number:02d}")
        self.play(m.FadeIn(h), run_time=0.4)
        return h

    def show_code(self, first, last, size=17):
        c = place_code(code(first, last, size))
        self.play(m.FadeIn(c), run_time=0.6)
        return c

    def focus(self, c, first, last, script, *extra, box=None):
        """Box lines first..last of a code block while the voice explains them."""
        new = mark(c, first, last)
        anim = m.Create(new) if box is None else m.ReplacementTransform(box, new)
        self.say(script, m.AnimationGroup(anim, *extra))
        return new

    def qa(self, question, answer_lines, answer_speech, spoken=None):
        self.clear(0.4)
        q = qa_card(question).move_to([0, 2.0, 0])
        if q.width > 13:
            q.scale_to_fit_width(13)
        a = lines_block(answer_lines, size=27, color=FG, buff=0.28)
        if a.width > 12.4:
            a.scale_to_fit_width(12.4)
        a.next_to(q, m.DOWN, buff=0.7).align_to(q, m.LEFT).shift(m.RIGHT * 0.4)
        self.say("Question: " + (spoken or question), m.FadeIn(q, shift=m.DOWN * 0.2))
        self.say(answer_speech, m.LaggedStart(*[m.FadeIn(x, shift=m.RIGHT * 0.15) for x in a], lag_ratio=0.35))
        self.wait(0.4)

    # ── film ───────────────────────────────────────────────────────────────────────────
    def construct(self) -> None:
        self.intro()
        self.ch_node()
        self.ch_time()
        self.ch_predict()
        self.ch_odom_imu()
        self.ch_task1()
        self.ch_scan()
        self.ch_task2()
        self.ch_setup()
        self.ch_loop()
        self.ch_stats()
        self.ch_fusion()
        self.ch_pure()
        self.ch_launch()
        self.ch_results()
        self.ch_quiz()

    # ── 00 ─────────────────────────────────────────────────────────────────────────────
    def intro(self):
        k = T("MOBILE ROBOTS · EXERCISE 5 · CODE WALKTHROUGH", 24, AMBER)
        t = T("icp_node.cpp, line by line", 58, FG, weight=m.BOLD)
        s = T("every block, every choice, and the questions you might be asked", 28, MUTED)
        g = m.VGroup(k, t, s).arrange(m.DOWN, buff=0.35)
        self.say("This is the long walkthrough of exercise five. We go through the ICP node block by block, in the order the data "
                 "flows: what each part does, why it is written this way, and which questions you might get in the Testat.",
                 m.LaggedStart(m.FadeIn(k), m.Write(t), m.FadeIn(s), lag_ratio=0.4))
        self.play(m.FadeOut(g))
        items = ["01  the node and its callbacks", "02  time, and the reset when the bag loops", "03  predict",
                 "04  odometry and IMU", "05  TASK 1: getTransform", "06  scan to points", "07  TASK 2: the initial guess",
                 "08  ICP set-up: the inverse trick", "09  TASK 3: the ICP loop", "10  statistics and result",
                 "11  ICP into the filter", "12  pure-ICP mode", "13  launch parameters", "14  results on the bag",
                 "15  quick quiz"]
        left = lines_block(items[:8], 26, FG, buff=0.24)
        right = lines_block(items[8:], 26, FG, buff=0.24)
        cols = m.VGroup(left, right).arrange(m.RIGHT, buff=1.0, aligned_edge=m.UP).move_to([0, -0.2, 0])
        h = header("Roadmap")
        self.say("The code shown is the fixed version from your repository. Where it differs from the team's original, "
                 "I say what the original did and why it was changed.",
                 m.AnimationGroup(m.FadeIn(h), m.LaggedStart(*[m.FadeIn(x) for x in [*left, *right]], lag_ratio=0.08)))

    # ── 01 ─────────────────────────────────────────────────────────────────────────────
    def ch_node(self):
        self.chapter(1, "The node and its callbacks")
        c = self.show_code(82, 96)
        n = notes(c, ["one class: LioNode : rclcpp::Node", "three independent subscriptions",
                   "one shared state_ and filter_time_", "no synchronizer (unlike Exercise 4)"])
        b = self.focus(c, 82, 84, "The node is one class, LioNode, which inherits from r c l c p p node. "
                       "First it creates [a TF broadcaster, to publish our pose, and a TF buffer with a listener, which getTransform will ask.]",
                       m.FadeIn(n[0]))
        b = self.focus(c, 86, 96, "Then three independent subscriptions: wheel odometry, the IMU, and the two dimensional lidar scan. "
                       "Each callback runs at its own sensor's rate, and all of them update one shared state.",
                       m.FadeIn(n[1]), m.FadeIn(n[2]), box=b)
        self.say("Unlike exercise four, there is no message synchronizer. A slow sensor can't hold back a fast one, and no message is thrown away.",
                 m.FadeIn(n[3]))
        self.clear(0.3)
        self.add(header("The node and its callbacks", "01"))
        c = self.show_code(57, 67)
        n = notes(c, ["use_icp, icp_only: the three modes", "icp_gain: how much to trust ICP",
                   "base_frame: base_footprint", "set by the launch file, no recompiling"])
        b = self.focus(c, 57, 62, "The parameters come from the launch file. use ICP and ICP only switch between the three modes, "
                       "and ICP gain scales how much the filter trusts ICP.", m.FadeIn(n[0]), m.FadeIn(n[1]))
        self.focus(c, 66, 67, "The base frame is base footprint, the frame every measurement is converted into. "
                   "Because these are parameters, you can switch modes in the launch file without recompiling.",
                   m.FadeIn(n[2]), m.FadeIn(n[3]), box=b)
        self.qa("Why a class with members instead of global variables?",
                ["The node keeps its memory together and private.", "Callbacks become member functions with access to state_.",
                 "It's the standard ROS 2 style; Exercise 4's skeleton used globals."],
                "The node's memory, the state, the last scan and the TF objects, is kept together and private, and the callbacks "
                "become member functions that can reach it. It's the standard ROS two style. Exercise four's skeleton used globals instead.")
        self.qa("What happens to the TF broadcaster and the TF listener?",
                ["broadcaster: publishes our transform odom → base_footprint", "listener: subscribes /tf and /tf_static, fills the buffer",
                 "getTransform asks the buffer for sensor mountings"],
                "The broadcaster publishes our own result, odom to base footprint. The listener subscribes to slash TF and slash TF static "
                "and fills the buffer, and getTransform asks that buffer how each sensor is mounted.")

    # ── 02 ─────────────────────────────────────────────────────────────────────────────
    def ch_time(self):
        self.chapter(2, "Time, and the reset when the bag loops", "02 · fixed")
        c = self.show_code(111, 129)
        n = notes(c, ["dt = t − filter_time_", "a few ms backwards: normal → ignore", "more than 1 s backwards: bag looped",
                   "→ resetFilter: origin, initial Ex,", "   forget the last scan"])
        b = self.focus(c, 111, 111, "Every callback first computes d t, the time since the last update by any sensor. "
                       "Odometry and IMU messages interleave, so a stamp a few milliseconds older than the last one is normal, "
                       "and that message is simply ignored.", m.FadeIn(n[0]), m.FadeIn(n[1]))
        b = self.focus(c, 120, 129, "A jump back of more than one second means the bag started over, or Minot ran init again. "
                       "Then reset filter puts the robot back at the origin with the initial uncertainty, and forgets the last scan.",
                       m.FadeIn(n[2]), m.FadeIn(n[3]), m.FadeIn(n[4]), box=b)
        self.clear(0.3)
        self.add(header("Time, and the reset when the bag loops", "02 · fixed"))
        c = self.show_code(195, 202)
        self.focus(c, 196, 200, "This check now sits in the odometry, the IMU and the scan callback. "
                   "The scan callback needs its own, because in pure ICP mode the other two never advance filter time.")
        self.qa("What went wrong when the bag looped in the original?",
                ["Each callback returned on dt ≤ 0 without updating its stored time.", "After the loop every message was 'older' than the end of the bag,",
                 "so the node ignored everything. On the bag: last transform at 44.5 s."],
                "Each callback returned on d t smaller or equal zero, without updating its stored time. After the loop, every message "
                "was older than the stored end of the bag, so the node ignored everything forever. On the bag, the last transform came at "
                "forty four and a half seconds, and then the robot froze.")
        self.qa("Why a threshold of one second, and not zero?",
                ["Odom (50 Hz) and IMU (125 Hz) arrive interleaved:", "a few ms backwards happens all the time.",
                 "Resetting on those would reset constantly. 1 s is far above that,", "and far below the 40 s bag length."],
                "Odometry at fifty hertz and IMU at a hundred and twenty five hertz arrive interleaved, so a few milliseconds backwards "
                "happens all the time. A threshold of zero would reset constantly. One second is far above that jitter and far below the bag length.")

    # ── 03 ─────────────────────────────────────────────────────────────────────────────
    def ch_predict(self):
        self.chapter(3, "predict: the motion model")
        c = self.show_code(149, 166)
        n = notes(c, ["state x = [x, y, θ, v, ω, a]", "A: constant-acceleration model", "Eu: process noise per second, × dt",
                   "skip dt ≤ 10 µs"])
        b = self.focus(c, 152, 162, "Predict is the Exercise four filter. The state has six entries: position, heading, speed, turn rate "
                       "and acceleration. The matrix A moves it forward by d t: cos theta and sin theta turn the forward speed "
                       "into x and y, and the half a d t squared terms add the acceleration.", m.FadeIn(n[0]), m.FadeIn(n[1]))
        b = self.focus(c, 164, 166, "E u is the process noise: how much the model may be wrong per second. It is multiplied by d t, "
                       "so a longer gap without measurements adds more uncertainty.", m.FadeIn(n[2]), box=b)
        self.focus(c, 149, 150, "And predict returns at once if d t is below ten microseconds: two messages with the same stamp "
                   "need no prediction between them.", m.FadeIn(n[3]), box=b)
        self.qa("Why is predict called in every callback?",
                ["Every measurement belongs to its own time stamp.", "Before correcting with it, the state is moved to that time.",
                 "One shared filter_time_ for all sensors."],
                "Every measurement belongs to its own time. Before the filter corrects with it, it moves the state forward to exactly that time. "
                "That's why all callbacks share one filter time.")
        self.qa("Is anything missing before the ICP correction?",
                ["The scan callback does not predict to the scan's time first.", "It corrects the state of the last odom/IMU message,",
                 "a few ms earlier: small at these rates, but not strictly right."],
                "Yes, one small thing: the scan callback doesn't predict to the scan's own time stamp first. It corrects the state of the "
                "last odometry or IMU message, a few milliseconds earlier. At these rates that's small, but strictly it should predict first.")

    # ── 04 ─────────────────────────────────────────────────────────────────────────────
    def ch_odom_imu(self):
        self.chapter(4, "Odometry and IMU", "04 · IMU fixed")
        c = self.show_code(204, 216)
        n = notes(c, ["twist in child_frame_id", "→ rotate into base_footprint", ".linear(): rotation only"])
        b = self.focus(c, 204, 206, "The odometry callback reads the forward speed and the turn rate from the twist. "
                       "The twist is given in the message's child frame.", m.FadeIn(n[0]))
        self.focus(c, 208, 216, "If that frame isn't base footprint, the vectors are rotated into it with dot linear, the rotation part "
                   "of the transform. In our bag the child frame already is base footprint, so nothing changes.",
                   m.FadeIn(n[1]), m.FadeIn(n[2]), box=b)
        self.clear(0.3)
        self.add(header("Odometry and IMU", "04 · IMU fixed"))
        c = self.show_code(282, 296)
        n = notes(c, ["topic: /imu/data_raw (was /ouster/imu)", "z = [ω]   (Ez 0.05)", "a only with imu_use_acc:=true",
                   "/ouster/imu: 0.006 rad/s offset", "= 14° over the 40 s bag"])
        self.focus(c, 283, 296, "The IMU callback now uses the Phidget IMU on I M U data raw, and only its gyroscope: z is just the turn rate. "
                   "The forward acceleration is only added when I M U use acc is set.", m.FadeIn(n[0]), m.FadeIn(n[1]), m.FadeIn(n[2]))
        self.say("The original used the Ouster's IMU. Its gyroscope reads zero point zero zero six radians per second while standing still. "
                 "Over the forty second bag that is fourteen degrees, and the filter trusted it.", m.FadeIn(n[3]), m.FadeIn(n[4]))
        self.qa("Why rotate IMU and odom vectors with .linear() only?",
                ["Speeds and accelerations are directions, not positions.", "A mounting offset doesn't change a direction,",
                 "only the rotation applies. Points use .matrix() (rotate + shift)."],
                "Speeds and accelerations are directions, not positions. Where a sensor is mounted doesn't change a direction, only how it is "
                "turned, so only the rotation applies. Whole point clouds use dot matrix, rotation and shift.",
                spoken="Why are IMU and odometry vectors rotated with dot linear only?")
        self.qa("Why is the IMU's acceleration not used any more?",
                ["Phidget: TF says turned +90°, the data says forward = +acc.y", "(TF would make it −acc.y: the wrong sign).",
                 "Ouster IMU: 0.5 m/s² forward while standing still.", "A wrong acceleration only hurts v; the wheels measure v well."],
                "For the Phidget, TF says the sensor is turned by plus ninety degrees, but the data says forward is plus acc y. "
                "With TF it would get the wrong sign. The Ouster IMU reads half a metre per second squared while standing. "
                "A wrong acceleration only hurts the speed, and the wheels measure the speed well anyway.")

    # ── 05 ─────────────────────────────────────────────────────────────────────────────
    def ch_task1(self):
        self.chapter(5, "TASK 1: getTransform", "05 · TASK 1")
        c = self.show_code(312, 329)
        n = notes(c, ["lookupTransform(target, source, time, 0.1 s)", "→ Eigen::Isometry3d (rotation + shift)",
                   "true, or warn (max every 2 s) + false"])
        b = self.focus(c, 312, 317, "Task one. Inside a try block, look up transform asks the TF buffer: where is the source frame, relative to "
                       "base footprint, at the message's time? It waits at most a tenth of a second.", m.FadeIn(n[0]))
        b = self.focus(c, 319, 321, "transform to Eigen turns the ROS message into an Eigen isometry, a rotation plus a translation, "
                       "written into the output parameter, and the function returns true.", m.FadeIn(n[1]), box=b)
        self.focus(c, 323, 329, "If TF can't answer, the exception is caught, a warning is printed at most every two seconds, and it returns false.",
                   m.FadeIn(n[2]), box=b)
        self.qa("lookupTransform(base_frame_, source_frame, …): which way round?",
                ["Target first, then source.", "The result converts data FROM the sensor frame INTO the base frame:",
                 "p_base = T · p_sensor"],
                "Target first, then source. The result converts data from the sensor frame into the base frame: p base equals T times p sensor.",
                spoken="lookupTransform with base frame first and source frame second: which way does it convert?")
        self.qa("Why the message's time stamp, and why a 0.1 s timeout?",
                ["Transforms can change over time: ask for the one valid", "when the data was measured (needed for moving joints).",
                 "At startup /tf_static may not have arrived yet:", "waiting briefly avoids failing the first messages."],
                "Transforms can change over time, so you ask for the one valid when the data was measured. For a fixed mount it makes no "
                "difference, but it's the right habit. The short timeout helps at startup, when TF static may not have arrived yet.")
        self.qa("What happens without getTransform on this robot?",
                ["os_lidar is turned 180° and sits 0.54 m up.", "Real scan pair: −0.68 m instead of +0.68 m.",
                 "ICP would make the robot drive backwards", "(the 'inverted' warning on the sheet)."],
                "On this robot the lidar frame is turned by a hundred and eighty degrees. With our real scan pair, ICP then says minus zero "
                "point six eight metres instead of plus: the robot would drive backwards. That's the inverted warning on the exercise sheet.")

    # ── 06 ─────────────────────────────────────────────────────────────────────────────
    def ch_scan(self):
        self.chapter(6, "Scan to points")
        c = self.show_code(336, 346)
        n = notes(c, ["each ray: angle, range r", "point = (r cos α, r sin α)", "skip inf / NaN / out of range",
                   "~722 rays → ~250 points"])
        b = self.focus(c, 337, 346, "The scan callback turns every ray into a point: x is r times cos of the angle, y is r times sin. "
                       "The angle starts at angle min and grows by angle increment per ray.", m.FadeIn(n[0]), m.FadeIn(n[1]))
        self.focus(c, 341, 341, "Rays that hit nothing come as infinity, because the launch file sets use inf. Those, and anything outside "
                   "the range limits, are skipped. Of about seven hundred rays, roughly two hundred and fifty points remain.",
                   m.FadeIn(n[2]), m.FadeIn(n[3]), box=b)
        self.clear(0.3)
        self.add(header("Scan to points", "06"))
        c = self.show_code(349, 368)
        n = notes(c, ["whole cloud → base_footprint", "no transform → skip the scan", "first scan: just remember it"])
        b = self.focus(c, 352, 360, "Then the whole cloud is moved into the robot frame with transform point cloud and the full matrix. "
                       "Without a transform, the scan is skipped, because points in the wrong frame are worse than none.",
                       m.FadeIn(n[0]), m.FadeIn(n[1]))
        self.focus(c, 362, 368, "The very first scan has nothing to compare with, so it is only stored as the last cloud.",
                   m.FadeIn(n[2]), box=b)
        self.qa("Why is the whole scan skipped when the transform is missing?",
                ["ICP compares two clouds. One in the wrong frame", "gives a wrong motion, and the filter would trust it.",
                 "Odom and IMU fall back to raw values instead:", "for them the base frame is a good guess."],
                "ICP compares two clouds, and one in the wrong frame gives a wrong motion that the filter would trust. So the scan is dropped. "
                "Odometry and IMU fall back to their raw values instead, because for them the base frame is a reasonable guess.")

    # ── 07 ─────────────────────────────────────────────────────────────────────────────
    def ch_task2(self):
        self.chapter(7, "TASK 2: the initial guess", "07 · TASK 2")
        c = self.show_code(384, 392)
        n = notes(c, ["vx = x(3), w = x(4) from the filter", "guess = (vx·dt, 0, w·dt)", "sideways 0: wheels can't slide"])
        self.focus(c, 384, 392, "Task two takes the speed and the turn rate from the Kalman state, and predicts the motion since the last scan: "
                   "speed times d t forward, nothing sideways, turn rate times d t.", m.FadeIn(n[0]), m.FadeIn(n[1]), m.FadeIn(n[2]))
        self.clear(0.3)
        self.add(header("TASK 2: the initial guess", "07 · TASK 2 · real scans while turning"))
        pair = DEMO["pairs"]["turn"]
        view = View(scale=0.085, center=(5.0, 0.0), origin=(-2.0, -0.6), half_w=3.6, half_h=3.0, radius=0.032)
        tgt = view.dots(pair["tgt"], TEAL)
        z_steps, g_steps = pair["steps"], pair["guess_steps"]
        keep = [i for i in range(len(z_steps[0]["points"])) if view.inside(z_steps[0]["points"][i])]

        def at(steps, i, color):
            pts = steps[i]["points"]
            return m.VGroup(*[m.Dot(view.point(pts[j]), radius=view.radius, color=color,
                                    fill_opacity=1.0 if view.inside(pts[j]) else 0.0) for j in keep])

        src = at(z_steps, 0, VIOLET)
        self.say("Why it matters: here the robot turns on the spot, almost fifty degrees within half a second.",
                 m.AnimationGroup(m.FadeIn(tgt), m.FadeIn(src)))
        fail = card(["start at zero", "9 point pairs left", "result: 11 m sideways"], size=22, title="stuck", title_color=RED) \
            .move_to([4.3, 0.8, 0])
        self.say("Started at zero, most points are more than a metre from any partner. Only nine pairs survive, and ICP returns nonsense.",
                 m.AnimationGroup(m.Succession(*[m.Transform(src, at(z_steps, i, RED), run_time=0.5) for i in range(1, len(z_steps))]),
                                  m.FadeIn(fail)))
        ok = card([f"{pair['guess_inliers']} point pairs", f"turn {pair['guess_delta'][2]:+.3f} rad",
                   f"wheels {pair['truth'][2]:+.3f} rad"], size=22, title="from the guess", title_color=GREEN).move_to([4.3, -1.6, 0])
        self.say("Started at the filter's guess, it converges in thirteen iterations to minus zero point eight four radians, what the wheels say.",
                 m.Succession(m.Transform(src, at(g_steps, 0, VIOLET), run_time=0.6),
                              m.Succession(*[m.Transform(src, at(g_steps, i, GREEN), run_time=0.3) for i in range(1, len(g_steps))]),
                              m.FadeIn(ok)))
        self.qa("Why is the sideways guess exactly zero?",
                ["A wheeled robot can't slide sideways (non-holonomic).", "On an arc it ends up about v·dt·sin(ω·dt/2) to the side:",
                 "under a millimetre at 20 Hz. ICP corrects that easily."],
                "A wheeled robot can't slide sideways. Strictly, on an arc it ends up slightly to the side, but at twenty scans per second "
                "that's under a millimetre, and ICP corrects it easily.")
        self.qa("Why does pure-ICP mode start at zero?",
                ["In icp_only the filter doesn't run, so x(3) and x(4) stay 0.", "ICP must find the whole motion itself.",
                 "Fine for 0.05 s steps; it fails on big gaps or fast turns."],
                "In ICP only mode the filter doesn't run, so the speed and turn rate in the state stay zero. ICP has to find the whole motion "
                "itself. Between two scans fifty milliseconds apart that works, but on big gaps or fast turns it fails, as we just saw.")

    # ── 08 ─────────────────────────────────────────────────────────────────────────────
    def ch_setup(self):
        self.chapter(8, "ICP set-up: the inverse trick")
        c = self.show_code(486, 495)
        n = notes(c, ["guess = robot motion", "ICP moves points, not the robot", "points move the opposite way",
                   "→ invert the guess (T_guess_point)"])
        self.focus(c, 486, 495, "The guess describes how the robot moved. But ICP moves the old points onto the new ones, and points move the "
                   "opposite way: drive one metre towards a wall, and the wall comes one metre closer. So the guess becomes a three by three "
                   "matrix and is inverted.", m.LaggedStart(*[m.FadeIn(x) for x in n], lag_ratio=0.5))
        self.clear(0.3)
        self.add(header("ICP set-up: the inverse trick", "08"))
        c = self.show_code(504, 517)
        n = notes(c, ["move every old point by the guess", "T_total starts at the guess", "kd-tree on the target, built once"])
        b = self.focus(c, 504, 512, "Every old point is moved by that inverted guess, so ICP starts close to the answer.", m.FadeIn(n[0]))
        self.focus(c, 514, 517, "The total transform starts there, and a k d tree is built once on the new scan.",
                   m.FadeIn(n[1]), m.FadeIn(n[2]), box=b)
        self.qa("Why build the kd-tree on the target and not the source?",
                ["We ask: nearest TARGET point for each source point.", "The searched set goes in the tree.",
                 "The target never moves, so one tree serves all 20 iterations."],
                "Each iteration asks: which target point is nearest to this source point? The set you search goes in the tree. "
                "And the target never moves, so one tree serves all twenty iterations.")

    # ── 09 ─────────────────────────────────────────────────────────────────────────────
    def ch_loop(self):
        self.chapter(9, "TASK 3: the ICP loop", "09 · TASK 3")
        c = self.show_code(521, 539)
        n = notes(c, ["1  nearest neighbour per old point", "keep if distance² < 1.0 (1 m)", "fewer than 3 pairs → stop"])
        b = self.focus(c, 526, 535, "Step one: for every old point, the k d tree finds the single nearest new point. The pair is kept if the "
                       "squared distance is below one, so closer than one metre.", m.FadeIn(n[0]), m.FadeIn(n[1]))
        self.focus(c, 537, 539, "With fewer than three pairs, a rotation and a shift can't be determined reliably, so the loop stops.",
                   m.FadeIn(n[2]), box=b)
        self.clear(0.3)
        self.add(header("TASK 3: the ICP loop", "09 · TASK 3"))
        c = self.show_code(541, 557)
        n = notes(c, ["2  centroids s̄, t̄", "3  H = Σ (s − s̄)(t − t̄)ᵀ   (2×2)"])
        b = self.focus(c, 541, 550, "Step two: the centre of each group of paired points.", m.FadeIn(n[0]))
        b = self.focus(c, 552, 557, "Step three: the cross covariance H, a two by two matrix of the centred pairs. Subtracting the centres "
                       "removes the shift, so H holds only how the shapes are turned against each other.", m.FadeIn(n[1]), box=b)
        self.clear(0.3)
        self.add(header("TASK 3: the ICP loop", "09 · TASK 3"))
        c = self.show_code(559, 569)
        n = notes(c, ["4  SVD H = U Σ Vᵀ,  R = V Uᵀ", "   det R < 0 → flip V's last column", "   t = t̄ − R s̄"])
        b = self.focus(c, 559, 562, "Step four: the singular value decomposition of H gives the best rotation directly, R equals V times U "
                       "transposed. This is the Kabsch method, a closed form least squares solution, no searching.", m.FadeIn(n[0]))
        self.focus(c, 563, 569, "If R came out as a mirror image, its determinant is negative, and flipping V's last column gives the best proper "
                   "rotation. Then the shift t moves the rotated old centre onto the new centre.", m.FadeIn(n[1]), m.FadeIn(n[2]), box=b)
        self.clear(0.3)
        self.add(header("TASK 3: the ICP loop", "09 · TASK 3"))
        c = self.show_code(571, 590, size=16)
        n = notes(c, ["5  move old points by R, t", "6  T_total = T_step · T_total", "7  stop: |t| < 0.1 mm and |angle| < 0.0001 rad",
                   "   at most 20 iterations"])
        b = self.focus(c, 571, 578, "Step five moves every old point by R and t, so in the next round the nearest neighbours, and the pairs, change.",
                       m.FadeIn(n[0]))
        b = self.focus(c, 580, 584, "Step six writes R and t into a three by three step matrix and multiplies it onto the total, from the left, "
                       "because it happens after all earlier steps.", m.FadeIn(n[1]), box=b)
        self.focus(c, 586, 589, "Step seven stops when the step is tiny: less than a tenth of a millimetre and a ten thousandth of a radian. "
                   "Otherwise it repeats, at most twenty times.", m.FadeIn(n[2]), m.FadeIn(n[3]), box=b)
        self.clear(0.3)
        self.add(header("TASK 3 on real scans", "09 · two scans 0.5 s apart, one corner zoomed"))
        pair = DEMO["pairs"]["straight"]
        steps = pair["steps"]
        view = View(scale=0.95, center=(-14.0, 12.0), origin=(-1.6, -0.1), half_w=4.8, half_h=2.6, radius=0.05)
        keep = source_indices(view, steps)
        tgt = view.dots(pair["tgt"], TEAL)
        src = step_to(view, steps, 0, keep)
        it = counter("iteration 0", [5.0, 1.8, 0])
        self.add(view.frame())
        self.say("Here it is on real data: violet, the old scan, teal, the new one, zoomed into one corner.",
                 m.AnimationGroup(m.FadeIn(src), m.FadeIn(tgt), m.FadeIn(it)))
        anims = []
        for i in range(len(steps) - 1):
            ln = pair_lines(view, steps[i])
            anims.append(m.Succession(m.Create(ln, run_time=0.5),
                                      m.AnimationGroup(m.Transform(src, step_to(view, steps, i + 1, keep)), m.FadeOut(ln),
                                                       m.Transform(it, counter(f"iteration {i + 1}", [5.0, 1.8, 0])), run_time=0.8)))
        res = card([f"forward {pair['delta'][0]:+.3f} m", f"wheels {pair['truth'][0]:+.3f} m", f"{pair['inliers']} pairs"],
                   size=21, title="result").move_to([5.0, -0.6, 0])
        self.say("Pair, align, move, and again. After eight iterations the step is below the threshold. The robot moved zero point six eight "
                 "metres forward; the wheels say zero point six three.", m.AnimationGroup(m.Succession(*anims), m.FadeIn(res)))
        self.qa("Why compare squared distances against 1.0?",
                ["The kd-tree returns squared distances.", "Comparing d² < 1.0 means d < 1 m without a square root.",
                 "Careful: d = exactly 1 m is rejected (1.0 is not < 1.0)."],
                "The k d tree already returns squared distances. Comparing the square with one means closer than one metre, without "
                "computing a square root. One trap in worked examples: exactly one metre is rejected, because one is not smaller than one.",
                spoken="Why compare squared distances against one?")
        self.qa("Why one metre, and not more or less?",
                ["Too big: pairs between different objects (wrong matches).", "Too small: no pairs when the guess is a bit off.",
                 "Between scans 0.05 s apart the robot moves ~5 cm,", "so 1 m leaves a large margin."],
                "Too big, and points pair up with different objects. Too small, and there are no pairs when the guess is a bit off. "
                "Between two scans fifty milliseconds apart the robot moves about five centimetres, so one metre leaves a large margin.")
        self.qa("Where does R = V·Uᵀ come from?",
                ["We want R minimising Σ ‖R sᵢ + t − tᵢ‖².", "With centred points that's maximising trace(R H).",
                 "SVD of H gives the maximum at R = V Uᵀ (Kabsch / Arun 1987)."],
                "We want the rotation that minimises the summed squared distances. With centred points that's the same as maximising the trace "
                "of R times H, and the singular value decomposition gives that maximum at R equals V U transposed.",
                spoken="Where does R equals V times U transposed come from?")
        self.qa("Why T_step · T_total, and not T_total · T_step?",
                ["T_total maps the original points to where they were", "before this step. The new step comes after it.",
                 "Products apply right to left, so the newest goes on the left."],
                "T total maps the original points to where they were before this step, and the new step is applied after that. Matrix "
                "products apply from right to left, so the newest step goes on the left.",
                spoken="Why T step times T total, and not the other way round?")

    # ── 10 ─────────────────────────────────────────────────────────────────────────────
    def ch_stats(self):
        self.chapter(10, "Statistics and result")
        c = self.show_code(593, 612, size=16)
        n = notes(c, ["pair again after the loop (< 1 m)", "N = inliers", "residual_rms: typical leftover gap",
                   "avg_radius: how far away they are"])
        self.focus(c, 598, 612, "After the loop, the pairs are formed once more to measure the fit: N, the number of inliers, the root mean "
                   "square of the leftover gaps, and the average distance of the points from the robot. On the bag, typically two hundred "
                   "and fifty points, sixteen centimetres, and twenty two metres.", m.LaggedStart(*[m.FadeIn(x) for x in n], lag_ratio=0.4))
        self.clear(0.3)
        self.add(header("Statistics and result", "10"))
        c = self.show_code(622, 628)
        self.focus(c, 622, 624, "Then the total transform is inverted back into robot motion, the other half of the inverse trick: "
                   "forward, sideways and turn angle.")
        self.qa("Does a small residual mean the match is right?",
                ["No. Along a wall or in a corridor, point-to-point ICP slides:", "every point still has a close partner,",
                 "so the residual is small while the motion is wrong.", "It measures fit, not whether the motion is determined."],
                "No. Along a long wall, point to point ICP slides: every point still finds a close partner, so the residual is small while "
                "the motion is wrong. The residual measures how well points fit, not whether the motion is determined.")

    # ── 11 ─────────────────────────────────────────────────────────────────────────────
    def ch_fusion(self):
        self.chapter(11, "ICP into the Kalman filter", "11 · noise fixed")
        c = self.show_code(398, 406)
        n = notes(c, ["only if at least 10 pairs", "z = [Δx/dt, Δθ/dt]: a speed,", "like the wheels (same H rows)"])
        self.focus(c, 398, 402, "With at least ten pairs, the ICP result becomes a measurement. It's turned into a speed: the forward shift and "
                   "the turn, divided by the time between the scans, the same kind of measurement as the wheels.",
                   m.LaggedStart(*[m.FadeIn(x) for x in n], lag_ratio=0.4))
        self.clear(0.3)
        self.add(header("ICP into the Kalman filter", "11 · noise fixed"))
        c = self.show_code(420, 439, size=16)
        n = notes(c, ["σ_v = rms / dt        ∈ [0.05, 5]", "σ_ω = rms / r̄ / dt   ∈ [0.01, 2]", "Ez = diag(σ²) / icp_gain",
                   "bag: σ_v ≈ 3.3 m/s, σ_ω ≈ 0.16 rad/s"], size=21)
        b = self.focus(c, 420, 424, "The uncertainty comes from the leftover gap, about sixteen centimetres per scan. Divided by d t it becomes a "
                       "speed error. For the turn, a rotation moves a point at distance r by r times the angle, so it is also divided by the "
                       "average radius.", m.FadeIn(n[0]), m.FadeIn(n[1]))
        self.focus(c, 432, 439, "Squared into E z, and divided by ICP gain: a gain of two means trust it twice as much. On the bag the speed "
                   "error is about three metres per second, so the speed comes from the wheels, while ICP still counts for the turn rate.",
                   m.FadeIn(n[2]), m.FadeIn(n[3]), box=b)
        old = m.MathTex(r"\text{original: }\ \sigma = \frac{\mathrm{rms}}{\sqrt{N}},\quad \sigma_\omega = \frac{\sigma}{\bar r}\ (\text{no }\Delta t)",
                        font_size=34, color=RED).move_to(n.get_center())
        self.say("The original divided by the square root of N, about sixteen, and forgot d t for the turn rate. That made ICP far too trusted.",
                 m.Succession(m.FadeOut(n, run_time=0.4), m.Write(old)))
        self.clear(0.3)
        self.add(header("ICP into the Kalman filter", "11"))
        c = self.show_code(441, 454, size=16)
        self.focus(c, 453, 454, "The update itself is the usual Kalman correction, with the Joseph form for the covariance, which stays symmetric "
                   "and positive even with rounding errors.")
        self.qa("Why was σ = rms / √N replaced?",
                ["√N assumes N independent measurements.", "Lidar points aren't: neighbours lie on the same wall,",
                 "make the same error, slide together.", "Bag: ICP looked ~16× too precise → fused path 2.7 m short."],
                "Dividing by the square root of N assumes N independent measurements. Lidar points aren't independent: neighbours lie on the "
                "same wall, make the same error and slide together. On the bag ICP looked about sixteen times too precise, and the fused path "
                "came out two point seven metres short.",
                spoken="Why was sigma equals r m s over the square root of N replaced?")
        self.qa("Why must σ_ω be divided by dt?",
                ["z = Δθ/dt is a rate in rad/s.", "rms / r̄ is an angle error per scan in rad.",
                 "Divided by dt (0.05 s) it becomes a rate error.", "Without it: 20× too small, stuck at the 0.005 floor."],
                "The measurement is a rate, radians per second, but residual over radius is an angle error per scan. Dividing by d t, "
                "fifty milliseconds, turns it into a rate error. Without it, the error was twenty times too small and sat at its lower limit.",
                spoken="Why must the turn rate error, sigma omega, be divided by d t?")
        self.qa("What does icp_gain do, and what value would you choose?",
                ["Ez is divided by it: 2 = trust ICP twice as much.", "Simulation (fixed formula): gain 0.01 → 2.0 m end error,",
                 "2 → 4.5 m, 10 → 9.9 m. On the open field: keep it ≤ 2.", "It can't fix sliding; it only changes the trust."],
                "E z is divided by the gain, so two means trust ICP twice as much. In the simulation with a corridor, a gain of zero point "
                "zero one ended two metres off, two ended four and a half, ten almost ten. The gain can't fix sliding, it only changes how much "
                "the filter listens.",
                spoken="What does ICP gain do, and what value would you choose?")

    # ── 12 ─────────────────────────────────────────────────────────────────────────────
    def ch_pure(self):
        self.chapter(12, "Pure-ICP mode")
        c = self.show_code(457, 473)
        n = notes(c, ["zero guess (filter not running)", "delta is in the robot frame", "rotate by θ into odom, add to the pose",
                   "publish TF from here", "current scan → last scan, in every mode"])
        b = self.focus(c, 459, 459, "In pure ICP mode, ICP starts at zero, because the filter doesn't run.", m.FadeIn(n[0]))
        b = self.focus(c, 461, 468, "Its result is a motion in the robot frame, so it is rotated by the current heading into the odom frame and "
                       "added to the pose, like the odometry in exercise two. This branch publishes the transform itself.",
                       m.FadeIn(n[1]), m.FadeIn(n[2]), m.FadeIn(n[3]), box=b)
        self.focus(c, 471, 472, "In every mode, the current scan then becomes the last scan, even if ICP failed. Comparing against an ever older "
                   "scan would make the gap, and the error, grow.", m.FadeIn(n[4]), box=b)

    # ── 13 ─────────────────────────────────────────────────────────────────────────────
    def ch_launch(self):
        self.chapter(13, "Launch parameters")
        start = next(i for i, ln in enumerate(LAUNCH) if "params={" in ln) + 1
        snippet = "\n".join(LAUNCH[start:start + 12])
        c = m.Code(code_string=snippet, language="python", line_numbers_from=start + 1,
                   paragraph_config={"font_size": 18, "line_spacing": 0.45},
                   background_config={"stroke_color": "#33414d"})
        place_code(c)
        n = notes(c, ["target_frame os_lidar: heights from the lidar", "2–6 m above lidar = 2.5–6.5 m above ground",
                   "0.0087 rad = 0.5° → ~722 rays", "0.45–50 m: ignore the robot itself", "use_inf: empty rays = inf"])
        self.play(m.FadeIn(c))
        self.say("The slice is set in the launch file. Heights are measured in the os lidar frame, so two to six metres above the lidar is "
                 "two and a half to six and a half above the ground. The angle step of zero point zero zero eight seven radians is half a degree, "
                 "about seven hundred rays. The minimum range of forty five centimetres cuts off the robot itself.",
                 m.LaggedStart(*[m.FadeIn(x) for x in n], lag_ratio=0.5))
        self.qa("Why start the slice at 2 m, and what if you change it?",
                ["Below: people, vehicles, tall grass — they move.", "Above: facades, roofs, tree tops — mostly rigid.",
                 "Higher min_height: fewer moving objects, but fewer points.", "Lower max_height: cuts leafy canopies, which change every scan."],
                "Below two metres, people and vehicles move. Above, facades and roofs stay put. A higher minimum height removes more moving "
                "objects but leaves fewer points. A lower maximum height cuts off leafy tree tops, which change shape every scan. "
                "On an open field it's a trade between enough points and rigid points.")
        self.qa("Would a finer angle_increment or a smaller range_max help?",
                ["Finer angle: more points, a little better (sim: 18.1 → 14.9 m),", "costs computation.",
                 "Smaller range_max: WORSE (sim: 20.4 m). Far features are", "often the only evidence of forward motion."],
                "A finer angle step gives more points and helps a little, at the cost of computation. A smaller maximum range made it worse in "
                "the simulation, because along a corridor or an open field the far features are often the only evidence of forward motion.",
                spoken="Would a finer angle increment, or a smaller maximum range, help?")

    # ── 14 ─────────────────────────────────────────────────────────────────────────────
    def ch_results(self):
        self.chapter(14, "Results on the bag")
        s, ox, oy = 0.36, -5.6, 1.1

        def scr(x, y):
            return [ox - y * s, oy + x * s, 0]

        def poly(pts, color):
            v = m.VMobject(color=color, stroke_width=4)
            v.set_points_as_corners([scr(p[1], p[2]) for p in pts])
            return v

        wheels, icp = poly(DEMO["path_wheels"], AMBER), poly(DEMO["path_icp"], VIOLET)
        wl = T("wheels: 26.3 m", 22, AMBER)
        wl.move_to(wheels.get_end() + m.UP * 0.35).align_to(wheels.get_end(), m.RIGHT)
        il = T("ICP alone: 22.3 m", 22, VIOLET)
        il.move_to(icp.get_end() + m.DOWN * 0.75).align_to(icp.get_end(), m.RIGHT)
        self.say("On the real bag, the wheels end twenty six metres down the field. ICP alone ends at twenty two: about fifteen percent short, "
                 "and it creeps by about fifteen centimetres while the robot stands still.",
                 m.AnimationGroup(m.Create(wheels), m.FadeIn(wl), m.Create(icp), m.FadeIn(il)))
        rows = [("end of the 40 s drive", "original", "fixed"), ("no ICP", "4.88, −26.22, −78.6°", "1.74, −26.34, −90.0°"),
                ("ICP + filter", "2.16, −23.48, −85.3°", "2.25, −26.34, −88.0°"), ("bag with --loop", "frozen at 44.5 s", "resets, continues")]
        tbl = m.VGroup()
        for r, row in enumerate(rows):
            for j, txt in enumerate(row):
                cell = T(txt, 21, MUTED if r == 0 else (GREEN if j == 2 else FG))
                cell.move_to([[-3.8, 0.9, 4.6][j], -1.2 - r * 0.5, 0])
                tbl.add(cell)
        self.say("With the three fixes, the filter without ICP matches exercise four again, the fused filter keeps the wheels' distance with "
                 "the heading near the gyroscope, and the bag can loop.", m.FadeIn(tbl))
        self.clear(0.3)
        self.add(header("Results on the bag", "14"))
        why = card(["point-to-point ICP slides along walls and fences", "few rigid features 2–6 m up on an open field",
                    "far points sparse: 0.5° at 20 m ≈ 17 cm apart", "leafy canopies change shape every scan",
                    "fixed 1 m pair limit"], size=24, title="why does the ICP result look like this?").move_to([-2.6, 0.2, 0])
        alt = card(["point-to-line / point-to-plane ICP", "(PL-ICP, GICP in PCL)", "outlier rejection",
                    "degeneracy check before trusting ICP", "NDT"], size=24, title="better approaches", title_color=GREEN) \
            .move_to([4.1, 0.2, 0]).scale(0.92)
        self.say("This is the question on the sheet. Point to point ICP slides along walls, the slice has few rigid features on an open field, "
                 "far points are sparse, and leafy tree tops change shape.", m.FadeIn(why))
        self.say("Better: point to line or point to plane ICP, which measures the distance to the line through the neighbours, so sliding "
                 "along a wall costs nothing. Plus outlier rejection, and a check whether the geometry determines the motion at all.",
                 m.FadeIn(alt))

    # ── 15 ─────────────────────────────────────────────────────────────────────────────
    def ch_quiz(self):
        self.chapter(15, "Quick quiz")
        qs = [("What does getTransform return, and in which direction?", "An isometry from the sensor frame into base_footprint, or false.",
               "What does getTransform return, and in which direction?", "A rotation and shift from the sensor frame into base footprint, or false."),
              ("Why is the initial guess sideways zero?", "Wheeled robots can't slide sideways.", None, None),
              ("Why invert the guess and the result?", "ICP moves points, which move opposite to the robot.", None, None),
              ("Why R = V·Uᵀ, and what if det R < 0?", "Best least-squares rotation; flip V's last column.",
               "Why R equals V times U transposed, and what if its determinant is negative?",
               "It's the best least squares rotation. If the determinant is negative, flip the last column of V."),
              ("Why does ICP enter the filter as a speed?", "It measures change between scans, not an absolute pose.", None, None),
              ("Which three things were fixed?", "Loop reset, ICP noise (no √N, ω/dt), /imu/data_raw.",
               None, "The reset when the bag loops, the ICP noise formula, and the IMU topic.")]
        for q, a, q_sp, a_sp in qs:
            self.clear(0.3)
            qc = qa_card(q).move_to([0, 1.0, 0])
            if qc.width > 13:
                qc.scale_to_fit_width(13)
            ac = T(a, 28, GREEN).next_to(qc, m.DOWN, buff=0.8)
            self.say(q_sp or q, m.FadeIn(qc))
            self.wait(1.5)
            self.say(a_sp or a, m.FadeIn(ac))
        self.clear(0.4)
        end = lines_block(["That's the whole node.", "Short version: the overview video.",
                           "Details: the study guide and the cheatsheet."], 34, FG)
        self.say("That's the whole node. For a short version, watch the overview video, and for details, read the study guide and the cheatsheet. "
                 "Good luck in the Testat.", m.FadeIn(end))
        self.wait(1)
