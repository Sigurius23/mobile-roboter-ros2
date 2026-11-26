"""Exercise 5 code walkthrough in four parts, line by line, with the reason for every choice.

For someone new to C++ and to the task: every part starts with where the code sits in the node,
says in plain words what it is for, then reads it line by line (what, why, what goes wrong otherwise).

Render one part:  manimgx render ex5_walk.py Part1Node   (Part2Frames, Part3ICP, Part4Fusion)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import manimgx as m

from common import AMBER, DEMO, FG, GREEN, MUTED, RED, TEAL, VIOLET, T, card, dots, header, lines_block, qa_card
from fine import Walk
from icp_anim import View, counter, pair_lines, source_indices, step_to
from parts import Part

LAUNCH = (Path(__file__).parent.parent.parent / "ros2_ws" / "src" / "icp_odom" / "launch" / "icp_odom.launch.py") \
    .read_text(encoding="utf-8").splitlines()


# ════════════════════════════════════════════════════════════════════════════════════════
class Part1Node(Part):
    """How the node is built: parameters, TF, subscriptions, time, predict, wheels and gyro."""

    def construct(self) -> None:
        self.opening(1, "How the node is built",
                     ["the whole node in one picture", "parameters and subscriptions", "time, and the reset when the bag loops",
                      "predict: the motion model", "wheels and gyroscope"],
                     "[This is part one of four.] We read the ICP node of exercise five line by line, and for every line we ask why "
                     "it is written that way. Part one starts with [the whole node in one picture.] Then we look at [how the node is "
                     "set up], [how it keeps track of time], [how it predicts the motion], and [how the wheels and the gyroscope feed "
                     "the filter.] Parts two and three cover the three tasks, and part four covers how ICP enters the filter.")
        self.tour(["scan", "callback", "task1", "task2", "task3", "filter", "pose", "odomimu"], [
            "Everything starts with [a lidar scan]: a ring of distance measurements, twenty times a second. "
            "For every scan, the node runs a function called [scan callback.]",
            "[Task one] converts the scan from the sensor's frame into the robot's frame. "
            "[Task two] guesses how far the robot moved since the last scan. "
            "[Task three], the ICP loop, shifts the old scan onto the new one and measures the real motion.",
            "That motion goes into [a Kalman filter], which combines it with the other sensors. The filter's result is "
            "[the robot's pose], which the node publishes on TF.",
            "[The wheels and the gyroscope] feed the same filter through their own callbacks. That's where part one starts.",
        ])

        # ── parameters ──────────────────────────────────────────────────────────────────
        h = self.section("Parameters", "01 · the constructor")
        w = Walk(self, 57, 62)
        w.show()
        w.step([57, 58],
               "[The constructor runs once, when the node starts.] First it [declares a parameter] called ICP only, "
               "with the default [false], and then [reads its value] into a member variable. "
               "[In C plus plus,] the arrow on this means: call a function of this node object. "
               "[Why parameters?] The launch file can set them, so you switch modes without recompiling.",
               tokens=[(57, "declare_parameter", "announce a setting"), (57, "false", "its default"),
                       (58, "get_parameter", "read the value from the launch file")],
               cpp=["this->f( … ): call function f of this node      as_bool(): read the value as true or false"],
               why=["the launch file sets them: switch modes without recompiling"],
               instead=["constants in the code: change, rebuild, restart for every test"])
        w.step([57, 59],
               "[Two switches give three modes.] [ICP only] means ICP alone, without the filter. "
               "[Use ICP] means the filter with ICP added. [With both off,] the filter runs on wheels and gyroscope only, "
               "like in exercise four. That's useful to compare against.",
               tokens=[(57, '"icp_only"', "ICP alone, no filter"), (59, '"use_icp"', "filter plus ICP")],
               why=["three modes: wheels + gyro only,  filter + ICP (default),  ICP alone"])
        w.step([61, 62],
               "[The third parameter is ICP gain.] [It sets how much the filter trusts ICP.] "
               "[Part four shows] what it does and which value to choose.",
               tokens=[(61, "1.0", "default: trust ICP as measured")],
               why=["icp_gain divides the ICP uncertainty: 2 means trust ICP twice as much (part 4)"])
        w.done()

        w = Walk(self, 82, 84)
        w.show()
        w.step(82,
               "[Next come the TF objects.] [The broadcaster] publishes our result, the robot's pose. "
               "[In C plus plus,] make shared creates the object and a pointer to it that cleans up after itself. "
               "[The pose is the node's output:] other programs, like RViz, read it from TF.",
               tokens=[(82, "tf_broadcaster_", "publishes our pose on TF")],
               cpp=["std::make_shared<X>( … ): create an X and a pointer that frees it when nobody needs it any more"],
               why=["the pose odom → base_footprint is what the node produces"])
        w.step([83, 84],
               "[The buffer and the listener belong together.] [The buffer] stores every transform it has heard about. "
               "[The listener] subscribes to slash TF and slash TF static, and fills the buffer. "
               "[We need them] because task one asks this buffer how each sensor is mounted.",
               tokens=[(83, "tf_buffer_", "stores all transforms"), (84, "tf_listener_", "fills it from /tf and /tf_static")],
               why=["TASK 1 (getTransform) asks this buffer where each sensor is mounted"])
        w.done()

        w = Walk(self, 86, 96)
        w.show()
        w.step([86, 87, 88],
               "[Now the subscriptions.] [Create subscription] tells ROS: call me when a message arrives on [the odom topic.] "
               "The function it calls is [odom callback.] "
               "[In C plus plus,] std bind packs that member function together with this object, so ROS can call it. "
               "[Each sensor gets its own callback,] and each one runs at its own sensor's rate.",
               tokens=[(86, "create_subscription", "listen to a topic"), (87, '"/odom"', "wheel odometry"),
                       (88, "odomCallback", "runs for every message")],
               cpp=["std::bind(&LioNode::odomCallback, this, _1): call odomCallback of this object, with the message"],
               why=["three independent callbacks, each at its sensor's own rate"])
        w.step(87,
               "[The second argument is the quality of service.] [Keep last ten] means up to ten messages wait if the callback "
               "is busy. [Reliable] means none get lost on the way. [If the node is briefly slow,] it catches up instead of "
               "losing measurements.",
               tokens=[(87, "KeepLast(10)", "queue of 10 messages"), (87, "reliable()", "no lost messages")],
               why=["a short delay in the node doesn't cost measurements"])
        w.step([90, 91, 92, 94, 95, 96],
               "[The same again] for [the IMU], and for [the lidar scan.] "
               "[Why not wait for all three together,] like the synchronizer in exercise four? "
               "Because then a slow sensor holds back a fast one, and messages that don't fit get thrown away.",
               tokens=[(91, "imu_topic", "the gyroscope"), (95, '"/ouster/scan"', "the lidar scan")],
               why=["no synchronizer: a slow sensor never holds back a fast one, and no message is thrown away"],
               instead=["a message_filters synchronizer (Exercise 4): waits for all three, drops what doesn't match"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.qa("Why a class with members instead of global variables?",
                ["The node's memory stays together and private.", "Callbacks are member functions that can reach state_.",
                 "It's the usual ROS 2 style."],
                "[The node's memory,] the state, the last scan and the TF objects, stays together and private. "
                "[The callbacks become member functions,] so they can reach that memory directly. "
                "[It's the usual ROS two style.] Exercise four's skeleton used globals instead.")

        # ── time ────────────────────────────────────────────────────────────────────────
        self.plain("Time, and the reset when the bag loops", "02 · FIX 1",
                   ["Every message carries the time it was measured.", "The filter remembers the time of its last update.",
                    "When the bag starts over, time jumps back by 40 s."],
                   "[Every message carries a time stamp,] the moment it was measured. "
                   "[The filter remembers the time of its last update], and moves its state forward by the difference. "
                   "[When you play the bag in a loop,] time suddenly jumps back by forty seconds. "
                   "The original code didn't handle that, so this part was fixed.")
        h = self.section("Time, and the reset when the bag loops", "02 · FIX 1")
        w = Walk(self, 195, 202)
        w.show()
        w.step(195,
               "[This is in the odometry callback.] [dt] is [the time since the last update,] by any sensor. "
               "[Predict needs it] to move the state forward to the moment of this message.",
               tokens=[(195, "dt", "seconds since the last update"), (195, "filter_time_", "time of the last update")],
               why=["predict moves the state forward by dt before every correction"])
        w.step([201, 202],
               "[Look at the last check first.] [If dt is zero or negative,] the message is a little older than the last one. "
               "[That's normal:] odometry at fifty hertz and IMU at a hundred and twenty five hertz arrive interleaved. "
               "The message is simply skipped.",
               tokens=[(201, "dt <= 0", "a slightly older message")],
               why=["odom (50 Hz) and IMU (125 Hz) interleave: a few ms backwards happens all the time"])
        w.step([196, 197, 198, 199, 200],
               "[A big jump back is different.] If time goes back by more than [K time jump back,] which is one second, "
               "the bag has started over. Then [reset filter] starts again at the origin. "
               "[The original code returned here] without updating its stored time. After the loop, every message looked too "
               "old, and the robot froze.",
               tokens=[(196, "kTimeJumpBack", "1 second"), (198, "resetFilter", "start over at the origin")],
               why=["a jump back of more than 1 s: the bag looped, or Minot ran init again"],
               instead=["original: return on dt ≤ 0 forever; after the loop the robot froze at 44.5 s"])
        w.done()

        w = Walk(self, 123, 129)
        w.show()
        w.step([126, 127, 128],
               "[Reset filter does three things.] [It puts the state back] to the origin, with the starting uncertainty. "
               "[It takes the new time] as its reference. [And it forgets the last scan.] "
               "[That last one matters:] comparing a scan from the end of the bag with one from its start would look like a "
               "huge jump, and the filter would believe it.",
               tokens=[(126, "initialState()", "origin, starting uncertainty"), (127, "filter_time_", "the new time"),
                       (128, "have_last_scan_", "forget the old scan")],
               why=["a scan from the bag's end against one from its start would look like a huge jump"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.qa("Why a threshold of one second, and not zero?",
                ["Odom and IMU interleave: a few ms backwards is normal.", "A threshold of 0 would reset all the time.",
                 "1 s is far above that jitter and far below the 40 s bag."],
                "[Odometry and IMU arrive interleaved,] so a few milliseconds backwards happens all the time. "
                "[With a threshold of zero,] the filter would reset constantly. "
                "[One second is far above that jitter,] and far below the length of the bag.")

        # ── predict ─────────────────────────────────────────────────────────────────────
        self.where("filter", [
            "[Predict belongs to the Kalman filter.] Every callback calls it first, to move the state forward to the time of "
            "its message. Only then does it correct the state with the measurement.",
        ])
        self.plain("predict: the motion model", "03",
                   ["state: x, y, heading θ, speed v, turn rate ω, acceleration a",
                    "predict: where would the robot be after dt, if nothing changes?",
                    "and how much less sure are we?"],
                   "[The filter's state is six numbers]: position x and y, the heading, the speed, the turn rate and the "
                   "acceleration. [Predict asks]: if the robot keeps moving like this, where is it after d t seconds? "
                   "[And it also asks how much less sure] we are after that time.")
        h = self.section("predict: the motion model", "03")
        w = Walk(self, 157, 166)
        w.show()
        w.step([157, 159],
               "[These lines fill the matrix A.] [Position x grows] by the cosine of the heading, times speed, times d t. "
               "[Position y grows] by the sine. [The speed points along the heading,] and cosine and sine split it into an "
               "x part and a y part.",
               tokens=[(157, "c * dt", "x += cos θ · v · dt"), (159, "ss * dt", "y += sin θ · v · dt")],
               why=["the speed points along the heading θ; cos θ and sin θ split it into x and y"])
        w.step([158, 160],
               "[The acceleration adds] [one half a d t squared], the formula from school physics.",
               tokens=[(158, "0.5 * dt * dt", "½ · a · dt²")])
        w.step([161, 162],
               "[The last two lines] say: [the heading grows] by turn rate times d t, and [the speed grows] by acceleration "
               "times d t. [Everything else stays the same,] because A starts as the identity matrix.",
               tokens=[(161, "A(2, 4)", "θ += ω · dt"), (162, "A(3, 5)", "v += a · dt")],
               why=["A starts as the identity: what isn't changed here stays the same"])
        w.step([164, 165, 166],
               "[E u is the process noise.] It says how wrong the model may be, [per second.] "
               "[It's multiplied by d t,] [because the longer we predict without a measurement,] the less sure we can be.",
               tokens=[(165, "0.05", "per second (here for v and ω)"), (166, "Eu *= dt", "scaled by the time")],
               why=["the longer without a measurement, the larger the uncertainty"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.qa("Why is predict called in every callback?",
                ["Every measurement belongs to its own moment.", "The state is moved to that moment first,",
                 "so all sensors share one filter_time_."],
                "[Every measurement belongs to its own moment.] [Before the filter corrects with it,] it moves the state "
                "forward to exactly that moment. [That's why all callbacks share one filter time.]")

        # ── odometry ────────────────────────────────────────────────────────────────────
        self.where("odomimu", [
            "[Now the wheels and the gyroscope.] Their callbacks run between the scans, many times a second, and keep the "
            "filter up to date.",
        ])
        h = self.section("Wheel odometry", "04")
        w = Walk(self, 204, 206)
        w.show()
        w.step(204,
               "[The odometry message] says in which frame its speeds are given: [the child frame I D.]",
               tokens=[(204, "child_frame_id", "the frame of the speeds")])
        w.step([205, 206],
               "[Then the two speeds.] [Linear x is the forward speed], and [angular z is the turn rate.] "
               "[A robot with two driven wheels] measures exactly these two.",
               tokens=[(205, "linear.x", "forward speed"), (206, "angular.z", "turn rate")],
               why=["a differential drive measures forward speed and turn rate"])
        w.done()

        w = Walk(self, 208, 215)
        w.show()
        w.step(208,
               "[If that frame isn't] [the robot's base frame,] the speeds must be rotated. "
               "[In our bag] the child frame already is base footprint, so this block is skipped. It's there for other robots.",
               tokens=[(208, "!=", "only for another frame")],
               why=["in this bag child_frame_id is base_footprint, so nothing changes"])
        w.step(211, "[Here, task one] [looks up how that frame is turned.]",
               tokens=[(211, "getTransform", "TASK 1, part 2")])
        w.step([213, 214],
               "[Then both vectors are rotated] with [dot linear], [the rotation part] of the transform. "
               "[Only the rotation,] because speeds are directions. Where a sensor is mounted doesn't change a direction, "
               "only how it's turned.",
               tokens=[(213, ".linear()", "rotation only")],
               cpp=[".linear(): the 3×3 rotation of an Eigen::Isometry3d      .matrix(): rotation and shift"],
               why=["speeds are directions: a mounting offset doesn't change them, only a rotation does"])
        w.done()

        w = Walk(self, 224, 234)
        w.show()
        w.step([224, 225, 226],
               "[Now the correction.] [The measurement z] holds [the forward speed] and [the turn rate.]",
               tokens=[(224, "z(2)", "two measured numbers"), (225, "v_lin.x()", "forward speed"),
                       (226, "v_ang.z()", "turn rate")])
        w.step([228, 229, 230],
               "[E z is the measurement noise,] [a variance of zero point one] for both. "
               "[Smaller means more trusted.] The gyroscope gets zero point zero five, so for turning it counts more than the wheels.",
               tokens=[(229, "0.1", "variance of the wheel speeds")],
               why=["smaller variance = more trust; the gyro gets 0.05, so it counts more for turning"])
        w.step([232, 233, 234],
               "[H says which state entries the sensor measures.] [Row zero] measures entry three, the speed. "
               "[Row one] measures entry four, the turn rate. "
               "[The rest of the correction] is the Kalman update from exercise four.",
               tokens=[(233, "H(0, 3)", "z(0) is entry 3: v"), (234, "H(1, 4)", "z(1) is entry 4: ω")],
               why=["the rest is the usual Kalman update from Exercise 4"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        # ── IMU ─────────────────────────────────────────────────────────────────────────
        h = self.section("The IMU", "04 · FIX 3")
        w = Walk(self, 71, 72)
        w.show()
        w.step(71,
               "[Which IMU do we use?] The robot has two. The default is now [the Phidget IMU on I M U data raw.] "
               "[The original used the Ouster's IMU.] Its gyroscope reads zero point zero zero six radians per second while "
               "standing still. Over the forty second bag, that adds up to fourteen degrees.",
               tokens=[(71, '"/imu/data_raw"', "the Phidget IMU")],
               why=["/ouster/imu: offset of 0.006 rad/s at standstill = 14° over the 40 s bag"])
        w.done()

        w = Walk(self, 283, 290)
        w.show()
        w.step(283,
               "[In the IMU callback,] the number of measured values depends on [I M U use acc.] "
               "[In C plus plus,] question mark and colon mean: two if it's true, otherwise one. "
               "[The acceleration is off by default.] The Phidget's TF gives it the wrong sign, and the Ouster reads half a metre "
               "per second squared while standing.",
               tokens=[(283, "imu_use_acc_ ? 2 : 1", "2 values, or only 1")],
               cpp=["a ? b : c  means: b if a is true, otherwise c"],
               why=["acceleration is off: Phidget's TF has the wrong sign, the Ouster reads 0.5 m/s² standing"],
               instead=["using it: a wrong acceleration only spoils the speed, which the wheels measure well"])
        w.step([288, 289, 290],
               "[So normally, z holds just one number:] [the turn rate from the gyroscope], with [a variance of zero point "
               "zero five.] [A gyroscope measures turning directly,] while wheels slip in turns.",
               tokens=[(288, "ang_imu.z()", "gyro turn rate"), (289, "0.05", "trusted more than the wheels")],
               why=["a gyroscope measures turning directly; wheels slip in turns"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.closing(["Part 1: the node, time, predict, wheels and gyro.", "Next, part 2: TASK 1, scan to points, TASK 2."],
                     "[That's how the node is built.] [In part two,] we follow a lidar scan into the node: task one, "
                     "turning the scan into points, and task two.")


# ════════════════════════════════════════════════════════════════════════════════════════
class Part2Frames(Part):
    """TASK 1 (getTransform), scan to points, TASK 2 (the initial guess)."""

    def construct(self) -> None:
        self.opening(2, "Frames, points and the guess",
                     ["TASK 1: getTransform", "scan to points", "TASK 2: the initial guess"],
                     "[This is part two of four.] We follow one lidar scan into the node. First [task one], which converts "
                     "between frames. Then [how a scan becomes points], and finally [task two], the guess where ICP starts.")
        self.task1()
        self.scan_to_points()
        self.task2()
        self.closing(["Part 2: TASK 1, scan to points, TASK 2.", "Next, part 3: TASK 3, the ICP loop."],
                     "[That was task one and task two.] [In part three,] ICP takes the guess and finds the real motion.")

    # ── TASK 1 ──────────────────────────────────────────────────────────────────────────
    def task1(self):
        self.where("task1", [
            "Here is where [task one] sits. Every time the lidar sends a scan, the node runs scan callback. "
            "The first thing it does is call task one, getTransform.",
            "The odometry and [IMU callbacks] call it too, for their own messages.",
        ], callers=["called in:", "scanCallback, line 352", "odomCallback, line 211", "imuCallback, line 276"])

        h = header("What getTransform is for", "TASK 1")
        self.play(m.FadeIn(h), run_time=0.5)
        src = DEMO["pairs"]["straight"]["src"]
        off = DEMO["lidar_offset"]
        in_lidar = [[-(x - off[0]), -(y - off[1])] for x, y in src]
        left = dots(in_lidar, VIOLET, 0.065, origin=(-3.4, -0.3), radius=0.028, limit=(2.9, 2.5))
        right = dots(src, TEAL, 0.065, origin=(3.4, -0.3), radius=0.028, limit=(2.9, 2.5))
        lt = T("the lidar's own numbers (os_lidar)", 22, VIOLET).move_to([-3.4, 2.4, 0])
        rt = T("the robot's frame (base_footprint)", 22, TEAL).move_to([3.4, 2.4, 0])
        tri = lambda x: m.Triangle(color=AMBER, fill_color=AMBER, fill_opacity=1).scale(0.12).move_to([x, -0.3, 0])
        rl, rr = tri(-3.4), tri(3.4)
        self.say("Every sensor measures relative to itself. On this robot, [the lidar] sits fifty four centimetres up "
                 "and is turned around by a hundred and eighty degrees. So in the lidar's own numbers, "
                 "[a wall in front of the robot shows up behind it.]",
                 m.AnimationGroup(m.FadeIn(lt), m.FadeIn(rl), m.FadeIn(left), run_time=0.8),
                 m.Indicate(left, scale_factor=1.04))
        self.say("getTransform tells us how to [convert the lidar's points into the robot's own frame], called base footprint. "
                 "Then everything the node compares is measured from the same place.",
                 m.AnimationGroup(m.FadeIn(rt), m.FadeIn(rr), m.TransformFromCopy(left, right)))
        bad = T("without it, our real scan pair gives −0.68 m: the robot seems to drive backwards", 24, RED).move_to([0, -3.4, 0])
        self.say("Without it, ICP would compare points in the wrong frame. On our real scan pair, "
                 "[the robot would seem to drive backwards.] Now let's read the function.", m.FadeIn(bad, run_time=0.6))
        self.play(*[m.FadeOut(x) for x in (h, left, right, lt, rt, rl, rr, bad)], run_time=0.5)

        h = header("TASK 1: getTransform", "TASK 1")
        self.play(m.FadeIn(h), run_time=0.5)
        w = Walk(self, 308, 313)
        w.show()
        w.step([308, 309, 310],
               "[First, the signature.] The function gets three things: [the name of the sensor's frame], "
               "[the time the data was measured], and [an empty transform called T out], which it fills in. "
               "It returns [true if that worked], and false if not. "
               "[In C plus plus,] bool means true or false, and the ampersand means the function works on the caller's own "
               "variable instead of a copy. That's how it can fill in T out. "
               "[Why not throw an error instead?] Because the callers react differently to a failure. The scan callback drops "
               "the scan. Odometry and IMU keep their raw values. With true or false, each caller handles that with a simple if.",
               tokens=[(308, "source_frame", "sensor frame, e.g. os_lidar"), (309, "time", "when it was measured"),
                       (310, "T_out", "filled in by the function"), (308, "bool", "true if it worked")],
               cpp=["bool: true or false      &: works on the caller's own variable, no copy (so T_out can be filled in)"],
               why=["each caller reacts differently to a failure: the scan is skipped, odom and IMU keep raw values"],
               instead=["throwing an error: every callback would need its own try/catch (std::optional would also work)"])
        w.step(312,
               "[Next comes the try block.] In C plus plus, [if something inside try fails], "
               "[the program jumps to the matching catch block instead of crashing.] "
               "[Here, the lookup fails] when TF has no answer yet. That happens right after startup, "
               "before the static transforms have arrived.",
               tokens=[(312, "try", "the lookup may fail")],
               cpp=["try { … } catch { … }: if something inside try fails, the program jumps to catch instead of crashing"],
               why=["right after startup, /tf_static may not be in the buffer yet, so the lookup fails"],
               instead=["no try/catch: the node crashes on its first message"])
        w.done()

        w = Walk(self, 314, 320)
        w.show()
        w.step(315,
               "[Inside the try block,] we ask [the TF buffer] for the transform. [Look up transform] does the asking, "
               "and [the answer is stored in tf msg.] [The arrow] calls a function on the buffer object. "
               "A TF listener keeps that buffer filled. [We use the buffer] because the lidar is not mounted directly on the "
               "robot's base frame. It sits six frames away, and the buffer multiplies that whole chain for us.",
               tokens=[(315, "tf_buffer_", "kept filled by the TF listener"), (315, "lookupTransform", "ask for one transform"),
                       (315, "tf_msg", "the answer")],
               cpp=["a->f( … ): call function f on the object that a points to"],
               why=["os_lidar → base_footprint is a chain of 6 static transforms; the buffer combines them"],
               instead=["subscribing to /tf_static ourselves: we would have to chain the transforms by hand"])
        w.step(316,
               "[The first two arguments are the frames.] [The target comes first]: that's base footprint, the robot. "
               "[The source comes second]: that's the sensor. [In this order,] the result moves sensor points into the "
               "robot frame. If you swap them, you get the opposite transform, and the scan points end up in the wrong place.",
               tokens=[(316, "base_frame_", "1  target: the robot"), (316, "source_frame", "2  source: the sensor")],
               why=["lookupTransform(target, source) gives T with  p_robot = T · p_sensor"],
               instead=["swapped: you get the inverse transform, and the points end up in the wrong place"])
        w.step(316,
               "[The third argument is the time.] We pass [the message's own time stamp], not the current time. "
               "For a sensor bolted to the robot, that makes no difference. For moving joints, or a bag played back later, "
               "only the stamp gives the right answer. The last argument says [how long to wait: at most a tenth of a second.] "
               "[We keep it short] because the node runs in a single thread. While it waits, no other callback can run, "
               "and a new scan arrives every fifty milliseconds.",
               tokens=[(316, "time", "3  the message's time stamp"), (316, "rclcpp::Duration::from_seconds(0.1)", "4  wait at most 0.1 s")],
               why=["the message's stamp is right for moving joints and for bag playback",
                    "one thread: while the lookup waits, no other callback runs (it only waits right after startup)"],
               instead=["no timeout: the first lookups fail at startup;  a long timeout: odom and IMU messages pile up"])
        w.step(319,
               "[Now the conversion.] [transformToEigen] turns the ROS message into an Eigen isometry, a rotation plus a shift, "
               "and [stores it in T out.] [We convert] because the callers work with Eigen. For speeds and gyro readings they use "
               "only the rotation part, called linear. For point clouds they use the full matrix.",
               tokens=[(319, "tf2::transformToEigen", "ROS message → Eigen isometry"), (319, "T_out", "the caller's transform")],
               why=[".linear() is the rotation only, for directions (speeds, gyro);  .matrix() adds the shift, for points"],
               instead=["converting the quaternion by hand: easy to get the order or a sign wrong"])
        w.step(320, "[Then the function returns] [true], so the caller knows T out is valid.",
               tokens=[(320, "true", "T_out is valid")])
        w.done()

        w = Walk(self, 322, 329)
        w.show()
        w.step(322,
               "[If anything went wrong, we land in the catch block.] It catches [TransformException], the base class of all "
               "lookup errors. [So one block covers] an unknown frame, a time outside the buffer, and frames that aren't connected.",
               tokens=[(322, "tf2::TransformException", "base class of all lookup errors")],
               why=["one catch covers LookupException, ExtrapolationException, ConnectivityException and the others"])
        w.step([325, 326, 327],
               "[Here we print a warning.] [It's throttled] to [once every two seconds], "
               "[because this function runs for every message], up to a hundred and twenty five times a second. "
               "A normal warning would flood the terminal.",
               tokens=[(325, "RCLCPP_WARN_THROTTLE", "a warning, but rarely"), (325, "2000", "at most every 2000 ms")],
               why=["getTransform runs for every odom, IMU and scan message: up to 125 times a second"],
               instead=["RCLCPP_WARN: hundreds of identical lines per second"])
        w.step(328,
               "[Finally, the function returns false,] and [the caller decides] what to do. [The scan callback skips the scan]: "
               "a scan in the wrong frame would give ICP a wrong motion, and the filter would trust it. "
               "Odometry and IMU continue with their raw values.",
               tokens=[(328, "false", "the caller decides")],
               why=["a scan in the wrong frame gives a wrong ICP motion that the filter trusts, so it's better to skip it"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.qa("lookupTransform(base_frame_, source_frame, …): which way round?",
                ["Target first, then source.", "The result converts FROM the sensor frame INTO the base frame:",
                 "p_base = T · p_sensor"],
                "[Target first, then source.] [The result converts data from the sensor frame into the base frame.] "
                "[In a formula: p base equals T times p sensor.]",
                spoken="lookupTransform with the base frame first and the source frame second: which way does it convert?")

    # ── scan to points ──────────────────────────────────────────────────────────────────
    def scan_to_points(self):
        h = header("What a scan is", "scan to points")
        self.play(m.FadeIn(h), run_time=0.4)
        src = DEMO["pairs"]["straight"]["src"]
        o = (0.0, -0.4)
        pts = dots(src, TEAL, 0.065, origin=o, radius=0.028, limit=(6.2, 2.9))
        bot = m.Triangle(color=AMBER, fill_color=AMBER, fill_opacity=1).scale(0.14).move_to([o[0], o[1], 0])
        step = max(1, len(pts) // 18)
        rays = m.VGroup(*[m.Line([o[0], o[1], 0], d.get_center(), color=AMBER, stroke_width=1.2, stroke_opacity=0.6)
                          for d in list(pts)[::step]])
        note = T("each ray: an angle and a distance r   →   x = r·cos(angle),  y = r·sin(angle)", 24, FG).move_to([0, -3.5, 0])
        self.say("[A lidar scan is a list of distances.] The lidar turns around and [measures one distance every half degree], "
                 "about seven hundred and twenty per turn. [Where a ray hits something, we get a point.] "
                 "The code turns [each angle and distance into x and y.]",
                 m.FadeIn(bot, run_time=0.5), m.Create(rays, run_time=0.8), m.FadeIn(pts, run_time=0.6), m.FadeIn(note, run_time=0.6))
        self.play(*[m.FadeOut(x) for x in (h, pts, bot, rays, note)], run_time=0.4)

        h = self.section("Scan to points", "scan callback, lines 337–346")
        w = Walk(self, 337, 346)
        w.show()
        w.step(337, "[The first ray] points in the direction [angle min.]",
               tokens=[(337, "angle_min", "angle of the first ray")])
        w.step(338,
               "[The loop goes over every ray.] [Ranges] holds one distance per ray, about seven hundred and twenty of them. "
               "[In C plus plus,] a for loop has three parts: start at zero, continue while i is smaller than the size, "
               "and add one after each round.",
               tokens=[(338, "scan->ranges.size()", "about 722 rays")],
               cpp=["for (i = 0; i < n; ++i): repeat for i = 0, 1, 2, … n−1"])
        w.step([340, 341],
               "[A ray only becomes a point if it's usable.] [It must be finite]: rays that hit nothing come as infinity, "
               "because the launch file sets use inf. [And it must be inside the range limits.] [The minimum, forty five "
               "centimetres, cuts off the robot itself.]",
               tokens=[(341, "std::isfinite(r)", "hit something"), (341, "r >= scan->range_min", "not the robot itself")],
               why=["inf: the ray hit nothing (use_inf in the launch file);  range_min 0.45 m: the robot's own body"])
        w.step(343,
               "[Then the polar to x y conversion.] [x is r times the cosine of the angle], and [y is r times the sine.] "
               "[Of about seven hundred and twenty rays,] roughly two hundred and fifty points remain on the bag.",
               tokens=[(343, "r * std::cos(angle)", "x"), (343, "r * std::sin(angle)", "y")],
               why=["from about 722 rays, roughly 250 points remain on the bag"])
        w.step(345, "[After each ray,] the angle grows by [the angle increment], half a degree.",
               tokens=[(345, "angle_increment", "0.5° per ray")])
        w.done()

        w = Walk(self, 352, 360)
        w.show()
        w.step(352, "[Next, task one] [gives the lidar's mounting], as we saw earlier.",
               tokens=[(352, "getTransform", "TASK 1")])
        w.step(354,
               "[If it worked,] [transform point cloud] moves every point into the robot frame. It uses [the full matrix], "
               "[because points are positions:] they need both the rotation and the shift.",
               tokens=[(354, "transformPointCloud", "every point into base_footprint"), (354, "T_base_lidar", "rotation and shift")],
               why=["points are positions: they need rotation and shift (.matrix()), unlike speeds"])
        w.step([356, 357, 358, 359],
               "[If it failed,] the function [returns], and this scan is skipped. "
               "[Points in the wrong frame are worse than none,] because ICP would measure a wrong motion and the filter would trust it.",
               tokens=[(359, "return", "skip this scan")],
               why=["ICP on a scan in the wrong frame gives a wrong motion that the filter would trust"])
        w.done()

        w = Walk(self, 362, 368)
        w.show()
        w.step([362, 363, 364, 365, 366, 367, 368],
               "[The very first scan] [has nothing to compare with.] [It's only stored] as the last scan, together with its time. "
               "[ICP always needs two scans,] an old one and a new one.",
               tokens=[(362, "!have_last_scan_", "the first scan"), (364, "*last_cloud_base_", "kept for next time")],
               why=["ICP compares two scans: the first one is only stored"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

    # ── TASK 2 ──────────────────────────────────────────────────────────────────────────
    def task2(self):
        self.where("task2", [
            "[Task two] comes right after that, [still inside scan callback.] The new scan is now in the robot frame. "
            "Next, ICP in task three will shift the old scan until it lines up with the new one. "
            "Task two decides where that search starts.",
            "Why does the start matter? ICP only finds the nearest fit. If it starts far away from the real motion, "
            "it can lock onto the wrong fit.",
        ], callers=["computed in:", "scanCallback, lines 384–392", "used by: icp( … ), line 396"])
        h = header("TASK 2: the initial guess", "TASK 2")
        self.play(m.FadeIn(h), run_time=0.5)
        g = DEMO["pairs"]["straight"]["guess"]
        v, om = g[0] / 0.5, g[2] / 0.5
        tp = DEMO["pairs"]["turn"]
        w = Walk(self, 384, 392)
        w.show()
        w.step([384, 385],
               "[The guess starts from the Kalman filter's state], the six numbers from part one: position x and y, heading, "
               "speed, turn rate and acceleration. [Entry three is the speed] and [entry four is the turn rate.] "
               "[C plus plus counts from zero,] so entry zero is x. "
               "[We take them from the filter] rather than from the raw wheels, because the filter combines wheels, "
               "gyroscope and ICP. So it's our best estimate of how the robot is moving.",
               tokens=[(384, "state_.x(3)", f"speed v, e.g. {v:.2f} m/s"), (385, "state_.x(4)", f"turn rate ω, e.g. {om:+.3f} rad/s")],
               cpp=["state_.x(3): entry 3 of the state, counting from 0:  x, y, θ, v, ω, a"],
               why=["the filter combines wheels, gyro and ICP: the best estimate of the current motion"],
               instead=["raw wheel speed: slip in turns goes straight into the guess;  the last ICP result is also common"])
        w.step(390,
               "[The forward part] is [speed times the time since the last scan.] With a scan every fifty milliseconds, "
               "that's about six and a half centimetres here. [We ignore the acceleration], because over fifty milliseconds "
               "it adds less than a millimetre.",
               tokens=[(390, "vx * dt_scan", f"{v:.2f} m/s × 0.05 s = {v * 0.05:.3f} m")],
               why=["over 50 ms, constant speed is accurate enough:  ½·a·dt² < 1 mm"])
        w.step(391, "[The sideways part] is [zero.] [Wheels can't slide sideways], and on a curve the real value is under a millimetre.",
               tokens=[(391, "0.0", "no sideways slide")],
               why=["a wheeled robot can't move sideways; on a curve the offset is ≈ v·dt·sin(ω·dt/2) < 1 mm"])
        w.step(392,
               "[The turn] is [the turn rate times the time since the last scan.] [Next, let's see why the guess matters,] "
               "on real scans.",
               tokens=[(392, "w * dt_scan", f"{om:+.3f} rad/s × 0.05 s")],
               why=[f"real turning pair: from zero, 9 pairs and a nonsense result;  from the guess, {tp['guess_inliers']} pairs"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        # real scans while turning
        h = header("Why the guess matters", "TASK 2 · real scans while turning")
        self.play(m.FadeIn(h), run_time=0.4)
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
        legend = lines_block(["violet: the old scan", "teal: the new scan"], 22, MUTED, buff=0.12).move_to([4.3, 2.4, 0])
        self.say("[Here the robot turns on the spot,] almost fifty degrees within half a second. [Violet is the old scan, teal the new one.]",
                 m.AnimationGroup(m.FadeIn(tgt), m.FadeIn(src), run_time=0.6), m.FadeIn(legend, run_time=0.5))
        fail = card(["start at zero", "9 point pairs left", "result: nonsense"], size=22, title="from zero", title_color=RED) \
            .move_to([4.3, 0.6, 0])
        self.say("[Started at zero,] most points are more than a metre from any partner. [Only nine pairs survive,] and ICP returns nonsense.",
                 m.Succession(*[m.Transform(src, at(z_steps, i, RED), run_time=0.4) for i in range(1, len(z_steps))]),
                 m.FadeIn(fail, run_time=0.5))
        ok = card([f"{pair['guess_inliers']} point pairs", f"turn {pair['guess_delta'][2]:+.3f} rad",
                   f"wheels {pair['truth'][2]:+.3f} rad"], size=22, title="from the guess", title_color=GREEN).move_to([4.3, -1.7, 0])
        self.say("[Started at the filter's guess,] it lines up within a few iterations, and [the turn matches the wheels.]",
                 m.Succession(m.Transform(src, at(g_steps, 0, VIOLET), run_time=0.5),
                              *[m.Transform(src, at(g_steps, i, GREEN), run_time=0.25) for i in range(1, len(g_steps))]),
                 m.FadeIn(ok, run_time=0.5))
        self.wait(0.5)
        self.clear()

        self.qa("Why is the sideways guess exactly zero?",
                ["A wheeled robot can't slide sideways.", "On an arc it ends up about v·dt·sin(ω·dt/2) to the side:",
                 "under a millimetre at 20 scans per second."],
                "[A wheeled robot can't slide sideways.] [Strictly, on a curve it ends up slightly to the side,] "
                "[but at twenty scans per second that's under a millimetre,] and ICP corrects it easily.")
        self.qa("Why does pure-ICP mode start at zero?",
                ["With icp_only the filter doesn't run, so x(3) and x(4) stay 0.", "ICP has to find the whole motion itself.",
                 "Fine for 0.05 s steps; it fails on big gaps or fast turns."],
                "[With ICP only, the filter doesn't run,] so the speed and the turn rate in the state stay zero. "
                "[ICP has to find the whole motion itself.] "
                "[Between two scans fifty milliseconds apart that works,] but on big gaps or fast turns it fails, as we just saw.")


# ════════════════════════════════════════════════════════════════════════════════════════
class Part3ICP(Part):
    """TASK 3: the ICP function, set-up, the loop, statistics and result."""

    def construct(self) -> None:
        self.opening(3, "TASK 3: the ICP loop",
                     ["the idea, on real scans", "set-up: the inverse trick", "one round, step by step", "statistics and result"],
                     "[This is part three of four,] the heart of the exercise. First [the idea, on real scans.] Then [the set-up], "
                     "[one round of the loop, step by step], and [what the function returns.]")
        self.where("task3", [
            "[Task three] is the function I C P. Scan callback calls it with the old scan, the new scan, and the guess from task two.",
            "[The function itself] starts at line four hundred and seventy five.",
        ], callers=["called in:", "scanCallback, line 396", "the function:", "icp( … ), lines 475–629"])
        self.idea()
        self.setup()
        self.loop()
        self.stats()
        self.questions()
        self.closing(["Part 3: TASK 3, the ICP loop.", "Next, part 4: ICP in the filter, results, quiz."],
                     "[That's the whole ICP function.] [In part four,] we see how its result enters the Kalman filter, "
                     "and how well it works on the real bag.")

    def idea(self):
        h = header("The idea, on real scans", "TASK 3 · two scans 0.5 s apart, one corner zoomed")
        self.play(m.FadeIn(h), run_time=0.4)
        pair = DEMO["pairs"]["straight"]
        steps = pair["steps"]
        view = View(scale=0.95, center=(-14.0, 12.0), origin=(-1.6, -0.1), half_w=4.8, half_h=2.6, radius=0.05)
        keep = source_indices(view, steps)
        tgt = view.dots(pair["tgt"], TEAL)
        src = step_to(view, steps, 0, keep)
        it = counter("round 0", [5.0, 1.8, 0])
        legend = lines_block(["violet: old scan", "teal: new scan", "orange: pairs"], 22, MUTED, buff=0.12).move_to([5.0, 0.6, 0])
        self.say("[Here are two real scans,] half a second apart, zoomed into one corner. [Violet is the old scan, teal the new one.] "
                 "The robot drove forward, so the same walls show up in slightly different places.",
                 m.AnimationGroup(m.FadeIn(view.frame()), m.FadeIn(src), m.FadeIn(tgt), m.FadeIn(it), run_time=0.6),
                 m.FadeIn(legend, run_time=0.5))
        ln = pair_lines(view, steps[0])
        self.say("[ICP pairs every old point with the nearest new point.] [Then it computes the one rotation and shift] that brings "
                 "the pairs closest together, and moves the old scan by it.",
                 m.Create(ln, run_time=0.6),
                 m.AnimationGroup(m.Transform(src, step_to(view, steps, 1, keep)), m.FadeOut(ln),
                                  m.Transform(it, counter("round 1", [5.0, 1.8, 0])), run_time=0.8))
        anims = []
        for i in range(1, len(steps) - 1):
            ln = pair_lines(view, steps[i])
            anims.append(m.Succession(m.Create(ln, run_time=0.35),
                                      m.AnimationGroup(m.Transform(src, step_to(view, steps, i + 1, keep)), m.FadeOut(ln),
                                                       m.Transform(it, counter(f"round {i + 1}", [5.0, 1.8, 0])), run_time=0.5)))
        res = card([f"forward {pair['delta'][0]:+.3f} m", f"wheels {pair['truth'][0]:+.3f} m", f"{pair['inliers']} pairs"],
                   size=21, title="result").move_to([5.0, -1.4, 0])
        self.say("[Then it pairs again, and moves again,] until the step is tiny. "
                 "[The total of all steps is the robot's motion:] zero point six eight metres forward. The wheels say zero point six three.",
                 m.Succession(*anims), m.FadeIn(res, run_time=0.5))
        self.wait(0.4)
        self.clear()

    def setup(self):
        self.plain("ICP moves points, not the robot", "set-up: the inverse trick",
                   ["The guess says how the ROBOT moved.", "ICP moves the old POINTS onto the new ones.",
                    "Points move the opposite way: drive 1 m towards a wall,", "and the wall comes 1 m closer."],
                   "[The guess from task two says how the robot moved.] [But ICP moves the old points] onto the new ones. "
                   "[Points move the opposite way.] If you drive one metre towards a wall, [the wall comes one metre closer.] "
                   "So the code turns the guess around before it starts.")
        h = self.section("Set-up: the inverse trick", "TASK 3 · lines 486–517")
        w = Walk(self, 486, 495)
        w.show()
        w.step(488,
               "[The guess becomes a three by three matrix,] [starting as the identity], which means no motion. "
               "[In C plus plus,] Matrix three d is a three by three matrix of decimal numbers. "
               "[Why three by three for a flat motion?] It holds the rotation and the shift in one matrix, "
               "and two motions combine by simply multiplying their matrices.",
               tokens=[(488, "Eigen::Matrix3d::Identity()", "3×3, starts as 'no motion'")],
               cpp=["Eigen::Matrix3d: a 3×3 matrix of doubles"],
               why=["rotation and shift in one matrix; two motions combine by multiplying"])
        w.step([489, 490, 491, 492, 493, 494],
               "[These six lines fill it in.] [The cosine] and [the minus sine] of the guessed turn go top left, "
               "and [the forward shift] goes into the last column. [The pattern] is the standard two D motion matrix.",
               tokens=[(489, "c_g", "cos of the turn"), (490, "-s_g", "−sin"), (491, "initial_guess_robot(0)", "forward shift")],
               why=["( cos −sin x | sin cos y | 0 0 1 ): rotate by the turn, then shift by x, y"])
        w.step(495,
               "[Then it's inverted.] [Dot inverse] turns the robot's motion into the points' motion, "
               "[because points move the opposite way.]",
               tokens=[(495, ".inverse()", "robot motion → point motion")],
               why=["the robot drives forward, so the points come closer: the opposite motion"])
        w.done()

        w = Walk(self, 506, 512)
        w.show()
        w.step([506, 507, 508, 509, 510, 511, 512],
               "[Every old point is moved by the inverted guess:] [rotate, then shift.] "
               "[So ICP starts close to the answer] and only has to fix what the guess got wrong.",
               tokens=[(509, "c0 * p.x - s0 * p.y + tx_gp", "rotate, then shift")],
               why=["ICP starts close to the answer and only fixes what the guess got wrong"])
        w.done()

        w = Walk(self, 514, 517)
        w.show()
        w.step(514,
               "[T total collects all steps.] [It starts at the guess,] and each round of the loop adds its step.",
               tokens=[(514, "T_total", "all steps so far, starts at the guess")])
        w.step([516, 517],
               "[A k d tree is built on the new scan.] [A k d tree] finds the nearest point quickly, without checking all "
               "two hundred and fifty. It's built on [the new scan,] [because that's the set we search in,] and the new scan "
               "never moves. So one tree serves all twenty rounds.",
               tokens=[(516, "kdtree", "fast nearest-point search"), (517, "tgt_cloud", "the new scan")],
               why=["we search the new scan, and it never moves: one tree serves all 20 rounds"],
               instead=["a tree on the old scan: it moves every round, so it would have to be rebuilt"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

    def loop(self):
        self.plain("One round of the loop", "TASK 3",
                   ["1  pair every old point with its nearest new point", "2, 3  centres of both sets, and how they're turned",
                    "4  best rotation and shift", "5, 6  move the old points, add the step to the total", "7  stop when the step is tiny"],
                   "[Each round has seven steps. Step one pairs every old point with its nearest new point.] [Two and three] compute "
                   "the centres of both point sets, and how the shapes are turned against each other. [Four] finds the best rotation "
                   "and shift. [Five and six] move the old points and add the step to the total. [Seven] stops when the step is tiny.")
        h = self.section("TASK 3: the ICP loop", "TASK 3 · lines 519–590")
        w = Walk(self, 519, 525)
        w.show()
        w.step(519,
               "[The loop] runs [at most twenty rounds.] [It usually stops earlier;] twenty only caps the time per scan.",
               tokens=[(519, "iter < 20", "at most 20 rounds")],
               why=["it usually converges earlier; 20 caps the time per scan (a new scan comes every 50 ms)"])
        w.step([522, 523],
               "[Each round starts with two empty lists:] [the old points that found a partner,] and [their partners.] "
               "[In C plus plus,] a vector is a list that can grow.",
               tokens=[(522, "src_pts", "old points with a partner"), (523, "tgt_pts", "their partners")],
               cpp=["std::vector<T>: a list of T that can grow"])
        w.done()

        w = Walk(self, 526, 535, max_chars=84)
        w.show()
        w.step(526,
               "[Step one: pairs.] The loop goes over [every old point.] "
               "[In C plus plus,] this for loop gives each element in turn, by reference, without copying it.",
               tokens=[(526, "src_transformed->points", "every old point")],
               cpp=["for (const auto &p : list): p is each element in turn (no copy)"])
        w.step(529,
               "[The k d tree finds] [the one nearest new point.] The pair is kept only if [the squared distance is below one], "
               "so closer than one metre. [The tree already returns squared distances,] so comparing with one needs no square root.",
               tokens=[(529, "nearestKSearch(p, 1", "the 1 nearest new point"), (529, "nn_dist_sq[0] < 1.0", "closer than 1 m")],
               why=["the tree returns squared distances: d² < 1 means d < 1 m, without a square root"])
        w.step([531, 532, 533],
               "[If it's close enough,] [the partner is looked up] by its index, and [both points are added] to the lists.",
               tokens=[(531, "nn_idx[0]", "index of the partner"), (532, "emplace_back", "add to the list")])
        w.done()

        w = Walk(self, 537, 539)
        w.show()
        w.step([538, 539],
               "[If fewer than three pairs are left,] the loop [stops.] [With so few pairs,] the rotation and shift can't be "
               "determined reliably.",
               tokens=[(538, "< 3", "too few pairs")],
               why=["too few pairs: rotation and shift aren't determined reliably"])
        w.done()

        w = Walk(self, 541, 550)
        w.show()
        w.step([542, 543, 544, 545, 546, 547, 548],
               "[Step two: the centres.] [The centre of the old points] and [the centre of their partners] are just averages: "
               "add everything up, then divide by the count. [Once the rotation is known,] the shift is just the difference "
               "of the two centres.",
               tokens=[(542, "src_mean", "centre of the old points"), (543, "tgt_mean", "centre of the partners")],
               why=["once the rotation is known, the shift is just the difference of the centres"])
        w.step([549, 550],
               "[Here's the division.] [Static cast] turns the count into a decimal number. [In C plus plus,] it's the safe, "
               "explicit way to convert a type.",
               tokens=[(549, "static_cast<double>", "count as a decimal number")],
               cpp=["static_cast<double>(n): convert n to a double, on purpose and visibly"])
        w.done()

        w = Walk(self, 552, 557)
        w.show()
        w.step(556,
               "[Step three: the matrix H.] For every pair it takes [the old point minus its centre,] times [the new point minus "
               "its centre, turned into a row.] [In C plus plus,] dot transpose turns a column into a row, so column times row "
               "gives a two by two matrix. [Why subtract the centres?] That removes the shift. What's left in H is only how the "
               "two shapes are turned against each other.",
               tokens=[(556, "src_pts[i] - src_mean", "old point, centred"),
                       (556, "(tgt_pts[i] - tgt_mean).transpose()", "new point, centred, as a row")],
               cpp=[".transpose(): column → row;  column × row gives a 2×2 matrix"],
               why=["subtracting the centres removes the shift: H only holds how the shapes are turned"])
        w.done()

        w = Walk(self, 559, 569, max_chars=84)
        w.show()
        w.step(560,
               "[Step four: the rotation.] [The singular value decomposition] splits H into three simpler matrices, U, sigma and V. "
               "[This is the Kabsch method:] it gives the best rotation directly, with no searching.",
               tokens=[(560, "JacobiSVD", "H = U · Σ · Vᵀ")],
               why=["Kabsch method: the best rotation in closed form, no search"])
        w.step(562,
               "[The best rotation is] [R equals V times U transposed.] The questions at the end show where that comes from.",
               tokens=[(562, "V * svd.matrixU().transpose()", "R = V · Uᵀ")])
        w.step([563, 564, 565, 566, 567, 568],
               "[One check.] [If R's determinant is negative,] R is a mirror image, not a rotation. "
               "[Flipping the last column of V] gives the best real rotation. [A mirror image can't happen] to a real robot.",
               tokens=[(563, "R.determinant() < 0", "a mirror image"), (566, "V.col(1) *= -1.0", "flip the last column")],
               why=["determinant −1 means a reflection; a robot can only rotate"])
        w.step(569,
               "[Then the shift:] [the new centre minus the rotated old centre.] [After rotating,] this moves the old centre "
               "exactly onto the new one.",
               tokens=[(569, "tgt_mean - R * src_mean", "the shift t")],
               why=["after rotating, t moves the old centre onto the new centre"])
        w.done()

        w = Walk(self, 571, 578)
        w.show()
        w.step([574, 575],
               "[Step five: every old point] is moved by [this round's rotation and shift.] "
               "[In the next round,] the nearest neighbours change, so the pairs get better.",
               tokens=[(574, "R(0, 0) * p.x + R(0, 1) * p.y + t(0)", "rotate, then shift")],
               why=["next round, the nearest neighbours change, so the pairs improve"])
        w.step([576, 577],
               "[The results are stored as float] [with static cast,] [because P C L stores points as float.]",
               tokens=[(576, "static_cast<float>", "back to float")],
               why=["PCL points use float, the Eigen math uses double"])
        w.done()

        w = Walk(self, 580, 589)
        w.show()
        w.step([581, 582, 583],
               "[Step six: the step becomes a matrix.] [R goes top left,] and [t goes into the last column.] "
               "[In C plus plus,] block picks a part of a matrix: its size in the angle brackets, its corner in the round ones.",
               tokens=[(582, "block<2, 2>(0, 0)", "R, top left"), (583, "block<2, 1>(0, 2)", "t, last column")],
               cpp=[".block<rows, cols>(row, col): the part of the matrix starting at (row, col)"])
        w.step(584,
               "[Then it's added to the total:] [T step times T total.] [The newest step goes on the left,] because it happens "
               "after all the earlier ones, and matrix products apply from right to left.",
               tokens=[(584, "T_step * T_total", "newest step on the left")],
               why=["products apply right to left; the new step comes after all earlier ones"])
        w.step([587, 588, 589],
               "[Step seven: stop when the step is tiny.] [Less than a tenth of a millimetre] and [less than a ten thousandth of a "
               "radian.] [More rounds would change nothing] you could measure.",
               tokens=[(588, "t.norm() < 1e-4", "under 0.1 mm"), (588, "std::abs(step_angle) < 1e-4", "under 0.0001 rad")],
               why=["converged: more rounds change nothing measurable"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

    def stats(self):
        h = self.section("Statistics and result", "TASK 3 · lines 593–628")
        w = Walk(self, 600, 609)
        w.show()
        w.step([600, 601, 602],
               "[After the loop, the pairs are formed once more,] [with the same one metre limit,] to measure how good the fit is.",
               tokens=[(602, "distSq[0] < 1.0", "same 1 m limit")])
        w.step([605, 606, 607], "[For each pair,] [the squared gap is added up.]",
               tokens=[(607, "dx * dx + dy * dy", "squared gap")])
        w.step(608,
               "[And the partner's distance] [from the robot.] [Part four needs it:] a small turn moves far points a lot.",
               tokens=[(608, "std::sqrt(q.x * q.x + q.y * q.y)", "distance from the robot")],
               why=["a turn by α moves a point at distance r by r·α (used in part 4)"])
        w.step(609, "[And N counts] [the pairs.]", tokens=[(609, "++N", "number of pairs")])
        w.done()

        w = Walk(self, 614, 620)
        w.show()
        w.step(618,
               "[The residual] is [the root mean square of the gaps:] the typical leftover distance between partners.",
               tokens=[(618, "std::sqrt(sumSq / static_cast<double>(N))", "typical leftover gap")])
        w.step(619,
               "[And the average radius.] [On the bag,] [typically two hundred and fifty pairs, sixteen centimetres, "
               "and twenty two metres.]",
               tokens=[(619, "sumRadius / static_cast<double>(N)", "average distance")],
               why=["on the bag: N ≈ 248 pairs, rms ≈ 0.165 m, radius ≈ 21.9 m (medians)"])
        w.done()

        w = Walk(self, 622, 628, max_chars=96)
        w.show()
        w.step(622,
               "[Finally, the inverse trick in reverse.] [T total] says how the points moved. [Inverting it] gives how the robot moved.",
               tokens=[(622, "T_total.inverse()", "point motion → robot motion")],
               why=["second half of the inverse trick: back to the robot's motion"])
        w.step(624,
               "[The result is three numbers:] [forward], [sideways], and [the turn angle], read from the matrix with atan two.",
               tokens=[(624, "T_robot(0, 2)", "forward"), (624, "T_robot(1, 2)", "sideways"),
                       (624, "atan2(T_robot(1, 0), T_robot(0, 0))", "turn angle")])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

    def questions(self):
        self.qa("Why one metre as the pair limit, and not more or less?",
                ["Too big: points pair up with other objects.", "Too small: no pairs when the guess is a bit off.",
                 "Between scans 0.05 s apart the robot moves ~5 cm."],
                "[Too big, and points pair up with different objects.] [Too small, and there are no pairs] when the guess is a "
                "bit off. [Between two scans fifty milliseconds apart,] the robot moves about five centimetres, so one metre "
                "leaves a large margin.")
        self.qa("Where does R = V·Uᵀ come from?",
                ["We want the R that minimises Σ ‖R sᵢ + t − tᵢ‖².", "With centred points: maximise trace(R H).",
                 "The SVD of H gives that maximum at R = V Uᵀ (Kabsch)."],
                "[We want the rotation that makes the summed squared gaps smallest.] [With centred points,] that's the same as "
                "maximising the trace of R times H. [The singular value decomposition gives that maximum] at R equals V U transposed.",
                spoken="Where does R equals V times U transposed come from?")
        self.qa("Why T_step · T_total, and not T_total · T_step?",
                ["T_total maps the original points to before this step.", "The new step is applied after it.",
                 "Products apply right to left: the newest goes on the left."],
                "[T total maps the original points] to where they were before this step. [The new step is applied after that.] "
                "[Matrix products apply from right to left,] so the newest step goes on the left.",
                spoken="Why T step times T total, and not the other way round?")
        self.qa("Does a small residual mean the match is right?",
                ["No. Along a long wall, point-to-point ICP slides:", "every point still has a close partner,",
                 "so the residual is small while the motion is wrong."],
                "[No. Along a long wall, point to point ICP can slide.] [Every point still finds a close partner,] "
                "[so the residual stays small while the motion is wrong.] It measures fit, not whether the motion is determined.")


# ════════════════════════════════════════════════════════════════════════════════════════
class Part4Fusion(Part):
    """ICP into the Kalman filter, pure-ICP mode, launch parameters, results, quiz."""

    def construct(self) -> None:
        self.opening(4, "ICP in the filter, and results",
                     ["ICP into the Kalman filter", "how sure is ICP?", "pure-ICP mode", "launch parameters",
                      "results on the bag, and a quiz"],
                     "[This is the last part.] We follow ICP's result [into the Kalman filter], see [how the filter decides how much "
                     "to trust it], look at [the pure ICP mode] and [the launch parameters], and finish with [the results on the bag, "
                     "and a short quiz.]")
        self.fusion()
        self.pure()
        self.launch()
        self.results()
        self.quiz()

    def fusion(self):
        self.where("filter", [
            "[ICP's result goes into the Kalman filter,] still inside scan callback, right after task three.",
            "[This block] runs only when use ICP is on and ICP only is off.",
        ], callers=["in scanCallback:", "lines 394–455"])
        h = self.section("ICP into the Kalman filter", "lines 398–402")
        w = Walk(self, 398, 402)
        w.show()
        w.step(398,
               "[First a check:] [at least ten pairs.] [With fewer,] the match isn't trustworthy, and the scan is not used.",
               tokens=[(398, "res.inliers >= 10", "only a solid match")],
               why=["fewer than 10 pairs: the result can't be trusted, so it's skipped"])
        w.step([401, 402],
               "[ICP measured a change between two scans.] [Divided by d t,] the forward shift becomes [a speed,] and the turn "
               "becomes a turn rate. [That's the same kind of measurement as the wheels,] so the filter can use the same H rows.",
               tokens=[(401, "res.delta(0) / dt_scan", "forward speed"), (402, "res.delta(2) / dt_scan", "turn rate")],
               why=["a change between scans, divided by dt, is a speed: the same measurement as the wheels"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.plain("How sure is ICP?", "FIX 2",
                   ["The filter weighs each sensor by its uncertainty σ.", "ICP's σ comes from the leftover gap (the residual).",
                    "The original made ICP look about 16 times too precise."],
                   "[The filter weighs every sensor by its uncertainty,] sigma. [For ICP, sigma comes from the leftover gap,] "
                   "the residual from part three. [The original formula made ICP look about sixteen times too precise,] "
                   "so this block was fixed.")
        h = self.section("How sure is ICP?", "FIX 2 · lines 420–424")
        w = Walk(self, 420, 424)
        w.show()
        w.step(420,
               "[The position error per scan] is [the residual itself,] about sixteen centimetres on the bag. "
               "[The leftover gap is exactly how far off one scan's position can be.]",
               tokens=[(420, "residual_rms", "≈ 0.165 m per scan")],
               why=["the leftover gap is the position error of one scan"])
        w.step(421,
               "[For the angle,] the residual is divided by [the average radius.] [A turn by a small angle] moves a point at "
               "distance r by r times that angle. So the angle error is the gap divided by r. The max with one avoids dividing "
               "by almost zero.",
               tokens=[(421, "std::max(avg_radius, 1.0)", "radius, at least 1 m")],
               why=["a turn by α moves a point at distance r by r·α, so α ≈ gap / r"])
        w.step([423, 424],
               "[Both are divided by d t:] [the position error becomes a speed error.] [Clamp keeps them] inside sensible limits. "
               "[The measurement is a speed,] so its error must be a speed error too.",
               tokens=[(423, "sigma_trans / dt_scan", "position error → speed error"), (423, "0.05, 5.0", "limits")],
               why=["z is a speed, so its error must be a speed: divide by dt"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.plain("What the original did", "FIX 2",
                   ["σ = rms / √N,  with N ≈ 248:  √N ≈ 16,  so 16 times too small",
                    "σ_ω without / dt:  20 times too small",
                    "ICP overruled wheels and gyro: fused path 2.7 m short"],
                   "[The original divided the residual by the square root of N.] With about two hundred and fifty pairs, "
                   "that's sixteen times smaller. [It also forgot d t for the turn rate,] twenty times too small. "
                   "[So ICP overruled the wheels and the gyroscope,] and the fused path came out two point seven metres short.")

        h = self.section("ICP into the Kalman filter", "lines 432–454")
        w = Walk(self, 432, 439)
        w.show()
        w.step([433, 434], "[The variances go into E z:] [sigma squared.]",
               tokens=[(433, "sigma_v * sigma_v", "variance = σ²")])
        w.step([436, 437, 438, 439],
               "[Then E z is divided by] [ICP gain.] A gain of two halves the variance, so ICP counts twice as much. "
               "[In C plus plus,] slash equals means: divide and store. "
               "[On the bag,] sigma v is about three metres per second, so the speed comes from the wheels, while ICP still "
               "counts for the turn rate.",
               tokens=[(438, "Ez /= icp_gain_", "gain 2: half the variance")],
               cpp=["x /= y  means  x = x / y"],
               why=["bag: σ_v ≈ 3.3 m/s, so v comes from the wheels; σ_ω ≈ 0.15 rad/s, so ICP counts for ω"])
        w.done()

        w = Walk(self, 445, 454, max_chars=92)
        w.show()
        w.step([445, 446],
               "[The update is the usual Kalman correction.] [S] is the uncertainty of the difference, and [K] is the Kalman gain.",
               tokens=[(445, "S =", "uncertainty of the difference"), (446, "K =", "Kalman gain")])
        w.step(447, "[The innovation] is [measured minus expected.]",
               tokens=[(447, "z - H * state_.x", "measured − expected")])
        w.step(454,
               "[The covariance uses the Joseph form.] [It's longer,] [but it stays symmetric and positive,] even with rounding errors.",
               tokens=[(454, "(I - K * H) * state_.Ex", "Joseph form")],
               why=["stays symmetric and positive even with rounding errors"],
               instead=["(I − K H) · Ex: shorter, but rounding can make it lose symmetry"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

        self.qa("Why was σ = rms / √N replaced?",
                ["√N assumes N independent measurements.", "Neighbouring lidar points lie on the same wall:",
                 "they make the same error and slide together."],
                "[Dividing by the square root of N assumes N independent measurements.] [Lidar points aren't independent:] "
                "neighbours lie on the same wall, [they make the same error, and slide together.]",
                spoken="Why was sigma equals r m s over the square root of N replaced?")
        self.qa("What does icp_gain do, and what value would you choose?",
                ["Ez is divided by it: 2 means trust ICP twice as much.", "Simulation: gain 0.01 → 2.0 m end error, 2 → 4.5 m, 10 → 9.9 m.",
                 "It can't fix sliding; it only changes the trust."],
                "[E z is divided by the gain,] so two means trust ICP twice as much. [In the simulation with a corridor,] a gain "
                "of zero point zero one ended two metres off, two ended four and a half, and ten almost ten. "
                "[The gain can't fix sliding.] It only changes how much the filter listens.",
                spoken="What does ICP gain do, and what value would you choose?")

    def pure(self):
        self.where("pose", [
            "[In pure ICP mode,] the filter doesn't run. ICP's motion is added straight to the pose, and the node publishes it.",
        ])
        h = self.section("Pure-ICP mode", "lines 459–472")
        w = Walk(self, 459, 468)
        w.show()
        w.step(459,
               "[Here ICP starts at] [zero,] [because the filter doesn't run] in this mode, so there's no speed to guess from.",
               tokens=[(459, "Eigen::Vector3d::Zero()", "start at zero")],
               why=["no filter in this mode: no speed to make a guess from"])
        w.step([462, 463, 464, 465],
               "[The motion is in the robot's frame.] [It's rotated by the heading] into the odom frame, [and added to the pose,] "
               "[like the odometry in exercise two.]",
               tokens=[(464, "c * delta(0) - s * delta(1)", "rotated into odom"), (464, "+=", "added to the pose")],
               why=["delta is in the robot frame: rotate by θ before adding (as in Exercise 2)"])
        w.step(468, "[This branch] [publishes the transform itself.]",
               tokens=[(468, "publishTF", "publish the pose")])
        w.done()
        w = Walk(self, 471, 472)
        w.show()
        w.step([471, 472],
               "[In every mode, at the end,] [the new scan becomes the old one,] even if ICP failed. "
               "[Comparing against an ever older scan] would make the gap, and the error, grow.",
               tokens=[(471, "*last_cloud_base_", "the new scan becomes the old one")],
               why=["always the latest scan: an ever older one would make the gap and the error grow"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)

    def launch(self):
        self.plain("Where the scan comes from", "launch parameters",
                   ["The Ouster lidar gives a 3D point cloud.", "pointcloud_to_laserscan cuts a horizontal slice",
                    "and turns it into a 2D scan for our node."],
                   "[The Ouster lidar gives a three D point cloud.] [A node called point cloud to laser scan cuts a horizontal "
                   "slice out of it,] [and turns it into the two D scan our node reads.] Its parameters are in the launch file.")
        h = self.section("Launch parameters", "icp_odom.launch.py")
        w = Walk(self, 21, 31, src=LAUNCH, language="python")
        w.show()
        w.step(21, "[Heights are measured] from [the lidar's own frame.]",
               tokens=[(21, "'os_lidar'", "heights from the lidar")])
        w.step([23, 24],
               "[The slice] goes from [two] to [six metres above the lidar.] [That's two and a half to six and a half metres "
               "above the ground:] facades, roofs and tree tops, but not people or cars, which move.",
               tokens=[(23, "2.0", "2 m above the lidar"), (24, "6.0", "6 m above the lidar")],
               why=["2–6 m above the lidar = 2.5–6.5 m above ground: rigid things, not people or cars"])
        w.step(27,
               "[The angle step] is [zero point zero zero eight seven radians,] half a degree, about seven hundred and twenty rays.",
               tokens=[(27, "0.0087", "0.5° per ray")])
        w.step([29, 30],
               "[The range limits:] [forty five centimetres] cuts off the robot itself, and [fifty metres] is the far limit.",
               tokens=[(29, "0.45", "skip the robot"), (30, "50.0", "far limit")])
        w.step(31, "[And use inf] [reports empty rays as infinity,] [which our loop skips.]",
               tokens=[(31, "True", "empty rays = inf")],
               why=["our scan loop skips rays that aren't finite"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)
        self.qa("Why start the slice at 2 m, and what if you change it?",
                ["Below 2 m: people and vehicles move.", "Higher min_height: fewer moving objects, but fewer points.",
                 "Lower max_height: cuts leafy tree tops, which change every scan."],
                "[Below two metres, people and vehicles move.] [A higher minimum height removes more moving objects,] but leaves "
                "fewer points. [A lower maximum height cuts off leafy tree tops,] which change shape every scan.")

    def results(self):
        h = header("Results on the bag", "40 s drive, dlg_feldtage_24:cut")
        self.play(m.FadeIn(h), run_time=0.4)
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
        self.say("[On the real bag, the wheels] end twenty six metres down the field. [ICP alone] ends at twenty two, about "
                 "fifteen percent short.",
                 m.AnimationGroup(m.Create(wheels), m.FadeIn(wl), run_time=1.0),
                 m.AnimationGroup(m.Create(icp), m.FadeIn(il), run_time=1.0))
        rows = [("end of the 40 s drive", "original", "fixed"), ("no ICP", "4.88, −26.22, −78.6°", "1.74, −26.34, −90.0°"),
                ("ICP + filter", "2.16, −23.48, −85.3°", "2.25, −26.34, −88.0°"), ("bag with --loop", "frozen at 44.5 s", "resets, continues")]
        tbl_rows = []
        for r, row in enumerate(rows):
            g = m.VGroup()
            for j, txt in enumerate(row):
                cell = T(txt, 21, MUTED if r == 0 else (GREEN if j == 2 else FG))
                cell.move_to([[-3.8, 0.9, 4.6][j], -1.2 - r * 0.5, 0])
                g.add(cell)
            tbl_rows.append(g)
        self.say("[With the three fixes,] [the filter without ICP matches exercise four again,] [the fused filter keeps the "
                 "wheels' distance,] [and the bag can loop.]",
                 *[m.FadeIn(g, run_time=0.5) for g in tbl_rows])
        self.clear()
        h = header("Why does ICP fall short here?", "the question on the sheet")
        self.play(m.FadeIn(h), run_time=0.4)
        why = card(["point-to-point ICP slides along walls and fences", "few rigid features 2–6 m up on an open field",
                    "far points are sparse: 0.5° at 20 m ≈ 17 cm apart", "leafy tree tops change shape every scan"],
                   size=24, title="why").move_to([-2.6, 0.2, 0])
        alt = card(["point-to-line ICP (PL-ICP), GICP", "outlier rejection", "check the geometry before trusting ICP", "NDT"],
                   size=24, title="better approaches", title_color=GREEN).move_to([4.1, 0.2, 0]).scale(0.92)
        self.say("[This is the question on the sheet.] Point to point ICP slides along walls, the slice has few rigid features on "
                 "an open field, far points are sparse, and leafy tree tops change shape.", m.FadeIn(why, run_time=0.6))
        self.say("[Better approaches:] point to line ICP measures the distance to the line through the neighbours, so sliding "
                 "along a wall costs nothing. Add outlier rejection, and a check whether the geometry pins down the motion at all.",
                 m.FadeIn(alt, run_time=0.6))
        self.clear()

    def quiz(self):
        qs = [("What does getTransform return, and in which direction?",
               "A transform from the sensor frame into base_footprint, or false.", None,
               "A rotation and shift from the sensor frame into base footprint, or false."),
              ("Why is the initial guess sideways zero?", "Wheeled robots can't slide sideways.", None, None),
              ("Why invert the guess and the result?", "ICP moves points, which move opposite to the robot.", None, None),
              ("Why R = V·Uᵀ, and what if det R < 0?", "Best least-squares rotation; flip V's last column.",
               "Why R equals V times U transposed, and what if its determinant is negative?",
               "It's the best least squares rotation. If the determinant is negative, flip the last column of V."),
              ("Why does ICP enter the filter as a speed?", "It measures a change between scans, not a position.", None, None),
              ("Which three things were fixed?", "Loop reset, ICP noise (no √N, σ_ω / dt), /imu/data_raw.",
               None, "The reset when the bag loops, the ICP noise formula, and the IMU topic.")]
        h = header("Quick quiz", "pause the video and answer first")
        self.play(m.FadeIn(h), run_time=0.4)
        for q, a, q_sp, a_sp in qs:
            qc = qa_card(q).move_to([0, 1.0, 0])
            if qc.width > 13:
                qc.scale_to_fit_width(13)
            ac = T(a, 28, GREEN).next_to(qc, m.DOWN, buff=0.8)
            if ac.width > 12.6:
                ac.scale_to_fit_width(12.6)
            self.say("[" + (q_sp or q) + "]", m.FadeIn(qc, run_time=0.5))
            self.wait(1.5)
            self.say("[" + (a_sp or a) + "]", m.FadeIn(ac, run_time=0.5))
            self.wait(0.6)
            self.play(m.FadeOut(qc), m.FadeOut(ac), run_time=0.3)
        self.closing(["That's the whole node.", "Short version: the overview video.", "Details: the study guide and the cheatsheet."],
                     "[That's the whole node.] [For a short version, watch the overview video,] [and for details, read the study "
                     "guide and the cheatsheet.] Good luck in the Testat.")


# ════════════════════════════════════════════════════════════════════════════════════════
RVIZ_CLIP = Path(__file__).parent / "data" / "rviz_clip"   # tools/rviz_run.sh + capture_rviz.py + prep_rviz.py
RVIZ_FPS = 255 / 44.9                                      # frames grabbed per second of the bag


class Clip(m.Group):
    """A screen recording (a folder of JPEG frames) that plays in real time once added to the scene."""

    def __init__(self, folder, fps, height, center):
        super().__init__()
        self.files = sorted(Path(folder).glob("*.jpg"))
        if not self.files:
            raise FileNotFoundError(f"no frames in {folder}: run tools/rviz_run.sh with tools/capture_rviz.py first")
        self.fps, self.h, self.c = fps, height, center
        self.t, self.i = 0.0, -1
        self._show(0)
        self.add_updater(self._tick)

    def _tick(self, mob, dt):
        self.t += dt
        i = min(int(self.t * self.fps), len(self.files) - 1)
        if i != self.i:
            self._show(i)

    def _show(self, i):
        img = m.ImageMobject(str(self.files[i]), scale_to_resolution=0)
        img.scale_to_fit_height(self.h).move_to(self.c)
        if self.submobjects:
            self.remove(*self.submobjects)
        self.add(img)
        self.i = i

    @property
    def duration(self):
        return len(self.files) / self.fps


class Part5Extras(Part):
    """The node live in RViz, a known weakness, and the questions the first four parts left out."""

    def construct(self) -> None:
        self.opening(5, "Live in RViz, and loose ends",
                     ["the node running on the bag, in RViz", "a known weakness: no predict before the ICP correction",
                      "four more Testat questions"],
                     "[This is a short extra part.] First we watch [the node running on the real bag, in RViz.] Then a "
                     "[small weakness in the code] that an examiner might ask about, and [four more questions] that the "
                     "first four parts left out.")
        self.how_to_run()
        self.rviz()
        self.weakness()
        self.questions()
        self.closing(["That's everything.", "Practise: explain getTransform, the inverse trick", "and the noise fix in your own words."],
                     "[That's everything.] [The best test now is to explain getTransform, the inverse trick] [and the noise fix "
                     "out loud, in your own words.] Good luck.")

    def how_to_run(self):
        self.plain("How this recording was made", "running the node",
                   ["pointcloud_to_laserscan: 3D cloud → 2D scan (launch file parameters)",
                    "icp_node with use_icp:=true, icp_gain:=2.0: filter + ICP",
                    "ros2 bag play dlg_feldtage_24:cut --clock --remap /tf:=/tf_old",
                    "RViz: fixed frame odom, LaserScan /ouster/scan, TF, the path"],
                   "[First, the scan converter] cuts the slice out of the three D cloud, with the launch file's parameters. "
                   "[Then our node, with ICP and the filter on.] [The bag plays with its own TF remapped away,] because the bag "
                   "already contains the robot's recorded odom transform. Without the remap, two publishers would fight over "
                   "the same transform. [RViz shows the scans, the frames, and the path.]")

    def rviz(self):
        h = header("The node live in RViz", "filter + ICP on dlg_feldtage_24:cut, real time, seen from above")
        self.play(m.FadeIn(h), run_time=0.4)
        clip = Clip(RVIZ_CLIP, RVIZ_FPS, height=5.7, center=[-2.75, -0.75, 0])
        frame = m.SurroundingRectangle(clip, buff=0.02, color=MUTED, stroke_width=1.5)
        self.add(frame, clip)
        notes = [T(s, 22, c) for s, c in [("teal: lidar scans, kept for 60 s", TEAL),
                                           ("orange: the path, TF odom → base_footprint", AMBER),
                                           ("axes: the frames odom and base_footprint", FG),
                                           ("sharp walls: motion and scans agree", GREEN),
                                           ("one grid square: 5 m", MUTED)]]
        col = m.VGroup(*notes).arrange(m.DOWN, aligned_edge=m.LEFT, buff=0.32)
        if col.width > 5.0:
            col.scale_to_fit_width(5.0)
        col.move_to([4.15, -0.4, 0])
        start = self.time
        self.say("[Here is the node running on the real bag,] in real time, seen from above. "
                 "[The teal points are the lidar scans.] RViz keeps each scan on screen for a minute, so together they draw a map "
                 "of the field.", m.FadeIn(frame, run_time=0.4), m.FadeIn(notes[0], run_time=0.5))
        self.say("[The orange line is the robot's path:] the transform odom to base footprint that our node publishes. "
                 "[The two small sets of axes] are the frames odom, where the robot started, and base footprint, the robot itself.",
                 m.FadeIn(notes[1], run_time=0.5), m.FadeIn(notes[2], run_time=0.5))
        self.say("[Now watch the walls and the fences.] Every scan is drawn where our node says the robot was. "
                 "If the motion estimate were wrong, the same wall would show up twice, shifted or smeared. "
                 "[Here the walls stay mostly sharp,] so the motion and the scans agree.",
                 m.Indicate(frame, scale_factor=1.0, color=AMBER), m.FadeIn(notes[3], run_time=0.5))
        self.say("[One grid square is five metres.] By the end, the robot has driven about twenty six metres, "
                 "the same distance the wheels measured.", m.FadeIn(notes[4], run_time=0.5))
        rest = clip.duration - (self.time - start)
        if rest > 0:
            self.wait(rest + 0.5)
        self.wait(1.0)
        clip.clear_updaters()
        self.clear()

    def weakness(self):
        self.plain("A known weakness", "not fixed, but worth knowing",
                   ["The odom and IMU callbacks call predict before they correct.",
                    "The scan callback doesn't: ICP corrects the state of the last odom or IMU message,",
                    "a few milliseconds before the scan."],
                   "[Every odometry and IMU callback calls predict before it corrects.] [The scan callback doesn't.] "
                   "So ICP corrects the state of the last odometry or IMU message, [a few milliseconds before the scan was taken.]")
        h = self.section("A known weakness", "odomCallback vs. scanCallback")
        w = Walk(self, 218, 221)
        w.show()
        w.step(221,
               "[In the odometry callback,] [predict moves the state] to the message's time [before the correction.]",
               tokens=[(221, "predict(state_, dt)", "state moved to this message's time")],
               why=["first predict to the measurement's time, then correct"])
        w.done()
        w = Walk(self, 394, 398)
        w.show()
        w.step([394, 395, 396, 397, 398],
               "[In scan callback,] [the correction starts right after ICP,] with no predict in between. "
               "[Strictly, it should first predict] to the scan's time stamp and update filter time.",
               tokens=[(396, "icp(", "then straight to the correction")],
               why=["missing: state_ = predict(state_, scan time − filter_time_);  filter_time_ = scan time"])
        w.done()
        self.play(m.FadeOut(h), run_time=0.4)
        self.plain("Why it barely matters here", "a known weakness",
                   ["The gap is at most a few ms (odom 50 Hz, IMU 125 Hz).", "ICP measures speeds, v and ω, not a position.",
                    "Speeds hardly change in a few ms, so the error is tiny."],
                   "[The gap is at most a few milliseconds,] because odometry and IMU arrive fifty and a hundred and twenty five "
                   "times a second. [And ICP measures speeds, not a position.] [A speed hardly changes in a few milliseconds,] "
                   "so the error is tiny. If you're asked, name the fix: predict to the scan's time first.")

    def questions(self):
        self.qa("A pair is exactly 1 m apart. Is it kept?",
                ["No. The test is nn_dist_sq[0] < 1.0.", "1.0 < 1.0 is false, so exactly 1 m is rejected.",
                 "A classic trap in worked examples."],
                "[No. The test is: squared distance smaller than one.] [One is not smaller than one,] so a pair exactly one "
                "metre apart is rejected. [That's a classic trap] in worked examples.",
                spoken="A pair is exactly one metre apart. Is it kept?")
        self.qa("Why must σ_ω be divided by dt?",
                ["z = Δθ / dt is a rate, in rad/s.", "rms / r̄ is an angle error per scan, in rad.",
                 "Without / dt (0.05 s) it was 20× too small."],
                "[The measurement is a turn rate, radians per second.] [Residual over radius is an angle error per scan,] in "
                "radians. Dividing by d t turns it into a rate error. [Without it, the error was twenty times too small,] "
                "and ICP overruled the gyroscope.",
                spoken="Why must the turn rate error, sigma omega, be divided by d t?")
        self.qa("Would a finer angle step or a smaller range_max help?",
                ["Finer angle: more points, a little better (sim: 18.1 → 14.9 m),", "but more computation.",
                 "Smaller range_max: worse (sim: 20.4 m). Far features are often the only clue to forward motion."],
                "[A finer angle step gives more points and helps a little,] [at the cost of computation.] "
                "[A smaller maximum range made it worse in the simulation,] because on an open field the far features are "
                "often the only clue to forward motion.",
                spoken="Would a finer angle step, or a smaller maximum range, help?")
        self.qa("What do the TF broadcaster and the TF listener do?",
                ["broadcaster: publishes our result, odom → base_footprint", "listener: subscribes /tf and /tf_static, fills the buffer",
                 "getTransform asks that buffer how each sensor is mounted"],
                "[The broadcaster publishes our own result,] odom to base footprint. [The listener subscribes to TF and TF static] "
                "and fills the buffer. [getTransform then asks that buffer] how each sensor is mounted.",
                spoken="What do the TF broadcaster and the TF listener do?")
