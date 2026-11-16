# Übung 4: the Kalman filter, decision by decision

This file explains every piece of `src/kalman_node.cpp`: what each variable is for, why it has the value it has, and what the alternatives would have been. The order follows the data: a measurement pair arrives, the callback runs, then `predict`, then the two corrections, then the transform goes out.

Everything here was built and run on 16 Nov 2026 against `dlg_feldtage_24:cut`. The test results are at the end.

---

## 1. The big picture

The robot has two sensors that both say something about its motion, and both are wrong in different ways:

| Sensor | Measures | Good at | Bad at |
|---|---|---|---|
| Wheel odometry (`/odom`, 50 Hz) | forward speed `v`, yaw rate `ω` | `v`: wheels roll, speed is accurate | `ω`: four-wheel skid steering slips sideways in every turn |
| IMU (`/imu/data_raw`, 125 Hz) | yaw rate `ω` (gyro), acceleration (accelerometer) | `ω`: a gyro measures rotation directly, no slip | position: you would have to integrate acceleration twice, errors explode |

A Kalman filter keeps one best guess of the state **and** how sure it is about each part of it. Every time new measurements arrive it does two things:

1. **Predict**: "Given what I believed 20 ms ago and how robots move, where is it now?" The guess moves forward, and the uncertainty grows.
2. **Correct**: "The sensors say this. How much should I believe them compared to my prediction?" The guess moves toward the measurement, and the uncertainty shrinks.

How much the guess moves toward each measurement is decided by the **Kalman gain** `K`. `K` comes purely from the uncertainties: a precise sensor and an unsure prediction give a large `K`, the other way round gives a small `K`. That is why the noise matrices `Eu` and `Ez` are the real "tuning knobs" of the filter.

One callback in our node runs this pipeline:

```
odom + imu pair arrives
  └─ callback()
       ├─ dt from the timestamps
       ├─ predict(state, dt)          model step:  x = A x,  Ex = A Ex Aᵀ + Eu
       ├─ correct(state, odom, dt)    sensor step: v and ω from the wheels
       ├─ correct(state, imu, dt)     sensor step: ω and a from the IMU
       └─ send TF odom → base_footprint
```

---

## 2. The state vector and why it has six entries

```cpp
enum StateIndex : int { PX = 0, PY = 1, THETA = 2, V = 3, OMEGA = 4, ACC = 5 };
constexpr int STATE_SIZE = 6;

struct State
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd Ex = Eigen::MatrixXd::Identity(6, 6);
};
```

### What is in `x`

`x = [p_x, p_y, θ, v, ω, a]`

- **`p_x`, `p_y`, `θ`**: the pose. This is what we actually want, because it goes out as the TF transform. No sensor measures it directly.
- **`v`, `ω`**: speed and yaw rate. They are in the state because the sensors measure them. A Kalman filter can only use a measurement if the state contains (a function of) the measured quantity, because `H` has to map state → measurement.
- **`a`**: forward acceleration. It is in the state because the IMU measures it, and because it lets the model predict how `v` changes between measurements (`v += a·dt`).

Why not more? A 3D state (z, roll, pitch, vertical speed …) would need gravity handling and more sensors. The sheet keeps the problem 2D on purpose. Why not fewer? Without `ω` in the state we could not fuse the two yaw-rate sensors, and fusing them is the whole point of this exercise.

`v` and `a` are in the **robot's own frame** (along its forward axis), while `p_x`, `p_y` are in the **odom frame**. That mix is deliberate: the sensors measure in the robot frame, and we want the pose in the odom frame. The transition matrix `A` does the conversion with `cos θ` / `sin θ` (section 4).

### Why the `enum StateIndex`

The skeleton writes `state.x(2)` and the sheet writes `A(0,3)`. That works, but nobody can read `A(1,5)` without looking up the table. With the enum the code says `A(PY, ACC)`: "how much acceleration flows into the y position". The numbers are identical (`PY == 1`, `ACC == 5`), so every formula from the sheet still matches one to one. It is a plain `enum` with `int` values rather than an `enum class`, because Eigen's `operator()` takes integers and an `enum class` would need a cast at every use.

`STATE_SIZE = 6` is `constexpr` so it is known at compile time and replaces the magic number 6 in every `Identity(...)`/`Zero(...)` call.

### Why `Eigen::VectorXd` / `Eigen::MatrixXd` and not fixed-size types

The `X` means "size decided at runtime" and the `d` means `double`. That is what the skeleton's `State` struct uses, so I kept it: the given code (`state.x(2)`, `State()` in the reset) works with it unchanged.

The alternative would be `Eigen::Matrix<double, 6, 6>` and `Eigen::Matrix<double, 6, 1>`. Those are faster (no heap allocation) and the compiler would reject size mistakes such as multiplying a 2×6 by a 2×6. For a 6×6 filter at 50 Hz the speed difference does not matter, and changing the struct would change the given skeleton, so dynamic sizes are the safer choice here.

`double` rather than `float`: the covariance matrices are multiplied and inverted every step, and rounding errors accumulate. `double` costs nothing noticeable and keeps `Ex` symmetric and positive for longer.

### Why `x` starts at zero and `Ex` at the identity

- **`x = 0`**: the odom frame is *defined* to start where the robot starts, facing along x, standing still. So position 0, heading 0, speed 0 is exactly right, not a guess.
- **`Ex = I`**: variance 1 for every entry. This says "a bit unsure about everything". The value is not critical: within a few steps the measurements dominate. The two extremes would be worse. `Ex = 0` would claim perfect knowledge, so the first corrections would barely move the estimate. A huge `Ex` (for example 10⁶) is fine for `v`/`ω`/`a` but would make the first steps numerically jumpy.

---

## 3. The globals and the callback

```cpp
builtin_interfaces::msg::Time last_time;
nav_msgs::msg::Odometry last_odom;
sensor_msgs::msg::Imu last_imu;
bool first_cb = true;
State state;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
```

All of these come from the skeleton. Globals rather than members of a node class, because the node in `main` is a plain `rclcpp::Node` and the callback is a free function. I kept that structure so the solution matches the sheet line by line in the Testat.

| Variable | Purpose |
|---|---|
| `state` | The filter's belief, carried from one callback to the next. This is what makes the filter *recursive*: each step starts from the previous result. |
| `last_time` | Stamp of the previous odom message, used to compute `dt`. |
| `last_odom`, `last_imu` | The previous messages, used for `dt_odom` and `dt_imu`. |
| `first_cb` | `true` until we have one message to compute a time difference against. |
| `tf_broadcaster` | Sends the result on `/tf`. A `shared_ptr` because it is created in `main` after the node exists, but is used in the callback. |

### Why `last_time` starts at the maximum value

```cpp
last_time.sec = std::numeric_limits<int32_t>::max();
```

That is a trick in the given code: the first `dt = odom_time - last_time` is then hugely negative, so the first callback takes the `dt < 0` branch. That branch resets the state and sets `first_cb = true`. The same branch also handles the bag looping back to its start (the time jumps backwards). So one code path covers both "just started" and "bag restarted".

The order of what happens:

1. **Callback 1:** `dt < 0`, so the state is reset and the function returns. `last_time` is now valid.
2. **Callback 2:** `dt > 0` but `first_cb` is true, so no filtering happens. `last_odom` and `last_imu` get stored.
3. **Callback 3 onward:** the full pipeline runs.

### The three time differences

```cpp
auto dt = (odom_time - last_time).seconds();
double dt_odom = (odom_time - last_odom_time).seconds();
double dt_imu = (imu_time - last_imu_time).seconds();
```

- **`dt`** goes into `predict`: how far forward in time the model has to move the state. It is measured on odom stamps because the published TF carries the odom stamp, so the state should describe the robot at that moment.
- **`dt_odom`** scales the odometry noise. In our node it equals `dt`, because `last_time` and `last_odom` are both updated every callback.
- **`dt_imu`** scales the IMU noise. It is measured on IMU stamps because the synchronizer may pair an IMU message that is a few milliseconds off from the odom message, so the IMU's own interval is the honest one.

Stamps are taken from the message headers, not from the wall clock (`now()`). That way the filter is correct when the bag plays at a different speed, and when the sensor data arrives late.

### The pipeline

```cpp
state = predict(state, dt);
state = correct(state, *odom, dt_odom);
state = correct(state, *imu, dt_imu);
```

- **Predict before correct**, always. The correction compares measurements taken *now* with a prediction for *now*, so first we have to move the old belief to the present.
- **Odom first, then IMU.** For independent sensor noises, two sequential corrections give the same result as one joint correction with both sensors stacked into a 4-row `H`, so the order does not change the answer. Doing them separately keeps each `H` and `Ez` small and readable.
- **`*odom`** dereferences the `SharedPtr`, because `correct` takes a `const &` to the message. That avoids copying the message, which carries 36-element covariance arrays.
- **`state = f(state, ...)`**: the functions take `State` by value and return the new one, as in the skeleton. Copying a 6×6 matrix (288 bytes) is far cheaper than any risk of half-updated state.

### The log line

```cpp
static int counter = 0;
if (++counter % 100 == 0) { RCLCPP_INFO(rclcpp::get_logger("kalman_node"), ...); }
```

I added this so you can watch the state in the terminal. `RCLCPP_INFO_THROTTLE` would be nicer, but it needs a node clock, and the callback is a free function without access to the node. The callback runs at the odom rate (50 Hz), so every 100th call means about every 2 s. `static` keeps the counter alive between calls without adding another global.

---

## 4. `predict`: the motion model

```cpp
const double theta = s.x(THETA);
const double c = std::cos(theta);
const double s_theta = std::sin(theta);
```

`cos θ` and `sin θ` are computed once, because they appear four times. The name is `s_theta` rather than `s`, because `s` is already the `State` parameter, and shadowing it would be a bug waiting to happen.

### The transition matrix `A`

`A(i, j)` = "how much of state `j` goes into state `i` during `dt`". Row = target, column = source. The new state is `x_new = A · x`, so row `i` of `A` is the formula for the new value of state `i`.

Start with `A = I`: every state keeps its own value (`x_i,new = 1 · x_i`). Then add the physical couplings:

| Entry | Value | Formula it creates | Why |
|---|---|---|---|
| `A(PX, V)` (sheet: `A(0,3)`) | `cos θ · dt` | `p_x += v · cos θ · dt` | Distance travelled is `v·dt` along the robot's heading. Its x part in the odom frame is the `cos θ` share. |
| `A(PX, ACC)` (`A(0,5)`) | `½ · cos θ · dt²` | `p_x += ½ a cos θ dt²` | Constant acceleration adds `½ a t²` of distance (from `d = v t + ½ a t²`), again projected on x. |
| `A(PY, V)` (`A(1,3)`) | `sin θ · dt` | `p_y += v · sin θ · dt` | Same as `A(PX,V)`, but the y share of the heading is `sin θ`. |
| `A(PY, ACC)` (`A(1,5)`) | `½ · sin θ · dt²` | `p_y += ½ a sin θ dt²` | Same as `A(PX,ACC)` for y. |
| `A(THETA, OMEGA)` (`A(2,4)`) | `dt` | `θ += ω · dt` | Heading changes by yaw rate × time, as in Übung 2. |
| `A(V, ACC)` (`A(3,5)`) | `dt` | `v += a · dt` | Speed changes by acceleration × time. |

Nothing else is coupled. In particular `ω` and `a` keep their values (`A(OMEGA, OMEGA) = A(ACC, ACC) = 1`): the model assumes they stay constant over the 20 ms step. That "constant acceleration" assumption is exactly what `Eu(ACC, ACC) = 1000` admits is often wrong (see below).

**Sheet question: why does the result go into `A(0,3)`, and what does that entry mean?** Row 0 is the formula for the new `p_x`, and column 3 is `v`. So `A(0,3)` is the factor by which the speed contributes to the x position in one step, here `cos θ · dt`.

**`A(X,X)` (the diagonal):** 1 everywhere. Every state carries over to the next step unchanged unless something else is added to it. A diagonal entry below 1 would make the value decay toward zero by itself. That is sometimes used for velocities ("the robot slows down if nobody drives"), but not here.

**Why `θ` from the start of the step and not the midpoint (as in my Übung 2 node)?** The sheet defines `A` from the current state. Over 20 ms the robot turns at most about 0.03 rad, so the difference is tiny, and the IMU correction fixes `θ` every step anyway.

### A known simplification: this is not quite a textbook EKF

`A` contains `cos θ` and `sin θ`, so the motion model is **nonlinear** in `θ`. A textbook *Extended* Kalman filter propagates the covariance with the **Jacobian** `G = ∂(A x)/∂x`. `G` equals `A` plus two extra entries:

```
G(PX, THETA) = −sin θ · (v·dt + ½·a·dt²)
G(PY, THETA) =  cos θ · (v·dt + ½·a·dt²)
```

Those entries say "uncertainty in the heading turns into uncertainty in the position", which is physically true. The sheet uses `A` for both the state and the covariance, so I did the same. The result is that `Ex` underestimates position uncertainty and misses the correlation between heading and position. The effect on the estimate is small here: no sensor measures position directly, so the position only gets nudged through those correlations during corrections. This is a good point to raise in the Testat if you are asked "what makes it *extended*?".

### Process noise `Eu`

```cpp
Eu(PX, PX) = 1.0;  Eu(PY, PY) = 1.0;  Eu(THETA, THETA) = 1.0;
Eu(V, V) = 10.0;   Eu(OMEGA, OMEGA) = 10.0;
Eu(ACC, ACC) = 1000.0;
Eu *= dt;
```

`Eu` says how much uncertainty the *model* adds per step: "even if my last state was perfect, how wrong can the prediction be?" Only the diagonal is set, which means we assume the model errors of different states are independent.

- **Position and heading: 1.** These follow almost exactly from `v` and `ω` via kinematics, so the model is fairly good for them.
- **Speed and yaw rate: 10.** The model says "constant", but the driver can change them at any time, so the model is worse here.
- **Acceleration: 1000.** The model assumes constant acceleration, but acceleration jumps all the time (bumps on the field, motor commands). We basically do not trust the predicted `a` at all. In effect, `a` is taken almost completely from the IMU each step.

**Why `*= dt`:** uncertainty has to grow with the time we predict without looking. 40 ms without a measurement should add twice the uncertainty of 20 ms. Multiplying by `dt` makes `Eu` a rate ("variance per second"). Without it, the same values would mean very different things at 50 Hz and at 10 Hz.

`Eu` is created inside `predict` rather than once globally, because it depends on `dt`. Building a 6×6 matrix 50 times per second is negligible work.

### The prediction itself

```cpp
s.x = A * s.x;
s.Ex = A * s.Ex * A.transpose() + Eu;
s.x(THETA) = wrap_angle(s.x(THETA));
```

- **`A * s.x`**: applies all the formulas from the table at once.
- **`A Ex Aᵀ + Eu`**: moves the uncertainty through the same model and then adds the model's own error. The `Aᵀ` is there because covariance is "squared": if `x` is scaled by `A`, its covariance is scaled by `A` from both sides. The `Bu` term from the lecture is missing because nobody tells the filter what the motors were commanded to do. The sheet says the same.
- **`wrap_angle`**: keeps `θ` within ±π. `atan2(sin, cos)` is used instead of `fmod`, because `fmod` gets negative angles and the ±π boundary wrong. Wrapping is safe here because no sensor measures `θ` directly, so there is never an innovation like `θ_measured − θ_predicted` that could jump by 2π.

---

## 5. `kalman_update`: the shared correction step

The sheet says the IMU correction is "largely the same" as the odometry one. Instead of writing the three formulas twice, I put them into one helper. Both `correct` functions only build `z`, `H` and `Ez` and call it. That way a fix or change to the math happens in one place, and the two corrections cannot drift apart.

```cpp
State kalman_update(State s, const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & Ez)
{
  const Eigen::MatrixXd S = H * s.Ex * H.transpose() + Ez;
  const Eigen::MatrixXd K = s.Ex * H.transpose() * S.inverse();
  const Eigen::VectorXd innovation = z - H * s.x;

  s.x = s.x + K * innovation;
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  s.Ex = (I - K * H) * s.Ex;
  s.x(THETA) = wrap_angle(s.x(THETA));
  return s;
}
```

Line by line, matched to the sheet's formulas:

| Code | Sheet | Meaning |
|---|---|---|
| `S = H Ex Hᵀ + Ez` | the bracket in `K` | **Innovation covariance**: how uncertain the *expected* measurement is (our state uncertainty seen through `H`) plus the sensor's own noise. A 2×2 matrix, one row/column per measured value. |
| `K = Ex Hᵀ S⁻¹` | `K = Σ̄ Hᵀ (H Σ̄ Hᵀ + Σ_Z)⁻¹` | **Kalman gain**, 6×2: for each of the 6 states, how much each of the 2 measurement errors should move it. |
| `innovation = z − H x` | `(z − H x̄)` | **Innovation**: what the sensor says minus what we predicted it would say. If the prediction was perfect, this is 0 and nothing changes. |
| `x = x + K · innovation` | `μ = x̄ + K(z − H x̄)` | Move the state toward the measurement, as far as `K` allows. |
| `Ex = (I − K H) Ex` | `Σ = (I − K H) Σ̄` | Shrink the uncertainty of the states the measurement told us about. |

Why `K` has entries for states that were not measured: `K` is 6×2, not 2×2. If `Ex` contains a correlation between, say, `v` and `p_x` (built up by `A` over previous steps), then measuring `v` also nudges `p_x`. That is the filter "using what it learned about how states relate".

**Why `S.inverse()` and not something fancier:** `S` is 2×2, symmetric and positive definite (it is a covariance plus positive noise), so a direct inverse is exact and cheap. For large measurement vectors you would use `S.ldlt().solve(...)` instead of forming an inverse, but that is not needed for 2×2.

**Why not the Joseph form** `(I−KH) Ex (I−KH)ᵀ + K Ez Kᵀ`: that form keeps `Ex` symmetric even with rounding errors, and is the robust choice for long runs. I used the sheet's shorter formula so the code matches the lecture. With `double` and 40 s loops, `Ex` stayed well-behaved in the tests.

**Parameter types:** `State s` by value, because we modify and return a copy, as in the skeleton. `z`, `H`, `Ez` by `const &`, because they are only read and copying them would be wasted work. The local results are `const`, so they cannot be changed by accident after they are computed.

---

## 6. `correct(odom)`: wheel odometry

```cpp
Eigen::VectorXd z(2);
z << odom.twist.twist.linear.x,   // v
     odom.twist.twist.angular.z;  // omega
```

**`z` (the measurement vector):** the two numbers the wheels give us, in a fixed order, because the rows of `H` and `Ez` must use the same order. The `<<` comma initializer fills the vector in that order. `twist.twist.linear.x` is the forward speed in the robot frame, the same field as in Übung 2. `linear.y` and `linear.z` are always 0 for a ground robot that cannot drive sideways, so there is nothing to measure there.

The pose part of the odom message (`pose.pose`) is **not** used. It is the robot's own integration of the same wheel speeds, so feeding it in would count the same information twice and pretend we know the position much better than we do.

```cpp
Eigen::MatrixXd Ez = Eigen::MatrixXd::Zero(2, 2);
Ez(0, 0) = 0.1;   // x_vel
Ez(1, 1) = 100.0; // yaw_vel
Ez *= dt;
```

**Sheet question: what does this matrix say?** It is the sensor noise covariance: how much we distrust each measured value. 0.1 for `v` means "wheel speed is reliable". 100 for `ω` means "wheel yaw rate is nearly useless", which is the slip problem from Übung 2. The ratio is what matters: `ω` from the wheels is trusted 1000 times less than `v`. The off-diagonal entries are 0, because we assume the errors in `v` and `ω` are independent.

**`*= dt`:** the sheet asks to scale the error over time. Because `Eu` is also scaled by `dt`, the *ratio* between model trust and sensor trust stays about the same at any message rate: a higher rate means less added model noise per step, but also less trusted measurements per step. Without the scaling, running the same filter at 100 Hz instead of 50 Hz would quietly change how much it believes the sensors.

```cpp
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_SIZE);
H(0, V) = 1;
H(1, OMEGA) = 1;
```

**Sheet question: why 2 rows and 6 columns?** `H` maps the 6-dimensional state to the 2-dimensional measurement space: `H · x = [v, ω]`. One row per measured value (2), one column per state entry (6). A 1 in `H(0, V)` means "the first measured value is directly the state's `v`". Everything else is 0 because the wheels say nothing about position or acceleration.

---

## 7. `correct(imu)`: the IMU, and why it uses `acc.y`

```cpp
Eigen::VectorXd z(2);
z << imu.angular_velocity.z,     // omega
     imu.linear_acceleration.y;  // a (forward)
```

### `angular_velocity.z`

The gyro's z axis is the yaw rate. The IMU frame `imu_phidget` is turned 90° around z relative to the robot, but a rotation around z leaves the z axis pointing up. So the gyro's z value is directly the robot's `ω`. In the bag, the gyro z and the wheel `ω` correlate at +0.96, which confirms the axis and the sign.

### `linear_acceleration.y`, not `.x`

This is the one place where I deliberately did **not** do the obvious thing. The sheet says "lineare Beschleunigung", and the natural guess is `linear_acceleration.x`. But the IMU is mounted turned by 90°, so its x axis points *sideways* on this robot.

I checked this against the data. I compared each IMU axis with the forward acceleration we can compute from the wheels (the change of `v_odom` over time, smoothed over 0.5 s):

| IMU signal | Correlation with forward acceleration from the wheels |
|---|---|
| `linear_acceleration.x` | +0.03 (no relation) |
| `linear_acceleration.y` | **+0.74** |
| `−linear_acceleration.y` | −0.74 |

So forward acceleration is `+acc.y`. Using `.x` would feed the sideways (centripetal) acceleration into the filter as if it were forward acceleration.

**Why not look up the rotation from TF, as in my Übung 2 node?** Two reasons:

1. The skeleton has no TF listener, and the mounting is fixed, so a constant is simpler and has no startup race.
2. The bag's `/tf_static` says `imu_phidget` is turned +90°. That would make forward = `−acc.y`, which is the *opposite sign* of what the data shows. So either the TF or the sensor's own axes are mislabelled. Data beats labels: the measured correlation is the ground truth here.

**Gravity:** `data_raw` contains gravity (the mean `acc.z` is 9.85 m/s²). It only shows up in z, which we do not use. A slope of a few degrees leaks a little gravity into y: at standstill `acc.y` averages −0.36 m/s². That bias is real, and you can see it in the log as `a = −0.38` while the robot stands still. Its effect on the pose is small, because the odometry pins `v` to 0 at every step. Only the `½·a·dt²` term moves the position, about 3 cm over 6 s of standing in the test. A better filter would add the bias as a 7th state, which is what packages like `robot_localization` do.

```cpp
Ez(0, 0) = 0.1; // yaw_vel
Ez(1, 1) = 0.1; // x_acc
```

**`Ez` values:** 0.1 for both, as given on the sheet: the IMU is built to measure exactly these two things. Compare with the odometry: `ω` is 0.1 here versus 100 there, so the filter takes its yaw rate almost entirely from the gyro. `v` comes from the wheels (0.1), because the IMU does not measure speed at all.

```cpp
H(0, OMEGA) = 1;
H(1, ACC) = 1;
```

Same idea as for odometry: row 0 picks `ω`, row 1 picks `a`.

---

## 8. Things I changed or added beyond the sheet

| Change | Why |
|---|---|
| Headers `.hpp` instead of `.h` for `tf2_ros`, `tf2`, `message_filters` | In Jazzy the `.h` versions are deprecated and print warnings. The build is now warning-free. |
| `<depend>eigen</depend>` in `package.xml` | The code needs Eigen, but the given `package.xml` did not declare it, so `rosdep` would not install it on a fresh machine. |
| `#include <cmath>` and `<limits>` | For `cos`/`sin`/`atan2` and `numeric_limits`. They happened to compile via other headers, but relying on that breaks with other compilers. |
| `enum StateIndex`, `STATE_SIZE` | Readability, see section 2. |
| `wrap_angle` | Keeps `θ` in ±π, see section 4. |
| `kalman_update` helper | One implementation of the correction formulas, see section 5. |
| `linear_acceleration.y` | The IMU is mounted turned, see section 7. |
| Log line every ~2 s | So you can watch the filter work. |

Everything else (globals, callback, sync, TF output, the reset logic) is the given skeleton, unchanged.

---

## 9. Synchronisation, and the optional task

```cpp
using SyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>;
auto sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), odom_sub, imu_sub);
```

The synchronizer only calls `callback` with **one odom message and one IMU message whose stamps are close**. `ApproximateTime` rather than `ExactTime`, because two different sensors never have identical stamps. `10` is the queue size per topic: how many messages it keeps while looking for a matching partner.

The consequence: the callback runs at the rate of the slower sensor (odom, 50 Hz). Each odom message is paired with one IMU message, so **75 of the 125 IMU messages per second are thrown away**.

**Optional task: is a version without synchronisation better?** Yes, in principle. Give each topic its own callback. Each callback first runs `predict` with the time since the *last update from either sensor*, then corrects with only its own sensor. Then:

- every IMU message is used (125 Hz instead of 50 Hz), so `ω` and `a` are tracked more finely;
- a missing or late odom message no longer blocks the IMU update;
- the state is always predicted to the exact time of each measurement, instead of using one IMU sample at an approximate odom time.

The price: you need one shared "last update time" for the predictions, and the TF output would come at the combined rate. On this bag the gain would be small, because the robot mostly drives straight, but on a robot that turns a lot it matters. This is the design real fusion packages use.

---

## 10. Test results

Bag `dlg_feldtage_24:cut` played in a loop with `--remap /tf:=/tf_old`. The node's transform `odom → base_footprint` was recorded from `/tf`, and the pose just before each loop restart is the end of the 40 s drive. Both loops gave the same numbers.

| Estimate | End x | End y | End heading |
|---|---|---|---|
| Wheels only (Übung 2, same as the robot's own `pose.pose`) | 3.38 m | −26.37 m | −86.6° |
| Gyro integrated alone | | | −90.5° |
| **This Kalman filter, sheet values** | **1.46 m** | **−26.17 m** | **−90.3°** |
| Experiment: trust swapped (wheel `ω` noise 0.1, gyro `ω` noise 100) | 3.42 m | −26.22 m | −86.4° |

What this shows:

- With the sheet's values, the heading follows the gyro (−90.3° vs −90.5°). The roughly 4° that the wheels lose to slip in the turn is corrected. That 4° alone moves the end point by about 2 m sideways after 25 m of driving, which is the 3.38 m vs 1.46 m in x.
- `v` follows the wheels: the y distance (−26.2 m) matches the wheel-only result.
- When the trust is swapped, the result jumps back to the wheels-only answer (−86.4°). That is the answer to the sheet's "try out the values" question: **the ratio of the `ω` entries in the two `Ez` matrices decides which sensor the heading comes from.**
- The reset on bag loop works: the log shows the pose back at (0, 0) after each restart.
- Build: `colcon build --packages-select kalman`, no warnings.

Not checked: RViz was not opened. TF was checked from the recorded `/tf` messages instead.

## 11. Run it

```bash
ros2 bag play $(marina resolve dlg_feldtage_24:cut) --loop --remap /tf:=/tf_old
```

```bash
ros2 run kalman kalman_node
```

```bash
rviz2
```

In RViz set Fixed Frame to `odom` and add the TF display (and `/ouster/points` if you want the point cloud).
