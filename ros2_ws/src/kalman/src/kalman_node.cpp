#include <cstdio>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"

#include <Eigen/Dense>

using namespace std::chrono_literals;

// State vector indices, so the matrices below read as A(PX, V) instead of A(0, 3).
// x = [p_x, p_y, theta, v, omega, a]
enum StateIndex : int { PX = 0, PY = 1, THETA = 2, V = 3, OMEGA = 4, ACC = 5 };
constexpr int STATE_SIZE = 6;

builtin_interfaces::msg::Time last_time;

nav_msgs::msg::Odometry last_odom;
sensor_msgs::msg::Imu last_imu;
bool first_cb = true;

struct State
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd Ex = Eigen::MatrixXd::Identity(6, 6);
};

std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

State state;

// Keep the heading in [-pi, pi] so it does not grow without bound over many turns.
double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Prediction: move the state forward by dt with a constant-acceleration model.
//   x_pred  = A * x
//   Ex_pred = A * Ex * A^T + Eu
State predict(State s, double dt)
{
  const double theta = s.x(THETA);
  const double c = std::cos(theta);
  const double s_theta = std::sin(theta);

  // Transition model. Row = target state, column = source state:
  // A(i, j) says how much of state j flows into state i during dt.
  // The diagonal is 1: without any other influence every state keeps its value.
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

  // v and a are measured along the robot's own x axis (forward), but p_x/p_y
  // live in the odom frame. cos/sin(theta) rotate "forward" into odom x/y.
  A(PX, V) = c * dt;                     // p_x += v * cos(theta) * dt
  A(PX, ACC) = 0.5 * c * dt * dt;        // p_x += 1/2 * a * cos(theta) * dt^2
  A(PY, V) = s_theta * dt;               // p_y += v * sin(theta) * dt
  A(PY, ACC) = 0.5 * s_theta * dt * dt;  // p_y += 1/2 * a * sin(theta) * dt^2
  A(THETA, OMEGA) = dt;                  // theta += omega * dt
  A(V, ACC) = dt;                        // v += a * dt
  // omega and a: constant over dt (diagonal 1, no other entries).

  // Process noise: how much we distrust the model per second.
  // Scaled by dt: the longer we predict without a measurement, the less we know.
  Eigen::MatrixXd Eu = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  Eu(PX, PX) = 1.0;        // x position
  Eu(PY, PY) = 1.0;        // y position
  Eu(THETA, THETA) = 1.0;  // yaw
  Eu(V, V) = 10.0;         // x_vel
  Eu(OMEGA, OMEGA) = 10.0; // yaw_vel
  Eu(ACC, ACC) = 1000.0;   // x_acc
  Eu *= dt;

  s.x = A * s.x;
  s.Ex = A * s.Ex * A.transpose() + Eu;
  s.x(THETA) = wrap_angle(s.x(THETA));

  return s;
}

// Shared Kalman correction step, used by both sensors:
//   K  = Ex H^T (H Ex H^T + Ez)^-1
//   x  = x + K (z - H x)
//   Ex = (I - K H) Ex
State kalman_update(State s, const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & Ez)
{
  // S = innovation covariance: uncertainty of the predicted measurement plus sensor noise.
  const Eigen::MatrixXd S = H * s.Ex * H.transpose() + Ez;
  // S is 2x2 and symmetric positive definite, so a direct inverse is cheap and stable.
  const Eigen::MatrixXd K = s.Ex * H.transpose() * S.inverse();
  // Innovation: what the sensor says minus what we expected it to say.
  const Eigen::VectorXd innovation = z - H * s.x;

  s.x = s.x + K * innovation;
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  s.Ex = (I - K * H) * s.Ex;
  s.x(THETA) = wrap_angle(s.x(THETA));

  return s;
}

// Correction with wheel odometry: measures v (twist.linear.x) and omega (twist.angular.z).
State correct(State s, const nav_msgs::msg::Odometry &odom, double dt)
{
  Eigen::VectorXd z(2);
  z << odom.twist.twist.linear.x,   // v
       odom.twist.twist.angular.z;  // omega

  // Sensor noise: v from the wheels is good, omega from the wheels is bad
  // (skid steering: the wheels slip in every turn).
  Eigen::MatrixXd Ez = Eigen::MatrixXd::Zero(2, 2);
  Ez(0, 0) = 0.1;   // x_vel
  Ez(1, 1) = 100.0; // yaw_vel
  Ez *= dt;         // same time scaling as Eu, so the ratio between model and sensor stays fixed

  // Sensor model: picks the measured entries out of the 6D state.
  // 2 rows = 2 measured values, 6 columns = 6 state entries.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_SIZE);
  H(0, V) = 1;      // x_vel
  H(1, OMEGA) = 1;  // yaw_vel

  return kalman_update(s, z, H, Ez);
}

// Correction with the IMU: measures omega (gyro z) and the forward acceleration a.
State correct(State s, const sensor_msgs::msg::Imu &imu, double dt)
{
  // The IMU in dlg_feldtage_24:cut is mounted turned by 90 deg (frame imu_phidget):
  // the robot's forward acceleration shows up on the IMU's y axis.
  // Measured in the bag: corr(d v_odom/dt, acc.y) = +0.74, corr(.., acc.x) = +0.03.
  // The gyro z axis points up in both frames, so omega needs no conversion.
  Eigen::VectorXd z(2);
  z << imu.angular_velocity.z,     // omega
       imu.linear_acceleration.y;  // a (forward)

  // The IMU is built to measure exactly these values, so we trust it much more.
  Eigen::MatrixXd Ez = Eigen::MatrixXd::Zero(2, 2);
  Ez(0, 0) = 0.1; // yaw_vel
  Ez(1, 1) = 0.1; // x_acc
  Ez *= dt;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_SIZE);
  H(0, OMEGA) = 1;  // yaw_vel
  H(1, ACC) = 1;    // x_acc

  return kalman_update(s, z, H, Ez);
}

void callback(const nav_msgs::msg::Odometry::SharedPtr odom, const sensor_msgs::msg::Imu::SharedPtr imu)
{
  rclcpp::Time odom_time = odom->header.stamp;
  rclcpp::Time imu_time = imu->header.stamp;

  auto dt = (odom_time - last_time).seconds();
  last_time = odom->header.stamp;

  if (dt < 0)
  {
    // loopback
    first_cb = true;
    state = State();
    return;
  }

  if (!first_cb)
  {
    rclcpp::Time last_odom_time = last_odom.header.stamp;
    rclcpp::Time last_imu_time = last_imu.header.stamp;

    double dt_odom = (odom_time - last_odom_time).seconds();
    double dt_imu = (imu_time - last_imu_time).seconds();

    // Pipeline: predict to the time of this measurement pair, then correct with both sensors.
    state = predict(state, dt);
    state = correct(state, *odom, dt_odom);
    state = correct(state, *imu, dt_imu);

    // Print the estimate about every 2 s (callback runs at the odom rate, 50 Hz).
    static int counter = 0;
    if (++counter % 100 == 0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("kalman_node"),
        "x=%.2f m  y=%.2f m  theta=%.1f deg  v=%.2f m/s  omega=%.2f rad/s  a=%.2f m/s^2",
        state.x(PX), state.x(PY), state.x(THETA) * 180.0 / M_PI, state.x(V), state.x(OMEGA), state.x(ACC));
    }

    auto tf_msg = geometry_msgs::msg::TransformStamped();

    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.header.stamp = odom->header.stamp;

    auto q = tf2::Quaternion();
    q.setRPY(0, 0, state.x(2));
    tf_msg.transform.translation.x = state.x(0);
    tf_msg.transform.translation.y = state.x(1);
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();

    tf_broadcaster->sendTransform(tf_msg);
  }

  first_cb = false;
  last_odom = *odom;
  last_imu = *imu;
}

int main(int argc, char **argv)
{
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>;

  last_time.sec = std::numeric_limits<int32_t>::max();
  last_time.nanosec = std::numeric_limits<int32_t>::max();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kalman_node");

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub(node, "odom");
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub(node, "imu/data_raw");
  auto sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), odom_sub, imu_sub);
  sync_->registerCallback(&callback);

  rclcpp::spin(node);
  return 0;
}
