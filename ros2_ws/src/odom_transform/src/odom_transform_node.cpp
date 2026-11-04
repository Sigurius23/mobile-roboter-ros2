#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

using std::placeholders::_1;

// Integrates the 2D pose of a differential-drive robot from /odom
// (twist.linear.x and twist.angular.z) and publishes it to TF as
// <frame_name> -> <odom child_frame_id>.
class OdomTransformNode : public rclcpp::Node
{
public:
  OdomTransformNode()
  : Node("odom_transform_node")
  {
    // Step 7: name of the new odom frame, e.g. --ros-args -p frame_name:=my_odom
    frame_name_ = declare_parameter<std::string>("frame_name", "my_odom");

    // Step 8 (optional): while linear.x == 0 (turning on the spot), take the
    // yaw rate from the IMU gyro instead of the wheel odometry.
    // In dlg_feldtage_24:cut the wheels still report ~0.02 m/s during the turn
    // on the spot, so a strict == 0 would almost never switch to the IMU.
    use_imu_ = declare_parameter<bool>("use_imu", false);
    standstill_speed_ = declare_parameter<double>("standstill_speed", 0.05);

    // TransformBroadcaster, not StaticTransformBroadcaster: the pose changes
    // with every odometry message. Static transforms go to /tf_static and are
    // meant for frames that never move (e.g. sensor mounts).
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Best effort is compatible with both reliable and best-effort publishers.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&OdomTransformNode::on_odom, this, _1));

    if (use_imu_) {
      // Only needed to rotate the gyro reading from the IMU frame into the
      // robot frame (static transform from the bag's /tf_static).
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", rclcpp::SensorDataQoS(),
        std::bind(&OdomTransformNode::on_imu, this, _1));
    }

    RCLCPP_INFO(
      get_logger(), "Publishing TF %s -> <odom child_frame_id>, IMU yaw rate %s",
      frame_name_.c_str(), use_imu_ ? "on" : "off");
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Some drivers leave child_frame_id empty; TF rejects empty frame names.
    base_frame_ = msg->child_frame_id.empty() ? "base_link" : msg->child_frame_id;

    const rclcpp::Time stamp(msg->header.stamp);
    if (last_stamp_ && stamp < *last_stamp_) {
      // Time jumped back: the bagfile started its next loop.
      RCLCPP_INFO(get_logger(), "Time jumped back, resetting pose to the origin");
      x_ = 0.0;
      y_ = 0.0;
      yaw_ = 0.0;
    } else if (last_stamp_) {
      const double dt = (stamp - *last_stamp_).seconds();
      const double v = msg->twist.twist.linear.x;
      double omega = msg->twist.twist.angular.z;

      bool omega_from_imu = false;
      if (use_imu_ && std::abs(v) < standstill_speed_ && imu_samples_ > 0) {
        omega = imu_yaw_rate_sum_ / imu_samples_;
        omega_from_imu = true;
      }

      if (dt > 0.0) {
        // Midpoint rule: use the heading halfway through the interval.
        const double yaw_mid = yaw_ + 0.5 * omega * dt;
        x_ += v * std::cos(yaw_mid) * dt;
        y_ += v * std::sin(yaw_mid) * dt;
        yaw_ = std::atan2(std::sin(yaw_ + omega * dt), std::cos(yaw_ + omega * dt));
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "x=%.2f m  y=%.2f m  yaw=%.1f deg  (v=%.2f m/s, omega=%.2f rad/s from %s)",
        x_, y_, yaw_ * 180.0 / M_PI, v, omega, omega_from_imu ? "IMU" : "odom");
    }
    last_stamp_ = stamp;
    imu_yaw_rate_sum_ = 0.0;
    imu_samples_ = 0;

    publish_transform(msg->header.stamp);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (base_frame_.empty()) {
      return;  // no odometry yet, robot frame unknown
    }

    if (!imu_to_base_) {
      if (msg->header.frame_id.empty() || msg->header.frame_id == base_frame_) {
        imu_to_base_ = tf2::Quaternion::getIdentity();
      } else {
        try {
          const auto t = tf_buffer_->lookupTransform(
            base_frame_, msg->header.frame_id, tf2::TimePointZero);
          imu_to_base_ = tf2::Quaternion(
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "No transform %s -> %s yet: %s",
            msg->header.frame_id.c_str(), base_frame_.c_str(), e.what());
          return;
        }
      }
    }

    const tf2::Vector3 gyro_imu(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    const tf2::Vector3 gyro_base = tf2::quatRotate(*imu_to_base_, gyro_imu);
    imu_yaw_rate_sum_ += gyro_base.z();
    ++imu_samples_;
  }

  void publish_transform(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_name_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw_);
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform);
  }

  // Parameters
  std::string frame_name_;
  bool use_imu_;
  double standstill_speed_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Integrated pose in frame_name_
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  std::optional<rclcpp::Time> last_stamp_;
  std::string base_frame_;

  // IMU yaw rate samples since the last odometry message
  std::optional<tf2::Quaternion> imu_to_base_;
  double imu_yaw_rate_sum_ = 0.0;
  int imu_samples_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTransformNode>());
  rclcpp::shutdown();
  return 0;
}
