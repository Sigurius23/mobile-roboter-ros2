#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include <string>
#include <cmath>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include <cstdlib>

using namespace std::chrono_literals;

class LioNode : public rclcpp::Node
{
public:
  struct State
  {
    // x, y, theta, v_lin, v_ang, a_lin
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd Ex = Eigen::MatrixXd::Identity(6, 6);
  };

  struct ICPResult
  {
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    int inliers = 0;
    double residual_rms = 0.0;
    double avg_radius = 0.0;
  };

  LioNode() : Node("icp_odom")
  {
    this->declare_parameter("icp_only", false);
    icp_only_ = this->get_parameter("icp_only").as_bool();
    this->declare_parameter("use_icp", true);
    use_icp_ = this->get_parameter("use_icp").as_bool();
    this->declare_parameter("icp_gain", 1.0);
    icp_gain_ = this->get_parameter("icp_gain").as_double();

    RCLCPP_INFO(this->get_logger(), "Parameters: use_icp=%s icp_only=%s",
                use_icp_ ? "true" : "false", icp_only_ ? "true" : "false");
    this->declare_parameter("base_frame", "base_footprint");
    base_frame_ = this->get_parameter("base_frame").as_string();

    // FIX 3: /ouster/imu has a gyro offset of about 0.006 rad/s at standstill
    // (about 14 deg over the 40 s bag). The Phidget IMU on /imu/data_raw does not.
    this->declare_parameter("imu_topic", "/imu/data_raw");
    const std::string imu_topic = this->get_parameter("imu_topic").as_string();
    // Acceleration from the IMU is off by default: the Phidget's TF says the
    // sensor is turned +90 deg, but its data says the opposite sign (forward
    // acceleration is +acc.y, TF would make it -acc.y), and /ouster/imu reads
    // 0.5 m/s^2 forward while standing. A wrong acceleration only hurts v.
    this->declare_parameter("imu_use_acc", false);
    imu_use_acc_ = this->get_parameter("imu_use_acc").as_bool();
    RCLCPP_INFO(this->get_logger(), "IMU: %s, acceleration %s",
                imu_topic.c_str(), imu_use_acc_ ? "used" : "not used");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&LioNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&LioNode::imuCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/ouster/scan", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&LioNode::scanCallback, this, std::placeholders::_1));

    state_ = initialState();
  }

private:
  bool icp_only_;
  bool use_icp_;
  double icp_gain_ = 1.0;
  bool imu_use_acc_ = false;
  std::string base_frame_;

  // Odom (50 Hz) and IMU (125 Hz) messages interleave, so a stamp a few ms older
  // than the last one is normal. Only a jump back of more than 1 s means the bag
  // looped (ros2 bag play --loop) or Minot ran "init" again.
  static constexpr double kTimeJumpBack = 1.0;

  static State initialState()
  {
    State s;
    s.Ex.diagonal() << 0.1, 0.1, 0.1, 1.0, 1.0, 10.0;
    return s;
  }

  // FIX 1: start over at the origin when time jumps back. Before, every callback
  // returned on dt <= 0 without updating its stored stamp, so after the first loop
  // all following messages were "too old" and the node never published again.
  void resetFilter(const rclcpp::Time &t)
  {
    RCLCPP_INFO(this->get_logger(), "Time jumped back, resetting the filter to the origin");
    state_ = initialState();
    filter_time_ = t;
    have_last_scan_ = false;
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_base_{new pcl::PointCloud<pcl::PointXYZ>()};
  bool have_last_scan_ = false;
  rclcpp::Time last_scan_stamp_;

  rclcpp::Time filter_time_;
  bool first_filter_time_ = true;
  State state_;

  State predict(State s, double dt)
  {
    if (dt <= 1e-5)
      return s;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
    double th = s.x(2);
    double c = cos(th);
    double ss = sin(th);

    A(0, 3) = c * dt;
    A(0, 5) = c * 0.5 * dt * dt;
    A(1, 3) = ss * dt;
    A(1, 5) = ss * 0.5 * dt * dt;
    A(2, 4) = dt;
    A(3, 5) = dt;

    Eigen::MatrixXd Eu = Eigen::MatrixXd::Identity(6, 6);
    Eu.diagonal() << 0.001, 0.001, 0.001, 0.05, 0.05, 0.5;
    Eu *= dt;

    State s_;
    s_.x = A * s.x;
    s_.x(2) = normalize_angle(s_.x(2));
    s_.Ex = A * s.Ex * A.transpose() + Eu;
    return s_;
  }

  double normalize_angle(double a)
  {
    while (a > M_PI)
      a -= 2.0 * M_PI;
    while (a < -M_PI)
      a += 2.0 * M_PI;
    return a;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time t = msg->header.stamp;

    if (first_filter_time_)
    {
      filter_time_ = t;
      first_filter_time_ = false;
      return;
    }

    double dt = (t - filter_time_).seconds();
    if (dt < -kTimeJumpBack)
    {
      resetFilter(t);
      return;
    }
    if (dt <= 0)
      return;

    std::string twist_frame = msg->child_frame_id;
    Eigen::Vector3d v_lin(msg->twist.twist.linear.x, msg->twist.twist.linear.y, 0.0);
    Eigen::Vector3d v_ang(0.0, 0.0, msg->twist.twist.angular.z);

    if (twist_frame != base_frame_)
    {
      Eigen::Isometry3d T_base_child;
      if (getTransform(twist_frame, t, T_base_child))
      {
        v_lin = T_base_child.linear() * v_lin;
        v_ang = T_base_child.linear() * v_ang;
      }
    }

    if (icp_only_)
      return;

    state_ = predict(state_, dt);

    // EKF correct
    Eigen::VectorXd z(2);
    z(0) = v_lin.x(); // transformed linear x
    z(1) = v_ang.z(); // transformed angular z

    Eigen::MatrixXd Ez = Eigen::MatrixXd::Zero(2, 2);
    Ez(0, 0) = 0.1;
    Ez(1, 1) = 0.1;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
    H(0, 3) = 1;
    H(1, 4) = 1;

    Eigen::MatrixXd S = H * state_.Ex * H.transpose() + Ez;
    Eigen::MatrixXd K = state_.Ex * H.transpose() * S.inverse();

    state_.x = state_.x + K * (z - H * state_.x);
    state_.x(2) = normalize_angle(state_.x(2));
    state_.Ex = (Eigen::MatrixXd::Identity(6, 6) - K * H) * state_.Ex;

    filter_time_ = t;
    publishTF(t);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time t = msg->header.stamp;

    if (first_filter_time_)
    {
      filter_time_ = t;
      first_filter_time_ = false;
      return;
    }

    double dt = (t - filter_time_).seconds();
    if (dt < -kTimeJumpBack)
    {
      resetFilter(t);
      return;
    }
    if (dt <= 0)
      return;

    if (icp_only_)
      return;

    state_ = predict(state_, dt);

    Eigen::Vector3d acc_imu(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d ang_imu(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    Eigen::Isometry3d T_base_imu;
    if (getTransform(msg->header.frame_id, t, T_base_imu))
    {
      acc_imu = T_base_imu.linear() * acc_imu;
      ang_imu = T_base_imu.linear() * ang_imu;
    }

    // EKF correct. Measurement: yaw rate, plus forward acceleration only if imu_use_acc.
    const int rows = imu_use_acc_ ? 2 : 1;
    Eigen::VectorXd z(rows);
    Eigen::MatrixXd Ez = Eigen::MatrixXd::Zero(rows, rows);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, 6);

    z(0) = ang_imu.z(); // Yaw rate in base frame
    Ez(0, 0) = 0.05;
    H(0, 4) = 1; // v_ang
    if (imu_use_acc_)
    {
      z(1) = acc_imu.x(); // Linear acc x in base frame
      Ez(1, 1) = 0.5;
      H(1, 5) = 1; // a_lin
    }

    Eigen::MatrixXd S = H * state_.Ex * H.transpose() + Ez;
    Eigen::MatrixXd K = state_.Ex * H.transpose() * S.inverse();

    state_.x = state_.x + K * (z - H * state_.x);
    state_.x(2) = normalize_angle(state_.x(2));
    state_.Ex = (Eigen::MatrixXd::Identity(6, 6) - K * H) * state_.Ex;

    filter_time_ = t;
  }

  bool getTransform(const std::string &source_frame,
                    const rclcpp::Time &time,
                    Eigen::Isometry3d &T_out)
  {
    try
    {
      // Ask TF: "where is source_frame relative to base_frame_ at this time?"
      geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(
          base_frame_, source_frame, time, rclcpp::Duration::from_seconds(0.1));

      // Convert the ROS message into an Eigen transform (rotation + translation)
      T_out = tf2::transformToEigen(tf_msg);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      // Transform not available (yet): tell the caller it failed
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "getTransform %s -> %s failed: %s",
                           source_frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    rclcpp::Time current_scan_stamp = scan->header.stamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_lidar(new pcl::PointCloud<pcl::PointXYZ>());
    double angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      float r = scan->ranges[i];
      if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max)
      {
        cloud_in_lidar->emplace_back(r * std::cos(angle), r * std::sin(angle), 0.0f);
      }
      angle += scan->angle_increment;
    }

    // extrinsic
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Isometry3d T_base_lidar;

    if (getTransform(scan->header.frame_id, current_scan_stamp, T_base_lidar))
    {
      pcl::transformPointCloud(*cloud_in_lidar, *cloud_in_base, T_base_lidar.matrix());
    }
    else
    {
      // skip this scan without transform
      return;
    }

    if (!have_last_scan_)
    {
      *last_cloud_base_ = *cloud_in_base;
      have_last_scan_ = true;
      last_scan_stamp_ = current_scan_stamp;
      return;
    }

    double dt_scan = (current_scan_stamp - last_scan_stamp_).seconds();
    if (dt_scan < -kTimeJumpBack)
    {
      // Bag looped. Needed here too: with icp_only the odom/IMU callbacks never
      // advance filter_time_, so they would not notice the jump.
      resetFilter(current_scan_stamp);
      *last_cloud_base_ = *cloud_in_base;
      have_last_scan_ = true;
      last_scan_stamp_ = current_scan_stamp;
      return;
    }
    if (dt_scan <= 0.001)
      return;

    double vx = state_.x(3);
    double w = state_.x(4);

    // Initial guess for ICP: how far did the robot move since the last scan?
    // distance = speed * time, in robot coordinates (forward = x, left = y)
    Eigen::Vector3d guess_robot_base;
    guess_robot_base(0) = vx * dt_scan; // forward movement [m]
    guess_robot_base(1) = 0.0;          // sideways movement [m], a wheeled robot can't drive sideways
    guess_robot_base(2) = w * dt_scan;  // turn [rad]

    if (use_icp_ && !icp_only_)
    {
      auto res = icp(last_cloud_base_, cloud_in_base, guess_robot_base);

      if (res.inliers >= 10)
      {
        // using velocities from ICP measurement
        double meas_vx = res.delta(0) / dt_scan;
        double meas_w = res.delta(2) / dt_scan;

        double residual_rms = res.residual_rms;
        int N = res.inliers;
        double avg_radius = res.avg_radius;

        // FIX 2: uncertainty of the ICP motion.
        // Before: residual_rms / sqrt(N). That treats every matched point as an
        // independent measurement (median N = 248 in dlg_feldtage_24:cut), so the
        // ICP looked ~16x more precise than it is: sigma_v ~0.2 m/s against 0.1
        // variance for the wheels, so v came mostly from the ICP. But neighbouring
        // lidar points have strongly correlated errors, and on the open field the
        // geometry is weak (the ICP alone measured ~15 % less distance than the
        // wheels). Now the residual itself is the position error per scan.
        // Also: sigma_w used to be an angle error (rad) although omega is a rate
        // (rad/s). Without the division by dt it always sat at the 0.005 floor,
        // so the ICP fully overruled IMU and wheels for the yaw rate.
        (void)N;
        const double sigma_trans = residual_rms;
        const double sigma_angle = residual_rms / std::max(avg_radius, 1.0);

        double sigma_v = std::clamp(sigma_trans / dt_scan, 0.05, 5.0);
        double sigma_w = std::clamp(sigma_angle / dt_scan, 0.01, 2.0);
        RCLCPP_DEBUG(this->get_logger(), "icp: N=%d rms=%.3f r=%.1f v=%.2f (sd %.2f) w=%.3f (sd %.3f)",
                     N, residual_rms, avg_radius, meas_vx, sigma_v, meas_w, sigma_w);

        Eigen::VectorXd z(2);
        z(0) = meas_vx;
        z(1) = meas_w;

        Eigen::MatrixXd Ez = Eigen::MatrixXd::Zero(2, 2);
        Ez(0, 0) = sigma_v * sigma_v;
        Ez(1, 1) = sigma_w * sigma_w;

        if (icp_gain_ > 1e-6)
        {
          Ez /= icp_gain_;
        }

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
        H(0, 3) = 1;
        H(1, 4) = 1;

        Eigen::MatrixXd S = H * state_.Ex * H.transpose() + Ez;
        Eigen::MatrixXd K = state_.Ex * H.transpose() * S.inverse();
        Eigen::VectorXd innovation = z - H * state_.x;
        Eigen::VectorXd delta_x = K * innovation;

        state_.x = state_.x + delta_x;
        state_.x(2) = normalize_angle(state_.x(2));

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        state_.Ex = (I - K * H) * state_.Ex * (I - K * H).transpose() + K * Ez * K.transpose();
      }
    }
    else if (icp_only_)
    {
      auto res = icp(last_cloud_base_, cloud_in_base, Eigen::Vector3d::Zero());

      Eigen::Vector3d delta = res.delta;
      double c = cos(state_.x(2));
      double s = sin(state_.x(2));
      state_.x(0) += c * delta(0) - s * delta(1);
      state_.x(1) += s * delta(0) + c * delta(1);
      state_.x(2) += delta(2);
      state_.x(2) = normalize_angle(state_.x(2));
      publishTF(current_scan_stamp);
    }

    *last_cloud_base_ = *cloud_in_base;
    last_scan_stamp_ = current_scan_stamp;
  }

  ICPResult icp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &src_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &tgt_cloud,
                const Eigen::Vector3d &initial_guess_robot)
  {
    if (src_cloud->empty() || tgt_cloud->empty())
    {
      RCLCPP_WARN(this->get_logger(), "icp: source or target cloud empty (src=%zu tgt=%zu)",
                  src_cloud->points.size(), tgt_cloud->points.size());
      return ICPResult();
    }

    double c_g = std::cos(initial_guess_robot(2));
    double s_g = std::sin(initial_guess_robot(2));
    Eigen::Matrix3d T_guess_robot = Eigen::Matrix3d::Identity();
    T_guess_robot(0, 0) = c_g;
    T_guess_robot(0, 1) = -s_g;
    T_guess_robot(0, 2) = initial_guess_robot(0);
    T_guess_robot(1, 0) = s_g;
    T_guess_robot(1, 1) = c_g;
    T_guess_robot(1, 2) = initial_guess_robot(1);
    Eigen::Matrix3d T_guess_point = T_guess_robot.inverse();

    double theta_gp = std::atan2(T_guess_point(1, 0), T_guess_point(0, 0));
    double tx_gp = T_guess_point(0, 2);
    double ty_gp = T_guess_point(1, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    src_transformed->reserve(src_cloud->points.size());

    double c0 = cos(theta_gp);
    double s0 = sin(theta_gp);
    for (const auto &p : src_cloud->points)
    {
      pcl::PointXYZ q = p;
      q.x = c0 * p.x - s0 * p.y + tx_gp;
      q.y = s0 * p.x + c0 * p.y + ty_gp;
      src_transformed->push_back(q);
    }

    Eigen::Matrix3d T_total = T_guess_point;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(tgt_cloud);

    for (int iter = 0; iter < 20; ++iter)
    {
      // 1) Pairs: for every source point, find the closest target point (kd-tree)
      std::vector<Eigen::Vector2d> src_pts;
      std::vector<Eigen::Vector2d> tgt_pts;
      std::vector<int> nn_idx(1);
      std::vector<float> nn_dist_sq(1);
      for (const auto &p : src_transformed->points)
      {
        // only keep pairs closer than 1 m (squared distance < 1.0), same limit as the statistics below
        if (kdtree.nearestKSearch(p, 1, nn_idx, nn_dist_sq) > 0 && nn_dist_sq[0] < 1.0)
        {
          const auto &q = (*tgt_cloud)[nn_idx[0]];
          src_pts.emplace_back(p.x, p.y);
          tgt_pts.emplace_back(q.x, q.y);
        }
      }

      // need at least 3 pairs to compute a rotation + translation
      if (src_pts.size() < 3)
        break;

      // 2) Centroids (average point) of both pair sets
      Eigen::Vector2d src_mean = Eigen::Vector2d::Zero();
      Eigen::Vector2d tgt_mean = Eigen::Vector2d::Zero();
      for (size_t i = 0; i < src_pts.size(); ++i)
      {
        src_mean += src_pts[i];
        tgt_mean += tgt_pts[i];
      }
      src_mean /= static_cast<double>(src_pts.size());
      tgt_mean /= static_cast<double>(tgt_pts.size());

      // 3) Cross-covariance H of the centered pairs
      Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
      for (size_t i = 0; i < src_pts.size(); ++i)
      {
        H += (src_pts[i] - src_mean) * (tgt_pts[i] - tgt_mean).transpose();
      }

      // 4) SVD gives the best rotation R, then the translation t
      Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix2d V = svd.matrixV();
      Eigen::Matrix2d R = V * svd.matrixU().transpose();
      if (R.determinant() < 0)
      {
        // reflection instead of rotation: flip the last column and recompute
        V.col(1) *= -1.0;
        R = V * svd.matrixU().transpose();
      }
      Eigen::Vector2d t = tgt_mean - R * src_mean;

      // 5) Move the source points with this step's R and t
      for (auto &p : src_transformed->points)
      {
        double x_new = R(0, 0) * p.x + R(0, 1) * p.y + t(0);
        double y_new = R(1, 0) * p.x + R(1, 1) * p.y + t(1);
        p.x = static_cast<float>(x_new);
        p.y = static_cast<float>(y_new);
      }

      // 6) Add this step to the total transformation
      Eigen::Matrix3d T_step = Eigen::Matrix3d::Identity();
      T_step.block<2, 2>(0, 0) = R;
      T_step.block<2, 1>(0, 2) = t;
      T_total = T_step * T_total;

      // 7) Stop when the step is tiny (converged)
      double step_angle = std::atan2(R(1, 0), R(0, 0));
      if (t.norm() < 1e-4 && std::abs(step_angle) < 1e-4)
        break;
    }

    // compute statistics for covariance estimation
    int N = 0;
    double sumSq = 0.0;
    double sumRadius = 0.0;
    std::vector<int> idx(1);
    std::vector<float> distSq(1);
    for (const auto &p : *src_transformed)
    {
      if (kdtree.nearestKSearch(p, 1, idx, distSq) > 0)
      {
        if (distSq[0] < 1.0)
        {
          const auto &q = (*tgt_cloud)[idx[0]];
          double dx = q.x - p.x;
          double dy = q.y - p.y;
          sumSq += dx * dx + dy * dy;
          sumRadius += std::sqrt(q.x * q.x + q.y * q.y);
          ++N;
        }
      }
    }

    double residual_rms = 0.0;
    double avg_radius = 0.0;
    if (N > 0)
    {
      residual_rms = std::sqrt(sumSq / static_cast<double>(N));
      avg_radius = sumRadius / static_cast<double>(N);
    }

    Eigen::Matrix3d T_robot = T_total.inverse();
    ICPResult res;
    res.delta = Eigen::Vector3d(T_robot(0, 2), T_robot(1, 2), atan2(T_robot(1, 0), T_robot(0, 0)));
    res.inliers = N;
    res.residual_rms = residual_rms;
    res.avg_radius = avg_radius;
    return res;
  }

  void publishTF(rclcpp::Time t)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = t;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = base_frame_;

    tf2::Quaternion q;
    q.setRPY(0, 0, state_.x(2));
    tf_msg.transform.translation.x = state_.x(0);
    tf_msg.transform.translation.y = state_.x(1);
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(tf_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioNode>());
  rclcpp::shutdown();
  return 0;
}
