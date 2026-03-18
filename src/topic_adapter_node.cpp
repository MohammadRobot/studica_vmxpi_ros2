// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
// TopicAdapterNode centralizes API compatibility shims used by launch files:
// - /scan_raw -> /scan frame normalization
// - IMU aliasing/fallback
// - Nav2 cmd_vel/odom topic adaptation
// - TF relay for controller-specific topics

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TopicAdapterNode : public rclcpp::Node {
public:
  TopicAdapterNode() : Node("topic_adapter") {
    // Enable only the adapters needed by the current launch profile.
    enable_scan_relay_ = this->declare_parameter<bool>("enable_scan_relay", false);
    enable_imu_relay_ = this->declare_parameter<bool>("enable_imu_relay", false);
    enable_nav2_bridge_ = this->declare_parameter<bool>("enable_nav2_bridge", false);
    enable_tf_relay_ = this->declare_parameter<bool>("enable_tf_relay", false);

    if (enable_scan_relay_) {
      setupScanRelay();
    }
    if (enable_imu_relay_) {
      setupImuRelay();
    }
    if (enable_nav2_bridge_) {
      setupNav2Bridge();
    }
    if (enable_tf_relay_) {
      setupTfRelay();
    }

    if (!enable_scan_relay_ && !enable_imu_relay_ && !enable_nav2_bridge_ &&
        !enable_tf_relay_) {
      RCLCPP_WARN(this->get_logger(), "No adapters enabled. Set one of: enable_scan_relay, "
                                      "enable_imu_relay, enable_nav2_bridge, enable_tf_relay.");
    }
  }

private:
  void setupScanRelay() {
    scan_input_topic_ =
        this->declare_parameter<std::string>("scan_input_topic", "/scan_raw");
    scan_output_topic_ =
        this->declare_parameter<std::string>("scan_output_topic", "/scan");
    scan_output_frame_id_ =
        this->declare_parameter<std::string>("scan_output_frame_id", "laser_scan_frame");

    // Offer reliable QoS on /scan so RViz and other reliable subscribers can connect.
    // Reliable publishers remain compatible with best-effort subscribers.
    auto scan_output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        scan_output_topic_, scan_output_qos);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TopicAdapterNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Scan relay enabled: %s -> %s (frame_id=%s)",
                scan_input_topic_.c_str(), scan_output_topic_.c_str(),
                scan_output_frame_id_.c_str());
  }

  void setupImuRelay() {
    imu_input_topic_ = this->declare_parameter<std::string>(
        "imu_input_topic", "/imu_sensor_broadcaster/imu");
    imu_output_topic_ = this->declare_parameter<std::string>("imu_output_topic", "/imu");
    imu_use_odom_fallback_ =
        this->declare_parameter<bool>("imu_use_odom_fallback", false);
    imu_fallback_odom_topic_ = this->declare_parameter<std::string>(
        "imu_fallback_odom_topic", "/robot_base_controller/odom");
    imu_fallback_frame_id_ =
        this->declare_parameter<std::string>("imu_fallback_frame_id", "imu_link");
    imu_zero_epsilon_ =
        this->declare_parameter<double>("imu_zero_epsilon", 1e-9);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        imu_output_topic_, rclcpp::SensorDataQoS());
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TopicAdapterNode::imuCallback, this, std::placeholders::_1));

    if (imu_use_odom_fallback_) {
      imu_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          imu_fallback_odom_topic_, rclcpp::SensorDataQoS(),
          std::bind(&TopicAdapterNode::imuOdomFallbackCallback, this,
                    std::placeholders::_1));
    }

    RCLCPP_INFO(this->get_logger(),
                "IMU relay enabled: %s -> %s (odom fallback: %s from %s)",
                imu_input_topic_.c_str(), imu_output_topic_.c_str(),
                imu_use_odom_fallback_ ? "enabled" : "disabled",
                imu_fallback_odom_topic_.c_str());
  }

  void setupNav2Bridge() {
    nav2_input_cmd_vel_topic_ = this->declare_parameter<std::string>(
        "input_cmd_vel_topic", "/cmd_vel");
    nav2_output_cmd_vel_topic_ = this->declare_parameter<std::string>(
        "output_cmd_vel_topic", "/robot_base_controller/cmd_vel");
    nav2_input_odom_topic_ = this->declare_parameter<std::string>(
        "input_odom_topic", "/robot_base_controller/odom");
    nav2_output_odom_topic_ =
        this->declare_parameter<std::string>("output_odom_topic", "/odom");
    nav2_cmd_vel_frame_id_ =
        this->declare_parameter<std::string>("cmd_vel_frame_id", "base_link");

    nav2_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        nav2_input_cmd_vel_topic_, 10,
        std::bind(&TopicAdapterNode::nav2CmdVelCallback, this,
                  std::placeholders::_1));
    nav2_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        nav2_output_cmd_vel_topic_, 10);

    nav2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        nav2_input_odom_topic_, 10,
        std::bind(&TopicAdapterNode::nav2OdomCallback, this,
                  std::placeholders::_1));
    nav2_odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(nav2_output_odom_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "Nav2 bridge enabled: %s (Twist) -> %s (TwistStamped), %s -> %s",
                nav2_input_cmd_vel_topic_.c_str(),
                nav2_output_cmd_vel_topic_.c_str(),
                nav2_input_odom_topic_.c_str(),
                nav2_output_odom_topic_.c_str());
  }

  void setupTfRelay() {
    tf_input_topic_ = this->declare_parameter<std::string>(
        "tf_input_topic", "/mecanum_base_controller/tf_odometry");
    tf_output_topic_ = this->declare_parameter<std::string>("tf_output_topic", "/tf");

    auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(tf_output_topic_, tf_qos);
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_input_topic_, tf_qos,
        std::bind(&TopicAdapterNode::tfCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TF relay enabled: %s -> %s",
                tf_input_topic_.c_str(), tf_output_topic_.c_str());
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    sensor_msgs::msg::LaserScan out = *msg;
    out.header.frame_id = scan_output_frame_id_;
    scan_pub_->publish(out);
  }

  bool imuHasSignal(const sensor_msgs::msg::Imu &msg) const {
    auto nz = [this](double v) { return std::fabs(v) > imu_zero_epsilon_; };
    return nz(msg.orientation.x) || nz(msg.orientation.y) || nz(msg.orientation.z) ||
           nz(msg.orientation.w) || nz(msg.angular_velocity.x) ||
           nz(msg.angular_velocity.y) || nz(msg.angular_velocity.z) ||
           nz(msg.linear_acceleration.x) || nz(msg.linear_acceleration.y) ||
           nz(msg.linear_acceleration.z);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const bool has_signal = imuHasSignal(*msg);
    if (!has_signal && imu_use_odom_fallback_ && !imu_nonzero_seen_) {
      // Suppress all-zero IMU samples until a real IMU signal appears.
      return;
    }
    imu_nonzero_seen_ = imu_nonzero_seen_ || has_signal;
    imu_pub_->publish(*msg);
  }

  void imuOdomFallbackCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!imu_use_odom_fallback_ || imu_nonzero_seen_) {
      return;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header = msg->header;
    imu_msg.header.frame_id = imu_fallback_frame_id_;
    imu_msg.orientation = msg->pose.pose.orientation;
    imu_msg.angular_velocity = msg->twist.twist.angular;
    // Odom doesn't include linear acceleration.
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    imu_msg.orientation_covariance = {1000000.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                                      0.0,       0.0, 0.0005};
    imu_msg.angular_velocity_covariance = {1000000.0, 0.0, 0.0, 0.0, 1000000.0,
                                           0.0,       0.0, 0.0, 0.0005};
    imu_msg.linear_acceleration_covariance = {0.5, 0.0, 0.0, 0.0, 0.5,
                                              0.0, 0.0, 0.0, 0.5};

    imu_pub_->publish(imu_msg);
  }

  void nav2CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = this->now();
    stamped.header.frame_id = nav2_cmd_vel_frame_id_;
    stamped.twist = *msg;
    nav2_cmd_vel_pub_->publish(stamped);
  }

  void nav2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    nav2_odom_pub_->publish(*msg);
  }

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    tf_pub_->publish(*msg);
  }

  bool enable_scan_relay_{false};
  bool enable_imu_relay_{false};
  bool enable_nav2_bridge_{false};
  bool enable_tf_relay_{false};

  std::string scan_input_topic_;
  std::string scan_output_topic_;
  std::string scan_output_frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  std::string imu_input_topic_;
  std::string imu_output_topic_;
  bool imu_use_odom_fallback_{false};
  std::string imu_fallback_odom_topic_;
  std::string imu_fallback_frame_id_;
  double imu_zero_epsilon_{1e-9};
  bool imu_nonzero_seen_{false};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::string nav2_input_cmd_vel_topic_;
  std::string nav2_output_cmd_vel_topic_;
  std::string nav2_input_odom_topic_;
  std::string nav2_output_odom_topic_;
  std::string nav2_cmd_vel_frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr nav2_cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav2_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav2_odom_pub_;

  std::string tf_input_topic_;
  std::string tf_output_topic_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
