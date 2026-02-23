#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class Nav2TopicBridge : public rclcpp::Node {
public:
  Nav2TopicBridge() : Node("nav2_topic_bridge") {
    input_cmd_vel_topic_ = this->declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    output_cmd_vel_topic_ =
        this->declare_parameter<std::string>("output_cmd_vel_topic", "/diffbot_base_controller/cmd_vel");
    input_odom_topic_ =
        this->declare_parameter<std::string>("input_odom_topic", "/diffbot_base_controller/odom");
    output_odom_topic_ = this->declare_parameter<std::string>("output_odom_topic", "/odom");
    cmd_vel_frame_id_ = this->declare_parameter<std::string>("cmd_vel_frame_id", "base_link");

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_cmd_vel_topic_, 10,
        std::bind(&Nav2TopicBridge::cmdVelCallback, this, std::placeholders::_1));
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_topic_, 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, 10,
        std::bind(&Nav2TopicBridge::odomCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Nav2 cmd bridge: %s (Twist) -> %s (TwistStamped)",
                input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Nav2 odom bridge: %s -> %s", input_odom_topic_.c_str(),
                output_odom_topic_.c_str());
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = this->now();
    stamped.header.frame_id = cmd_vel_frame_id_;
    stamped.twist = *msg;
    cmd_vel_pub_->publish(stamped);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_pub_->publish(*msg); }

  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string cmd_vel_frame_id_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2TopicBridge>());
  rclcpp::shutdown();
  return 0;
}
