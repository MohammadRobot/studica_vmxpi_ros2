#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <math.h>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node") {
    const auto cmd_vel_topic = this->declare_parameter<std::string>(
        "cmd_vel_topic", "/robot_base_controller/cmd_vel");
    const auto scan_topic =
        this->declare_parameter<std::string>("scan_topic", "/scan");

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic, 10);
    timer_ = this->create_wall_timer(200ms,
                                     std::bind(&Patrol::timer_callback, this));
    sub_laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Patrol using scan topic '%s' and cmd_vel "
                                    "topic '%s'",
                scan_topic.c_str(), cmd_vel_topic.c_str());
  }

private:
  float direction_ = 0.0;
  float linearVelocityX = 0.0;
  float angularVelocityZ = 0.0;

  int laserScanPoints = 0;
  int laserAngularRange = 360;

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = linearVelocityX;
    message.angular.z = angularVelocityZ;

    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    twist_stamped_msg.twist = message;
    twist_stamped_msg.header.stamp = this->now();
    twist_stamped_msg.header.frame_id =
        "base_link"; // Or your robot's base frame

    RCLCPP_INFO(this->get_logger(),
                "linearVelocityX[%f]    angularVelocityZ[%f]   direction_[%f]",
                linearVelocityX, angularVelocityZ, direction_);

    publisher_->publish(twist_stamped_msg);
  }

  float normalize_angle(float angle) {
    angle = atan2(sin(angle), cos(angle));
    return angle;
  }

  int getAngelScanPoint(int angle) {
    int point = static_cast<int>(
        round(angle * ((double)laserScanPoints / laserAngularRange)));
    return point;
  }

  float getDirection_(float angle) {
    angle *= (M_PI / 180);
    direction_ = normalize_angle(angle);
    direction_ -= M_PI / 2;
    return direction_;
  }

  void
  scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    if (scan_msg->ranges.empty()) {
      return;
    }

    float frontMinDistance = scan_msg->range_max;
    float frontMinDistanceAngle = 0.0;
    float maxDistance = 0.0;
    float maxDistanceAngle = 0.0;

    laserScanPoints =
        static_cast<int>(round(((scan_msg->angle_max - scan_msg->angle_min) /
                                scan_msg->angle_increment)));

    int startPoint = getAngelScanPoint(90);
    int endPoint = getAngelScanPoint(270);
    int centerStartPoint = getAngelScanPoint(140);
    int centerEndPoint = getAngelScanPoint(220);

    for (int i = startPoint; i < endPoint; i++) {
      if (scan_msg->ranges[i] >= scan_msg->range_min &&
          scan_msg->ranges[i] <= scan_msg->range_max &&
          scan_msg->ranges[i] > maxDistance) {
        maxDistance = scan_msg->ranges[i];
        maxDistanceAngle =
            ((i - startPoint) / ((double)laserScanPoints / laserAngularRange));
      }
    }

    for (int i = centerStartPoint; i < centerEndPoint; i++) {
      if (scan_msg->ranges[i] >= scan_msg->range_min &&
          scan_msg->ranges[i] <= scan_msg->range_max &&
          scan_msg->ranges[i] < frontMinDistance) {
        frontMinDistance = scan_msg->ranges[i];
        frontMinDistanceAngle =
            ((i - startPoint) / ((double)laserScanPoints / laserAngularRange));
      }
    }

    if (frontMinDistance <= 1.0) {
      linearVelocityX = 0.2;
      direction_ = getDirection_(frontMinDistanceAngle);
      angularVelocityZ = direction_ * -1;
    } else if (frontMinDistance <= 2.0) {
      linearVelocityX = 0.5;
      direction_ = getDirection_(maxDistanceAngle);
      angularVelocityZ = direction_ / 2;
    } else {
      linearVelocityX = 0.5;
      angularVelocityZ = 0.0;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserScan_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
