#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuTopicRelay : public rclcpp::Node {
public:
  ImuTopicRelay() : Node("imu_topic_relay") {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/imu_sensor_broadcaster/imu");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/imu");

    imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuTopicRelay::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "IMU relay: %s -> %s", input_topic_.c_str(),
                output_topic_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) { imu_pub_->publish(*msg); }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTopicRelay>());
  rclcpp::shutdown();
  return 0;
}
