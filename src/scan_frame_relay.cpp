#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFrameRelay : public rclcpp::Node {
public:
  ScanFrameRelay() : Node("scan_frame_relay") {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/scan_raw");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "laser_scan_frame");

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanFrameRelay::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Scan relay: %s -> %s (frame_id=%s)", input_topic_.c_str(),
                output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    sensor_msgs::msg::LaserScan out = *msg;
    out.header.frame_id = output_frame_id_;
    scan_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
