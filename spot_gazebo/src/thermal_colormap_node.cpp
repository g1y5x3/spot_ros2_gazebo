#include <exception>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "spot_gazebo/thermal_colormap.hpp"

class ThermalColormapNode : public rclcpp::Node
{
public:
  ThermalColormapNode()
  : Node("thermal_colormap")
  {
    const auto input_topic = this->declare_parameter<std::string>(
      "input_topic", "/spot/camera/thermal_camera");
    const auto output_topic = this->declare_parameter<std::string>(
      "output_topic", "/spot/camera/thermal/colormap");

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    input_qos.best_effort();
    input_qos.durability_volatile();

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    output_qos.reliable();
    output_qos.durability_volatile();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      output_topic, output_qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic, input_qos,
      std::bind(
        &ThermalColormapNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "Thermal colormap: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    try {
      const auto thermal = cv_bridge::toCvCopy(
        message, sensor_msgs::image_encodings::MONO16);
      const cv::Mat colorized = spot_gazebo::thermalColormap(thermal->image);
      auto output = cv_bridge::CvImage(
        message->header, sensor_msgs::image_encodings::BGR8, colorized)
        .toImageMsg();
      publisher_->publish(*output);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Thermal colormap failed: %s", error.what());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalColormapNode>());
  rclcpp::shutdown();
  return 0;
}
