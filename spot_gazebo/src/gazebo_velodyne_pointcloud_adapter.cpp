#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class GazeboVelodynePointCloudAdapter : public rclcpp::Node
{
public:
  GazeboVelodynePointCloudAdapter()
  : Node("gazebo_velodyne_pointcloud_adapter")
  {
    // Declare parameters
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("input_topic", "/spot/lidar/points");
    this->declare_parameter("output_topic", "/spot/lidar/points_transformed");
    this->declare_parameter("scan_rate", 10.0);       // Hz — lidar rotation rate
    this->declare_parameter("num_scan_lines", 16);     // vertical scan lines
    this->declare_parameter("vertical_fov_min", -15.0); // degrees
    this->declare_parameter("vertical_fov_max", 15.0);  // degrees

    target_frame_ = this->get_parameter("target_frame").as_string();
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    scan_rate_ = this->get_parameter("scan_rate").as_double();
    num_scan_lines_ = this->get_parameter("num_scan_lines").as_int();
    vert_fov_min_rad_ = this->get_parameter("vertical_fov_min").as_double() * M_PI / 180.0;
    vert_fov_max_rad_ = this->get_parameter("vertical_fov_max").as_double() * M_PI / 180.0;

    scan_period_ = 1.0 / scan_rate_;

    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber QoS (Best Effort for Gazebo/Sensor input)
    rclcpp::QoS input_qos(10);
    input_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    input_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // Publisher QoS (Reliable for RViz/Downstream compatibility)
    rclcpp::QoS output_qos(10);
    output_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    output_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, output_qos);

    // Subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      input_qos,
      std::bind(
        &GazeboVelodynePointCloudAdapter::pointcloud_callback,
        this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Gazebo Velodyne adapter: %s -> %s (target: %s, scan_rate: %.1f Hz, lines: %d)",
      input_topic.c_str(), output_topic.c_str(), target_frame_.c_str(),
      scan_rate_, num_scan_lines_);
  }

private:
  /// Check whether a PointCloud2 message contains a field with the given name.
  static bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    for (const auto & f : cloud.fields) {
      if (f.name == name) return true;
    }
    return false;
  }

  /// Compute ring index from the elevation angle of a point (in lidar frame,
  /// before any TF transform).  Maps the vertical FOV linearly onto
  /// [0, num_scan_lines_-1].
  uint16_t computeRing(float x, float y, float z) const
  {
    float range_xy = std::sqrt(x * x + y * y);
    float elevation = std::atan2(z, range_xy);                       // radians

    // Clamp to configured vertical FOV
    elevation = std::max(static_cast<float>(vert_fov_min_rad_),
                         std::min(static_cast<float>(vert_fov_max_rad_), elevation));

    float ratio = (elevation - static_cast<float>(vert_fov_min_rad_))
                / static_cast<float>(vert_fov_max_rad_ - vert_fov_min_rad_);
    int ring = static_cast<int>(std::round(ratio * (num_scan_lines_ - 1)));
    return static_cast<uint16_t>(std::max(0, std::min(num_scan_lines_ - 1, ring)));
  }

  /// Compute the per-point time offset (seconds) from the azimuth angle.
  /// Azimuth 0 → time 0;  one full revolution → scan_period_.
  static float computeTimeOffset(float x, float y, double scan_period)
  {
    // atan2 returns [-pi, pi].  Shift to [0, 2*pi).
    float azimuth = std::atan2(y, x);
    if (azimuth < 0.0f) azimuth += 2.0f * static_cast<float>(M_PI);
    return static_cast<float>((azimuth / (2.0 * M_PI)) * scan_period);
  }

  /// Build a new PointCloud2 that is guaranteed to contain the fields
  /// x, y, z, intensity, ring, time — adding any that are missing.
  sensor_msgs::msg::PointCloud2 addVelodyneFields(
    const sensor_msgs::msg::PointCloud2 & cloud_in) const
  {
    const bool need_time = !hasField(cloud_in, "time");
    const bool need_ring = !hasField(cloud_in, "ring");
    const bool need_intensity = !hasField(cloud_in, "intensity");

    // If nothing to add, return a copy
    if (!need_time && !need_ring && !need_intensity) {
      return cloud_in;
    }

    const uint32_t num_points = cloud_in.width * cloud_in.height;

    // --- Create iterators for reading the input cloud -----------------------
    sensor_msgs::PointCloud2ConstIterator<float> in_x(cloud_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(cloud_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(cloud_in, "z");

    // --- Build the output cloud with the full Velodyne field set ------------
    sensor_msgs::msg::PointCloud2 cloud_out;
    cloud_out.header = cloud_in.header;
    cloud_out.height = 1;
    cloud_out.width  = num_points;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.is_bigendian = cloud_in.is_bigendian;

    // Define the output fields
    sensor_msgs::PointCloud2Modifier modifier(cloud_out);
    modifier.setPointCloud2Fields(6,
      "x",         1, sensor_msgs::msg::PointField::FLOAT32,
      "y",         1, sensor_msgs::msg::PointField::FLOAT32,
      "z",         1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring",      1, sensor_msgs::msg::PointField::UINT16,
      "time",      1, sensor_msgs::msg::PointField::FLOAT32);

    // Create write iterators
    sensor_msgs::PointCloud2Iterator<float>    out_x(cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float>    out_y(cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float>    out_z(cloud_out, "z");
    sensor_msgs::PointCloud2Iterator<float>    out_int(cloud_out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(cloud_out, "ring");
    sensor_msgs::PointCloud2Iterator<float>    out_time(cloud_out, "time");

    // Optional input intensity reader
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> in_int_ptr;
    if (!need_intensity) {
      in_int_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
        cloud_in, "intensity");
    }

    // Optional input ring reader
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> in_ring_ptr;
    if (!need_ring) {
      in_ring_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(
        cloud_in, "ring");
    }

    for (uint32_t i = 0; i < num_points;
         ++i, ++in_x, ++in_y, ++in_z,
         ++out_x, ++out_y, ++out_z, ++out_int, ++out_ring, ++out_time)
    {
      const float px = *in_x;
      const float py = *in_y;
      const float pz = *in_z;

      *out_x = px;
      *out_y = py;
      *out_z = pz;

      // Intensity
      if (in_int_ptr) {
        *out_int = **in_int_ptr;
        ++(*in_int_ptr);
      } else {
        *out_int = 0.0f;
      }

      // Ring
      if (in_ring_ptr) {
        *out_ring = **in_ring_ptr;
        ++(*in_ring_ptr);
      } else {
        *out_ring = computeRing(px, py, pz);
      }

      // Per-point time offset (seconds)
      *out_time = computeTimeOffset(px, py, scan_period_);
    }

    return cloud_out;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::string source_frame = msg->header.frame_id;
    if (source_frame.empty()) {
       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         "Received PointCloud2 with empty frame_id");
       return;
    }

    try {
      // 1. Add velodyne fields (time, ring) BEFORE the TF transform so that
      //    azimuth / elevation are computed in the lidar frame.
      sensor_msgs::msg::PointCloud2 cloud_augmented = addVelodyneFields(*msg);

      // 2. TF-transform into the target frame (if needed)
      if (source_frame != target_frame_) {
        geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(
            target_frame_, source_frame, msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1));

        sensor_msgs::msg::PointCloud2 cloud_out;
        tf2::doTransform(cloud_augmented, cloud_out, transform_stamped);
        publisher_->publish(cloud_out);
      } else {
        publisher_->publish(cloud_augmented);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Transform error: %s", ex.what());
    }
  }

  std::string target_frame_;
  double scan_rate_;
  double scan_period_;
  int num_scan_lines_;
  double vert_fov_min_rad_;
  double vert_fov_max_rad_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboVelodynePointCloudAdapter>());
  rclcpp::shutdown();
  return 0;
}
