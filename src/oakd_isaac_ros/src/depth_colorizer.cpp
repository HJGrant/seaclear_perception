// C++/composable port of scripts/depth_colorizer.py -- colorizes the OAK-D's
// native 16-bit depth image (millimeters) into an 8-bit BGR image for RTP
// streaming (x264enc can't encode raw 16-bit depth directly).
//
// Ported to a composable node (rather than the plain Python one) so it can
// be loaded into the same container as the RTP video_output nodes and
// benefit from rclcpp intra-process comms -- see perception.launch.py.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class depth_colorizer : public rclcpp::Node
{
public:
  explicit depth_colorizer(const rclcpp::NodeOptions & options)
  : Node("depth_colorizer", options)
  {
    this->declare_parameter<std::string>("depth_topic", "/oak/stereo/image_raw");
    this->declare_parameter<std::string>("output_topic", "/oak/stereo/depth_color");
    this->declare_parameter<double>("min_depth_m", 0.2);
    this->declare_parameter<double>("max_depth_m", 20.0);

    min_depth_m_ = this->get_parameter("min_depth_m").as_double();
    max_depth_m_ = this->get_parameter("max_depth_m").as_double();
    const std::string depth_topic = this->get_parameter("depth_topic").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();

    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

    // SharedPtr callback (not ConstSharedPtr) to match this codebase's
    // existing intra-process-friendly pattern (see composable_node_video_output.cpp).
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&depth_colorizer::depth_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "DepthColorizer started: %s -> %s (range %.1f-%.1f m)",
      depth_topic.c_str(), output_topic.c_str(), min_depth_m_, max_depth_m_);
  }

private:
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr depth_msg)
  {
    try {
      // depthai_ros_driver publishes depth as 16UC1, millimeters.
      cv::Mat depth_raw = cv_bridge::toCvShare(depth_msg)->image;

      cv::Mat depth_m;
      depth_raw.convertTo(depth_m, CV_32F, 1.0 / 1000.0);

      // Invalid/zero-range pixels come through as 0 -- treat as max range
      // so they colorize as "far" rather than "closest", which would be
      // misleading.
      cv::Mat invalid = (depth_m <= 0.0f);

      cv::Mat clipped;
      cv::max(depth_m, static_cast<float>(min_depth_m_), clipped);
      cv::min(clipped, static_cast<float>(max_depth_m_), clipped);

      cv::Mat norm = (clipped - static_cast<float>(min_depth_m_)) /
        static_cast<float>(max_depth_m_ - min_depth_m_) * 255.0f;

      cv::Mat norm_u8;
      norm.convertTo(norm_u8, CV_8U);
      norm_u8.setTo(255, invalid);

      cv::Mat color_map;
      cv::applyColorMap(norm_u8, color_map, cv::COLORMAP_VIRIDIS);

      auto color_msg = cv_bridge::CvImage(
        depth_msg->header, "bgr8", color_map).toImageMsg();
      color_pub_->publish(*color_msg);

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to colorize depth: %s", e.what());
    }
  }

  double min_depth_m_;
  double max_depth_m_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(depth_colorizer)
