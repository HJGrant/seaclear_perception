#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "foxglove_msgs/msg/compressed_video.hpp"

namespace oakd_isaac_ros {

class H264ToFoxgloveBridge : public rclcpp::Node {
public:
  explicit H264ToFoxgloveBridge(const rclcpp::NodeOptions & options)
  : Node("h264_to_foxglove_bridge", options) {
    
    // Subscriber to H264 data from Isaac ROS Encoder
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "input/h264", 10,
      std::bind(&H264ToFoxgloveBridge::callback, this, std::placeholders::_1));

    // Publisher for Foxglove Video Panel
    pub_ = this->create_publisher<foxglove_msgs::msg::CompressedVideo>(
      "output/foxglove", 10);
  }

private:
  void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    auto video_msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();

    // Copying metadata
    video_msg->timestamp = msg->header.stamp;
    video_msg->frame_id = msg->header.frame_id;
    video_msg->format = "h264";

    // Transferring the payload (the H264 bitstream)
    video_msg->data = msg->data;

    pub_->publish(std::move(video_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr pub_;
};

}  // namespace oakd_isaac_ros

// Register the component with the ROS 2 system
RCLCPP_COMPONENTS_REGISTER_NODE(oakd_isaac_ros::H264ToFoxgloveBridge)