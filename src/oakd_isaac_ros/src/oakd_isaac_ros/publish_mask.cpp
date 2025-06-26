#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Replace with actual message type
#include "vision_msgs/msg/detection2_d_array.hpp"

class DetectionMaskNode : public rclcpp::Node
{
public:
  DetectionMaskNode()
  : Node("detection_mask_node")
  {
    // Parameters
    this->declare_parameter<std::string>("target_class", "person");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 640);

    target_class_ = this->get_parameter("target_class").as_string();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();

    // Subscriber
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections_output", 10,
      std::bind(&DetectionMaskNode::detectionCallback, this, std::placeholders::_1));

    // Publisher
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/binary_mask", 10);
  }

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    // Create blank mask
    cv::Mat mask = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);

    
    for (const auto& detection : msg->detections) {
        if (detection.results.empty()) continue;

        const auto& class_id = detection.results[0].class_id;
        if (class_id != target_class_) continue;

        const auto& bbox = detection.bbox;
        cv::RotatedRect rotated_rect(
            cv::Point2f(bbox.center.x, bbox.center.y),
            cv::Size2f(bbox.size_x, bbox.size_y)
        );

        cv::Point2f vertices[4];
        rotated_rect.points(vertices);
        std::vector<cv::Point> contour;
        for (int i = 0; i < 4; ++i) {
            contour.push_back(vertices[i]);
        }
        cv::fillConvexPoly(mask, contour, cv::Scalar(255));
    }


    // Convert to ROS Image message
    auto msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
    msg_out->header.stamp = this->now();
    mask_pub_->publish(*msg_out);
  }

  std::string target_class_;
  int image_width_;
  int image_height_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionMaskNode>());
  rclcpp::shutdown();
  return 0;
}
