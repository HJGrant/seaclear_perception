#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace oakd_isaac_ros {

class DNNDepthVisualizer : public rclcpp::Node {
public:
    explicit DNNDepthVisualizer(const rclcpp::NodeOptions & options)
    : Node("dnn_depth_visualizer", options) {
        // Declare parameters
        this->declare_parameter<double>("min_disp", 0.0);
        this->declare_parameter<double>("max_disp", 256.0);

        // Publisher for the colorized image
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_color", 10);

        // Subscriber for the Disparity image
        disp_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
            "disparity", 10,
            std::bind(&DNNDepthVisualizer::disparityCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ DNN Depth Visualizer Node Started.");
    }

private:
    void disparityCallback(const stereo_msgs::msg::DisparityImage::SharedPtr msg) {
        try {
            // 1. Convert ROS Disparity Image to OpenCV Mat
            // Disparity is usually 32FC1 (32-bit float, 1 channel)
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, msg->image.encoding);
            cv::Mat disp_mat = cv_ptr->image;

            // 2. Get normalization params
            double min_val = this->get_parameter("min_disp").as_double();
            double max_val = this->get_parameter("max_disp").as_double();

            // 3. Normalize and Convert to 8-bit
            cv::Mat norm_mat;
            disp_mat.convertTo(norm_mat, CV_8UC1, 255.0 / (max_val - min_val), -min_val);

            // 4. Apply Colormap (Viridis)
            cv::Mat color_mat;
            cv::applyColorMap(norm_mat, color_mat, cv::COLORMAP_VIRIDIS);

            // 5. Convert back to ROS message and publish
            auto color_msg = cv_bridge::CvImage(msg->header, "bgr8", color_mat).toImageMsg();
            color_pub_->publish(*color_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disp_sub_;
};

} // namespace oakd_isaac_ros

// Register as a component
RCLCPP_COMPONENTS_REGISTER_NODE(oakd_isaac_ros::DNNDepthVisualizer)