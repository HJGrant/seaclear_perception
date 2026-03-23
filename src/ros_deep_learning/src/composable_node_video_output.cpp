#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "image_converter.h"
#include <jetson-utils/videoOutput.h>

namespace jetson_ros
{
class VideoOutputNode : public rclcpp::Node
{
public:
    explicit VideoOutputNode(const rclcpp::NodeOptions& options)
    : Node("video_output", options), m_stream(nullptr), m_image_cvt(nullptr)
    {
        // Declare Parameters
        std::string resource_str = "";
        std::string codec_str = "";
        int video_bitrate = 2000000;

        this->declare_parameter("resource", resource_str);
        this->declare_parameter("codec", codec_str);
        this->declare_parameter("bitrate", video_bitrate);

        this->get_parameter("resource", resource_str);
        this->get_parameter("codec", codec_str);
        this->get_parameter("bitrate", video_bitrate);

        if (resource_str.empty()) {
            RCLCPP_ERROR(this->get_logger(), "resource param not set");
            return;
        }

        // Setup Video Options
        videoOptions video_options;
        if (!codec_str.empty())
            video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());
        
        video_options.bitRate = video_bitrate;

        // Create Stream & Converter
        m_stream = videoOutput::Create(resource_str.c_str(), video_options);
        m_image_cvt = new imageConverter();

        if (!m_stream || !m_image_cvt) {
            RCLCPP_ERROR(this->get_logger(), "failed to initialize jetson-utils");
            return;
        }

        if (!m_stream->Open()) {
            RCLCPP_ERROR(this->get_logger(), "failed to open stream");
            return;
        }

        // Create Subscriber with Intra-process compatibility
        // Using ConstPtr (SharedPtr) is key for Zero-Copy
        m_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "image_in", 
            rclcpp::SensorDataQoS().keep_last(1), // Use SensorData QoS for video
            std::bind(&VideoOutputNode::img_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Video Output Component Ready: %s", resource_str.c_str());
    }

    ~VideoOutputNode()
    {
        if (m_stream) delete m_stream;
        if (m_image_cvt) delete m_image_cvt;
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr input)
    {
        // convert sensor_msgs to jetson-utils format (GPU memory)
        if (!m_image_cvt || !m_image_cvt->Convert(input)) {
            RCLCPP_INFO(this->get_logger(), "failed to convert image");
            return;
        }

        // Render to WebRTC/Display
        m_stream->Render(m_image_cvt->ImageGPU(), m_image_cvt->GetWidth(), m_image_cvt->GetHeight());

        // Update overlay
        char str[256];
        sprintf(str, "%ux%u | %.1f FPS", m_image_cvt->GetWidth(), m_image_cvt->GetHeight(), m_stream->GetFrameRate());
        m_stream->SetStatus(str);
    }

    videoOutput* m_stream;
    imageConverter* m_image_cvt;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
};
} // namespace jetson_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(jetson_ros::VideoOutputNode)