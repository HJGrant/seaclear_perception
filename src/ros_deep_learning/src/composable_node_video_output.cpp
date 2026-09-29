#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "image_converter.h"
#include <jetson-utils/videoOutput.h>
#include <jetson-utils/cudaResize.h>
#include <jetson-utils/cudaMappedMemory.h>

namespace jetson_ros
{
class VideoOutputNode : public rclcpp::Node
{
public:
    explicit VideoOutputNode(const rclcpp::NodeOptions& options)
    : Node("video_output", options), m_stream(nullptr), m_image_cvt(nullptr),
      m_resized_gpu(nullptr), m_resized_cpu(nullptr), m_out_width(0), m_out_height(0)
    {
        // Declare Parameters
        std::string resource_str = "";
        std::string codec_str = "";
        int video_bitrate = 2000000;
        bool headless = true;
        // Optional GPU downscale before encoding (0 = stream at input size).
        // Lets the stream fit a low-bandwidth tether without changing the
        // camera resolution that the rest of the pipeline consumes.
        int output_width = 0;
        int output_height = 0;

        this->declare_parameter("resource", resource_str);
        this->declare_parameter("codec", codec_str);
        this->declare_parameter("bitrate", video_bitrate);
        this->declare_parameter("headless", headless);
        this->declare_parameter("output_width", output_width);
        this->declare_parameter("output_height", output_height);

        this->get_parameter("resource", resource_str);
        this->get_parameter("codec", codec_str);
        this->get_parameter("bitrate", video_bitrate);
        this->get_parameter("headless", headless);
        this->get_parameter("output_width", output_width);
        this->get_parameter("output_height", output_height);

        if (output_width > 0 && output_height > 0) {
            m_out_width = output_width;
            m_out_height = output_height;
            if (!cudaAllocMapped((void**)&m_resized_cpu, (void**)&m_resized_gpu,
                                 m_out_width * m_out_height * sizeof(imageConverter::PixelType))) {
                RCLCPP_ERROR(this->get_logger(), "failed to allocate resize buffer");
                return;
            }
        }

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
        // See node_video_output.cpp for why the headless toggle needs the
        // argv-based overload rather than the resource+options one.
        if (headless) {
            const char* headless_argv[] = { "video_output", "--headless" };
            m_stream = videoOutput::Create(resource_str.c_str(), 2, (char**)headless_argv, -1, video_options);
        } else {
            m_stream = videoOutput::Create(resource_str.c_str(), video_options);
        }
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
        if (m_resized_cpu) CUDA(cudaFreeHost(m_resized_cpu));
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr input)
    {
        // convert sensor_msgs to jetson-utils format (GPU memory)
        if (!m_image_cvt || !m_image_cvt->Convert(input)) {
            RCLCPP_INFO(this->get_logger(), "failed to convert image");
            return;
        }

        imageConverter::PixelType* frame = m_image_cvt->ImageGPU();
        uint32_t width = m_image_cvt->GetWidth();
        uint32_t height = m_image_cvt->GetHeight();

        if (m_resized_gpu && (width != m_out_width || height != m_out_height)) {
            if (CUDA_FAILED(cudaResize(frame, width, height,
                                       m_resized_gpu, m_out_width, m_out_height, FILTER_LINEAR))) {
                RCLCPP_INFO(this->get_logger(), "failed to resize image");
                return;
            }
            frame = m_resized_gpu;
            width = m_out_width;
            height = m_out_height;
        }

        // Render to WebRTC/Display
        m_stream->Render(frame, width, height);

        // Update overlay
        char str[256];
        sprintf(str, "%ux%u | %.1f FPS", width, height, m_stream->GetFrameRate());
        m_stream->SetStatus(str);
    }

    videoOutput* m_stream;
    imageConverter* m_image_cvt;
    imageConverter::PixelType* m_resized_gpu;
    imageConverter::PixelType* m_resized_cpu;
    uint32_t m_out_width;
    uint32_t m_out_height;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
};
} // namespace jetson_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(jetson_ros::VideoOutputNode)