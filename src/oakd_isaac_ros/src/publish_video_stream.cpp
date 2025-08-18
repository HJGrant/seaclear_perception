#include <stdio.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class oakd_video_stream : public rclcpp::Node
{
public:
    publish_video_stream(const rclcpp::NodeOptions & options)
    : Node("video_stream_node")
    {
        
    detection_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/oak/image_raw", 10,
      std::bind(&oakd_video_stream::writeToStreamCallback, this, std::placeholders::_1));

    }

    write_video_stream = this->create_publisher<foxglove_msgs::msg::CompressedVideo>("/oak/compressed_video", 10);

private:
    void writeToStreamCallback(){
        
    }

};

RCLCPP_COMPONENTS_REGISTER_NODE(oakd_video_stream)