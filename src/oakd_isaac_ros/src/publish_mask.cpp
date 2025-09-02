#include <stdio.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>

#include "vision_msgs/msg/detection2_d_array.hpp"

#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp_components/register_node_macro.hpp>

class publish_mask : public rclcpp::Node
{
public:
  publish_mask(const rclcpp::NodeOptions & options)
  : Node("detection_mask_node", options)
  {
    this->declare_parameter<int>("target_class_id", 0);
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 640);
    this->declare_parameter<double>("confidence_threshold", 0.3);

    class_ids_.resize(4);
    class_ids_[0] = "rov";
    class_ids_[1] = "plant";
    class_ids_[2] = "animal";
    class_ids_[3] = "trash";

    target_class_id_ = this->get_parameter("target_class_id").as_int();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();

    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections_output", 10,
      std::bind(&publish_mask::detectionCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/yolov8_encoder/resize/camera_info", 10,
      std::bind(&publish_mask::cameraInfoCallback, this, std::placeholders::_1));

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera0/detection/mask/camera_info", 10);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mask_overlay", 10);
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera0/detection/mask", 10);
    
    // publish the target point from the object detection
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/colrov/detected_objects/target", 10);

    rgb_sub_.subscribe(this, "/oak/rgb/image_raw_rgb8");
    resized_sub_.subscribe(this, "/yolov8_encoder/resize/image");

    sync_ = std::make_shared<Sync>(SyncPolicy(10), rgb_sub_, resized_sub_);
    sync_->registerCallback(std::bind(&publish_mask::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Subscribe to depth image and camera info (input topics)
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/oak/stereo/image_raw_depth", 10,
      std::bind(&publish_mask::depthCallback, this, std::placeholders::_1));

    depth_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/oak/stereo/camera_info", 10,
      std::bind(&publish_mask::depthCameraInfoCallback, this, std::placeholders::_1));

    // Publishers for republished depth image and camera info with updated timestamps
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/oak/stereo/image_raw", 10);
    depth_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/oak/stereo/camera_info", 10);
  }

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(detections_mutex_);
    latest_detections_ = msg;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    last_camera_info_ = msg;
  }

  // Store latest depth image for republishing later with RGB timestamp
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_image_ = msg;
  }

  // Store latest depth camera info for republishing later with RGB timestamp
  void depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_camera_info_ = msg;
  }

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & resized_msg)
  {
    vision_msgs::msg::Detection2DArray::SharedPtr detections_copy;
    {
      std::lock_guard<std::mutex> lock(detections_mutex_);
      if (!latest_detections_) {
        RCLCPP_WARN(this->get_logger(), "No detections received yet");
        return;
      }
      detections_copy = latest_detections_;
    }

    cv::Mat resized_image;
    try {
      resized_image = cv_bridge::toCvCopy(resized_msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in syncCallback: %s", e.what());
      return;
    }

    cv::Mat mask = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);

    // Select single best detection for the target class above threshold
    const vision_msgs::msg::Detection2D* best_det = nullptr;
    double best_score = -1.0;
    int best_cid = -1;

    for (const auto& detection : detections_copy->detections) {
      if (detection.results.empty()) continue;

      for (const auto& res : detection.results) {
        int cid = -1;
        try {
          cid = std::stoi(res.hypothesis.class_id);
        } catch (...) {
          continue;
        }
        if (cid == target_class_id_ && res.hypothesis.score >= confidence_threshold_) {
          if (res.hypothesis.score > best_score) {
            best_score = res.hypothesis.score;
            best_det = &detection;
            best_cid = cid;
          }
        }
      }
    }

    geometry_msgs::msg::Point target_point_msg;
    if (best_det) {
      const auto& bbox = best_det->bbox;

      // draw mask for the best detection only
      cv::Point2f center_pt(static_cast<float>(bbox.center.position.x), static_cast<float>(bbox.center.position.y));
      cv::RotatedRect rotated_rect(center_pt, cv::Size2f(bbox.size_x, bbox.size_y), 0.0f);

      cv::Point2f vertices[4];
      rotated_rect.points(vertices);
      std::vector<cv::Point> contour;
      contour.reserve(4);
      for (int i = 0; i < 4; ++i) {
        contour.emplace_back(vertices[i]);
      }
      cv::fillConvexPoly(mask, contour, cv::Scalar(255));

      // compute normalized target point using CameraInfo intrinsics
      if (last_camera_info_) {
        const auto& K = last_camera_info_->k; // fx=K[0], fy=K[4], cx=K[2], cy=K[5]
        const double fx = K[0];
        const double fy = K[4];
        const double cx = K[2];
        const double cy = K[5];
        target_point_msg.x = (bbox.center.position.x - cx) / fx;
        target_point_msg.y = (bbox.center.position.y - cy) / fy;
        target_point_msg.z = 0.0;
      } else {
        // fallback to pixel coordinates if intrinsics are unavailable
        target_point_msg.x = bbox.center.position.x;
        target_point_msg.y = bbox.center.position.y;
        target_point_msg.z = 0.0;
      }
    }

    cv::Mat overlay;
    resized_image.copyTo(overlay);
    double alpha = 0.5;

    cv::Mat red_mask = cv::Mat::zeros(mask.size(), CV_8UC3);
    red_mask.setTo(cv::Scalar(0, 0, 255), mask); // red where mask

    cv::addWeighted(red_mask, alpha, overlay, 1.0 - alpha, 0, overlay);

    if (best_det) {
      const auto& bbox = best_det->bbox;
      cv::Point2f center_pt(static_cast<float>(bbox.center.position.x),
                            static_cast<float>(bbox.center.position.y));

      // crosshair at center
      cv::drawMarker(overlay, center_pt, cv::Scalar(0, 255, 0),
                    cv::MARKER_CROSS, 12, 2);

      std::string text = class_ids_[best_cid] + ": " + std::to_string(best_score);

      int fontFace = cv::FONT_HERSHEY_SIMPLEX;
      double fontScale = 1.0;
      cv::Scalar color(0, 255, 0);
      int thickness = 2;

      cv::Point2f label_pt(
          center_pt.x,                  // same x as center
          center_pt.y - 15.0f           // 15 pixels above the crosshair
      );

      cv::putText(overlay, text, label_pt, fontFace, fontScale, color, thickness);

      // outline of the (axis-aligned) bbox we used to fill the mask
      cv::RotatedRect r(center_pt, cv::Size2f(bbox.size_x, bbox.size_y), 0.0f);
      cv::Point2f verts_f[4];
      r.points(verts_f);
      std::vector<cv::Point> poly(4);
      for (int i = 0; i < 4; ++i) poly[i] = verts_f[i];
      cv::polylines(overlay, poly, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    // Publish camera info with RGB timestamp
    if (last_camera_info_) {
      auto cam_info_to_publish = *last_camera_info_;
      cam_info_to_publish.header.stamp = rgb_msg->header.stamp;  // sync timestamp
      camera_info_pub_->publish(cam_info_to_publish);
    }

    // Publish mask and overlay with RGB timestamp
    std_msgs::msg::Header header;
    header.stamp = rgb_msg->header.stamp;
    header.frame_id = last_camera_info_ ? last_camera_info_->header.frame_id : "oak_rgb_camera_optical_frame";

    auto mask_msg = cv_bridge::CvImage(header, "mono8", mask).toImageMsg();
    mask_pub_->publish(*mask_msg);

    auto overlay_msg = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    overlay_pub_->publish(*overlay_msg);

    // Also publish the target point for visual servoing (if we found one)
    if (best_det) {
      target_pub_->publish(target_point_msg);
    }

    // Now republish the latest depth image and camera info with RGB timestamp
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (latest_depth_image_) {
        auto depth_msg = *latest_depth_image_;
        depth_msg.header.stamp = rgb_msg->header.stamp; // overwrite timestamp
        depth_pub_->publish(depth_msg);
      }
      if (latest_depth_camera_info_) {
        auto depth_cam_info = *latest_depth_camera_info_;
        depth_cam_info.header.stamp = rgb_msg->header.stamp; // overwrite timestamp
        depth_camera_info_pub_->publish(depth_cam_info);
      }
    }
  }

  int target_class_id_;
  int image_width_;
  int image_height_;
  double confidence_threshold_{};
  std::vector<std::string> class_ids_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> resized_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image> SyncPolicy;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  std::mutex detections_mutex_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;

  std::mutex depth_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_depth_camera_info_;

  sensor_msgs::msg::CameraInfo::SharedPtr last_camera_info_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(publish_mask)
