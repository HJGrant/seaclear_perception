// NITROS variant of publish_mask.cpp.
//
// Only the detection MASK publisher is converted to a NITROS-typed,
// GPU-resident publish (feeding mask_resize_node -> nvblox zero-copy,
// when this node is co-located in the same composable-node container).
// Depth/camera_info/overlay/target-point publishing is left identical to
// publish_mask.cpp (plain sensor_msgs types), matching the pattern
// NvbloxNode itself uses for camera_info (see nvblox_node.cpp: CameraInfo
// stays plain even for its NITROS-typed depth/color/mask image inputs).
//
// NitrosImageBuilder call verified against the real installed header
// (/opt/ros/humble/include/isaac_ros_nitros_image_type/nitros_image_builder.hpp).
// nitros_image_mono8_t::supported_type_name is still unverified -- it's
// confirmed by direct analogy to nitros_image_rgb8_t::supported_type_name
// (a real, used symbol in isaac_ros_tensor_proc) but hasn't been checked
// against nitros_image.hpp directly; if the build fails on that symbol,
// check the format-tag names declared there.

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
#include "vision_msgs/msg/detection2_d.hpp"

#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <cuda_runtime.h>
#include <isaac_ros_managed_nitros/managed_nitros_publisher.hpp>
#include <isaac_ros_nitros_image_type/nitros_image.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_builder.hpp>

class publish_mask_nitros : public rclcpp::Node
{
public:
  publish_mask_nitros(const rclcpp::NodeOptions & options)
  : Node("detection_mask_nitros_node", options)
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

    cudaError_t stream_err = cudaStreamCreate(&cuda_stream_);
    if (stream_err != cudaSuccess) {
      RCLCPP_ERROR(
        this->get_logger(), "cudaStreamCreate failed: %s", cudaGetErrorString(stream_err));
    }

    // Depth 1 (not the earlier 10) on these subscriptions and the sync
    // below -- the detection chain upstream only sustains ~27-36Hz against
    // a 60Hz camera, and a deep reliable queue here let stale frames back
    // up (growing GPU util and RTP latency over the session) instead of
    // always working the newest frame.
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections_output", 1,
      std::bind(&publish_mask_nitros::detectionCallback, this, std::placeholders::_1));

    selected_detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>("best_detection", 10);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/yolov8_encoder/resize/camera_info", 1,
      std::bind(&publish_mask_nitros::cameraInfoCallback, this, std::placeholders::_1));

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera0/detection/mask/camera_info", 10);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mask_overlay", 10);

    // NITROS-typed, GPU-resident mask publisher -- replaces the plain
    // sensor_msgs::msg::Image publisher from publish_mask.cpp.
    mask_pub_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "/camera0/detection/mask",
      nvidia::isaac_ros::nitros::nitros_image_mono8_t::supported_type_name);

    // publish the target point from the object detection
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/sc2/state_estimation_node/detected_objects/target", 10);

    rgb_sub_.subscribe(this, "/oak/rgb/image_raw_rgb8");
    resized_sub_.subscribe(this, "/yolov8_encoder/resize/image");

    sync_ = std::make_shared<Sync>(SyncPolicy(1), rgb_sub_, resized_sub_);
    sync_->registerCallback(std::bind(&publish_mask_nitros::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Subscribe to depth image and camera info (input topics)
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/oak/stereo/image_raw_depth", 1,
      std::bind(&publish_mask_nitros::depthCallback, this, std::placeholders::_1));

    depth_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/oak/stereo/camera_info", 1,
      std::bind(&publish_mask_nitros::depthCameraInfoCallback, this, std::placeholders::_1));

    // Publishers for republished depth image and camera info with updated timestamps
    // (left as plain sensor_msgs types -- see file header comment)
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/oak/stereo/image_raw", 10);
    depth_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/oak/stereo/camera_info", 10);
  }

  ~publish_mask_nitros() override
  {
    if (cuda_stream_) {
      cudaStreamDestroy(cuda_stream_);
    }
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

  // Copies a CV_8UC1 host cv::Mat to a device buffer and wraps it in a
  // NitrosImage. UNVERIFIED against the real NitrosImageBuilder API --
  // see file header comment.
  nvidia::isaac_ros::nitros::NitrosImage buildNitrosMask(
    const cv::Mat & mask, const std_msgs::msg::Header & header)
  {
    const size_t num_bytes = static_cast<size_t>(mask.rows) * static_cast<size_t>(mask.cols);

    void * device_ptr = nullptr;
    cudaError_t err = cudaMallocAsync(&device_ptr, num_bytes, cuda_stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(this->get_logger(), "cudaMallocAsync failed: %s", cudaGetErrorString(err));
      throw std::runtime_error("cudaMallocAsync failed in buildNitrosMask");
    }

    err = cudaMemcpyAsync(
      device_ptr, mask.data, num_bytes, cudaMemcpyHostToDevice, cuda_stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(this->get_logger(), "cudaMemcpyAsync failed: %s", cudaGetErrorString(err));
      cudaFreeAsync(device_ptr, cuda_stream_);
      throw std::runtime_error("cudaMemcpyAsync failed in buildNitrosMask");
    }
    cudaStreamSynchronize(cuda_stream_);

    return nvidia::isaac_ros::nitros::NitrosImageBuilder()
      .WithHeader(header)
      .WithEncoding("mono8")
      .WithDimensions(static_cast<uint32_t>(mask.rows), static_cast<uint32_t>(mask.cols))
      .WithGpuData(device_ptr)
      .Build();
  }

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & resized_msg)
  {
    vision_msgs::msg::Detection2DArray::SharedPtr detections_copy;
    {
      std::lock_guard<std::mutex> lock(detections_mutex_);
      if (!latest_detections_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No detections received yet");
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

    // 255 = masked -> nvblox multi_mapper routes it to the dynamic/occupancy
    // mapper, not the static TSDF. 0 = unmasked -> static TSDF. We want only
    // the detected target in the static TSDF, so background starts at 255
    // (masked/excluded) and the detected bbox is punched out to 0
    // (unmasked/included) below.
    cv::Mat mask(image_height_, image_width_, CV_8UC1, cv::Scalar(255));

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
      cv::fillConvexPoly(mask, contour, cv::Scalar(0));

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

    cv::Mat overlay = resized_image.clone();
    cv::Mat blended;
    double alpha = 0.5;

    // 1. Create a red image the same size as our frame
    cv::Mat red_img = cv::Mat(mask.size(), CV_8UC3, cv::Scalar(0, 0, 255));

    // 2. Blend the red image with the original image
    // Note: We use 1.0 for the background to keep it at full brightness
    cv::addWeighted(red_img, alpha, overlay, 1.0, 0, blended);

    // 3. Only apply the "blended" version over the detected target, not the
    // (now-inverted) background. mask is 0 on the target / 255 elsewhere
    // (see TSDF-gating comment above), so invert it just for this overlay
    // highlight -- this is purely a debug-visualization concern, independent
    // of the polarity nvblox needs.
    cv::Mat highlight_mask;
    cv::bitwise_not(mask, highlight_mask);
    blended.copyTo(overlay, highlight_mask);

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

    // Publish mask (NITROS) and overlay (plain) with RGB timestamp
    std_msgs::msg::Header header;
    header.stamp = rgb_msg->header.stamp;
    header.frame_id = last_camera_info_ ? last_camera_info_->header.frame_id : "oak_rgb_camera_optical_frame";

    try {
      mask_pub_->publish(buildNitrosMask(mask, header));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish NITROS mask: %s", e.what());
    }

    auto overlay_msg = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    overlay_pub_->publish(*overlay_msg);

    // Also publish the target point for visual servoing (if we found one)
    if (best_det) {
      target_pub_->publish(target_point_msg);

      vision_msgs::msg::Detection2D selected_msg = *best_det;
      selected_msg.header.stamp = rgb_msg->header.stamp; // SYNC THE TIMESTAMP
      selected_msg.header.frame_id = header.frame_id;
      selected_detection_pub_->publish(selected_msg);

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

  cudaStream_t cuda_stream_{};

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr selected_detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>> mask_pub_;
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

RCLCPP_COMPONENTS_REGISTER_NODE(publish_mask_nitros)
