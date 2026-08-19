// Multi-frame TSDF mapping support: periodically re-anchors a "temporary
// global frame" (map_frame, e.g. tsdf_map_origin) at the vehicle's current
// estimated pose, and clears nvblox's map at the same moment. Bounds how
// much state-estimate drift can corrupt the map's internal consistency --
// within one reset window the map is built relative to a fixed snapshot of
// the pose estimate, not the continuously-drifting live estimate.
//
// parent_frame -> base_frame is expected to be a live, continuously updated
// transform (either the real EKF's odom -> sc2_base_link, or the
// IMU-only fallback in imu_pose_estimate.launch.py -- this node doesn't
// care which). At each reset, it snapshots that transform and republishes
// it as a *static* parent_frame -> map_frame, which stays frozen until the
// next reset. nvblox's global_frame parameter should be set to map_frame.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

class map_frame_anchor : public rclcpp::Node
{
public:
  explicit map_frame_anchor(const rclcpp::NodeOptions & options)
  : Node("map_frame_anchor", options)
  {
    this->declare_parameter<std::string>("parent_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "sc2_base_link");
    this->declare_parameter<std::string>("map_frame", "tsdf_map_origin");
    this->declare_parameter<std::string>("trigger_topic", "/oak/stereo/image_raw");
    this->declare_parameter<int>("reset_every_n_frames", 100);
    this->declare_parameter<std::string>("clear_map_service", "/nvblox_node/clear_map");

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    reset_every_n_frames_ = static_cast<int>(this->get_parameter("reset_every_n_frames").as_int());
    const std::string trigger_topic = this->get_parameter("trigger_topic").as_string();
    const std::string clear_map_service = this->get_parameter("clear_map_service").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    clear_map_client_ = this->create_client<std_srvs::srv::Trigger>(clear_map_service);

    frame_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      trigger_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&map_frame_anchor::frameCallback, this, std::placeholders::_1));

    // Delay the first anchor -- at construction the tf buffer hasn't had a
    // chance to receive anything yet, so an immediate lookup would just
    // fail. By the time reset_every_n_frames triggers the *next* one, TF
    // should be populated regardless, but doing an early one too means the
    // map has a valid anchor from close to startup rather than sitting
    // unanchored for the whole first window.
    // Repeating, not one-shot -- the first attempt can easily race ahead of
    // the pose source actually being up (EKF, static TF publishers, etc.
    // are all starting up concurrently). Keeps retrying every 2s until the
    // first anchor actually succeeds, then stops. Without this, a failed
    // first attempt wouldn't retry again until reset_every_n_frames frames
    // had counted -- fine at the default (100, ~10s), but a real stall at
    // higher settings (5000 frames is ~500s at 10Hz depth).
    startup_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        if (resetAndAnchor()) {
          startup_timer_->cancel();
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "map_frame_anchor: will re-anchor '%s' -> '%s' as '%s' -> '%s' every %d frames of %s",
      parent_frame_.c_str(), base_frame_.c_str(), parent_frame_.c_str(), map_frame_.c_str(),
      reset_every_n_frames_, trigger_topic.c_str());
  }

private:
  void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &)
  {
    if (++frame_count_ >= reset_every_n_frames_) {
      frame_count_ = 0;
      resetAndAnchor();
    }
  }

  // Returns true if the anchor transform was successfully looked up and
  // published (regardless of whether the clear_map call itself succeeded --
  // that failure is logged separately and doesn't block the retry timer,
  // since the anchor itself is the more fundamental thing to get right).
  bool resetAndAnchor()
  {
    geometry_msgs::msg::TransformStamped current;
    try {
      current = tf_buffer_->lookupTransform(parent_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(
        this->get_logger(), "map_frame_anchor: could not look up %s -> %s (%s), skipping reset",
        parent_frame_.c_str(), base_frame_.c_str(), e.what());
      return false;
    }

    geometry_msgs::msg::TransformStamped anchor = current;
    anchor.header.stamp = this->get_clock()->now();
    anchor.header.frame_id = parent_frame_;
    anchor.child_frame_id = map_frame_;
    // Republishing a static transform for a child_frame_id that was
    // already broadcast replaces the previously latched value -- this is
    // what makes "frozen until the next reset" work.
    static_broadcaster_->sendTransform(anchor);

    if (!clear_map_client_->service_is_ready()) {
      RCLCPP_WARN(
        this->get_logger(),
        "map_frame_anchor: re-anchored '%s' but %s isn't available -- "
        "old geometry was NOT cleared",
        map_frame_.c_str(), clear_map_client_->get_service_name());
      return true;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    clear_map_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto result = future.get();
        if (!result->success) {
          RCLCPP_WARN(
            this->get_logger(), "map_frame_anchor: clear_map service reported failure: %s",
            result->message.c_str());
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "map_frame_anchor: re-anchored '%s' at current '%s' pose and requested a map clear",
      map_frame_.c_str(), base_frame_.c_str());
    return true;
  }

  std::string parent_frame_;
  std::string base_frame_;
  std::string map_frame_;
  int reset_every_n_frames_;
  int frame_count_ = 0;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_map_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(map_frame_anchor)
