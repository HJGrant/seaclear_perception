// Drop-in replacement for rclcpp_components' component_container_mt whose
// loaded nodes do not publish /parameter_events.
//
// Why: in Humble every node publishes /parameter_events (reliable, keeps up to
// 1000 samples), and every remote Humble node subscribes to it. With readers
// on other hosts (.95 on the robot, .117 topside), Fast DDS 2.6 sent ~15-18
// heartbeats/s per stored sample per writer -- ~56k pkt/s (~36 Mbit/s) from
// this container alone, ~6 Mbit/s of it across the tether (measured
// 2026-09-29). Parameters still work (get/set/list services, on-set callbacks);
// only the change notifications for external tools are no longer broadcast.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

class NoParamEventsComponentManager : public rclcpp_components::ComponentManager
{
public:
  using rclcpp_components::ComponentManager::ComponentManager;

protected:
  rclcpp::NodeOptions create_node_options(
    const std::shared_ptr<LoadNode::Request> request) override
  {
    auto options = rclcpp_components::ComponentManager::create_node_options(request);
    options.start_parameter_event_publisher(false);
    return options;
  }
};

// Same as Humble's component_container_mt main(), with the manager above.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<NoParamEventsComponentManager>();
  if (node->has_parameter("thread_num")) {
    const auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, thread_num);
  }
  node->set_executor(exec);
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}
