#include "interference/interference_timer_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node options
  rclcpp::NodeOptions options;

  // Create and spin the interference timer node
  auto node = std::make_shared<interference::InterferenceTimerNode>(options);

  RCLCPP_INFO(node->get_logger(), "Starting Interference Timer Node...");

  // Spin the node
  rclcpp::spin(node);

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
