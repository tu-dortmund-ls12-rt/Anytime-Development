#include "anytime_monte_carlo/anytime_server.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <rclcpp/logging.hpp>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: anytime_core::AnytimeActionServerBase<Anytime>(
    "anytime_action_server", options.use_intra_process_comms(true))
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action server");

  // Read the ros2 parameters
  std::string reactive_proactive_str = this->declare_parameter("is_reactive_proactive", "reactive");
  int batch_size = this->declare_parameter("batch_size", 1);

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");

  RCLCPP_INFO(this->get_logger(), "Monte Carlo Action Server initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  is_reactive_proactive: %s", reactive_proactive_str.c_str());
  RCLCPP_INFO(this->get_logger(), "  batch_size: %d", batch_size);

  RCLCPP_DEBUG(
    this->get_logger(), "is_reactive_proactive: %s",
    is_reactive_proactive ? "proactive" : "reactive");
  RCLCPP_DEBUG(this->get_logger(), "batch_size: %d", batch_size);

  // Create the Anytime management object based on the parameters
  this->anytime_management_ = create_anytime_management(this, is_reactive_proactive, batch_size);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<
  anytime_core::AnytimeBase<AnytimeActionServer::Anytime, AnytimeActionServer::GoalHandleType>>
AnytimeActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, int batch_size)
{
  if (is_reactive_proactive) {
    return std::make_shared<AnytimeManagement<true>>(node, batch_size);
  } else {
    return std::make_shared<AnytimeManagement<false>>(node, batch_size);
  }
}

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionServer)