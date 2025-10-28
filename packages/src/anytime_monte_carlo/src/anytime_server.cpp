#include "anytime_monte_carlo/anytime_server.hpp"

#include <rclcpp/logging.hpp>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: anytime_core::AnytimeActionServerBase<Anytime>("anytime_action_server", options)
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action server");

  // Create the action server
  this->action_server_ = rclcpp_action::create_server<Anytime>(
    this, "anytime",
    [this](const rclcpp_action::GoalUUID uuid, std::shared_ptr<const Anytime::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleType> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleType> goal_handle) {
      return this->handle_accepted(goal_handle);
    },
    rcl_action_server_get_default_options(),
    this->get_node_base_interface()->get_default_callback_group());

  // Read the ros2 parameters
  std::string reactive_proactive_str = this->declare_parameter("is_reactive_proactive", "reactive");
  int batch_size = this->declare_parameter("batch_size", 1);

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");

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