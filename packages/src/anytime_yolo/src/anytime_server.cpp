#include "anytime_yolo/anytime_server.hpp"

#include <rclcpp/logging.hpp>

#include <map>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: anytime_core::AnytimeActionServerBase<Anytime>(
    "anytime_action_server", options.use_intra_process_comms(true))
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action server");

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
  std::string passive_cooperative_str =
    this->declare_parameter("is_passive_cooperative", "passive");
  std::string sync_async_str = this->declare_parameter("is_sync_async", "sync");
  int batch_size = this->declare_parameter("batch_size", 1);
  std::string weights_path = this->declare_parameter("weights_path", "");

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");
  bool is_passive_cooperative = (passive_cooperative_str == "cooperative");
  bool is_sync_async = (sync_async_str == "async");

  RCLCPP_DEBUG(
    this->get_logger(), "is_reactive_proactive: %s",
    is_reactive_proactive ? "proactive" : "reactive");
  RCLCPP_DEBUG(
    this->get_logger(), "is_passive_cooperative: %s",
    is_passive_cooperative ? "cooperative" : "passive");
  RCLCPP_DEBUG(this->get_logger(), "is_sync_async: %s", is_sync_async ? "async" : "sync");
  RCLCPP_DEBUG(this->get_logger(), "batch_size: %d", batch_size);
  RCLCPP_DEBUG(this->get_logger(), "weights_path: %s", weights_path.c_str());

  this->anytime_management_ = create_anytime_management(
    this, is_reactive_proactive, is_passive_cooperative, is_sync_async, batch_size, weights_path);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<
  anytime_core::AnytimeBase<AnytimeActionServer::Anytime, AnytimeActionServer::GoalHandleType>>
AnytimeActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, bool is_passive_cooperative, bool is_sync_async,
  int batch_size, const std::string & weights_path)
{
  if (is_reactive_proactive) {
    if (is_passive_cooperative) {
      if (is_sync_async) {
        return std::make_shared<AnytimeManagement<true, true, true>>(
          node, batch_size, weights_path);
      } else {
        return std::make_shared<AnytimeManagement<true, true, false>>(
          node, batch_size, weights_path);
      }
    } else {
      if (is_sync_async) {
        return std::make_shared<AnytimeManagement<true, false, true>>(
          node, batch_size, weights_path);
      } else {
        return std::make_shared<AnytimeManagement<true, false, false>>(
          node, batch_size, weights_path);
      }
    }
  } else {
    if (is_passive_cooperative) {
      if (is_sync_async) {
        return std::make_shared<AnytimeManagement<false, true, true>>(
          node, batch_size, weights_path);
      } else {
        return std::make_shared<AnytimeManagement<false, true, false>>(
          node, batch_size, weights_path);
      }
    } else {
      if (is_sync_async) {
        return std::make_shared<AnytimeManagement<false, false, true>>(
          node, batch_size, weights_path);
      } else {
        return std::make_shared<AnytimeManagement<false, false, false>>(
          node, batch_size, weights_path);
      }
    }
  }
}
