#ifndef ANYTIME_CORE_ANYTIME_SERVER_HPP
#define ANYTIME_CORE_ANYTIME_SERVER_HPP

#include "anytime_core/anytime_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>

namespace anytime_core
{

template <typename ActionType>
class AnytimeActionServerBase : public rclcpp::Node
{
public:
  using GoalHandleType = rclcpp_action::ServerGoalHandle<ActionType>;

  explicit AnytimeActionServerBase(
    const std::string & node_name, rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : Node(node_name, options)
  {
  }

  virtual ~AnytimeActionServerBase() = default;

protected:
  rclcpp_action::Server<ActionType>::SharedPtr action_server_;
  std::shared_ptr<AnytimeBase<ActionType, GoalHandleType>> anytime_management_;

  // Common goal handling
  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename ActionType::Goal> goal)
  {
    anytime_management_->set_goal_handle_receive_time(this->now());
    RCLCPP_DEBUG(this->get_logger(), "Received goal request");
    (void)uuid;  // Suppress unused variable warning
    (void)goal;  // Suppress unused variable warning

    if (anytime_management_->is_running()) {
      RCLCPP_DEBUG(this->get_logger(), "Goal rejected: server is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_DEBUG(this->get_logger(), "Goal accepted: server is inactive");
    anytime_management_->set_goal_handle_accept_time(this->now());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Common cancel handling
  virtual rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleType> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    anytime_management_->notify_cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Common accepted handling
  virtual void handle_accepted(const std::shared_ptr<GoalHandleType> goal_handle)
  {
    anytime_management_->set_goal_processing_start_time(this->now());
    RCLCPP_DEBUG(this->get_logger(), "Setting goal handle for AnytimeManagement");
    anytime_management_->set_goal_handle(goal_handle);

    RCLCPP_DEBUG(this->get_logger(), "Resetting AnytimeManagement");
    anytime_management_->reset();

    RCLCPP_DEBUG(this->get_logger(), "Activating AnytimeManagement");
    anytime_management_->activate();

    RCLCPP_DEBUG(this->get_logger(), "Start AnytimeManagement");
    anytime_management_->start();
  }
};

}  // namespace anytime_core

#endif  // ANYTIME_CORE_ANYTIME_SERVER_HPP
