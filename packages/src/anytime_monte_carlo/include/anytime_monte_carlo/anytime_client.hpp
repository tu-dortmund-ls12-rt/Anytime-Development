#ifndef ANYTIME_CLIENT_HPP
#define ANYTIME_CLIENT_HPP

#include "anytime_core/anytime_client_base.hpp"
#include "anytime_interfaces/action/monte_carlo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Class definition for the Anytime Action Client
class AnytimeActionClient
: public anytime_core::AnytimeClientBase<anytime_interfaces::action::MonteCarlo>
{
public:
  // Type aliases for convenience
  using Anytime = anytime_interfaces::action::MonteCarlo;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

  // Constructor
  AnytimeActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Destructor
  ~AnytimeActionClient();

private:
  // Timer for sending goals
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  // Timer for cancel timeout
  rclcpp::TimerBase::SharedPtr cancel_timeout_timer_ = nullptr;

  // Function to send a goal to the action server
  void send_goal();

  // Callback for cancel timeout
  void cancel_timeout_callback();

  // Domain-specific implementations from base class
  void post_processing(const AnytimeGoalHandle::WrappedResult & result) override;
  void log_result(const AnytimeGoalHandle::WrappedResult & result) override;
  void process_feedback(
    AnytimeGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const Anytime::Feedback> feedback) override;
  void on_goal_rejected() override;
  void on_goal_accepted(AnytimeGoalHandle::SharedPtr goal_handle) override;
  void cleanup_after_result() override;
};

#endif  // ANYTIME_CLIENT_HPP
