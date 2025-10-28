#ifndef ANYTIME_MONTE_CARLO_ANYTIME_SERVER_HPP
#define ANYTIME_MONTE_CARLO_ANYTIME_SERVER_HPP

#include "anytime_core/anytime_server.hpp"
#include "anytime_interfaces/action/monte_carlo.hpp"
#include "anytime_monte_carlo/anytime_management.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AnytimeActionServer
: public anytime_core::AnytimeActionServerBase<anytime_interfaces::action::MonteCarlo>
{
public:
  using Anytime = anytime_interfaces::action::MonteCarlo;
  using GoalHandleType = rclcpp_action::ServerGoalHandle<Anytime>;

  explicit AnytimeActionServer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~AnytimeActionServer() override;

private:
  // factory function for creating anytime management
  std::shared_ptr<anytime_core::AnytimeBase<Anytime, GoalHandleType>> create_anytime_management(
    rclcpp::Node * node, bool is_reactive_proactive, int batch_size);
};

#endif  // ANYTIME_MONTE_CARLO_ANYTIME_SERVER_HPP