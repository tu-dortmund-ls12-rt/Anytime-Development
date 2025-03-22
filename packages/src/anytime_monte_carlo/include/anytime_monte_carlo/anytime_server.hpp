#include "anytime_interfaces/action/anytime.hpp"
#include "anytime_monte_carlo/monte_carlo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class AnytimeActionServer : public rclcpp::Node
{
public:
  AnytimeActionServer(rclcpp::NodeOptions options);
  ~AnytimeActionServer();

  using Anytime = anytime_interfaces::action::Anytime;
  using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

private:
  rclcpp_action::Server<Anytime>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Anytime::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  void execute(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  // factory function for monte carlo pi
  std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>> create_monte_carlo_pi(
    rclcpp::Node * node, bool is_reactive_proactive, bool is_single_multi, int batch_size);

  std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>> monte_carlo_pi_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionServer)