#ifndef INTERFERENCE__INTERFERENCE_TIMER_NODE_HPP_
#define INTERFERENCE__INTERFERENCE_TIMER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>

namespace interference
{

class InterferenceTimerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for InterferenceTimerNode
   *
   * @param options Node options
   */
  explicit InterferenceTimerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~InterferenceTimerNode();

private:
  /**
   * @brief Timer callback that performs busy-wait loop
   */
  void timer_callback();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int timer_period_ms_;    // Timer period in milliseconds
  int execution_time_ms_;  // Execution time (busy-wait duration) in milliseconds

  // Statistics
  size_t execution_count_;
};

}  // namespace interference

#endif  // INTERFERENCE__INTERFERENCE_TIMER_NODE_HPP_
