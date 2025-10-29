#ifndef ANYTIME_CORE_ANYTIME_BASE_HPP
#define ANYTIME_CORE_ANYTIME_BASE_HPP

#include "anytime_core/anytime_waitable.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>

namespace anytime_core
{

template <typename InterfaceType, typename GoalHandleType>
class AnytimeBase
{
public:
  virtual ~AnytimeBase() = default;

  // Domain-specific pure virtual functions that must be implemented by derived classes
  // These handle the actual computation logic
  virtual void compute_iteration() = 0;  // The actual computation work
  virtual void populate_feedback(std::shared_ptr<typename InterfaceType::Feedback> feedback) = 0;
  virtual void populate_result(std::shared_ptr<typename InterfaceType::Result> result) = 0;
  virtual void reset_domain_state() = 0;   // Reset domain-specific state
  virtual bool should_finish() const = 0;  // Check if computation should finish

  // Optional: callback to be invoked after each iteration (for async GPU operations)
  // Return nullptr for synchronous operations (default behavior for Monte Carlo)
  // Requirements for get_iteration_callback, sends feedback and calls notify_result
  // when batch is complete
  virtual void * get_iteration_callback() { return nullptr; }

  // Common reactive/proactive implementations
  virtual void reactive_function()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Reactive function called");
    compute();
    if (get_iteration_callback() == nullptr) {
      // Synchronous mode: send feedback and notify result immediately
      send_feedback();
      notify_result();
    }
    // Async mode: callback will handle notification
  }

  virtual void reactive_result_function()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Reactive result function called");
    bool should_finish_now = should_finish();
    bool should_cancel = goal_handle_->is_canceling();

    if ((should_finish_now || should_cancel) && is_running()) {
      calculate_result();
      notify_check_finish();
    } else if (is_running()) {
      notify_iteration();
    }
  }

  virtual void check_cancel_and_finish_reactive()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Check cancel and finish reactive function called");
    bool should_finish_now = should_finish();
    bool should_cancel = goal_handle_->is_canceling();

    if ((should_finish_now || should_cancel) && is_running()) {
      if (should_cancel) {
        goal_handle_->canceled(result_);
      } else {
        goal_handle_->succeed(result_);
      }

      deactivate();
      RCLCPP_DEBUG(
        node_->get_logger(), "Reactive function finished, should finish: %d, should cancel: %d",
        should_finish_now, should_cancel);
    } else if (!is_running()) {
      RCLCPP_DEBUG(node_->get_logger(), "Reactive function finished previously");
    }
  }

  virtual void proactive_function()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Proactive function called");
    compute();
    if (get_iteration_callback() == nullptr) {
      // Synchronous mode: notify result immediately
      notify_result();
    }
    // Async mode: callback will handle notification
  }

  virtual void proactive_result_function()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Proactive result function called");
    calculate_result();
    send_feedback();
    notify_check_finish();
  }

  virtual void check_cancel_and_finish_proactive()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Check cancel and finish proactive function called");
    bool should_finish_now = should_finish();
    bool should_cancel = goal_handle_->is_canceling();

    if ((should_finish_now || should_cancel) && is_running()) {
      if (should_cancel) {
        goal_handle_->canceled(result_);
      } else {
        goal_handle_->succeed(result_);
      }
      deactivate();
      RCLCPP_DEBUG(
        node_->get_logger(), "Proactive function finished, should finish: %d, should cancel: %d",
        should_finish_now, should_cancel);
    } else if (!is_running()) {
      RCLCPP_DEBUG(node_->get_logger(), "Proactive function finished previously");
    } else {
      notify_iteration();
    }
  }

  virtual void start()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Start function called");
    notify_iteration();
  }

  virtual void compute()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Compute function called");

    // Start timing
    auto start_time = node_->now();

    // Call domain-specific computation
    compute_iteration();

    // End timing
    auto end_time = node_->now();

    // Calculate computation time for this batch
    rclcpp::Duration computation_time = end_time - start_time;

    // Update the average computation time using incremental averaging
    // Using double precision to avoid integer overflow
    batch_count_++;
    if (batch_count_ == 1) {
      average_computation_time_ = computation_time;
    } else {
      // Compute incremental average: avg_new = avg_old + (value - avg_old) / count
      // Use double to avoid overflow with large nanosecond values
      double avg_ns = static_cast<double>(average_computation_time_.nanoseconds());
      double new_ns = static_cast<double>(computation_time.nanoseconds());
      double delta = new_ns - avg_ns;
      avg_ns = avg_ns + (delta / static_cast<double>(batch_count_));

      // Clamp to valid range to prevent overflow
      if (avg_ns < 0.0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Negative average computation time detected, resetting to current time");
        avg_ns = new_ns;
      }

      average_computation_time_ =
        rclcpp::Duration(std::chrono::nanoseconds(static_cast<int64_t>(avg_ns)));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "Average computation time: %f ms",
      average_computation_time_.nanoseconds() / 1e6);
  }

  virtual void send_feedback()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Send feedback function called");
    auto feedback = std::make_shared<typename InterfaceType::Feedback>();

    // Let derived class populate domain-specific feedback
    populate_feedback(feedback);

    if (goal_handle_) {
      goal_handle_->publish_feedback(feedback);
      RCLCPP_DEBUG(node_->get_logger(), "Feedback sent");
    } else {
      RCLCPP_WARN(node_->get_logger(), "Goal handle is null, cannot send feedback");
    }
  }

  virtual void calculate_result()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Calculate result function called");
    auto new_result = std::make_shared<typename InterfaceType::Result>();

    // Let derived class populate domain-specific result
    populate_result(new_result);

    result_ = new_result;
  }

  virtual void notify_cancel()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Notify cancel function");
    if (is_reactive_proactive_) {
      // Proactive: check finish immediately
      this->notify_check_finish();
    } else {
      // Reactive: process result first, then check finish
      this->notify_result();
    }
    RCLCPP_DEBUG(node_->get_logger(), "Notify cancel function finished");
  }

  virtual void reset()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Reset function called");
    result_ = std::make_shared<typename InterfaceType::Result>();

    // Reset domain-specific state
    reset_domain_state();

    batch_count_ = 0;
    average_computation_time_ = rclcpp::Duration(0, 0);
  }

  // Activation control
  void activate() { is_running_.store(true, std::memory_order_seq_cst); }
  void deactivate() { is_running_.store(false, std::memory_order_seq_cst); }
  bool is_running() const { return is_running_.load(std::memory_order_seq_cst); }

  // Goal handle management
  void set_goal_handle(std::shared_ptr<GoalHandleType> goal_handle) { goal_handle_ = goal_handle; }
  std::shared_ptr<GoalHandleType> get_goal_handle() { return goal_handle_; }

  // Waitable notifications
  void notify_iteration() { anytime_iteration_waitable_->notify(); }
  void notify_result() { anytime_result_waitable_->notify(); }
  void notify_check_finish() { anytime_check_finish_waitable_->notify(); }

protected:
  // Initialize waitables and callback group
  // Call this from derived class constructor with is_reactive_proactive template parameter
  template <bool isReactiveProactive>
  void initialize_anytime_base(rclcpp::Node * node, int batch_size)
  {
    node_ = node;
    batch_size_ = batch_size;

    // Store the mode
    is_reactive_proactive_ = isReactiveProactive;

    // Create callback group
    compute_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create and register waitables based on reactive/proactive mode
    if constexpr (isReactiveProactive) {
      // Proactive mode
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_function(); });
      anytime_result_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_result_function(); });
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_proactive(); });
    } else {
      // Reactive mode
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_function(); });
      anytime_result_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_result_function(); });
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_reactive(); });
    }

    // Add waitables to node
    node->get_node_waitables_interface()->add_waitable(
      anytime_iteration_waitable_, compute_callback_group_);
    node->get_node_waitables_interface()->add_waitable(
      anytime_result_waitable_, compute_callback_group_);
    node->get_node_waitables_interface()->add_waitable(
      anytime_check_finish_waitable_,
      node->get_node_base_interface()->get_default_callback_group());
  }

  // Node pointer for logging (must be set by derived class)
  rclcpp::Node * node_ = nullptr;

  // Waitables for synchronization
  std::shared_ptr<AnytimeWaitable> anytime_iteration_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_result_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_check_finish_waitable_;

  // Running state
  std::atomic<bool> is_running_{false};

  // Goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  // Feedback and result messages
  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
    std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
    std::make_shared<typename InterfaceType::Result>();

  // Callback group for compute operations
  rclcpp::CallbackGroup::SharedPtr compute_callback_group_;

  // Batch size configuration
  int batch_size_ = 1;

  // Performance metrics
  uint64_t batch_count_ = 0;
  rclcpp::Duration average_computation_time_{0, 0};

  // Mode tracking
  bool is_reactive_proactive_ = false;
};

}  // namespace anytime_core

#endif  // ANYTIME_CORE_ANYTIME_BASE_HPP