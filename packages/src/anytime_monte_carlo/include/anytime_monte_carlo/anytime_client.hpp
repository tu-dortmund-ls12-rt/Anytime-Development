// Copyright 2025 Anytime System
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ANYTIME_MONTE_CARLO__ANYTIME_CLIENT_HPP_
#define ANYTIME_MONTE_CARLO__ANYTIME_CLIENT_HPP_

#include <memory>

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
  explicit AnytimeActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

#endif  // ANYTIME_MONTE_CARLO__ANYTIME_CLIENT_HPP_
