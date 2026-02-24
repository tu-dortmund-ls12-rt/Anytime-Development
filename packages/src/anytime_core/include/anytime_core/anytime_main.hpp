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

#ifndef ANYTIME_CORE__ANYTIME_MAIN_HPP_
#define ANYTIME_CORE__ANYTIME_MAIN_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace anytime_core
{

/**
 * @brief Common main function template for Anytime action servers
 *
 * This template function consolidates the identical main function logic
 * used by both anytime_monte_carlo and anytime_yolo server implementations.
 *
 * @tparam ServerNode The specific server node type (e.g., AnytimeActionServer)
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return Exit code (0 for success)
 *
 * @example
 * int main(int argc, char* argv[]) {
 *   return anytime_core::anytime_server_main<AnytimeActionServer>(argc, argv);
 * }
 */
template<typename ServerNode>
int anytime_server_main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<ServerNode>(options);

  // Default to single-threaded
  std::string executor_type = "single";

  // Parse command-line args
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--is_single_multi" && i + 1 < argc) {
      executor_type = argv[i + 1];
      i++;  // Skip the next argument since we consumed it
    }
  }

  if (executor_type == "multi") {
    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } else {
    // Use single-threaded executor (default)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}

/**
 * @brief Common main function template for Anytime action clients
 *
 * This template function consolidates the identical main function logic
 * used by both anytime_monte_carlo and anytime_yolo client implementations.
 *
 * @tparam ClientNode The specific client node type (e.g., AnytimeActionClient)
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return Exit code (0 for success)
 *
 * @example
 * int main(int argc, char* argv[]) {
 *   return anytime_core::anytime_client_main<AnytimeActionClient>(argc, argv);
 * }
 */
template<typename ClientNode>
int anytime_client_main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<ClientNode>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

}  // namespace anytime_core

#endif  // ANYTIME_CORE__ANYTIME_MAIN_HPP_
