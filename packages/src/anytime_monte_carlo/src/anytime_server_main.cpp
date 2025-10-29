#include "anytime_core/anytime_main.hpp"
#include "anytime_monte_carlo/anytime_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  return anytime_core::anytime_server_main<AnytimeActionServer>(argc, argv);
}
