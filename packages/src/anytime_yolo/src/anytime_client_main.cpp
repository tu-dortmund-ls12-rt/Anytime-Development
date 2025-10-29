#include "anytime_core/anytime_main.hpp"
#include "anytime_yolo/anytime_client.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  return anytime_core::anytime_client_main<AnytimeActionClient>(argc, argv);
}
