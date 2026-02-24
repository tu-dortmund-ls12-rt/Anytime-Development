#include "anytime_yolo/yolo.hpp"

#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: warmup_engines <weights_path>" << std::endl;
    return 1;
  }

  const std::string weights_path = argv[1];
  std::cout << "Building/loading TensorRT engines from: " << weights_path << std::endl;

  try {
    AnytimeYOLO yolo(weights_path, false);
    std::cout << "All engines ready." << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "Engine warmup failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
