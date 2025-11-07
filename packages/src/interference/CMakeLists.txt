cmake_minimum_required(VERSION 3.8)
project(interference)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Create interference timer executable
add_executable(interference_timer_node
  src/interference_timer_node.cpp
  src/interference_timer_main.cpp
)

ament_target_dependencies(interference_timer_node
  rclcpp
)

# Install executables
install(TARGETS
  interference_timer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
