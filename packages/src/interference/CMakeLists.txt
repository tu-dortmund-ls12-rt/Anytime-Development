cmake_minimum_required(VERSION 3.8)
project(interference)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(anytime_tracing REQUIRED)

# Include directories
include_directories(include)

set(dependencies "rclcpp" "rclcpp_components" "anytime_tracing")

# Create interference timer library
add_library(interference_timer_lib SHARED
  src/interference_timer_node.cpp
)

# Include directories for library
target_include_directories(interference_timer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(interference_timer_lib ${dependencies})

# Register component
rclcpp_components_register_node(interference_timer_lib
  PLUGIN "interference::InterferenceTimerNode"
  EXECUTABLE interference_timer_component)

# Create interference timer executable
add_executable(interference_timer_node
  src/interference_timer_main.cpp
)

# Link executable with library
target_link_libraries(interference_timer_node interference_timer_lib)

# Install targets
install(TARGETS
  interference_timer_lib
  interference_timer_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
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

ament_export_include_directories(include)
ament_export_libraries(interference_timer_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
