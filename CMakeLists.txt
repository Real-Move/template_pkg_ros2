cmake_minimum_required(VERSION 3.5)
project(template_pkg_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(ros2_template_node_cpp src/ros2_template_node_cpp.cpp)
ament_target_dependencies(ros2_template_node_cpp rclcpp)

add_executable(hello_world_node src/hello_world_node.cpp)
ament_target_dependencies(hello_world_node rclcpp)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

message(STATUS "BUILD_TESTING: ${BUILD_TESTING}")

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_subdirectory(test)

endif()

# Install Python executables
install(PROGRAMS
  scripts/ros2_template_node_python.py
  scripts/hello_world_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
ros2_template_node_cpp
hello_world_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
