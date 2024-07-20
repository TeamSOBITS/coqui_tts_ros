cmake_minimum_required(VERSION 3.8)
project(coqui_tts_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sobits_msgs REQUIRED)

# Include directories
include_directories(
  ${rclpy_INCLUDE_DIRS}
  ${sobits_msgs_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  coqui_tts_ros/tts.py
  DESTINATION lib/${PROJECT_NAME}
)

# Package
ament_package()
