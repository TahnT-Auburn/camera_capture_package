cmake_minimum_required(VERSION 3.5)
project(camera_capture_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# arenasdk find file dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(OpenCV)
find_package(arena_sdk REQUIRED)

set(target_name capture)

add_executable(${target_name}
  src/camera_capture.cpp
  src/rclcpp_adapter/pixelformat_translation.cpp
)
ament_target_dependencies(
  ${target_name}
  rclcpp
  OpenCV
  arena_sdk
)

install(TARGETS
  ${target_name}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
