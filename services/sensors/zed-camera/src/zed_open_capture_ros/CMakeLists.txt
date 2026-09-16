cmake_minimum_required(VERSION 3.8)
project(zed_open_capture_ros)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_path(
  ZED_OPEN_CAPTURE_INCLUDE_DIR
  NAMES videocapture.hpp
  PATH_SUFFIXES zed-open-capture
  REQUIRED
)
find_library(
  ZED_OPEN_CAPTURE_LIBRARY
  NAMES zed_open_capture
  REQUIRED
)

add_executable(zed_open_capture_node src/zed_open_capture_node.cpp)
target_compile_features(zed_open_capture_node PUBLIC cxx_std_17)
target_compile_definitions(zed_open_capture_node PRIVATE VIDEO_MOD_AVAILABLE)
target_include_directories(
  zed_open_capture_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${ZED_OPEN_CAPTURE_INCLUDE_DIR}
)
ament_target_dependencies(
  zed_open_capture_node
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(
  zed_open_capture_node
  ${OpenCV_LIBRARIES}
  ${ZED_OPEN_CAPTURE_LIBRARY}
)

install(TARGETS zed_open_capture_node DESTINATION lib/${PROJECT_NAME})

ament_package()
