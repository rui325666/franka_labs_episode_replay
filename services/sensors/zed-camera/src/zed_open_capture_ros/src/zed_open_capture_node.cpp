#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "videocapture.hpp"

namespace
{

using sl_oc::video::FPS;
using sl_oc::video::RESOLUTION;

RESOLUTION parse_resolution(const std::string & value)
{
  if (value == "HD2K") {
    return RESOLUTION::HD2K;
  }
  if (value == "HD1080") {
    return RESOLUTION::HD1080;
  }
  if (value == "HD720") {
    return RESOLUTION::HD720;
  }
  if (value == "VGA") {
    return RESOLUTION::VGA;
  }
  throw std::invalid_argument("Unsupported resolution '" + value + "'");
}

FPS parse_fps(const int value)
{
  switch (value) {
    case 15:
      return FPS::FPS_15;
    case 30:
      return FPS::FPS_30;
    case 60:
      return FPS::FPS_60;
    case 100:
      return FPS::FPS_100;
    default:
      throw std::invalid_argument("Unsupported FPS " + std::to_string(value));
  }
}

bool is_supported_mode(const std::string & resolution, const int fps)
{
  if (resolution == "HD2K") {
    return fps == 15;
  }
  if (resolution == "HD1080") {
    return fps == 15 || fps == 30;
  }
  if (resolution == "HD720") {
    return fps == 15 || fps == 30 || fps == 60;
  }
  if (resolution == "VGA") {
    return fps == 15 || fps == 30 || fps == 60 || fps == 100;
  }
  return false;
}

}  // namespace

class ZedOpenCaptureNode final : public rclcpp::Node
{
public:
  ZedOpenCaptureNode()
  : Node("zed_open_capture_node")
  {
    const auto device_id_value = declare_parameter<int64_t>("device_id", -1);
    const auto expected_serial_number =
      declare_parameter<int64_t>("expected_serial_number", 0);
    const auto resolution_name = declare_parameter<std::string>("resolution", "HD720");
    const auto fps_parameter = declare_parameter<int64_t>("fps", 30);
    eye_ = declare_parameter<std::string>("eye", "left");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "head_camera_left_optical_frame");
    const auto publish_topic = declare_parameter<std::string>(
      "publish_topic", "/head_camera/zed_node/rgb/color/rect/image");

    if (eye_ != "left" && eye_ != "right") {
      throw std::invalid_argument("Parameter 'eye' must be 'left' or 'right'");
    }
    if (device_id_value < -1 || device_id_value > 63) {
      throw std::invalid_argument("Parameter 'device_id' must be -1 or between 0 and 63");
    }
    const int device_id = static_cast<int>(device_id_value);
    const int fps_value = static_cast<int>(fps_parameter);
    if (!is_supported_mode(resolution_name, fps_value)) {
      throw std::invalid_argument(
              "Unsupported camera mode " + resolution_name + "@" +
              std::to_string(fps_value));
    }

    sl_oc::video::VideoParams params;
    params.res = parse_resolution(resolution_name);
    params.fps = parse_fps(fps_value);
    params.verbose = sl_oc::VERBOSITY::INFO;

    camera_ = std::make_unique<sl_oc::video::VideoCapture>(params);
    if (!camera_->initializeVideo(device_id)) {
      throw std::runtime_error("Unable to initialize a ZED video device");
    }

    const int actual_serial_number = camera_->getSerialNumber();
    if (expected_serial_number > 0 && actual_serial_number != expected_serial_number) {
      throw std::runtime_error(
              "Opened ZED serial " + std::to_string(actual_serial_number) +
              ", expected " + std::to_string(expected_serial_number));
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      publish_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ZedOpenCaptureNode::publish_latest_frame, this));

    stats_started_at_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "ZED serial=%d device=%s mode=%s@%d eye=%s topic=%s",
      actual_serial_number, camera_->getDeviceName().c_str(), resolution_name.c_str(),
      fps_value, eye_.c_str(), publish_topic.c_str());
    RCLCPP_WARN(
      get_logger(),
      "zed-open-capture supplies an unrectified image; the compatibility topic name contains 'rect'");
  }

private:
  void publish_latest_frame()
  {
    const auto & frame = camera_->getLastFrame(5);
    if (frame.data == nullptr || frame.frame_id == 0 || frame.frame_id == last_frame_id_) {
      return;
    }
    last_frame_id_ = frame.frame_id;

    if (frame.width < 2 || frame.height == 0 || frame.channels != 2) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unexpected frame layout: %ux%u channels=%u", frame.width, frame.height,
        frame.channels);
      return;
    }

    const int eye_width = static_cast<int>(frame.width / 2);
    const int x_offset = eye_ == "left" ? 0 : eye_width;
    const cv::Mat stereo_yuyv(
      static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC2, frame.data);
    const cv::Mat eye_yuyv = stereo_yuyv(
      cv::Rect(x_offset, 0, eye_width, static_cast<int>(frame.height)));

    cv::Mat eye_bgr;
    cv::cvtColor(eye_yuyv, eye_bgr, cv::COLOR_YUV2BGR_YUYV);

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    if (frame.timestamp != 0) {
      header.stamp.sec = static_cast<int32_t>(frame.timestamp / 1000000000ULL);
      header.stamp.nanosec = static_cast<uint32_t>(frame.timestamp % 1000000000ULL);
    } else {
      header.stamp = now();
    }

    auto message = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::BGR8, eye_bgr).toImageMsg();
    publisher_->publish(*message);

    ++published_frame_count_;
    if (published_frame_count_ % 300 == 0) {
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stats_started_at_).count();
      RCLCPP_INFO(
        get_logger(), "Published %u frames (%.2f Hz), output=%dx%d",
        published_frame_count_, published_frame_count_ / elapsed, eye_bgr.cols, eye_bgr.rows);
    }
  }

  std::unique_ptr<sl_oc::video::VideoCapture> camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string eye_;
  std::string frame_id_;
  uint64_t last_frame_id_{0};
  uint32_t published_frame_count_{0};
  std::chrono::steady_clock::time_point stats_started_at_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZedOpenCaptureNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_open_capture_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
