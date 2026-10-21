#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>

namespace danmucamera
{
// A deliberately small, separate ROS publisher for a local V4L2 camera or
// one still image. The calibration processes never link to a camera SDK.
class TopicCameraDriver : public rclcpp::Node
{
public:
  TopicCameraDriver() : Node("qiji_topic_camera_driver")
  {
    device_ = declare_parameter<std::string>("device", "/dev/video0");
    image_file_ = declare_parameter<std::string>("image_file", "");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    fps_ = std::clamp(declare_parameter<double>("fps", 15.0), 1.0, 60.0);
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/qiji_test/image_raw");
    const auto command_topic = declare_parameter<std::string>(
      "command_topic", "/qiji_test/stream_command");
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/qiji_test/stream_status");
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic, rclcpp::QoS(1).reliable().transient_local());
    command_sub_ = create_subscription<std_msgs::msg::Bool>(
      command_topic, rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data) {start();} else {stop();}
      });
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / fps_)), [this]() {publishFrame();});
    publishStatus("stopped");
    if (declare_parameter<bool>("autostart", false)) {start();}
  }

private:
  void publishStatus(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  void start()
  {
    if (running_) {publishStatus("streaming"); return;}
    if (!image_file_.empty()) {
      still_ = cv::imread(image_file_, cv::IMREAD_COLOR);
      if (still_.empty()) {
        publishStatus("error:cannot_read_image_file");
        RCLCPP_ERROR(get_logger(), "无法读取图片文件: %s", image_file_.c_str());
        return;
      }
    } else {
      if (!capture_.open(device_, cv::CAP_V4L2)) {
        publishStatus("error:cannot_open_camera");
        RCLCPP_ERROR(get_logger(), "无法打开 V4L2 相机: %s", device_.c_str());
        return;
      }
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
      capture_.set(cv::CAP_PROP_FPS, fps_);
    }
    running_ = true;
    publishStatus("streaming");
    RCLCPP_INFO(get_logger(), "发布图像话题: %s",
      image_pub_->get_topic_name());
  }

  void stop()
  {
    running_ = false;
    capture_.release();
    still_.release();
    publishStatus("stopped");
  }

  void publishFrame()
  {
    if (!running_) {return;}
    cv::Mat frame;
    if (image_file_.empty()) {
      if (!capture_.read(frame) || frame.empty()) {
        stop();
        publishStatus("error:camera_read_failed");
        return;
      }
    } else {frame = still_;}
    if (!frame.isContinuous()) {frame = frame.clone();}
    sensor_msgs::msg::Image message;
    message.header.stamp = now();
    message.header.frame_id = "qiji_test_camera";
    message.height = static_cast<uint32_t>(frame.rows);
    message.width = static_cast<uint32_t>(frame.cols);
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = static_cast<uint32_t>(frame.cols * 3);
    message.data.resize(static_cast<size_t>(message.step) * message.height);
    std::memcpy(message.data.data(), frame.data, message.data.size());
    image_pub_->publish(std::move(message));
  }

  std::string device_, image_file_;
  int width_{1280}, height_{720};
  double fps_{15.0};
  bool running_{false};
  cv::VideoCapture capture_;
  cv::Mat still_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace danmucamera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<danmucamera::TopicCameraDriver>());
  rclcpp::shutdown();
  return 0;
}
