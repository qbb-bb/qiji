#include "ros_contract.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <opencv2/opencv.hpp>

#ifdef DANMUCAMERA_HAS_ORBBEC
#include <libobsensor/ObSensor.hpp>
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <memory>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace danmucamera
{

struct CameraDevice
{
  std::string id;
  std::string name;
  std::string backend;
  std::string signal;
};

struct CameraSelection
{
  std::string backend;
  std::string address;
  std::string signal;
};

CameraSelection parseSelection(const std::string & id)
{
  const auto first = id.find(':');
  const auto last = id.rfind(':');
  if (first == std::string::npos || last == first) {return {"v4l2", id, "rgb"};}
  return {id.substr(0, first), id.substr(first + 1, last - first - 1), id.substr(last + 1)};
}

// I/O 层：集中处理 V4L2、帧缓存、JPG 文件和 ROS I/O。
// 阻塞的 VideoCapture::read 只在 worker_ 中运行，绝不阻塞 ROS executor。
class IoNode : public rclcpp::Node
{
public:
  IoNode()
  : Node("calibration_io"), running_(true), streaming_(false), release_requested_(false)//赋初值
  {
    instance_lock_fd_ = ::open("/tmp/danmucamera_io_node.lock", O_CREAT | O_RDWR, 0666);
    if (instance_lock_fd_ < 0 || ::flock(instance_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      if (instance_lock_fd_ >= 0) {::close(instance_lock_fd_); instance_lock_fd_ = -1;}
      throw std::runtime_error(
        "已有一个 danmucamera io_node 正在运行；请关闭旧程序后再启动，避免相机状态错乱");
    }
    declare_parameter<std::string>("device", "auto");
    declare_parameter<std::string>(contract::extrinsic_input::kImageParameter, "");
    declare_parameter<std::string>(contract::extrinsic_input::kImageTypeParameter, "auto");
    declare_parameter<std::string>(contract::extrinsic_input::kCloudParameter, "");
    declare_parameter<std::string>(contract::extrinsic_input::kCommandParameter, "");
    declare_parameter<std::string>(contract::extrinsic_input::kStatusParameter, "");
    topic_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    declare_parameter<bool>("disable_ir_emitter", true);
    declare_parameter<std::vector<std::string>>("signals", std::vector<std::string>{"rgb"});
    output_directory_ = declare_parameter<std::string>("output_directory",
      "samples");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    camera_fps_ = declare_parameter<double>("fps", 30.0);
    preview_fps_ = declare_parameter<double>("preview_fps", 15.0);
    preview_width_ = declare_parameter<int>("preview_width", 1280);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 90);

    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kImageTopic, rclcpp::SensorDataQoS().keep_last(1));
    stream_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kStreamImageTopic, rclcpp::SensorDataQoS().keep_last(4));
    stream_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      contract::kStreamCloudTopic, rclcpp::SensorDataQoS().keep_last(1));
    captured_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kCapturedImageTopic, rclcpp::QoS(1).reliable());
    viewed_original_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kViewedOriginalTopic, rclcpp::QoS(1).reliable());
    viewed_error_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kViewedErrorTopic, rclcpp::QoS(1).reliable());
    viewed_undistorted_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kViewedUndistortedTopic, rclcpp::QoS(1).reliable());
    event_pub_ = create_publisher<std_msgs::msg::String>(contract::kCameraEventTopic, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(contract::kCameraStateTopic, 10);
    import_status_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kImportImageStatusTopic, 10);

    stream_service_ = create_service<std_srvs::srv::SetBool>(
      contract::kStreamingService,
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        setStreaming(request->data, *response);
      });
    emitter_service_ = create_service<std_srvs::srv::SetBool>(
      contract::kEmitterService,
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        setEmitterDisabled(request->data, *response);
      });
    capture_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kCaptureService,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {capture(*response);});
    save_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kSaveService,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {save(*response);});
    devices_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kRefreshDevicesService,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {listDevices(*response);});
    view_original_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kViewOriginalRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        publishFile(message->data, viewed_original_pub_);
      });
    view_error_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kViewErrorRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        publishFile(message->data, viewed_error_pub_);
      });
    view_undistorted_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kViewUndistortedRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        publishFile(message->data, viewed_undistorted_pub_);
      });
    import_image_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kImportImageRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {importImage(message->data);});
    delete_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kDeleteSampleTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {deleteSample(message->data);});

    worker_ = std::thread(&IoNode::captureLoop, this);
    const auto publish_period = std::chrono::duration<double>(1.0 / std::max(1.0, preview_fps_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
      std::bind(&IoNode::publishLatestFrame, this));
    RCLCPP_INFO(get_logger(), "I/O 节点就绪；硬件采集运行在独立线程");
  }

  ~IoNode() override
  {
    running_ = false;
    streaming_ = false;
    if (worker_.joinable()) {worker_.join();}
    releaseCameraResources();
#ifdef DANMUCAMERA_HAS_ORBBEC
    ob_context_.reset();
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
    realsense_context_.reset();
#endif
    if (instance_lock_fd_ >= 0) {
      ::flock(instance_lock_fd_, LOCK_UN);
      ::close(instance_lock_fd_);
    }
  }

private:
  std::vector<CameraDevice> discoverDevices()
  {
    std::vector<CameraDevice> result;
#ifdef DANMUCAMERA_HAS_ORBBEC
    try {
      std::lock_guard<std::recursive_mutex> sdk_lock(orbbec_mutex_);
      if (!ob_context_) {ob_context_ = std::make_unique<ob::Context>();}
      const auto devices = ob_context_->queryDeviceList();
      for (uint32_t i = 0; i < devices->getCount(); ++i) {
        const std::string serial = devices->getSerialNumber(i);
        const std::string name = devices->getName(i);
        result.push_back({"orbbec:" + serial + ":rgb", name, "orbbec", "rgb"});
        result.push_back({"orbbec:" + serial + ":ir-left", name, "orbbec", "ir-left"});
        result.push_back({"orbbec:" + serial + ":ir-right", name, "orbbec", "ir-right"});
        result.push_back({"orbbec:" + serial + ":depth", name, "orbbec", "depth"});
      }
    } catch (const std::exception &) {
      // 一个后端发现失败不应遮蔽其他相机。
    }
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
    try {
      std::lock_guard<std::recursive_mutex> sdk_lock(realsense_mutex_);
      if (!realsense_context_) {realsense_context_ = std::make_unique<rs2::context>();}
      for (auto && device : realsense_context_->query_devices()) {
        const std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
        result.push_back({"realsense:" + serial + ":rgb", name, "realsense", "rgb"});
        result.push_back({"realsense:" + serial + ":ir-left", name, "realsense", "ir-left"});
        result.push_back({"realsense:" + serial + ":ir-right", name, "realsense", "ir-right"});
        result.push_back({"realsense:" + serial + ":depth", name, "realsense", "depth"});
      }
    } catch (const std::exception &) {
    }
#endif
    for (int i = 63; i >= 0; --i) {
      const std::string path = "/dev/video" + std::to_string(i);
      if (!std::filesystem::exists(path)) {continue;}
      FILE * pipe = popen(
        ("v4l2-ctl -d " + path + " --list-formats 2>/dev/null").c_str(), "r");
      if (!pipe) {continue;}
      bool color = false;
      char buffer[128];
      while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        const std::string line(buffer);
        if (line.find("MJPG") != std::string::npos || line.find("YUYV") != std::string::npos ||
          line.find("RGB") != std::string::npos)
        {
          color = true;
          break;
        }
      }
      pclose(pipe);
      if (!color) {continue;}
      std::string name = "V4L2 Camera";
      std::ifstream file("/sys/class/video4linux/video" + std::to_string(i) + "/name");
      std::getline(file, name);
      result.push_back({"v4l2:" + path + ":rgb", name.empty() ? "V4L2 Camera" : name,
        "v4l2", "rgb"});
    }
    const auto topic = get_parameter("topic_input.name").as_string();
    const auto cloud_topic = get_parameter("topic_input.pointcloud_name").as_string();
    const std::string name = "ROS Camera";
    result.push_back({"ros-topic:configured:rgb", name, "ros-topic", "rgb"});
    result.push_back({"ros-topic:configured:depth", name, "ros-topic", "depth"});
    return result;
  }

  void setStreaming(bool enable, std_srvs::srv::SetBool::Response & response)
  {
    if (!enable) {
      if (!streaming_ && !release_requested_) {
        response.success = true;
        response.message = "相机已经关闭";
        return;
      }
      bool driver_stopped = true;
      if (topic_control_pub_) {
        std_msgs::msg::Bool command;
        command.data = false;
        uint64_t previous;
        {
          std::lock_guard<std::mutex> status_lock(topic_control_mutex_);
          previous = topic_control_sequence_;
        }
        topic_control_pub_->publish(command);
        std::unique_lock<std::mutex> status_lock(topic_control_mutex_);
        driver_stopped = topic_control_cv_.wait_for(status_lock, 3s,
          [this, previous]() {
            return topic_control_sequence_ > previous &&
              (topic_control_status_ == "stopped" || topic_control_status_ == "disconnected");
          });
      }
      streaming_ = false;
      release_requested_ = true;
      std::unique_lock<std::mutex> lock(release_mutex_);
      const bool released = release_cv_.wait_for(
        lock, 3s, [this]() {return !release_requested_.load();});
      response.success = released;
      response.message = !driver_stopped ? "已停止本程序订阅，但驱动未确认停止，请检查控制状态话题" :
        (released ? "相机采流已关闭，设备参数连接仍可保留" :
        "相机关闭超时，请稍后重试");
      publishEvent(response.message);
      return;
    }
    if (streaming_) {
      response.success = true;
      response.message = "相机已经在运行";
      return;
    }
    if (release_requested_) {
      response.success = false;
      response.message = "相机正在释放，请稍后重试";
      return;
    }

    std::string device_id = get_parameter("device").as_string();
    if (device_id == "auto") {
      const auto devices = discoverDevices();
      device_id = devices.empty() ? "" : devices.front().id;
    }
    if (device_id.empty()) {
      response.success = false;
      response.message = "未发现可用相机或信号";
      return;
    }
    // 分辨率可由 UI 在连接前通过 ROS 参数修改；每次打开设备时重新读取。
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    disable_ir_emitter_ = get_parameter("disable_ir_emitter").as_bool();
    selection_ = parseSelection(device_id);
    signals_ = get_parameter("signals").as_string_array();
    if (signals_.empty()) {signals_.push_back(selection_.signal);}
    if (selection_.backend == "v4l2") {signals_ = {"rgb"};}
    if (selection_.backend == "ros-topic") {
      signals_.erase(std::remove_if(signals_.begin(), signals_.end(),
        [](const std::string & signal) {return signal != "rgb" && signal != "depth";}),
        signals_.end());
    }
    if (selection_.backend != "ros-topic" &&
      std::find(signals_.begin(), signals_.end(), selection_.signal) == signals_.end()) {
      signals_.insert(signals_.begin(), selection_.signal);
    }
    disconnect_notified_ = false;
    std::string error;
    if (!openSelectedCamera(error)) {
      releaseCameraResources();
      response.success = false;
      response.message = error;
      return;
    }
    uint64_t control_initial_sequence = 0;
    if (topic_control_pub_) {
      const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
      while (topic_control_pub_->get_subscription_count() == 0 &&
        std::chrono::steady_clock::now() < discovery_deadline) {
        std::this_thread::sleep_for(20ms);
      }
      if (topic_control_pub_->get_subscription_count() == 0) {
        releaseCameraResources();
        response.success = false;
        response.message = "驱动命令话题没有订阅者；请启动实现控制协议的适配节点";
        return;
      }
      {
        std::lock_guard<std::mutex> status_lock(topic_control_mutex_);
        control_initial_sequence = topic_control_sequence_;
      }
      std_msgs::msg::Bool command;
      command.data = true;
      topic_control_pub_->publish(command);
      std::unique_lock<std::mutex> status_lock(topic_control_mutex_);
      const bool started = topic_control_cv_.wait_for(status_lock, 3s,
        [this, control_initial_sequence]() {
          return topic_control_sequence_ > control_initial_sequence &&
            (topic_control_status_ == "streaming" ||
            topic_control_status_ == "disconnected" ||
            topic_control_status_.rfind("error:", 0) == 0);
        });
      const auto status = topic_control_status_;
      status_lock.unlock();
      if (!started || status != "streaming") {
        std_msgs::msg::Bool stop_command;
        stop_command.data = false;
        topic_control_pub_->publish(stop_command);
        releaseCameraResources();
        response.success = false;
        response.message = started ? "驱动启动失败: " + status :
          "驱动 3 秒内未确认 streaming 状态";
        return;
      }
    }
    uint64_t initial_sequence;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      initial_sequence = frame_sequence_;
    }
    streaming_ = true;
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      const bool received_frame = frame_cv_.wait_for(lock, 5s,
        [this, initial_sequence]() {
          const bool ready = selection_.backend == "ros-topic" ?
            (!latest_frame_.empty() || !latest_cloud_.empty()) : !latest_frame_.empty();
          return !streaming_ || (frame_sequence_ > initial_sequence && ready);
        }) && streaming_ &&
        (selection_.backend == "ros-topic" ?
        (!latest_frame_.empty() || !latest_cloud_.empty()) : !latest_frame_.empty());
      if (!received_frame) {
        lock.unlock();
        if (topic_control_pub_) {
          std_msgs::msg::Bool stop_command;
          stop_command.data = false;
          topic_control_pub_->publish(stop_command);
        }
        streaming_ = false;
        release_requested_ = true;
        std::unique_lock<std::mutex> release_lock(release_mutex_);
        release_cv_.wait_for(release_lock, 3s, [this]() {
          return !release_requested_.load();
        });
        response.success = false;
        response.message = selection_.backend == "ros-topic" ?
          "5 秒内未收到 ROS 图像或点云；请检查话题名、网络发现、QoS 和发布端" :
          "相机未收到有效画面或设备已断开；请检查流格式、分辨率和 USB 连接";
        publishEvent(response.message);
        return;
      }
    }
    std::vector<std::string> received_signals;
    if (selection_.backend == "ros-topic") {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      if (signals_.size() > 1) {
        frame_cv_.wait_for(lock, 750ms, [this]() {
          return !latest_frame_.empty() && !latest_cloud_.empty();
        });
      }
      if (!latest_frame_.empty()) {received_signals.push_back("rgb");}
      if (!latest_cloud_.empty()) {received_signals.push_back("depth");}
      selection_.signal = !latest_frame_.empty() ? "rgb" : "depth";
    }
    response.success = true;
    response.message = "相机已启动: " + selection_.backend + "/" + selection_.signal;
    if (selection_.backend == "ros-topic") {
      response.message += "，可用信息流 ";
      for (size_t i = 0; i < received_signals.size(); ++i) {
        if (i != 0) {response.message += ",";}
        response.message += received_signals[i];
      }
    }
    if (selection_.signal != "depth") {
      response.message += "，实际分辨率 " + std::to_string(width_) + "x" +
        std::to_string(height_);
    }
    if (isInfraredSignal() && disable_ir_emitter_) {
      response.message += "，红外投射器已关闭（纯灰度）";
    }
    publishEvent(response.message);
    publishState("streaming", response.message);
  }

  void setEmitterDisabled(bool disable, std_srvs::srv::SetBool::Response & response)
  {
    const std::string backend = streaming_ ? selection_.backend :
      parseSelection(get_parameter("device").as_string()).backend;
    if (backend != "orbbec" && backend != "realsense") {
      response.message = "当前相机不支持红外投射器开关";
      return;
    }
    try {
      if (streaming_) {
#ifdef DANMUCAMERA_HAS_ORBBEC
        if (backend == "orbbec") {
          std::lock_guard<std::recursive_mutex> lock(orbbec_mutex_);
          if (!setOrbbecEmitterDisabled(disable, response.message)) {return;}
        }
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
        if (backend == "realsense") {
          std::lock_guard<std::recursive_mutex> lock(realsense_mutex_);
          if (!realsense_device_) {
            response.message = "RealSense 相机尚未打开";
            return;
          }
          auto sensor = realsense_device_->first<rs2::depth_sensor>();
          if (!sensor || !sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
            response.message = "此 RealSense 相机不支持红外投射器开关";
            return;
          }
          const float current = sensor.get_option(RS2_OPTION_EMITTER_ENABLED);
          if (!realsense_emitter_previous_) {realsense_emitter_previous_ = current;}
          sensor.set_option(RS2_OPTION_EMITTER_ENABLED, disable ? 0.0f : 1.0f);
          if ((sensor.get_option(RS2_OPTION_EMITTER_ENABLED) == 0.0f) != disable) {
            response.message = "红外投射器状态回读与设置不一致";
            return;
          }
          realsense_depth_sensor_ = sensor;
        }
#endif
      }
      const auto result = set_parameter(rclcpp::Parameter("disable_ir_emitter", disable));
      if (!result.successful) {
        response.message = "保存红外投射器设置失败: " + result.reason;
        return;
      }
      disable_ir_emitter_ = disable;
      response.success = true;
      response.message = disable ? "红外投射器已关闭" : "红外投射器已开启";
      if (!streaming_) {response.message += "，启动相机后生效";}
      publishEvent(response.message);
    } catch (const std::exception & exception) {
      response.message = std::string("切换红外投射器失败: ") + exception.what();
    }
  }

#ifdef DANMUCAMERA_HAS_ORBBEC
  bool setOrbbecEmitterDisabled(bool disable, std::string & error)
  {
    if (!orbbec_device_) {
      error = "Orbbec 相机尚未打开";
      return false;
    }
    if (orbbec_device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      if (!orbbec_emitter_previous_) {
        orbbec_emitter_previous_ = orbbec_device_->getBoolProperty(OB_PROP_LASER_BOOL);
      }
      orbbec_device_->setBoolProperty(OB_PROP_LASER_BOOL, !disable);
      if (orbbec_device_->getBoolProperty(OB_PROP_LASER_BOOL) != !disable) {
        error = "红外投射器状态回读与设置不一致";
        return false;
      }
      return true;
    }
    // Gemini 335 等新机型使用三态控制：0 关闭、1 开启、2 自动。
    if (orbbec_device_->isPropertySupported(
      OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      if (!orbbec_emitter_control_previous_) {
        orbbec_emitter_control_previous_ =
          orbbec_device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
      }
      orbbec_device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, disable ? 0 : 1);
      if (orbbec_device_->getIntProperty(OB_PROP_LASER_CONTROL_INT) != (disable ? 0 : 1)) {
        error = "红外投射器状态回读与设置不一致";
        return false;
      }
      return true;
    }
    error = "此 Orbbec 相机不支持红外投射器开关";
    return false;
  }
#endif

  bool openSelectedCamera(std::string & error)
  {
    try {
      if (selection_.backend == "ros-topic") {
        const auto topic = get_parameter("topic_input.name").as_string();
        const auto cloud_topic = get_parameter("topic_input.pointcloud_name").as_string();
        auto type = get_parameter("topic_input.type").as_string();
        const auto command_topic = get_parameter(
          contract::extrinsic_input::kCommandParameter).as_string();
        const auto status_topic = get_parameter(
          contract::extrinsic_input::kStatusParameter).as_string();
        if (command_topic.empty() != status_topic.empty() ||
          (!command_topic.empty() && (command_topic.front() != '/' ||
          status_topic.front() != '/' || command_topic == status_topic))) {
          error = "驱动控制的命令/状态话题须同时填写不同的完整绝对名称，或同时留空";
          return false;
        }
        const bool request_image = std::find(signals_.begin(), signals_.end(), "rgb") != signals_.end();
        const bool request_cloud = std::find(signals_.begin(), signals_.end(), "depth") != signals_.end();
        if (request_image && (topic == contract::kImageTopic ||
          topic == contract::kStreamImageTopic || topic == contract::kCapturedImageTopic)) {
          error = "输入话题不能是本程序发布的图像话题，否则会循环接收";
          return false;
        }
        if (request_cloud && cloud_topic == contract::kStreamCloudTopic) {
          error = "点云输入不能是本程序发布的点云话题";
          return false;
        }
        if (request_image && type != "auto" && type != "raw" && type != "compressed") {
          error = "topic_input.type 只能是 auto、raw 或 compressed";
          return false;
        }
        const auto topics = get_topic_names_and_types();
        const auto has_type = [&topics](const std::string & name, const char * message_type) {
            const auto found = topics.find(name);
            return found != topics.end() && std::find(found->second.begin(),
              found->second.end(), message_type) != found->second.end();
          };
        bool use_image = request_image && !topic.empty() && topic.front() == '/';
        bool use_cloud = request_cloud && !cloud_topic.empty() && cloud_topic.front() == '/';
        if (use_image && type == "auto") {
          if (has_type(topic, contract::extrinsic_input::kCompressedImageType)) {
            type = "compressed";
          } else if (has_type(topic, contract::extrinsic_input::kRawImageType)) {
            type = "raw";
          } else if (command_topic.empty()) {use_image = false;}
        } else if (use_image) {
          use_image = !command_topic.empty() || has_type(topic, type == "raw" ?
            contract::extrinsic_input::kRawImageType :
            contract::extrinsic_input::kCompressedImageType);
        }
        if (use_cloud) {
          use_cloud = !command_topic.empty() ||
            has_type(cloud_topic, contract::extrinsic_input::kCloudType);
        }
        if (!use_image && !use_cloud) {
          error = "ROS 图像与点云均未配置或未发现可用发布端；至少需要一路有效话题";
          return false;
        }
        if (request_image && !use_image) {
          RCLCPP_WARN(get_logger(), "图像话题不可用，继续使用可用的点云话题");
        }
        if (request_cloud && !use_cloud) {
          RCLCPP_WARN(get_logger(), "点云话题不可用，继续使用可用的图像话题");
        }
        signals_.clear();
        if (use_image) {signals_.push_back("rgb");}
        if (use_cloud) {signals_.push_back("depth");}
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          latest_frame_.release();
          latest_stream_frames_.clear();
          latest_cloud_.clear();
          topic_last_frame_time_ = {};
        }
        rclcpp::SubscriptionOptions options;
        options.callback_group = topic_callback_group_;
        if (!command_topic.empty()) {
          topic_control_pub_ = create_publisher<std_msgs::msg::Bool>(
            command_topic, rclcpp::QoS(1).reliable());
          topic_control_sub_ = create_subscription<std_msgs::msg::String>(
            status_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](std_msgs::msg::String::ConstSharedPtr message) {
              std::lock_guard<std::mutex> lock(topic_control_mutex_);
              topic_control_status_ = message->data;
              ++topic_control_sequence_;
              topic_control_cv_.notify_all();
            }, options);
        }
        const auto qos = rclcpp::SensorDataQoS().keep_last(1);
        if (use_image && (type == "compressed" || type == "auto")) {
          topic_compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            topic, qos,
            [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
              const cv::Mat encoded(1, static_cast<int>(message->data.size()), CV_8UC1,
                const_cast<uint8_t *>(message->data.data()));
              receiveTopicFrame(cv::imdecode(encoded, cv::IMREAD_UNCHANGED));
            }, options);
        }
        if (use_image && (type == "raw" || type == "auto")) {
          topic_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
            topic, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
              receiveRawTopicFrame(*message);
            }, options);
        }
        if (use_cloud) {
          topic_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
              receiveTopicCloud(*message);
            }, options);
        }
        topic_input_active_ = true;
        return true;
      }
      if (selection_.backend == "v4l2") {
        if (!capture_device_.open(selection_.address, cv::CAP_V4L2)) {
          error = "无法通过 V4L2 打开相机: " + selection_.address;
          return false;
        }
        capture_device_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        capture_device_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        capture_device_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        capture_device_.set(cv::CAP_PROP_FPS, camera_fps_);
        capture_device_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        width_ = static_cast<int>(capture_device_.get(cv::CAP_PROP_FRAME_WIDTH));
        height_ = static_cast<int>(capture_device_.get(cv::CAP_PROP_FRAME_HEIGHT));
        return true;
      }
#ifdef DANMUCAMERA_HAS_ORBBEC
      if (selection_.backend == "orbbec") {
        std::lock_guard<std::recursive_mutex> sdk_lock(orbbec_mutex_);
        const auto started_at = std::chrono::steady_clock::now();
        if (!ob_context_) {ob_context_ = std::make_unique<ob::Context>();}
        const auto context_ready_at = std::chrono::steady_clock::now();
        ob_context_->setDeviceChangedCallback(
          [this](std::shared_ptr<ob::DeviceList> removed,
            std::shared_ptr<ob::DeviceList>) {
            if (!removed) {return;}
            for (uint32_t i = 0; i < removed->getCount(); ++i) {
              if (selection_.backend == "orbbec" &&
                selection_.address == removed->getSerialNumber(i))
              {
                std::lock_guard<std::recursive_mutex> lock(orbbec_mutex_);
                notifyHardwareDisconnected(
                  "Orbbec SDK 检测到 Gemini 335 已拔出，已恢复为未连接状态");
                // 按 Orbbec 官方 hot_plugin 示例处理：设备已经消失时不再调用 stop()，
                // 而是在设备回调中直接释放 pipeline/device，避免阻塞在普通停流路径。
                orbbec_pipeline_.reset();
                orbbec_device_.reset();
                orbbec_emitter_previous_.reset();
                orbbec_emitter_control_previous_.reset();
                return;
              }
            }
          });
        auto devices = ob_context_->queryDeviceList();
        const auto enumerated_at = std::chrono::steady_clock::now();
        orbbec_device_ = devices->getDeviceBySN(selection_.address.c_str());
        const auto device_opened_at = std::chrono::steady_clock::now();
        if (disable_ir_emitter_) {
          if (!setOrbbecEmitterDisabled(true, error)) {return false;}
        }
        orbbec_pipeline_ = std::make_unique<ob::Pipeline>(orbbec_device_);
        auto config = std::make_shared<ob::Config>();
        for (const auto & signal : signals_) {
          const OBStreamType stream = signal == "ir-left" ? OB_STREAM_IR_LEFT :
            signal == "ir-right" ? OB_STREAM_IR_RIGHT :
            signal == "depth" ? OB_STREAM_DEPTH : OB_STREAM_COLOR;
          const OBFormat format = signal == "depth" ? OB_FORMAT_Y16 :
            signal == "rgb" ? OB_FORMAT_RGB : OB_FORMAT_Y8;
          const bool primary = signal == selection_.signal;
          config->enableVideoStream(
            stream, primary ? width_ : OB_WIDTH_ANY,
            primary ? height_ : OB_HEIGHT_ANY,
            static_cast<uint32_t>(camera_fps_), format);
        }
        orbbec_pipeline_->start(config);
        const auto stream_started_at = std::chrono::steady_clock::now();
        const auto millis = [](auto from, auto to) -> long long {
            return std::chrono::duration_cast<std::chrono::milliseconds>(to - from).count();
          };
        RCLCPP_INFO(
          get_logger(),
          "Gemini 启动分段耗时: Context %lld ms, 枚举 %lld ms, 打开设备 %lld ms, Pipeline %lld ms, 总计 %lld ms",
          millis(started_at, context_ready_at), millis(context_ready_at, enumerated_at),
          millis(enumerated_at, device_opened_at), millis(device_opened_at, stream_started_at),
          millis(started_at, stream_started_at));
        return true;
      }
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
      if (selection_.backend == "realsense") {
        std::lock_guard<std::recursive_mutex> sdk_lock(realsense_mutex_);
        if (!realsense_context_) {realsense_context_ = std::make_unique<rs2::context>();}
        realsense_pipeline_ = std::make_unique<rs2::pipeline>(*realsense_context_);
        rs2::config config;
        config.enable_device(selection_.address);
        for (const auto & signal : signals_) {
          const bool primary = signal == selection_.signal;
          const int stream_width = primary ? width_ : 0;
          const int stream_height = primary ? height_ : 0;
          if (signal == "rgb") {
            config.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_BGR8,
              static_cast<int>(camera_fps_));
          } else if (signal == "depth") {
            config.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16,
              static_cast<int>(camera_fps_));
          } else {
            const int index = signal == "ir-right" ? 2 : 1;
            config.enable_stream(RS2_STREAM_INFRARED, index, stream_width, stream_height, RS2_FORMAT_Y8,
              static_cast<int>(camera_fps_));
          }
        }
        const auto profile = realsense_pipeline_->start(config);
        realsense_device_ = profile.get_device();
        realsense_context_->set_devices_changed_callback(
          [this](rs2::event_information & information) {
            std::lock_guard<std::recursive_mutex> lock(realsense_mutex_);
            if (!realsense_device_.has_value() ||
              !information.was_removed(*realsense_device_))
            {
              return;
            }
            notifyHardwareDisconnected(
              "RealSense SDK 检测到 D415 已拔出，已恢复为未连接状态");
            // 设备已经消失，不再等待普通 stop()；直接丢弃失效的 pipeline。
            realsense_pipeline_.reset();
            realsense_device_.reset();
            realsense_depth_sensor_.reset();
            realsense_emitter_previous_.reset();
          });
        if (disable_ir_emitter_) {
          auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
          if (!depth_sensor || !depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
            realsense_pipeline_->stop();
            realsense_pipeline_.reset();
            error = "D415 不支持发射器开关，不能作为纯灰度标定流";
            return false;
          }
          realsense_emitter_previous_ = depth_sensor.get_option(RS2_OPTION_EMITTER_ENABLED);
          depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.0f);
          if (depth_sensor.get_option(RS2_OPTION_EMITTER_ENABLED) != 0.0f) {
            realsense_pipeline_->stop();
            realsense_pipeline_.reset();
            error = "D415 红外投射器关闭后回读仍为开启，拒绝进入灰度标定";
            return false;
          }
          realsense_depth_sensor_ = depth_sensor;
        }
        return true;
      }
#endif
      error = "所选 SDK 后端未编译进程序: " + selection_.backend;
    } catch (const std::exception & exception) {
      error = "打开 " + selection_.backend + "/" + selection_.signal + " 失败: " + exception.what();
      // pipeline::start() 失败时 SDK 也可能已经打开部分 V4L2 节点。若只返回错误而
      // 保留对象，下一次启动会被本进程自己以 Device or resource busy 阻塞。
      releaseCameraResources();
    }
    return false;
  }

  bool isInfraredSignal() const
  {
    return selection_.signal == "ir-left" || selection_.signal == "ir-right";
  }

  void receiveTopicFrame(const cv::Mat & image)
  {
    if (!topic_input_active_ || !streaming_ || image.empty()) {return;}
    cv::Mat frame;
    if (image.depth() != CV_8U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ROS 图像解码后不是 8 位图像，请检查发布端编码");
      return;
    }
    if (image.channels() == 4) {
      cv::cvtColor(image, frame, cv::COLOR_BGRA2BGR);
    } else if (image.channels() == 1 || image.channels() == 3) {
      frame = image.clone();
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ROS 图像通道数不受支持: %d", image.channels());
      return;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!topic_input_active_ || !streaming_) {return;}
    latest_frame_ = frame;
    latest_stream_frames_["rgb"] = frame;
    width_ = frame.cols;
    height_ = frame.rows;
    topic_last_frame_time_ = std::chrono::steady_clock::now();
    ++frame_sequence_;
    frame_cv_.notify_all();
  }

  void receiveRawTopicFrame(const sensor_msgs::msg::Image & message)
  {
    int type = -1;
    if (message.encoding == "bgr8" || message.encoding == "rgb8") {type = CV_8UC3;}
    else if (message.encoding == "bgra8" || message.encoding == "rgba8") {type = CV_8UC4;}
    else if (message.encoding == "mono8" || message.encoding == "8UC1") {type = CV_8UC1;}
    else if (message.encoding == "mono16" || message.encoding == "16UC1") {type = CV_16UC1;}
    if (type < 0 || message.width == 0 || message.height == 0 ||
      message.step < message.width * CV_ELEM_SIZE(type) ||
      message.data.size() < static_cast<size_t>(message.height) * message.step ||
      (type == CV_16UC1 && message.is_bigendian))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ROS 原始图像编码或缓冲区不受支持: %s", message.encoding.c_str());
      return;
    }
    const cv::Mat source(static_cast<int>(message.height), static_cast<int>(message.width),
      type, const_cast<uint8_t *>(message.data.data()), message.step);
    cv::Mat frame;
    if (message.encoding == "rgb8") {cv::cvtColor(source, frame, cv::COLOR_RGB2BGR);}
    else if (message.encoding == "rgba8") {cv::cvtColor(source, frame, cv::COLOR_RGBA2BGR);}
    else if (message.encoding == "bgra8") {cv::cvtColor(source, frame, cv::COLOR_BGRA2BGR);}
    else if (type == CV_16UC1) {source.convertTo(frame, CV_8UC1, 1.0 / 256.0);}
    else {frame = source;}
    receiveTopicFrame(frame);
  }

  void receiveTopicCloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    if (!topic_input_active_ || !streaming_ ||
      std::find(signals_.begin(), signals_.end(), "depth") == signals_.end()) {return;}
    int offsets[3] = {-1, -1, -1};
    for (const auto & field : message.fields) {
      const int index = field.name == "x" ? 0 : field.name == "y" ? 1 :
        field.name == "z" ? 2 : -1;
      if (index >= 0 && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1 && field.offset + sizeof(float) <= message.point_step) {
        offsets[index] = static_cast<int>(field.offset);
      }
    }
    if (offsets[0] < 0 || offsets[1] < 0 || offsets[2] < 0 || message.is_bigendian ||
      message.point_step == 0 || message.width == 0 || message.height == 0 ||
      message.row_step < message.width * message.point_step ||
      message.data.size() < static_cast<size_t>(message.height) * message.row_step) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "PointCloud2 需要小端 FLOAT32 的 x/y/z 字段及完整数据缓冲区");
      return;
    }
    const size_t total = static_cast<size_t>(message.width) * message.height;
    const size_t stride = std::max<size_t>(1, total / 12000);
    std::vector<cv::Vec3f> points;
    points.reserve(std::min<size_t>(total, 12000));
    for (size_t i = 0; i < total; i += stride) {
      const size_t row = i / message.width;
      const size_t col = i % message.width;
      const uint8_t * data = message.data.data() + row * message.row_step + col * message.point_step;
      cv::Vec3f point;
      for (int axis = 0; axis < 3; ++axis) {
        std::memcpy(&point[axis], data + offsets[axis], sizeof(float));
      }
      if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
        points.push_back(point);
      }
    }
    if (points.empty()) {return;}
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!topic_input_active_ || !streaming_) {return;}
    latest_cloud_ = std::move(points);
    topic_last_frame_time_ = std::chrono::steady_clock::now();
    ++frame_sequence_;
    frame_cv_.notify_all();
  }

  void notifyHardwareDisconnected(const std::string & detail)
  {
    if (disconnect_notified_.exchange(true)) {return;}
    streaming_ = false;
    release_requested_ = true;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_.release();
      frozen_frame_.release();
      frozen_from_import_ = false;
      ++frame_sequence_;
    }
    frame_cv_.notify_all();
    // 回调来自 SDK 内部线程；这里只改原子状态并发布消息，不在回调内 stop/reset SDK。
    publishEvent(detail);
    publishState("disconnected", detail);
  }

  void releaseCameraResources()
  {
    topic_input_active_ = false;
    topic_raw_sub_.reset();
    topic_compressed_sub_.reset();
    topic_cloud_sub_.reset();
    topic_control_sub_.reset();
    topic_control_pub_.reset();
    capture_device_.release();
#ifdef DANMUCAMERA_HAS_ORBBEC
    {
      std::lock_guard<std::recursive_mutex> sdk_lock(orbbec_mutex_);
      if (orbbec_pipeline_) {
        try {orbbec_pipeline_->stop();} catch (const std::exception &) {}
        orbbec_pipeline_.reset();
      }
      if (orbbec_device_ && orbbec_emitter_previous_.has_value()) {
        try {orbbec_device_->setBoolProperty(OB_PROP_LASER_BOOL, *orbbec_emitter_previous_);}
        catch (const std::exception &) {}
      }
      if (orbbec_device_ && orbbec_emitter_control_previous_.has_value()) {
        try {orbbec_device_->setIntProperty(
            OB_PROP_LASER_CONTROL_INT, *orbbec_emitter_control_previous_);}
        catch (const std::exception &) {}
      }
      orbbec_emitter_previous_.reset();
      orbbec_emitter_control_previous_.reset();
      orbbec_device_.reset();
    }
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
    {
      std::lock_guard<std::recursive_mutex> sdk_lock(realsense_mutex_);
      if (realsense_depth_sensor_.has_value() && realsense_emitter_previous_.has_value()) {
        try {
          realsense_depth_sensor_->set_option(
            RS2_OPTION_EMITTER_ENABLED, *realsense_emitter_previous_);
        } catch (const std::exception &) {}
      }
      if (realsense_pipeline_) {
        try {realsense_pipeline_->stop();} catch (const std::exception &) {}
        realsense_pipeline_.reset();
      }
      realsense_device_.reset();
      realsense_depth_sensor_.reset();
      realsense_emitter_previous_.reset();
    }
#endif
  }

  void captureLoop()
  {
    std::optional<std::chrono::steady_clock::time_point> read_failure_since;
    while (running_) {
      if (release_requested_) {
        releaseCameraResources();
        release_requested_ = false;
        release_cv_.notify_all();
        read_failure_since.reset();
      }
      if (!streaming_) {
        std::this_thread::sleep_for(20ms);
        continue;
      }
      if (topic_input_active_) {
        bool timed_out = false;
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          timed_out = (!latest_frame_.empty() || !latest_cloud_.empty()) &&
            std::chrono::steady_clock::now() - topic_last_frame_time_ >= 3s;
        }
        if (timed_out) {
          notifyHardwareDisconnected("连续 3 秒没有收到 ROS 主信息流，已恢复为未连接状态");
        }
        std::this_thread::sleep_for(20ms);
        continue;
      }
      std::map<std::string, cv::Mat> frames;
      if (!readSelectedFrames(frames) || frames[selection_.signal].empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (!read_failure_since.has_value()) {read_failure_since = now;}
        if (streaming_ && now - *read_failure_since >= 3s) {
          notifyHardwareDisconnected(
            "连续 3 秒没有收到有效画面，已恢复为未连接状态");
#ifdef DANMUCAMERA_HAS_ORBBEC
          if (selection_.backend == "orbbec") {
            std::lock_guard<std::recursive_mutex> sdk_lock(orbbec_mutex_);
            // 设备移除回调若因底层异常没有到达，也使用热拔插释放方式，
            // 不对已经消失的设备调用 pipeline.stop()。
            orbbec_pipeline_.reset();
            orbbec_device_.reset();
            orbbec_emitter_previous_.reset();
            orbbec_emitter_control_previous_.reset();
          }
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
          if (selection_.backend == "realsense") {
            std::lock_guard<std::recursive_mutex> sdk_lock(realsense_mutex_);
            realsense_pipeline_.reset();
            realsense_device_.reset();
            realsense_depth_sensor_.reset();
            realsense_emitter_previous_.reset();
          }
#endif
          read_failure_since.reset();
        }
        std::this_thread::sleep_for(10ms);
        continue;
      }
      read_failure_since.reset();
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = frames[selection_.signal];
      latest_stream_frames_ = std::move(frames);
      ++frame_sequence_;
      frame_cv_.notify_all();
    }
  }

  bool readSelectedFrames(std::map<std::string, cv::Mat> & outputs)
  {
    try {
      if (selection_.backend == "v4l2") {
        cv::Mat output;
        if (!capture_device_.read(output)) {return false;}
        outputs["rgb"] = std::move(output);
        return true;
      }
#ifdef DANMUCAMERA_HAS_ORBBEC
      if (selection_.backend == "orbbec") {
        std::lock_guard<std::recursive_mutex> sdk_lock(orbbec_mutex_);
        if (!orbbec_pipeline_) {return false;}
        const auto frames = orbbec_pipeline_->waitForFrames(100);
        if (!frames) {return false;}
        for (const auto & signal : signals_) {
          const OBFrameType type = signal == "ir-left" ? OB_FRAME_IR_LEFT :
            signal == "ir-right" ? OB_FRAME_IR_RIGHT :
            signal == "depth" ? OB_FRAME_DEPTH : OB_FRAME_COLOR;
          const auto frame = frames->getFrame(type);
          if (!frame) {continue;}
          const auto video = frame->as<ob::VideoFrame>();
          const int width = static_cast<int>(video->getWidth());
          const int height = static_cast<int>(video->getHeight());
          const auto format = frame->getFormat();
          cv::Mat output;
          if (format == OB_FORMAT_MJPG) {
            const cv::Mat bytes(1, static_cast<int>(frame->getDataSize()), CV_8UC1, frame->getData());
            output = cv::imdecode(bytes, cv::IMREAD_COLOR);
          } else if (format == OB_FORMAT_RGB || format == OB_FORMAT_BGR) {
            cv::Mat raw(height, width, CV_8UC3, frame->getData());
            if (format == OB_FORMAT_RGB) {cv::cvtColor(raw, output, cv::COLOR_RGB2BGR);}
            else {output = raw.clone();}
          } else if (format == OB_FORMAT_YUYV) {
            cv::Mat raw(height, width, CV_8UC2, frame->getData());
            cv::cvtColor(raw, output, cv::COLOR_YUV2BGR_YUY2);
          } else if (format == OB_FORMAT_Y8) {
            cv::Mat(height, width, CV_8UC1, frame->getData()).copyTo(output);
          } else if (format == OB_FORMAT_Y16 || format == OB_FORMAT_Z16) {
            cv::Mat raw(height, width, CV_16UC1, frame->getData());
            double max_value = 0.0;
            cv::minMaxLoc(raw, nullptr, &max_value);
            raw.convertTo(output, CV_8UC1, max_value > 0.0 ? 255.0 / max_value : 1.0);
            if (signal == "depth") {cv::applyColorMap(output, output, cv::COLORMAP_TURBO);}
          }
          if (!output.empty()) {outputs[signal] = std::move(output);}
        }
        return !outputs.empty();
      }
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
      if (selection_.backend == "realsense") {
        std::lock_guard<std::recursive_mutex> sdk_lock(realsense_mutex_);
        if (!realsense_pipeline_) {return false;}
        // D415 启动初期或 USB 抖动时，100 ms 内没帧是可恢复的短暂状态。
        // try_wait_for_frames 会用 false 表示超时，不会每 100 ms 抛异常并刷屏。
        // 250 ms 仍能保证关闭和热拔插快速取得 SDK 锁。
        rs2::frameset frames;
        if (!realsense_pipeline_->try_wait_for_frames(&frames, 250)) {return false;}
        for (const auto & signal : signals_) {
          rs2::frame selected_frame = signal == "rgb" ? frames.get_color_frame() :
            signal == "depth" ? frames.get_depth_frame() :
            frames.get_infrared_frame(signal == "ir-right" ? 2 : 1);
          if (!selected_frame) {continue;}
          const rs2::video_frame frame(selected_frame);
          const int width = frame.get_width();
          const int height = frame.get_height();
          cv::Mat output;
          if (signal == "rgb") {
            cv::Mat(height, width, CV_8UC3, const_cast<void *>(frame.get_data())).copyTo(output);
          } else if (signal == "depth") {
            cv::Mat raw(height, width, CV_16UC1, const_cast<void *>(frame.get_data()));
            raw.convertTo(output, CV_8UC1, 0.03);
            cv::applyColorMap(output, output, cv::COLORMAP_TURBO);
          } else {
            cv::Mat(height, width, CV_8UC1, const_cast<void *>(frame.get_data())).copyTo(output);
          }
          outputs[signal] = std::move(output);
        }
        return !outputs.empty();
      }
#endif
    } catch (const std::exception & exception) {
      // 真实 SDK 错误也做限频，避免采集线程在故障期间淹没 UI 日志。
      const auto now = std::chrono::steady_clock::now();
      const std::string detail = exception.what();
      if (detail != last_read_error_ ||
        now - last_read_error_log_time_ >= 1s)
      {
        publishEvent("读取相机失败: " + detail);
        last_read_error_ = detail;
        last_read_error_log_time_ = now;
      }
    }
    return false;
  }

  void publishLatestFrame()
  {
    cv::Mat frame;
    std::map<std::string, cv::Mat> streams;
    std::vector<cv::Vec3f> cloud;
    uint64_t sequence;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      sequence = frame_sequence_;
      if (!streaming_ || sequence == published_sequence_) {return;}
      frame = latest_frame_.clone();
      streams = latest_stream_frames_;
      cloud = latest_cloud_;
    }
    published_sequence_ = sequence;
    if (!frame.empty()) {publishImage(frame, image_pub_, true);}
    for (const auto & [signal, image] : streams) {
      if (!image.empty()) {publishImage(image, stream_image_pub_, true, signal);}
    }
    if (!cloud.empty() &&
      std::find(signals_.begin(), signals_.end(), "depth") != signals_.end()) {
      sensor_msgs::msg::PointCloud2 message;
      message.header.stamp = now();
      message.header.frame_id = "depth";
      message.height = 1;
      message.width = static_cast<uint32_t>(cloud.size());
      message.fields.resize(3);
      for (size_t axis = 0; axis < 3; ++axis) {
        message.fields[axis].name = std::string(1, "xyz"[axis]);
        message.fields[axis].offset = static_cast<uint32_t>(axis * sizeof(float));
        message.fields[axis].datatype = sensor_msgs::msg::PointField::FLOAT32;
        message.fields[axis].count = 1;
      }
      message.point_step = 3 * sizeof(float);
      message.row_step = message.width * message.point_step;
      message.data.resize(message.row_step);
      std::memcpy(message.data.data(), cloud.data(), message.data.size());
      message.is_dense = true;
      stream_cloud_pub_->publish(std::move(message));
    }
  }

  void capture(std_srvs::srv::Trigger::Response & response)
  {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!streaming_ || latest_frame_.empty()) {
        response.success = false;
        response.message = "相机尚未产生有效画面";
        return;
      }
      frozen_frame_ = latest_frame_.clone();
      frozen_from_import_ = false;
    }
    response.success = true;
    response.message = "已冻结当前帧（尚未保存）";
    publishImage(frozen_frame_, captured_pub_, false);
    publishEvent(response.message);
  }

  void save(std_srvs::srv::Trigger::Response & response)
  {
    cv::Mat frame;
    bool from_import = false;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (frozen_frame_.empty()) {
        response.success = false;
        response.message = "没有待保存画面，请先拍摄";
        return;
      }
      frame = frozen_frame_.clone();
      from_import = frozen_from_import_;
    }
    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error) {
      response.success = false;
      response.message = "无法创建目录: " + error.message();
      return;
    }
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto path = std::filesystem::absolute(output_directory_) /
      ((from_import ? std::string("imported") : selection_.backend + "_" + selection_.signal) +
      "_" + std::to_string(timestamp) + ".jpg");
    response.success = cv::imwrite(
      path.string(), frame, {cv::IMWRITE_JPEG_QUALITY, 100});
    response.message = response.success ? path.string() : "图像写入失败";
    publishEvent(response.success ? "已保存: " + path.string() : response.message);
  }

  void listDevices(std_srvs::srv::Trigger::Response & response)
  {
    const auto devices = discoverDevices();
    response.success = !devices.empty();
    for (const auto & device : devices) {
      response.message += device.id + "|" + device.name + "|" + device.backend + "|" +
        device.signal + "\n";
    }
    if (devices.empty()) {response.message = "未发现相机；请检查 USB、SDK 和 udev 权限";}
  }

  void publishImage(
    const cv::Mat & frame,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher,
    bool preview, const std::string & frame_id = "calibration_camera")
  {
    cv::Mat output = frame;
    if (preview && preview_width_ > 0 && frame.cols > preview_width_) {
      const double scale = static_cast<double>(preview_width_) / frame.cols;
      cv::resize(frame, output, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = now();
    message.header.frame_id = frame_id;
    message.format = "jpeg";
    cv::imencode(".jpg", output, message.data, {cv::IMWRITE_JPEG_QUALITY,
      preview ? jpeg_quality_ : 100});
    publisher->publish(std::move(message));
  }

  void publishFile(
    const std::string & path,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher)
  {
    const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty()) {
      publishEvent("无法读取图片: " + path);
      return;
    }
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = now();
    message.header.frame_id = path;
    // 结果查看使用无损传输，避免已有 JPEG 再次编码产生新的压缩损失。
    message.format = "png";
    cv::imencode(".png", image, message.data, {cv::IMWRITE_PNG_COMPRESSION, 3});
    publisher->publish(std::move(message));
  }

  void importImage(const std::string & path)
  {
    const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty()) {
      const std::string reason = "无法读取图片";
      publishEvent("选图失败，" + reason + ": " + path);
      publishImportStatus("failed", path, reason);
      return;
    }
    if (image.cols != width_ || image.rows != height_) {
      const std::string reason = "图片分辨率 " + std::to_string(image.cols) + "x" +
        std::to_string(image.rows) + " 与当前标定分辨率 " + std::to_string(width_) + "x" +
        std::to_string(height_) + " 不一致";
      publishEvent("选图失败：" + reason);
      publishImportStatus("failed", path, reason);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frozen_frame_ = image.clone();
      frozen_from_import_ = true;
    }
    publishImage(image, captured_pub_, false);
    publishEvent("已载入待标定图片（尚未保存）: " + path);
    publishImportStatus("loaded", path, "已载入并送往角点识别");
  }

  void publishImportStatus(
    const std::string & state, const std::string & path, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = state + "\t" + path + "\t" + detail;
    import_status_pub_->publish(message);
  }

  void deleteSample(const std::string & path)
  {
    const auto samples = std::filesystem::absolute(output_directory_).lexically_normal();
    const auto target = std::filesystem::absolute(path).lexically_normal();
    const std::string prefix = samples.string() + std::string(1, std::filesystem::path::preferred_separator);
    if (target.string().rfind(prefix, 0) != 0 || target.extension() != ".jpg") {
      publishEvent("拒绝删除样本目录之外的文件");
      return;
    }
    std::error_code error;
    const bool removed = std::filesystem::remove(target, error);
    publishEvent(removed ? "已删除: " + target.string() : "删除失败: " + error.message());
  }

  void publishEvent(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    event_pub_->publish(message);
  }

  void publishState(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = state + "\t" + detail;
    state_pub_->publish(message);
  }

  std::atomic_bool running_;//原子布尔，防止数据竞争导致的(bool)读取错误
  std::atomic_bool streaming_;
  std::atomic_bool release_requested_;
  std::atomic_bool disconnect_notified_{false};
  std::atomic_bool topic_input_active_{false};
  int instance_lock_fd_{-1};
  std::mutex release_mutex_;
  std::condition_variable release_cv_;
  std::thread worker_;
  cv::VideoCapture capture_device_;  // 仅采集线程读取。
  CameraSelection selection_{"v4l2", "", "rgb"};
  std::vector<std::string> signals_{"rgb"};
  bool disable_ir_emitter_{true};
#ifdef DANMUCAMERA_HAS_ORBBEC
  std::recursive_mutex orbbec_mutex_;
  std::unique_ptr<ob::Context> ob_context_;
  std::shared_ptr<ob::Device> orbbec_device_;
  std::unique_ptr<ob::Pipeline> orbbec_pipeline_;
  std::optional<bool> orbbec_emitter_previous_;
  std::optional<int> orbbec_emitter_control_previous_;
#endif
#ifdef DANMUCAMERA_HAS_REALSENSE
  std::recursive_mutex realsense_mutex_;
  std::unique_ptr<rs2::context> realsense_context_;
  std::unique_ptr<rs2::pipeline> realsense_pipeline_;
  std::optional<rs2::device> realsense_device_;
  std::optional<rs2::depth_sensor> realsense_depth_sensor_;
  std::optional<float> realsense_emitter_previous_;
#endif
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  cv::Mat latest_frame_;
  std::map<std::string, cv::Mat> latest_stream_frames_;
  std::vector<cv::Vec3f> latest_cloud_;
  cv::Mat frozen_frame_;
  bool frozen_from_import_{false};
  uint64_t frame_sequence_{0};
  uint64_t published_sequence_{0};
  std::string output_directory_;
  int width_;
  int height_;
  double camera_fps_;
  double preview_fps_;
  int preview_width_;
  int jpeg_quality_;
  std::string last_read_error_;
  std::chrono::steady_clock::time_point last_read_error_log_time_{};
  std::chrono::steady_clock::time_point topic_last_frame_time_{};
  rclcpp::CallbackGroup::SharedPtr topic_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr topic_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr topic_compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr topic_cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr topic_control_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_control_sub_;
  std::mutex topic_control_mutex_;
  std::condition_variable topic_control_cv_;
  std::string topic_control_status_;
  uint64_t topic_control_sequence_{0};
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr stream_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stream_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr captured_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr viewed_original_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr viewed_error_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr viewed_undistorted_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr import_status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emitter_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr devices_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr view_original_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr view_error_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr view_undistorted_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr import_image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delete_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace danmucamera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<danmucamera::IoNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "相机 I/O 节点启动失败: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
