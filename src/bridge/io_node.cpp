#include "ros_contract.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
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
// This process owns no camera hardware. Camera drivers run separately and
// publish standard ROS messages. The services below preserve the UI contract.
class IoNode : public rclcpp::Node
{
public:
  IoNode() : Node("calibration_io")
  {
    lock_fd_ = ::open("/tmp/danmucamera_io_node.lock", O_CREAT | O_RDWR, 0666);
    if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      if (lock_fd_ >= 0) {::close(lock_fd_);}
      throw std::runtime_error("已有一个标定 I/O 节点在运行");
    }
    declare_parameter<std::string>(contract::extrinsic_input::kImageParameter, "");
    declare_parameter<std::string>(contract::extrinsic_input::kCloudParameter, "");
    declare_parameter<std::string>(contract::extrinsic_input::kCommandParameter, "");
    declare_parameter<std::string>(contract::extrinsic_input::kStatusParameter, "");
    output_directory_ = declare_parameter<std::string>("output_directory", "samples");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    preview_width_ = declare_parameter<int>("preview_width", 1280);
    preview_fps_ = declare_parameter<double>("preview_fps", 15.0);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 90);
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kImageTopic, sensor_qos);
    stream_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kStreamImageTopic, sensor_qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      contract::kStreamCloudTopic, sensor_qos);
    captured_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kCapturedImageTopic, rclcpp::QoS(1).reliable());
    original_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kViewedOriginalTopic, rclcpp::QoS(1).reliable());
    error_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kViewedErrorTopic, rclcpp::QoS(1).reliable());
    undistorted_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kViewedUndistortedTopic, rclcpp::QoS(1).reliable());
    event_pub_ = create_publisher<std_msgs::msg::String>(contract::kCameraEventTopic, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(contract::kCameraStateTopic, 10);
    import_pub_ = create_publisher<std_msgs::msg::String>(contract::kImportImageStatusTopic, 10);

    stream_service_ = create_service<std_srvs::srv::SetBool>(
      contract::kStreamingService,
      [this](std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        setStreaming(request->data, *response);
      });
    emitter_service_ = create_service<std_srvs::srv::SetBool>(
      contract::kEmitterService,
      [](std_srvs::srv::SetBool::Request::SharedPtr,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        response->success = false;
        response->message = "投射器由外部驱动管理";
      });
    capture_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kCaptureService,
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {capture(*response);});
    save_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kSaveService,
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {save(*response);});
    discovery_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kRefreshTopicsService,
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {discoverTopics(*response);});
    original_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kViewOriginalRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        publishFile(message->data, original_pub_);
      });
    error_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kViewErrorRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        publishFile(message->data, error_pub_);
      });
    undistorted_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kViewUndistortedRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        publishFile(message->data, undistorted_pub_);
      });
    import_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kImportImageRequestTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {importImage(message->data);});
    delete_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kDeleteSampleTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {deleteSample(message->data);});
    preview_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, preview_fps_))),
      [this]() {publishPreview();});
    watchdog_timer_ = create_wall_timer(500ms, [this]() {checkTimeout();});
    RCLCPP_INFO(get_logger(), "Topic I/O 节点就绪；相机驱动运行在独立进程");
  }

  ~IoNode() override
  {
    streaming_ = false;
    if (lock_fd_ >= 0) {::flock(lock_fd_, LOCK_UN); ::close(lock_fd_);}
  }

private:
  static bool hasType(
    const std::map<std::string, std::vector<std::string>> & topics,
    const std::string & name, const char * type)
  {
    const auto found = topics.find(name);
    return found != topics.end() &&
      std::find(found->second.begin(), found->second.end(), type) != found->second.end();
  }

  static bool isInternal(const std::string & name)
  {
    static const std::set<std::string> internal{
      contract::kImageTopic, contract::kStreamImageTopic,
      contract::kStreamCloudTopic, contract::kCapturedImageTopic,
      contract::kCameraEventTopic, contract::kCameraStateTopic,
      contract::kCornerPreviewTopic, contract::kQualityResultTopic,
      contract::kQualityRequestTopic, contract::kCornerStatusTopic,
      contract::kGuideResultTopic, contract::kAlgorithmStatusTopic,
      contract::kAlgorithmResultTopic, contract::kAlgorithmReportTopic,
      contract::kViewOriginalRequestTopic, contract::kViewUndistortedRequestTopic,
      contract::kViewedUndistortedTopic, contract::kViewErrorRequestTopic,
      contract::kImportImageRequestTopic, contract::kImportImageStatusTopic,
      contract::kViewedOriginalTopic, contract::kViewedErrorTopic,
      contract::kDeleteSampleTopic};
    return internal.count(name) != 0;
  }

  void discoverTopics(std_srvs::srv::Trigger::Response & response)
  {
    const auto topics = get_topic_names_and_types();
    for (const auto & [name, types] : topics) {
      if (isInternal(name)) {continue;}
      const bool has_publisher = count_publishers(name) > 0;
      const bool has_subscriber = count_subscribers(name) > 0;
      const auto emit = [&](const char * type, const char * label, const char * role) {
          if (std::find(types.begin(), types.end(), type) != types.end() &&
            (std::string(role) == "command" ? has_subscriber : has_publisher)) {
            response.message += name + "|" + name + " [" + label + "]|" + role + "\n";
          }
        };
      emit(contract::extrinsic_input::kRawImageType, "Image", "rgb");
      emit(contract::extrinsic_input::kCompressedImageType, "CompressedImage", "rgb");
      emit(contract::extrinsic_input::kCloudType, "PointCloud2", "depth");
      emit(contract::extrinsic_input::kCameraInfoType, "CameraInfo", "camera_info");
      emit("std_msgs/msg/Bool", "Bool", "command");
      emit("std_msgs/msg/String", "String", "status");
    }
    response.success = true;
  }

  void clearSubscriptions()
  {
    raw_sub_.reset();
    compressed_sub_.reset();
    cloud_sub_.reset();
    control_sub_.reset();
    control_pub_.reset();
  }

  void setStreaming(bool enable, std_srvs::srv::SetBool::Response & response)
  {
    if (!enable) {
      streaming_ = false;
      const bool had_control = static_cast<bool>(control_pub_);
      bool confirmed_stop = true;
      if (control_pub_) {
        uint64_t before;
        {
          std::lock_guard<std::mutex> lock(control_mutex_);
          before = control_sequence_;
        }
        std_msgs::msg::Bool command;
        command.data = false;
        control_pub_->publish(command);
        std::unique_lock<std::mutex> lock(control_mutex_);
        confirmed_stop = control_cv_.wait_for(lock, 2s, [this, before]() {
          return control_sequence_ > before &&
            (control_status_ == "stopped" || control_status_ == "disconnected" ||
            control_status_.rfind("error:", 0) == 0);
        });
      }
      clearSubscriptions();
      response.success = true;
      response.message = !had_control ? "已停止订阅；外部驱动可独立运行" :
        confirmed_stop ? "已停止订阅与驱动" : "已停止订阅；驱动未在 2 秒内确认停止";
      publishEvent(response.message);
      publishState("stopped", response.message);
      return;
    }
    if (streaming_) {
      response.success = true;
      response.message = "输入已经在运行";
      return;
    }
    const auto image_name = get_parameter(contract::extrinsic_input::kImageParameter).as_string();
    const auto cloud_name = get_parameter(contract::extrinsic_input::kCloudParameter).as_string();
    const auto command_name = get_parameter(contract::extrinsic_input::kCommandParameter).as_string();
    const auto status_name = get_parameter(contract::extrinsic_input::kStatusParameter).as_string();
    if (command_name.empty() != status_name.empty()) {
      response.message = "驱动命令与状态话题必须同时配置或同时留空";
      return;
    }
    const auto topics = get_topic_names_and_types();
    const bool raw = hasType(topics, image_name, contract::extrinsic_input::kRawImageType);
    const bool compressed = hasType(
      topics, image_name, contract::extrinsic_input::kCompressedImageType);
    const bool use_image = !image_name.empty() && (raw || compressed) &&
      !isInternal(image_name) && count_publishers(image_name) > 0;
    const bool use_cloud = !cloud_name.empty() && !isInternal(cloud_name) &&
      hasType(topics, cloud_name, contract::extrinsic_input::kCloudType) &&
      count_publishers(cloud_name) > 0;
    if (!use_image && !use_cloud) {
      response.message = "未发现已选图像或点云话题；请刷新来源并选择可用话题";
      return;
    }
    clearSubscriptions();
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frame_.release();
      cloud_.clear();
      frame_sequence_ = 0;
      published_sequence_ = 0;
      last_data_time_ = {};
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (use_image && raw) {
      raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_name, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          receiveRawImage(*message);
        }, options);
    }
    if (use_image && compressed && !raw) {
      compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_name, qos,
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
          const cv::Mat data(1, static_cast<int>(message->data.size()), CV_8UC1,
            const_cast<uint8_t *>(message->data.data()));
          receiveImage(cv::imdecode(data, cv::IMREAD_UNCHANGED));
        }, options);
    }
    if (use_cloud) {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_name, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          receiveCloud(*message);
        }, options);
    }
    if (!command_name.empty()) {
      control_pub_ = create_publisher<std_msgs::msg::Bool>(
        command_name, rclcpp::QoS(1).reliable());
      control_sub_ = create_subscription<std_msgs::msg::String>(
        status_name, rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(control_mutex_);
          control_status_ = message->data;
          ++control_sequence_;
          control_cv_.notify_all();
        }, options);
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      while (control_pub_->get_subscription_count() == 0 &&
        std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(20ms);
      }
      if (control_pub_->get_subscription_count() == 0) {
        clearSubscriptions();
        response.message = "驱动命令话题没有订阅者";
        return;
      }
    }
    streaming_ = true;
    if (control_pub_) {
      uint64_t before;
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        before = control_sequence_;
      }
      std_msgs::msg::Bool command;
      command.data = true;
      control_pub_->publish(command);
      std::unique_lock<std::mutex> lock(control_mutex_);
      const bool confirmed = control_cv_.wait_for(lock, 3s, [this, before]() {
        return control_sequence_ > before &&
          (control_status_ == "streaming" || control_status_ == "disconnected" ||
          control_status_.rfind("error:", 0) == 0);
      });
      if (!confirmed || control_status_ != "streaming") {
        const auto detail = control_status_;
        lock.unlock();
        streaming_ = false;
        std_msgs::msg::Bool rollback;
        rollback.data = false;
        control_pub_->publish(rollback);
        std::this_thread::sleep_for(50ms);
        clearSubscriptions();
        response.message = confirmed ? "驱动启动失败: " + detail :
          "驱动 3 秒内未确认 streaming";
        return;
      }
    }
    std::unique_lock<std::mutex> lock(frame_mutex_);
    const bool received = frame_cv_.wait_for(lock, 5s, [this]() {
      return !streaming_ || !frame_.empty() || !cloud_.empty();
    }) && streaming_ && (!frame_.empty() || !cloud_.empty());
    if (!received) {
      lock.unlock();
      streaming_ = false;
      if (control_pub_) {
        std_msgs::msg::Bool rollback;
        rollback.data = false;
        control_pub_->publish(rollback);
        std::this_thread::sleep_for(50ms);
      }
      clearSubscriptions();
      response.message = "5 秒内未收到图像或点云；请检查发布端与 QoS";
      return;
    }
    if (use_image && use_cloud) {
      frame_cv_.wait_for(lock, 750ms, [this]() {
        return !frame_.empty() && !cloud_.empty();
      });
    }
    std::string streams;
    if (!frame_.empty()) {streams = "rgb";}
    if (!cloud_.empty()) {streams += (streams.empty() ? "" : ",") + std::string("depth");}
    response.success = true;
    response.message = "Topic 已启动，可用信息流 " + streams;
    if (!frame_.empty()) {
      response.message += "，实际分辨率 " + std::to_string(width_) + "x" +
        std::to_string(height_);
    }
    lock.unlock();
    publishEvent(response.message);
    publishState("streaming", response.message);
  }

  void receiveImage(const cv::Mat & image)
  {
    if (!streaming_ || image.empty()) {return;}
    cv::Mat frame;
    if (image.depth() != CV_8U) {return;}
    if (image.channels() == 4) {cv::cvtColor(image, frame, cv::COLOR_BGRA2BGR);}
    else if (image.channels() == 3 || image.channels() == 1) {frame = image.clone();}
    else {return;}
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!streaming_) {return;}
    frame_ = std::move(frame);
    width_ = frame_.cols;
    height_ = frame_.rows;
    last_data_time_ = std::chrono::steady_clock::now();
    ++frame_sequence_;
    frame_cv_.notify_all();
  }

  void receiveRawImage(const sensor_msgs::msg::Image & message)
  {
    int type = -1;
    if (message.encoding == "bgr8" || message.encoding == "rgb8") {type = CV_8UC3;}
    else if (message.encoding == "bgra8" || message.encoding == "rgba8") {type = CV_8UC4;}
    else if (message.encoding == "mono8" || message.encoding == "8UC1") {type = CV_8UC1;}
    else if (message.encoding == "mono16" || message.encoding == "16UC1") {type = CV_16UC1;}
    else if (message.encoding == "yuv422" || message.encoding == "yuv422_yuy2") {
      type = CV_8UC2;
    }
    if (type < 0 || !message.width || !message.height ||
      message.step < message.width * CV_ELEM_SIZE(type) ||
      message.data.size() < static_cast<size_t>(message.height) * message.step ||
      (type == CV_16UC1 && message.is_bigendian) ||
      (type == CV_8UC2 && message.width % 2 != 0)) {return;}
    const cv::Mat source(static_cast<int>(message.height), static_cast<int>(message.width),
      type, const_cast<uint8_t *>(message.data.data()), message.step);
    cv::Mat output;
    if (message.encoding == "rgb8") {cv::cvtColor(source, output, cv::COLOR_RGB2BGR);}
    else if (message.encoding == "rgba8") {cv::cvtColor(source, output, cv::COLOR_RGBA2BGR);}
    else if (message.encoding == "bgra8") {cv::cvtColor(source, output, cv::COLOR_BGRA2BGR);}
    else if (message.encoding == "yuv422") {
      cv::cvtColor(source, output, cv::COLOR_YUV2BGR_UYVY);
    } else if (message.encoding == "yuv422_yuy2") {
      cv::cvtColor(source, output, cv::COLOR_YUV2BGR_YUY2);
    }
    else if (type == CV_16UC1) {source.convertTo(output, CV_8UC1, 1.0 / 256.0);}
    else {output = source;}
    receiveImage(output);
  }

  void receiveCloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    if (!streaming_ || message.is_bigendian || !message.width || !message.height ||
      message.row_step < message.width * message.point_step ||
      message.data.size() < static_cast<size_t>(message.height) * message.row_step) {return;}
    int offset[3] = {-1, -1, -1};
    for (const auto & field : message.fields) {
      const int axis = field.name == "x" ? 0 : field.name == "y" ? 1 :
        field.name == "z" ? 2 : -1;
      if (axis >= 0 && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1 && field.offset + sizeof(float) <= message.point_step) {
        offset[axis] = static_cast<int>(field.offset);
      }
    }
    if (std::any_of(std::begin(offset), std::end(offset),
      [](int value) {return value < 0;})) {return;}
    const size_t total = static_cast<size_t>(message.width) * message.height;
    const size_t stride = std::max<size_t>(1, total / 12000);
    std::vector<cv::Vec3f> points;
    points.reserve(std::min<size_t>(total, 12000));
    for (size_t i = 0; i < total; i += stride) {
      const size_t row = i / message.width, col = i % message.width;
      const auto * data = message.data.data() + row * message.row_step +
        col * message.point_step;
      cv::Vec3f point;
      for (int axis = 0; axis < 3; ++axis) {
        std::memcpy(&point[axis], data + offset[axis], sizeof(float));
      }
      if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
        points.push_back(point);
      }
    }
    if (points.empty()) {return;}
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!streaming_) {return;}
    cloud_ = std::move(points);
    last_data_time_ = std::chrono::steady_clock::now();
    ++frame_sequence_;
    frame_cv_.notify_all();
  }

  void checkTimeout()
  {
    if (!streaming_) {return;}
    bool expired = false;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      expired = last_data_time_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() - last_data_time_ > 3s;
    }
    if (!expired || !streaming_.exchange(false)) {return;}
    if (control_pub_) {
      std_msgs::msg::Bool command;
      command.data = false;
      control_pub_->publish(command);
      std::this_thread::sleep_for(50ms);
    }
    clearSubscriptions();
    publishState("disconnected", "已超过 3 秒未收到所选 Topic 数据");
    publishEvent("Topic 数据中断，请刷新来源并检查发布端");
  }

  void publishPreview()
  {
    cv::Mat frame;
    std::vector<cv::Vec3f> cloud;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!streaming_ || frame_sequence_ == published_sequence_) {return;}
      published_sequence_ = frame_sequence_;
      frame = frame_.clone();
      cloud = cloud_;
    }
    if (!frame.empty()) {
      publishImage(frame, image_pub_, true);
      publishImage(frame, stream_image_pub_, true, "rgb");
    }
    if (!cloud.empty()) {
      sensor_msgs::msg::PointCloud2 message;
      message.header.stamp = now();
      message.header.frame_id = "depth";
      message.height = 1;
      message.width = static_cast<uint32_t>(cloud.size());
      message.point_step = 3 * sizeof(float);
      message.row_step = message.width * message.point_step;
      message.fields.resize(3);
      for (size_t axis = 0; axis < 3; ++axis) {
        message.fields[axis].name = std::string(1, "xyz"[axis]);
        message.fields[axis].offset = static_cast<uint32_t>(axis * sizeof(float));
        message.fields[axis].datatype = sensor_msgs::msg::PointField::FLOAT32;
        message.fields[axis].count = 1;
      }
      message.data.resize(message.row_step);
      std::memcpy(message.data.data(), cloud.data(), message.data.size());
      message.is_dense = true;
      cloud_pub_->publish(std::move(message));
    }
  }

  void capture(std_srvs::srv::Trigger::Response & response)
  {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!streaming_ || frame_.empty()) {
        response.message = "图像话题尚无有效画面";
        return;
      }
      frozen_frame_ = frame_.clone();
      frozen_from_import_ = false;
    }
    response.success = true;
    response.message = "已冻结当前图像";
    publishImage(frozen_frame_, captured_pub_, false);
  }

  void save(std_srvs::srv::Trigger::Response & response)
  {
    cv::Mat frame;
    bool from_import;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (frozen_frame_.empty()) {response.message = "没有待保存画面"; return;}
      frame = frozen_frame_.clone();
      from_import = frozen_from_import_;
    }
    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error) {response.message = "无法创建样本目录: " + error.message(); return;}
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto path = std::filesystem::absolute(output_directory_) /
      ((from_import ? "imported" : "ros-topic_rgb") + std::string("_") +
      std::to_string(timestamp) + ".jpg");
    response.success = cv::imwrite(path.string(), frame, {cv::IMWRITE_JPEG_QUALITY, 100});
    response.message = response.success ? path.string() : "图像写入失败";
    publishEvent(response.success ? "已保存: " + path.string() : response.message);
  }

  void publishImage(
    const cv::Mat & frame,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher,
    bool preview, const std::string & frame_id = "calibration_camera")
  {
    cv::Mat output = frame;
    if (preview && preview_width_ > 0 && frame.cols > preview_width_) {
      cv::resize(frame, output, cv::Size(), static_cast<double>(preview_width_) / frame.cols,
        static_cast<double>(preview_width_) / frame.cols, cv::INTER_AREA);
    }
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = now();
    message.header.frame_id = frame_id;
    message.format = "jpeg";
    cv::imencode(".jpg", output, message.data,
      {cv::IMWRITE_JPEG_QUALITY, preview ? jpeg_quality_ : 100});
    publisher->publish(std::move(message));
  }

  void publishFile(
    const std::string & path,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher)
  {
    const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty()) {publishEvent("无法读取图片: " + path); return;}
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = now();
    message.header.frame_id = path;
    message.format = "png";
    cv::imencode(".png", image, message.data, {cv::IMWRITE_PNG_COMPRESSION, 3});
    publisher->publish(std::move(message));
  }

  void importStatus(const std::string & state, const std::string & path,
    const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = state + "\t" + path + "\t" + detail;
    import_pub_->publish(message);
  }

  void importImage(const std::string & request)
  {
    std::string path = request;
    int expected_width = width_, expected_height = height_;
    if (request.rfind("v2\t", 0) == 0) {
      const size_t first = request.find('\t', 3);
      const size_t second = first == std::string::npos ?
        std::string::npos : request.find('\t', first + 1);
      if (second == std::string::npos) {return;}
      path = request.substr(second + 1);
      try {
        expected_width = std::stoi(request.substr(3, first - 3));
        expected_height = std::stoi(request.substr(first + 1, second - first - 1));
      } catch (const std::exception &) {
        importStatus("failed", path, "标定分辨率无效");
        return;
      }
    }
    if (expected_width <= 0 || expected_height <= 0 || path.empty()) {
      importStatus("failed", path, "标定分辨率或路径无效");
      return;
    }
    const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty()) {importStatus("failed", path, "无法读取图片"); return;}
    if (image.cols != expected_width || image.rows != expected_height) {
      importStatus("failed", path,
        "图片分辨率 " + std::to_string(image.cols) + "x" + std::to_string(image.rows) +
        " 与当前标定分辨率 " + std::to_string(expected_width) + "x" +
        std::to_string(expected_height) + " 不一致");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frozen_frame_ = image.clone();
      frozen_from_import_ = true;
    }
    publishImage(image, captured_pub_, false);
    importStatus("loaded", path, "已载入并送往角点识别");
  }

  void deleteSample(const std::string & path)
  {
    const auto samples = std::filesystem::absolute(output_directory_).lexically_normal();
    const auto target = std::filesystem::absolute(path).lexically_normal();
    const auto prefix = samples.string() + std::filesystem::path::preferred_separator;
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

  int lock_fd_{-1};
  std::atomic_bool streaming_{false};
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  cv::Mat frame_, frozen_frame_;
  bool frozen_from_import_{false};
  std::vector<cv::Vec3f> cloud_;
  uint64_t frame_sequence_{0}, published_sequence_{0};
  std::chrono::steady_clock::time_point last_data_time_{};
  int width_{1280}, height_{720}, preview_width_{1280}, jpeg_quality_{90};
  double preview_fps_{15};
  std::string output_directory_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::mutex control_mutex_;
  std::condition_variable control_cv_;
  std::string control_status_;
  uint64_t control_sequence_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_, stream_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr captured_pub_, original_pub_,
    error_pub_, undistorted_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_, state_pub_, import_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_service_, emitter_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_, save_service_,
    discovery_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr original_sub_, error_sub_,
    undistorted_sub_, import_sub_, delete_sub_;
  rclcpp::TimerBase::SharedPtr preview_timer_, watchdog_timer_;
};
}  // namespace danmucamera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<danmucamera::IoNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Topic I/O 节点启动失败: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
