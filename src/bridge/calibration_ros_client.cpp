#include "calibration_ros_client.hpp"

#include "ros_contract.hpp"

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QCoreApplication>
#include <QMetaObject>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace danmucamera
{

namespace
{
std::vector<rclcpp::Parameter> settingsParameters(const CalibrationSettings & settings)
{
  return {
    rclcpp::Parameter("board_type", settings.board_type.toStdString()),
    rclcpp::Parameter("board_width", settings.columns),
    rclcpp::Parameter("board_height", settings.rows),
    rclcpp::Parameter("square_size", settings.spacing),
    rclcpp::Parameter("marker_length", settings.marker_length),
    rclcpp::Parameter("dictionary_id", settings.dictionary_id),
    rclcpp::Parameter("camera_model", settings.camera_model.toStdString()),
    rclcpp::Parameter("distortion_model", settings.distortion_model.toStdString()),
    rclcpp::Parameter("automatic_initial", settings.automatic_initial),
    rclcpp::Parameter("initial_fx", settings.fx),
    rclcpp::Parameter("initial_fy", settings.fy),
    rclcpp::Parameter("initial_cx", settings.cx),
    rclcpp::Parameter("initial_cy", settings.cy),
    rclcpp::Parameter("initial_distortion",
      std::vector<double>(settings.initial_distortion.begin(), settings.initial_distortion.end()))
  };
}
}  // namespace

class CalibrationRosClient::Impl
{
public:
  explicit Impl(CalibrationRosClient & owner)
  : owner_(owner), node_(std::make_shared<rclcpp::Node>("calibration_ui")), running_(true)
  {
    corner_coverage_thresholds_.min_ratio = node_->declare_parameter<double>(
      "corner_coverage.min_ratio", 0.05);
    corner_coverage_thresholds_.max_ratio = node_->declare_parameter<double>(
      "corner_coverage.max_ratio", 0.25);
    if (!std::isfinite(corner_coverage_thresholds_.min_ratio) ||
      !std::isfinite(corner_coverage_thresholds_.max_ratio) ||
      corner_coverage_thresholds_.min_ratio < 0.0 ||
      corner_coverage_thresholds_.min_ratio >= corner_coverage_thresholds_.max_ratio ||
      corner_coverage_thresholds_.max_ratio > 1.0) {
      throw std::invalid_argument(
        "corner_coverage 阈值必须满足 0 <= min_ratio < max_ratio <= 1");
    }
    const auto centers_x = node_->declare_parameter<std::vector<double>>(
      "guide.center_x", std::vector<double>{});
    const auto centers_y = node_->declare_parameter<std::vector<double>>(
      "guide.center_y", std::vector<double>{});
    const auto legacy_angles = node_->declare_parameter<std::vector<double>>(
      "guide.angle_deg", std::vector<double>{});
    const auto roll = node_->declare_parameter<std::vector<double>>(
      "guide.roll_x_deg", std::vector<double>{});
    const auto pitch = node_->declare_parameter<std::vector<double>>(
      "guide.pitch_y_deg", std::vector<double>{});
    const auto yaw = node_->declare_parameter<std::vector<double>>(
      "guide.yaw_z_deg", std::vector<double>{});
    const auto areas = node_->declare_parameter<std::vector<double>>(
      "guide.area_ratio", std::vector<double>{});
    const size_t count = guide_settings_.centers.size();
    if (centers_x.size() == count && centers_y.size() == count &&
      (roll.size() == count || (roll.empty() && legacy_angles.size() == count)) &&
      (pitch.size() == count || (pitch.empty() && legacy_angles.size() == count)) &&
      (yaw.size() == count || (yaw.empty() && legacy_angles.size() == count)) &&
      areas.size() == count)
    {
      guide_settings_.targets_configured = true;
      for (size_t i = 0; i < count; ++i) {
        guide_settings_.centers[i] = QPointF(centers_x[i], centers_y[i]);
        guide_settings_.roll_x_deg[i] = roll.size() == count ? roll[i] : 0.0;
        guide_settings_.pitch_y_deg[i] = pitch.size() == count ? pitch[i] : 0.0;
        guide_settings_.yaw_z_deg[i] = yaw.size() == count ? yaw[i] : legacy_angles[i];
        guide_settings_.area_ratios[i] = areas[i];
      }
    }
    guide_settings_.max_orientation_error_deg =
      node_->declare_parameter<double>("guide.max_orientation_error_deg", -1.0);
    const auto camera_matrix = node_->declare_parameter<std::vector<double>>(
      "guide.camera_matrix", std::vector<double>{});
    guide_settings_.camera_matrix_invalid =
      !camera_matrix.empty() && camera_matrix.size() != 9;
    if (camera_matrix.size() == 9) {
      std::copy(camera_matrix.begin(), camera_matrix.end(), guide_settings_.camera_matrix.begin());
    }
    guide_settings_.distortion = node_->declare_parameter<std::vector<double>>(
      "guide.distortion", std::vector<double>{});
    guide_settings_.camera_width = node_->declare_parameter<int>("guide.camera_width", 0);
    guide_settings_.camera_height = node_->declare_parameter<int>("guide.camera_height", 0);
    guide_settings_.max_shape_corner_error_ratio = node_->declare_parameter<double>(
      "guide.max_shape_corner_error_ratio", -1.0);
    guide_settings_.visual_perspective_strength = node_->declare_parameter<double>(
      "guide.visual_perspective_strength", -1.0);
    guide_settings_.min_guide_coverage_ratio =
      node_->declare_parameter<double>("guide.min_guide_coverage_ratio", -1.0);
    guide_settings_.min_board_frame_ratio =
      node_->declare_parameter<double>("guide.min_board_frame_ratio", -1.0);
    guide_settings_.inside_margin_ratio =
      node_->declare_parameter<double>("guide.inside_margin_ratio", -1.0);
    guide_settings_.frame_edge_margin_ratio =
      node_->declare_parameter<double>("guide.frame_edge_margin_ratio", -1.0);
    guide_settings_.guide_color = QString::fromStdString(
      node_->declare_parameter<std::string>("guide.guide_color", ""));
    guide_settings_.ready_color = QString::fromStdString(
      node_->declare_parameter<std::string>("guide.ready_color", ""));
    guide_settings_.detected_color = QString::fromStdString(
      node_->declare_parameter<std::string>("guide.detected_color", ""));
    guide_settings_.guide_line_width =
      node_->declare_parameter<int>("guide.guide_line_width", 0);
    guide_settings_.ready_line_width =
      node_->declare_parameter<int>("guide.ready_line_width", 0);
    guide_settings_.detected_line_width =
      node_->declare_parameter<int>("guide.detected_line_width", 0);
    guide_settings_.corner_radius_px =
      node_->declare_parameter<double>("guide.corner_radius_px", -1.0);

    image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kImageTopic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveFrame(*message, owner_.onFrame);
      });
    stream_image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kStreamImageTopic, rclcpp::SensorDataQoS().keep_last(4),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveViewedFrame(*message, owner_.onStreamFrame);
      });
    stream_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      contract::kStreamCloudTopic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        if (!owner_.onStreamCloud || message->point_step != 3 * sizeof(float) ||
          message->data.size() < static_cast<size_t>(message->width) * message->point_step) {
          return;
        }
        std::vector<QVector3D> points;
        points.reserve(message->width);
        for (size_t i = 0; i < message->width; ++i) {
          float xyz[3];
          std::memcpy(xyz, message->data.data() + i * message->point_step, sizeof(xyz));
          points.emplace_back(xyz[0], xyz[1], xyz[2]);
        }
        dispatch([this, points = std::move(points)]() {
          if (owner_.onStreamCloud) {owner_.onStreamCloud(points);}
        });
      });
    captured_image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kCapturedImageTopic, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveFrame(*message, owner_.onCapturedFrame);
      });
    corner_preview_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kCornerPreviewTopic, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveFrame(*message, owner_.onCornerPreview);
      });
    viewed_original_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kViewedOriginalTopic, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveViewedFrame(*message, owner_.onViewedOriginal);
      });
    viewed_error_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kViewedErrorTopic, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveViewedFrame(*message, owner_.onViewedError);
      });
    viewed_undistorted_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kViewedUndistortedTopic, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        receiveViewedFrame(*message, owner_.onViewedUndistorted);
      });
    guide_result_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kGuideResultTopic, rclcpp::QoS(5).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QStringList fields = QString::fromStdString(message->data).split('\t');
        if (fields.size() < 26) {
          log("引导检测消息字段不足，无法读取角点结果");
          return;
        }
        GuideDetection result;
        result.frame_size = QSize(fields[0].toInt(), fields[1].toInt());
        result.detected = fields[2] == QStringLiteral("1");
        result.complete = fields[3] == QStringLiteral("1");
        result.points = fields[4].toInt();
        result.expected_points = fields[5].toInt();
        if (result.detected && result.points >= 3) {
          for (int i = 0; i < 4; ++i) {
            result.board_polygon << QPointF(
              fields[17 + 2 * i].toDouble(), fields[18 + 2 * i].toDouble());
          }
        }
        result.blur_warning = fields[10] == QStringLiteral("1");
        result.exposure_warning = fields[11] == QStringLiteral("1");
        result.laplacian_variance = fields[12].toDouble();
        result.mean_gray = fields[13].toDouble();
        result.warning_reason = fields[14];
        result.board_area_ratio = fields[15].toDouble();
        result.board_size_warning = fields[16] == QStringLiteral("1");
        // 实时角点结果先交给界面；姿态字段缺失时不能静默丢掉整帧。
        result.pose_valid = fields[25] == QStringLiteral("1") && fields.size() >= 36;
        if (result.pose_valid) {
          for (int i = 0; i < 9; ++i) {result.pose_rotation[i] = fields[26 + i].toDouble();}
        }
        result.pose_error = fields.size() >= 36 ? fields[35] :
          QStringLiteral("引导姿态数据格式不匹配，请关闭程序后完整重启");
        dispatch([this, result]() {
          if (owner_.onGuideDetection) {owner_.onGuideDetection(result);}
        });
      });
    event_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kCameraEventTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {log(message->data);});
    state_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kCameraStateTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QStringList fields = QString::fromStdString(message->data).split('\t');
        const QString state = fields.value(0);
        const QString detail = fields.mid(1).join(QStringLiteral("\t"));
        dispatch([this, state, detail]() {
          if (owner_.onCameraState) {owner_.onCameraState(state, detail);}
        });
      });
    import_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kImportImageStatusTopic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QStringList fields = QString::fromStdString(message->data).split('\t');
        const bool loaded = fields.value(0) == QStringLiteral("loaded");
        const QString path = fields.value(1);
        const QString detail = fields.mid(2).join(QStringLiteral("\t"));
        dispatch([this, loaded, path, detail]() {
          if (owner_.onImportStatus) {owner_.onImportStatus(loaded, path, detail);}
        });
      });
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kAlgorithmStatusTopic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QString text = QString::fromStdString(message->data);
        dispatch([this, text]() {if (owner_.onStatus) {owner_.onStatus(text);}});
      });
    result_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kAlgorithmResultTopic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QString text = QString::fromStdString(message->data);
        dispatch([this, text]() {if (owner_.onResult) {owner_.onResult(text);}});
      });
    quality_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kQualityResultTopic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QStringList fields = QString::fromStdString(message->data).split('\t');
        if (fields.size() < 276) {return;}
        ImageQualityMetrics quality;
        quality.laplacian_variance = fields[1].toDouble();
        quality.mean_gray = fields[2].toDouble();
        quality.blur_warning = fields[3] == QStringLiteral("1");
        quality.exposure_warning = fields[4] == QStringLiteral("1");
        quality.warning_reason = fields[5];
        for (int i = 0; i < 256; ++i) {quality.histogram[i] = fields[i + 6].toInt();}
        quality.board_area_ratio = fields[262].toDouble();
        quality.board_size_warning = fields[263] == QStringLiteral("1");
        quality.board_area_min_ratio = fields[264].toDouble();
        quality.board_area_max_ratio = fields[265].toDouble();
        for (int i = 0; i < 9; ++i) {quality.corner_grid_counts[i] = fields[266 + i].toInt();}
        const QString path = fields[0] == QStringLiteral("file") ?
          fields.mid(275).join(QStringLiteral("\t")) : QString();
        dispatch([this, path, quality]() {
          if (owner_.onImageQuality) {owner_.onImageQuality(path, quality);}
        });
      });
    corner_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kCornerStatusTopic, rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QString text = QString::fromStdString(message->data);
        const bool valid = text.startsWith("valid|");
        const QString detail = text.section('|', 1);
        dispatch([this, valid, detail]() {
          if (owner_.onCornerStatus) {owner_.onCornerStatus(valid, detail);}
        });
      });
    report_sub_ = node_->create_subscription<std_msgs::msg::String>(
      contract::kAlgorithmReportTopic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::vector<CalibrationReportItem> report;
        QString result_file;
        QString global_error_image;
        QString camera_matrix_file;
        QString distortion_matrix_file;
        const QStringList lines = QString::fromStdString(message->data).split('\n', Qt::SkipEmptyParts);
        for (const QString & line : lines) {
          const QStringList fields = line.split('\t');
          if (fields.size() == 2 && fields[0] == QStringLiteral("@result_file")) {
            result_file = fields[1];
            continue;
          }
          if (fields.size() == 2 && fields[0] == QStringLiteral("@global_error_image")) {
            global_error_image = fields[1];
            continue;
          }
          if (fields.size() == 2 && fields[0] == QStringLiteral("@camera_matrix_file")) {
            camera_matrix_file = fields[1];
            continue;
          }
          if (fields.size() == 2 && fields[0] == QStringLiteral("@distortion_matrix_file")) {
            distortion_matrix_file = fields[1];
            continue;
          }
          if (fields.size() >= 4) {
            report.push_back({fields[0], fields[2], fields[3], fields[1].toDouble()});
          } else if (fields.size() == 3) {
            report.push_back({fields[0], fields[2], QString(), fields[1].toDouble()});
          }
        }
        dispatch([this, report, result_file, global_error_image,
          camera_matrix_file, distortion_matrix_file]() {
          if (owner_.onReport) {
            owner_.onReport(
              report, result_file, global_error_image,
              camera_matrix_file, distortion_matrix_file);
          }
        });
      });

    stream_client_ = node_->create_client<std_srvs::srv::SetBool>(contract::kStreamingService);
    emitter_client_ = node_->create_client<std_srvs::srv::SetBool>(contract::kEmitterService);
    capture_client_ = node_->create_client<std_srvs::srv::Trigger>(contract::kCaptureService);
    save_client_ = node_->create_client<std_srvs::srv::Trigger>(contract::kSaveService);
    device_client_ = node_->create_client<std_srvs::srv::Trigger>(contract::kRefreshDevicesService);
    calibration_client_ = node_->create_client<std_srvs::srv::Trigger>(
      contract::kRunCalibrationService);
    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, contract::kCameraNode);
    algorithm_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, "/calibration_algorithm_bridge");
    view_original_pub_ = node_->create_publisher<std_msgs::msg::String>(
      contract::kViewOriginalRequestTopic, 10);
    view_error_pub_ = node_->create_publisher<std_msgs::msg::String>(
      contract::kViewErrorRequestTopic, 10);
    view_undistorted_pub_ = node_->create_publisher<std_msgs::msg::String>(
      contract::kViewUndistortedRequestTopic, 10);
    quality_request_pub_ = node_->create_publisher<std_msgs::msg::String>(
      contract::kQualityRequestTopic, 10);
    import_image_pub_ = node_->create_publisher<std_msgs::msg::String>(
      contract::kImportImageRequestTopic, 10);
    delete_pub_ = node_->create_publisher<std_msgs::msg::String>(contract::kDeleteSampleTopic, 10);
    executor_.add_node(node_);
    worker_ = std::thread([this]() {executor_.spin(); running_ = false;});
  }

  ~Impl()
  {
    executor_.cancel();
    if (worker_.joinable()) {worker_.join();}
  }

  template<typename Function>
  void dispatch(Function && function)
  {
    QMetaObject::invokeMethod(
      QCoreApplication::instance(), std::forward<Function>(function), Qt::QueuedConnection);
  }

  void receiveFrame(
    const sensor_msgs::msg::CompressedImage & message,
    const std::function<void(const QImage &)> & callback)
  {
    // 自动识别 JPEG/PNG；全局误差图使用无损 PNG，避免小字和单点在传输时产生压缩伪影。
    const QImage copied = QImage::fromData(
      message.data.data(), static_cast<int>(message.data.size()));
    if (copied.isNull()) {log("收到无法解码的压缩图像"); return;}
    dispatch([callback, copied]() {if (callback) {callback(copied);}});
  }

  void receiveViewedFrame(
    const sensor_msgs::msg::CompressedImage & message,
    const std::function<void(const QString &, const QImage &)> & callback)
  {
    const QImage copied = QImage::fromData(
      message.data.data(), static_cast<int>(message.data.size()));
    if (copied.isNull()) {log("收到无法解码的结果图像"); return;}
    const QString path = QString::fromStdString(message.header.frame_id);
    dispatch([callback, path, copied]() {if (callback) {callback(path, copied);}});
  }

  void log(const std::string & text)
  {
    const QString copied = QString::fromStdString(text);
    dispatch([this, copied]() {if (owner_.onLog) {owner_.onLog(copied);}});
  }

  CalibrationRosClient & owner_;
  rclcpp::Node::SharedPtr node_;
  GuideSettings guide_settings_;
  CornerCoverageThresholds corner_coverage_thresholds_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr stream_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr stream_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr captured_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr corner_preview_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guide_result_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr viewed_original_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr viewed_error_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr viewed_undistorted_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr import_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr quality_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr corner_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr report_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stream_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emitter_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr device_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibration_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> algorithm_parameter_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr view_original_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr view_error_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr view_undistorted_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr import_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delete_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread worker_;
  std::atomic_bool running_;
};

CalibrationRosClient::CalibrationRosClient()
: impl_(std::make_unique<Impl>(*this)) {}

CalibrationRosClient::~CalibrationRosClient() = default;

GuideSettings CalibrationRosClient::guideSettings() const
{
  return impl_->guide_settings_;
}

CornerCoverageThresholds CalibrationRosClient::cornerCoverageThresholds() const
{
  return impl_->corner_coverage_thresholds_;
}

bool CalibrationRosClient::spinSome()
{
  // 为保持窗口接口简单而保留此函数；真正的 ROS spin 已在后台线程中运行。
  return rclcpp::ok() && impl_->running_;
}

void CalibrationRosClient::refreshDevices()
{
  if (!impl_->device_client_->service_is_ready()) {
    impl_->log("设备发现服务未就绪");
    return;
  }
  impl_->device_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      std::vector<UiCameraDevice> devices;
      if (response->success) {
        const QStringList lines = QString::fromStdString(response->message)
          .split('\n', Qt::SkipEmptyParts);
        for (const QString & line : lines) {
          const QStringList fields = line.split('|');
          const QString signal = fields.value(3, QStringLiteral("rgb"));
          devices.push_back({
            fields.value(0), fields.value(1, QStringLiteral("Camera")),
            fields.value(2, QStringLiteral("v4l2")), signal,
            signal == QStringLiteral("rgb") || signal.startsWith(QStringLiteral("ir"))});
        }
      } else {
        impl_->log(response->message);
      }
      impl_->dispatch([this, devices]() {if (onDevices) {onDevices(devices);}});
    });
}

void CalibrationRosClient::refreshTopicInputs()
{
  if (!impl_->parameter_client_->service_is_ready()) {return;}
  impl_->parameter_client_->get_parameters(
    {contract::extrinsic_input::kImageParameter, contract::extrinsic_input::kCloudParameter,
      contract::extrinsic_input::kCommandParameter, contract::extrinsic_input::kStatusParameter},
    [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      const auto values = future.get();
      if (values.size() != 4) {return;}
      const QString image = QString::fromStdString(values[0].as_string());
      const QString cloud = QString::fromStdString(values[1].as_string());
      const QString command = QString::fromStdString(values[2].as_string());
      const QString status = QString::fromStdString(values[3].as_string());
      impl_->dispatch([this, image, cloud, command, status]() {
        if (onTopicDefaults) {onTopicDefaults(image, cloud);}
        if (onTopicControlDefaults) {onTopicControlDefaults(command, status);}
      });
    });
}

void CalibrationRosClient::refreshExtrinsicInput()
{
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {return;}
  impl_->algorithm_parameter_client_->get_parameters(
    {contract::extrinsic_input::kCameraInfoParameter,
      contract::extrinsic_input::kMaxPairDeltaParameter},
    [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      const auto values = future.get();
      if (values.size() != 2) {return;}
      const QString topic = QString::fromStdString(values[0].as_string());
      const double max_delta = values[1].as_double();
      impl_->dispatch([this, topic, max_delta]() {
        if (onExtrinsicInput) {onExtrinsicInput(topic, max_delta);}
      });
    });
}

void CalibrationRosClient::setExtrinsicInput(
  const QString & camera_info, double max_pair_delta_ms)
{
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {
    if (onExtrinsicInputSaved) {onExtrinsicInputSaved(false, "标定参数服务未就绪");}
    return;
  }
  impl_->algorithm_parameter_client_->set_parameters(
    {rclcpp::Parameter(contract::extrinsic_input::kCameraInfoParameter,
      camera_info.toStdString()),
      rclcpp::Parameter(contract::extrinsic_input::kMaxPairDeltaParameter,
      max_pair_delta_ms)},
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      const bool success = results.size() == 2 && std::all_of(results.begin(), results.end(),
        [](const auto & result) {return result.successful;});
      impl_->dispatch([this, success]() {
        if (onExtrinsicInputSaved) {
          onExtrinsicInputSaved(success, success ?
            QStringLiteral("外参输入设置已应用到当前运行的标定节点") :
            QStringLiteral("外参输入设置未能应用"));
        }
      });
    });
}

void CalibrationRosClient::connectCamera(
  const QString & device_id, int width, int height, bool disable_ir_emitter,
  const QStringList & stream_names, const QString & image_topic,
  const QString & cloud_topic, const QString & control_command,
  const QString & control_status)
{
  if (!impl_->parameter_client_->service_is_ready()) {
    impl_->log("相机参数服务未就绪");
    if (onConnectionChanged) {onConnectionChanged(false, "参数服务未就绪");}
    return;
  }
  std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("device", device_id.toStdString()),
      rclcpp::Parameter("width", width), rclcpp::Parameter("height", height),
      rclcpp::Parameter("disable_ir_emitter", disable_ir_emitter),
      rclcpp::Parameter("signals", [&stream_names]() {
        std::vector<std::string> values;
        for (const auto & signal : stream_names) {values.push_back(signal.toStdString());}
        return values;
      }())};
  if (device_id.startsWith(QStringLiteral("ros-topic:"))) {
    parameters.emplace_back("topic_input.name", image_topic.toStdString());
    parameters.emplace_back("topic_input.pointcloud_name", cloud_topic.toStdString());
    parameters.emplace_back(contract::extrinsic_input::kCommandParameter,
      control_command.toStdString());
    parameters.emplace_back(contract::extrinsic_input::kStatusParameter,
      control_status.toStdString());
  }
  const size_t expected = parameters.size();
  impl_->parameter_client_->set_parameters(
    parameters,
    [this, expected, topic_mode = device_id.startsWith(QStringLiteral("ros-topic:"))]
    (std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      if (results.size() != expected || !std::all_of(results.begin(), results.end(),
        [](const auto & result) {return result.successful;})) {
        impl_->dispatch([this, results]() {
          QString reason = QStringLiteral("设置相机参数失败");
          for (const auto & result : results) {
            if (!result.successful && !result.reason.empty()) {
              reason += QStringLiteral(": ") + QString::fromStdString(result.reason);
              break;
            }
          }
          if (onConnectionChanged) {onConnectionChanged(false, reason);}
        });
        return;
      }
      impl_->dispatch([this, topic_mode]() {
        if (onConnectionChanged) {onConnectionChanged(true, topic_mode ?
          "Topic 输入已配置，点击“启动相机”验证驱动和数据" :
          "相机已连接，点击“启动相机”开始采流");}
      });
    });
}

void CalibrationRosClient::startCamera()
{
  if (!impl_->stream_client_->service_is_ready()) {
    impl_->dispatch([this]() {
      if (onCameraCommandFailed) {onCameraCommandFailed("相机启动服务未就绪");}
    });
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  impl_->stream_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      impl_->dispatch([this, success, message]() {
        if (success && onStreamingChanged) {onStreamingChanged(true, message);}
        else if (!success && onCameraCommandFailed) {onCameraCommandFailed(message);}
      });
    });
}

void CalibrationRosClient::stopCamera()
{
  if (!impl_->stream_client_->service_is_ready()) {
    impl_->dispatch([this]() {
      if (onCameraCommandFailed) {onCameraCommandFailed("相机关闭服务未就绪");}
    });
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;
  impl_->stream_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      impl_->dispatch([this, success, message]() {
        if (success && onStreamingChanged) {onStreamingChanged(false, message);}
        else if (!success && onCameraCommandFailed) {onCameraCommandFailed(message);}
      });
    });
}

void CalibrationRosClient::setIrEmitterDisabled(bool disabled)
{
  if (!impl_->emitter_client_->service_is_ready()) {
    impl_->dispatch([this]() {
      if (onIrEmitterChanged) {onIrEmitterChanged(false, "红外投射器服务未就绪");}
    });
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = disabled;
  impl_->emitter_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      impl_->dispatch([this, success, message]() {
        if (onIrEmitterChanged) {onIrEmitterChanged(success, message);}
      });
    });
}

void CalibrationRosClient::disconnectCamera()
{
  if (!impl_->stream_client_->service_is_ready()) {
    impl_->dispatch([this]() {
      if (onCameraCommandFailed) {onCameraCommandFailed("相机服务未就绪");}
    });
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;
  impl_->stream_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      impl_->dispatch([this, success, message]() {
        if (!success) {
          if (onCameraCommandFailed) {onCameraCommandFailed(message);}
          return;
        }
        if (onStreamingChanged) {onStreamingChanged(false, message);}
        if (onConnectionChanged) {onConnectionChanged(false, "相机连接已断开");}
      });
    });
}

void CalibrationRosClient::captureFrame()
{
  if (!impl_->capture_client_->service_is_ready()) {
    impl_->log("拍摄服务未就绪");
    return;
  }
  impl_->capture_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      impl_->dispatch([this, success, message]() {
        if (onCaptureFinished) {onCaptureFinished(success, message);}
      });
    });
}

void CalibrationRosClient::captureForCalibration(
  const CalibrationSettings & settings)
{
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {
    impl_->log("算法参数服务未就绪");
    return;
  }
  impl_->algorithm_parameter_client_->set_parameters(settingsParameters(settings),
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      for (const auto & result : future.get()) {
        if (!result.successful) {impl_->log("标定板参数更新失败"); return;}
      }
      captureFrame();
    });
}

void CalibrationRosClient::startGuidedDetection(const CalibrationSettings & settings)
{
  const auto deliver = [this](bool success, const QString & message) {
      impl_->dispatch([this, success, message]() {
        if (onGuideStart) {onGuideStart(success, message);}
      });
    };
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {
    deliver(false, QStringLiteral("算法参数服务未就绪"));
    return;
  }
  auto parameters = settingsParameters(settings);
  const GuideSettings & guide = impl_->guide_settings_;
  const bool ui_intrinsics = settings.guide_use_initial &&
    settings.fx > 0 && settings.fy > 0;
  const bool configured = !ui_intrinsics &&
    guide.camera_matrix[0] > 0 && guide.camera_matrix[4] > 0;
  std::vector<double> K(guide.camera_matrix.begin(), guide.camera_matrix.end());
  const bool manual = !configured && ui_intrinsics;
  if (!configured) {
    K = manual ? std::vector<double>{settings.fx, 0, settings.cx,
      0, settings.fy, settings.cy, 0, 0, 1} : std::vector<double>(9, 0.0);
  }
  std::vector<double> D = configured ? guide.distortion : manual ?
    std::vector<double>(settings.initial_distortion.begin(), settings.initial_distortion.end()) :
    std::vector<double>{};
  if (D.empty()) {D = std::vector<double>(settings.camera_model == QStringLiteral("fisheye") ? 4 : 5, 0.0);}
  if (settings.camera_model == QStringLiteral("fisheye")) {D.resize(4);}
  else if (D.size() != 4 && D.size() != 5 && D.size() != 8) {D.resize(5);}
  parameters.emplace_back("guide_camera_matrix", K);
  parameters.emplace_back("guide_distortion", D);
  parameters.emplace_back("guide_camera_width",
    ui_intrinsics ? settings.guide_initial_width : guide.camera_width);
  parameters.emplace_back("guide_camera_height",
    ui_intrinsics ? settings.guide_initial_height : guide.camera_height);
  parameters.emplace_back("guide_board_physical_width", settings.board_physical_width);
  parameters.emplace_back("guide_board_physical_height", settings.board_physical_height);
  parameters.emplace_back("guide_enabled", true);
  impl_->algorithm_parameter_client_->set_parameters(parameters,
    [deliver](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        for (const auto & result : future.get()) {
          if (!result.successful) {
            deliver(false, QString::fromStdString(result.reason));
            return;
          }
        }
        deliver(true, QStringLiteral("实时标定板识别已启动"));
      } catch (const std::exception & error) {
        deliver(false, QString::fromUtf8(error.what()));
      }
    });
}

void CalibrationRosClient::stopGuidedDetection()
{
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {return;}
  impl_->algorithm_parameter_client_->set_parameters(
    {rclcpp::Parameter("guide_enabled", false)});
}

void CalibrationRosClient::importForCalibration(
  const QString & path, const CalibrationSettings & settings)
{
  const auto fail = [this, path](const QString & reason) {
      impl_->dispatch([this, path, reason]() {
        if (onImportStatus) {onImportStatus(false, path, reason);}
      });
    };
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {
    fail(QStringLiteral("算法参数服务未就绪"));
    return;
  }
  impl_->algorithm_parameter_client_->set_parameters(settingsParameters(settings),
    [this, path, fail](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        for (const auto & result : future.get()) {
          if (!result.successful) {
            fail(QStringLiteral("标定板参数更新失败：") +
              QString::fromStdString(result.reason));
            return;
          }
        }
      } catch (const std::exception & error) {
        fail(QStringLiteral("标定板参数请求失败：") + QString::fromUtf8(error.what()));
        return;
      }
      std_msgs::msg::String message;
      message.data = path.toStdString();
      impl_->import_image_pub_->publish(message);
    });
}

void CalibrationRosClient::saveFrame(const QString & batch_source)
{
  const auto deliver = [this, batch_source](bool success, const QString & message) {
      impl_->dispatch([this, batch_source, success, message]() {
        if (!batch_source.isEmpty()) {
          if (onBatchSaveFinished) {onBatchSaveFinished(batch_source, success, message);}
        } else if (onSaveFinished) {
          onSaveFinished(success, message);
        }
      });
    };
  if (!impl_->save_client_->service_is_ready()) {
    deliver(false, QStringLiteral("保存服务未就绪"));
    return;
  }
  impl_->save_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [deliver](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        const auto response = future.get();
        deliver(response->success, QString::fromStdString(response->message));
      } catch (const std::exception & error) {
        deliver(false, QStringLiteral("保存请求失败：") + QString::fromUtf8(error.what()));
      }
    });
}

void CalibrationRosClient::runCalibration()
{
  if (!impl_->calibration_client_->service_is_ready()) {
    impl_->log("标定服务未就绪");
    return;
  }
  impl_->calibration_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      impl_->dispatch([this, success, message]() {
        if (onCalibrationAccepted) {onCalibrationAccepted(success, message);}
      });
    });
}

void CalibrationRosClient::runCalibration(
  const QStringList & sample_paths, const CalibrationSettings & settings)
{
  if (!impl_->algorithm_parameter_client_->service_is_ready()) {
    impl_->log("算法参数服务未就绪");
    return;
  }
  std::vector<std::string> samples;
  samples.reserve(sample_paths.size());
  for (const QString & path : sample_paths) {samples.push_back(path.toStdString());}
  auto parameters = settingsParameters(settings);
  parameters.emplace_back("sample_paths", samples);
  parameters.emplace_back("corner_coverage.min_ratio", impl_->corner_coverage_thresholds_.min_ratio);
  parameters.emplace_back("corner_coverage.max_ratio", impl_->corner_coverage_thresholds_.max_ratio);
  impl_->algorithm_parameter_client_->set_parameters(parameters,
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      for (const auto & result : future.get()) {
        if (!result.successful) {impl_->log("标定板参数更新失败"); return;}
      }
      runCalibration();
    });
}

void CalibrationRosClient::viewImages(
  const QString & original_path, const QString & error_path,
  const QString & undistorted_path)
{
  std_msgs::msg::String message;
  if (!original_path.isEmpty()) {
    message.data = original_path.toStdString();
    impl_->view_original_pub_->publish(message);
  }
  if (!error_path.isEmpty()) {
    message.data = error_path.toStdString();
    impl_->view_error_pub_->publish(message);
  }
  if (!undistorted_path.isEmpty()) {
    message.data = undistorted_path.toStdString();
    impl_->view_undistorted_pub_->publish(message);
  }
}

void CalibrationRosClient::deleteSample(const QString & path)
{
  std_msgs::msg::String message;
  message.data = path.toStdString();
  impl_->delete_pub_->publish(message);
}


void CalibrationRosClient::requestImageQuality(const QString & path)
{
  std_msgs::msg::String request;
  request.data = path.toStdString();
  impl_->quality_request_pub_->publish(request);
}

}  // namespace danmucamera
