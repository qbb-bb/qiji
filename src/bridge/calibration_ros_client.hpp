#pragma once

#include <QImage>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QSize>
#include <QString>
#include <QStringList>
#include <QVector3D>

#include <array>
#include <functional>
#include <memory>
#include <vector>

namespace danmucamera
{

struct UiCameraDevice
{
  QString path;
  QString name;
  QString backend;
  QString signal;
  bool calibratable{false};
};

struct CalibrationSettings
{
  QString board_type{QStringLiteral("chessboard")};
  int columns{7};
  int rows{6};
  double spacing{0.06};
  double board_physical_width{0.0};
  double board_physical_height{0.0};
  double marker_length{0.04};
  int dictionary_id{0};
  QString camera_model{QStringLiteral("pinhole")};
  QString distortion_model{QStringLiteral("brown5")};
  bool automatic_initial{true};
  bool guide_use_initial{false};
  int guide_initial_width{0};
  int guide_initial_height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
  std::array<double, 8> initial_distortion{};
};

struct ImageQualityMetrics
{
  double laplacian_variance{0};
  double mean_gray{0};
  double board_area_ratio{-1.0};
  double board_area_min_ratio{0.0};
  double board_area_max_ratio{1.0};
  std::array<int, 9> corner_grid_counts{};
  bool board_size_warning{false};
  bool blur_warning{false};
  bool exposure_warning{false};
  QString warning_reason;
  std::array<int, 256> histogram{};
};

struct CornerCoverageThresholds
{
  double min_ratio{0.05};
  double max_ratio{0.25};
};

struct GuideSettings
{
  std::array<QPointF, 18> centers{};
  std::array<double, 18> roll_x_deg{};
  std::array<double, 18> pitch_y_deg{};
  std::array<double, 18> yaw_z_deg{};
  std::array<double, 18> area_ratios{};
  bool targets_configured{false};
  double max_orientation_error_deg{-1.0};
  std::array<double, 9> camera_matrix{};
  bool camera_matrix_invalid{false};
  std::vector<double> distortion;
  int camera_width{0};
  int camera_height{0};
  double max_shape_corner_error_ratio{-1.0};
  double visual_perspective_strength{-1.0};
  double min_guide_coverage_ratio{-1.0};
  double min_board_frame_ratio{-1.0};
  double inside_margin_ratio{-1.0};
  double frame_edge_margin_ratio{-1.0};
  QString guide_color;
  QString ready_color;
  QString detected_color;
  int guide_line_width{0};
  int ready_line_width{0};
  int detected_line_width{0};
  double corner_radius_px{-1.0};
};

struct GuideDetection
{
  QSize frame_size;
  QPolygonF board_polygon;
  bool detected{false};
  bool complete{false};
  bool pose_valid{false};
  std::array<double, 9> pose_rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  QString pose_error;
  int points{0};
  int expected_points{0};
  bool blur_warning{false};
  bool exposure_warning{false};
  double laplacian_variance{0};
  double mean_gray{0};
  double board_area_ratio{-1.0};
  bool board_size_warning{false};
  QString warning_reason;
};

struct CalibrationReportItem
{
  QString original_path;
  QString error_image_path;
  QString undistorted_image_path;
  double mean_error;
};

// UI 的唯一通信端口。窗口只调用这些“意图方法”并接收回调，不知道 ROS 接口名称、
// 消息类型、QoS、参数服务或 sensor_msgs 的内存布局。
class CalibrationRosClient
{
public:
  CalibrationRosClient();
  ~CalibrationRosClient();

  CalibrationRosClient(const CalibrationRosClient &) = delete;
  CalibrationRosClient & operator=(const CalibrationRosClient &) = delete;

  bool spinSome();
  void refreshDevices();
  void refreshTopicInputs();
  void connectCamera(
    const QString & device_id, int width, int height, bool disable_ir_emitter,
    const QStringList & stream_names = {}, const QString & image_topic = {},
    const QString & cloud_topic = {});
  void startCamera();
  void stopCamera();
  void setIrEmitterDisabled(bool disabled);
  void disconnectCamera();
  void captureFrame();
  void captureForCalibration(const CalibrationSettings & settings);
  GuideSettings guideSettings() const;
  CornerCoverageThresholds cornerCoverageThresholds() const;
  void startGuidedDetection(const CalibrationSettings & settings);
  void stopGuidedDetection();
  void importForCalibration(
    const QString & path, const CalibrationSettings & settings);
  void saveFrame(const QString & batch_source = {});
  void runCalibration();
  void runCalibration(
    const QStringList & sample_paths, const CalibrationSettings & settings);
  void viewImages(const QString & original_path, const QString & error_path = {},
    const QString & undistorted_path = {});
  void deleteSample(const QString & path);
  void requestImageQuality(const QString & path);

  std::function<void(const QImage &)> onFrame;
  std::function<void(const QString &, const QImage &)> onStreamFrame;
  std::function<void(const std::vector<QVector3D> &)> onStreamCloud;
  std::function<void(const QImage &)> onCapturedFrame;
  std::function<void(const QImage &)> onCornerPreview;
  std::function<void(const GuideDetection &)> onGuideDetection;
  std::function<void(bool, const QString &)> onGuideStart;
  std::function<void(const QString &, const QImage &)> onViewedOriginal;
  std::function<void(const QString &, const QImage &)> onViewedError;
  std::function<void(const QString &, const QImage &)> onViewedUndistorted;
  std::function<void(bool, const QString &)> onCornerStatus;
  std::function<void(const QString &, const ImageQualityMetrics &)> onImageQuality;
  std::function<void(
    const std::vector<CalibrationReportItem> &, const QString &, const QString &,
    const QString &, const QString &)> onReport;
  std::function<void(const QString &)> onLog;
  std::function<void(const QString &)> onStatus;
  std::function<void(const QString &)> onResult;
  std::function<void(const std::vector<UiCameraDevice> &)> onDevices;
  std::function<void(const QString &, const QString &)> onTopicDefaults;
  std::function<void(bool, const QString &)> onStreamingChanged;
  std::function<void(bool, const QString &)> onIrEmitterChanged;
  std::function<void(bool, const QString &)> onConnectionChanged;
  std::function<void(const QString &, const QString &)> onCameraState;
  std::function<void(const QString &)> onCameraCommandFailed;
  std::function<void(bool, const QString &)> onCaptureFinished;
  std::function<void(bool, const QString &)> onSaveFinished;
  std::function<void(const QString &, bool, const QString &)> onBatchSaveFinished;
  std::function<void(bool, const QString &)> onCalibrationAccepted;
  std::function<void(bool, const QString &, const QString &)> onImportStatus;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace danmucamera
