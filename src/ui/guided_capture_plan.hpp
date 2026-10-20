#pragma once

#include "calibration_ros_client.hpp"

#include <QPolygonF>
#include <QString>

namespace danmucamera
{

struct GuidedCaptureDecision
{
  QPolygonF target;
  QPolygonF observed;
  double coverage{0.0};
  double board_frame_ratio{-1.0};
  bool inside{false};
  bool orientation_ready{false};
  bool pose_valid{false};
  bool shape_mode{false};
  double shape_error_ratio{-1.0};
  QString shape_direction;
  double orientation_error_deg{-1.0};
  std::array<double, 3> target_xyz_deg{};
  std::array<double, 3> camera_rotation_correction_deg{};
  bool ready{false};
  QString instruction;
};

// 每张图由 config 中的投影中心、面积和相机光学坐标系 XYZ 姿态定义。
class GuidedCapturePlan
{
public:
  static constexpr int kImageCount = 18;
  static QString validationError(const GuideSettings & guide, const CalibrationSettings & board);
  static GuidedCaptureDecision evaluate(
    int step, const CalibrationSettings & board, const GuideSettings & guide,
    const GuideDetection & detection);
  static QPolygonF target(
    int step, const CalibrationSettings & board, const GuideSettings & guide,
    const QSize & frame);
  static QString instruction(int step);
};

}  // namespace danmucamera
