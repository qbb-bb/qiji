#include "guided_capture_plan.hpp"
#include "guided_pose.hpp"
#include <QColor>
#include <algorithm>
#include <cmath>

namespace danmucamera
{
namespace
{
bool fraction(double value) {return std::isfinite(value) && value >= 0.0 && value <= 1.0;}
BoardSpec boardSpec(const CalibrationSettings & board)
{
  BoardSpec result;
  result.type = board.board_type.toStdString();
  result.columns = board.columns;
  result.rows = board.rows;
  result.spacing = board.spacing;
  result.marker_length = board.marker_length;
  return result;
}
bool manualIntrinsics(const CalibrationSettings & board)
{
  return board.guide_use_initial && board.fx > 0 && board.fy > 0;
}
bool halfTurnSymmetry(const CalibrationSettings & board)
{
  return board.board_type == QStringLiteral("chessboard") ||
    board.board_type == QStringLiteral("circles_symmetric");
}
bool cameraFor(const CalibrationSettings & board, const GuideSettings & guide,
  const QSize & frame, GuideCamera & camera)
{
  if (frame.isEmpty()) {return false;}
  const bool ui_intrinsics = board.guide_use_initial && manualIntrinsics(board);
  const bool configured = !ui_intrinsics &&
    guide.camera_matrix[0] > 0 && guide.camera_matrix[4] > 0;
  if (configured) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {camera.K(r, c) = guide.camera_matrix[3 * r + c];}
    }
    camera.distortion = guide.distortion;
  } else if (manualIntrinsics(board)) {
    camera.K = cv::Matx33d(board.fx, 0, board.cx, 0, board.fy, board.cy, 0, 0, 1);
    camera.distortion.assign(board.initial_distortion.begin(), board.initial_distortion.end());
  } else {
    camera = visualGuideCamera(cv::Size(frame.width(), frame.height()),
      guide.visual_perspective_strength);
  }
  camera.fisheye = (configured || (manualIntrinsics(board))) &&
    board.camera_model == QStringLiteral("fisheye");
  if (camera.distortion.empty()) {camera.distortion.resize(camera.fisheye ? 4 : 5);}
  if (camera.fisheye) {camera.distortion.resize(4);}
  else if (camera.distortion.size() != 4 && camera.distortion.size() != 5 &&
    camera.distortion.size() != 8) {camera.distortion.resize(5);}
  if (!(camera.K(0, 0) > 0 && camera.K(1, 1) > 0) ||
    !std::isfinite(camera.K(0, 2)) || !std::isfinite(camera.K(1, 2))) {return false;}
  for (double d : camera.distortion) {if (!std::isfinite(d)) {return false;}}
  const int source_width = ui_intrinsics ? board.guide_initial_width : guide.camera_width;
  const int source_height = ui_intrinsics ? board.guide_initial_height : guide.camera_height;
  if ((configured || manualIntrinsics(board)) &&
    source_width > 0 && source_height > 0) {
    const double sx = static_cast<double>(frame.width()) / source_width;
    const double sy = static_cast<double>(frame.height()) / source_height;
    if (std::abs(sx - sy) > 0.01 * std::max(sx, sy)) {return false;}
    camera = scaledGuideCamera(camera,
      cv::Size(source_width, source_height),
      cv::Size(frame.width(), frame.height()));
  }
  return true;
}
QPolygonF normalized(const std::vector<cv::Point2d> & corners, const QSize & frame)
{
  QPolygonF polygon;
  for (const auto & corner : corners) {
    polygon << QPointF(corner.x / frame.width(), corner.y / frame.height());
  }
  return polygon;
}
double cross(const QPointF & a, const QPointF & b, const QPointF & p)
{
  return (b.x() - a.x()) * (p.y() - a.y()) -
    (b.y() - a.y()) * (p.x() - a.x());
}

double area(const QPolygonF & polygon)
{
  if (polygon.size() < 3) {return 0.0;}
  double twice_area = 0.0;
  for (int i = 0; i < polygon.size(); ++i) {
    const QPointF & a = polygon[i];
    const QPointF & b = polygon[(i + 1) % polygon.size()];
    twice_area += a.x() * b.y() - b.x() * a.y();
  }
  return std::abs(twice_area) / 2.0;
}

// Sutherland-Hodgman 裁剪：clip 为顺时针屏幕坐标下的凸四边形。
QPolygonF intersectConvex(const QPolygonF & subject, const QPolygonF & clip)
{
  QPolygonF output = subject;
  for (int edge = 0; edge < clip.size() && !output.isEmpty(); ++edge) {
    const QPointF & a = clip[edge];
    const QPointF & b = clip[(edge + 1) % clip.size()];
    const QPolygonF input = output;
    output.clear();
    QPointF previous = input.last();
    double previous_side = cross(a, b, previous);
    for (const QPointF & current : input) {
      const double current_side = cross(a, b, current);
      const bool previous_inside = previous_side >= -1e-9;
      const bool current_inside = current_side >= -1e-9;
      if (previous_inside != current_inside) {
        const double divisor = previous_side - current_side;
        if (std::abs(divisor) > 1e-12) {
          const double t = previous_side / divisor;
          output << previous + (current - previous) * t;
        }
      }
      if (current_inside) {output << current;}
      previous = current;
      previous_side = current_side;
    }
  }
  return output;
}
}  // namespace

QString GuidedCapturePlan::validationError(const GuideSettings & guide,
  const CalibrationSettings & board)
{
  if (!guide.targets_configured) {
    return QStringLiteral("guide 的中心、XYZ 姿态和面积数组必须各有 18 个值；yaw 可兼容旧 angle_deg。");
  }
  if (!(guide.max_orientation_error_deg > 0 && guide.max_orientation_error_deg <= 180) ||
    !std::isfinite(guide.max_orientation_error_deg)) {
    return QStringLiteral("max_orientation_error_deg 必须在 0～180 度之间。");
  }
  const GuideBoard model = guideBoard(boardSpec(board), board.board_physical_width,
    board.board_physical_height);
  if (!(model.width > 0 && model.height > 0)) {
    return QStringLiteral("标定板物理尺寸无效，请检查板型、间距和宽高。");
  }
  if (guide.camera_matrix_invalid) {
    return QStringLiteral("guide.camera_matrix 必须是 3×3 矩阵，恰好 9 个值。");
  }
  if (guide.camera_matrix[0] > 0 || guide.camera_matrix[4] > 0) {
    for (double value : guide.camera_matrix) {
      if (!std::isfinite(value)) {return QStringLiteral("guide.camera_matrix 含无效数值。");}
    }
    if (!(guide.camera_matrix[0] > 0 && guide.camera_matrix[4] > 0) ||
      std::abs(guide.camera_matrix[8] - 1.0) > 1e-6) {
      return QStringLiteral("guide.camera_matrix 的焦距必须为正，右下角必须为 1。");
    }
  }
  for (double value : guide.distortion) {
    if (!std::isfinite(value)) {return QStringLiteral("guide.distortion 含无效数值。");}
  }
  if (!guide.distortion.empty() &&
    ((board.camera_model == QStringLiteral("fisheye") && guide.distortion.size() != 4) ||
    (board.camera_model != QStringLiteral("fisheye") &&
    guide.distortion.size() != 4 && guide.distortion.size() != 5 &&
    guide.distortion.size() != 8))) {
    return QStringLiteral("guide.distortion 系数个数与针孔/鱼眼模型不匹配。");
  }
  if (!std::isfinite(guide.visual_perspective_strength) ||
    guide.visual_perspective_strength < 0.5 ||
    guide.visual_perspective_strength > 4.0) {
    return QStringLiteral("视觉透视强度需在 0.5～4.0 之间。");
  }
  if (!fraction(guide.max_shape_corner_error_ratio) ||
    guide.max_shape_corner_error_ratio == 0) {
    return QStringLiteral("画面四角形状误差比例配置无效。");
  }
  if ((guide.camera_width == 0) != (guide.camera_height == 0) ||
    guide.camera_width < 0 || guide.camera_height < 0) {
    return QStringLiteral("guide.camera_width 和 camera_height 必须同时有效，或同时为 0。");
  }
  if (!fraction(guide.min_guide_coverage_ratio) || guide.min_guide_coverage_ratio == 0 ||
    !fraction(guide.min_board_frame_ratio) || guide.min_board_frame_ratio == 0 ||
    !fraction(guide.inside_margin_ratio) || guide.inside_margin_ratio > 0.2 ||
    !fraction(guide.frame_edge_margin_ratio) ||
    guide.frame_edge_margin_ratio > 0.1) {
    return QStringLiteral("覆盖率、板面面积比例或边缘余量配置无效。");
  }
  for (int i = 0; i < kImageCount; ++i) {
    if (!fraction(guide.centers[i].x()) || !fraction(guide.centers[i].y()) ||
      !fraction(guide.area_ratios[i]) || guide.area_ratios[i] == 0 ||
      !std::isfinite(guide.roll_x_deg[i]) || std::abs(guide.roll_x_deg[i]) > 180 ||
      !std::isfinite(guide.pitch_y_deg[i]) || std::abs(guide.pitch_y_deg[i]) > 180 ||
      !std::isfinite(guide.yaw_z_deg[i]) || std::abs(guide.yaw_z_deg[i]) > 180) {
      return QStringLiteral("第 %1 张的中心、面积或 XYZ 角度配置无效。").arg(i + 1);
    }
  }
  if (!QColor(guide.guide_color).isValid() || !QColor(guide.ready_color).isValid() ||
    !QColor(guide.detected_color).isValid() || guide.guide_line_width < 1 ||
    guide.ready_line_width < 1 || guide.detected_line_width < 1 ||
    !std::isfinite(guide.corner_radius_px) || guide.corner_radius_px < 0) {
    return QStringLiteral("引导框颜色、线宽或圆角配置无效。");
  }
  return {};
}

QPolygonF GuidedCapturePlan::target(int step, const CalibrationSettings & board,
  const GuideSettings & guide, const QSize & frame)
{
  GuideCamera camera;
  if (!cameraFor(board, guide, frame, camera)) {return {};}
  const int i = std::clamp(step, 0, kImageCount - 1);
  GuidePose pose;
  std::string error;
  const bool visual_shape = !board.guide_use_initial &&
    !(guide.camera_matrix[0] > 0 && guide.camera_matrix[4] > 0) &&
    !manualIntrinsics(board);
  // 无内参的画面形状模式：先在像面中心形成 X/Y 双向透视，再整体平移到九宫格。
  // 这样目标位置不会改变板面的倾斜形状；已知 K 时仍采用真实离轴投影。
  const GuideBoard physical_board = guideBoard(boardSpec(board),
    board.board_physical_width, board.board_physical_height);
  const cv::Size frame_size(frame.width(), frame.height());
  const std::array<double, 3> xyz = {guide.roll_x_deg[i],
    guide.pitch_y_deg[i], guide.yaw_z_deg[i]};
  const double edge = guide.frame_edge_margin_ratio;
  auto overflow = [edge](const QPolygonF & polygon, double & dx, double & dy) {
      double min_x = 1.0, min_y = 1.0, max_x = 0.0, max_y = 0.0;
      for (const QPointF & corner : polygon) {
        min_x = std::min(min_x, corner.x());
        min_y = std::min(min_y, corner.y());
        max_x = std::max(max_x, corner.x());
        max_y = std::max(max_y, corner.y());
      }
      if (max_x - min_x > 1.0 - 2.0 * edge ||
        max_y - min_y > 1.0 - 2.0 * edge) {return false;}
      dx = min_x < edge ? edge - min_x :
        (max_x > 1.0 - edge ? 1.0 - edge - max_x : 0.0);
      dy = min_y < edge ? edge - min_y :
        (max_y > 1.0 - edge ? 1.0 - edge - max_y : 0.0);
      return true;
    };
  if (visual_shape) {
    if (!guideTarget(physical_board, camera, frame_size, {0.5, 0.5},
      guide.area_ratios[i], xyz, pose, error)) {return {};}
    QPolygonF polygon = normalized(pose.corners, frame);
    polygon.translate(guide.centers[i].x() - 0.5, guide.centers[i].y() - 0.5);
    double dx = 0.0, dy = 0.0;
    if (!overflow(polygon, dx, dy)) {return {};}
    polygon.translate(dx, dy);
    return polygon;
  }
  // 已知 K/D 时不能在像面上平移四角：离轴透视和畸变会改变四边形形状。
  // 若配置中心使投影越界，更新三维目标中心并重新 projectPoints。
  cv::Point2d center(guide.centers[i].x(), guide.centers[i].y());
  for (int attempt = 0; attempt < 16; ++attempt) {
    if (!guideTarget(physical_board, camera, frame_size, center,
      guide.area_ratios[i], xyz, pose, error)) {return {};}
    const QPolygonF polygon = normalized(pose.corners, frame);
    double dx = 0.0, dy = 0.0;
    if (!overflow(polygon, dx, dy)) {return {};}
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {return polygon;}
    center.x += dx;
    center.y += dy;
    if (center.x < 0.0 || center.x > 1.0 ||
      center.y < 0.0 || center.y > 1.0) {return {};}
  }
  return {};
}

QString GuidedCapturePlan::instruction(int step)
{
  const int index = std::clamp(step, 0, kImageCount - 1);
  return QStringLiteral("第 %1/18 张：将标定板移到荧光框，按框的透视方向倾斜并完整入画")
    .arg(index + 1);
}

GuidedCaptureDecision GuidedCapturePlan::evaluate(int step,
  const CalibrationSettings & board, const GuideSettings & guide,
  const GuideDetection & detection)
{
  GuidedCaptureDecision decision;
  const int i = std::clamp(step, 0, kImageCount - 1);
  decision.target_xyz_deg = {guide.roll_x_deg[i], guide.pitch_y_deg[i], guide.yaw_z_deg[i]};
  decision.target = target(step, board, guide, detection.frame_size);
  decision.instruction = instruction(step);
  decision.board_frame_ratio = detection.board_area_ratio;
  decision.pose_valid = detection.pose_valid;
  decision.shape_mode = !board.guide_use_initial &&
    !(guide.camera_matrix[0] > 0 && guide.camera_matrix[4] > 0) &&
    !manualIntrinsics(board);
  if (!decision.shape_mode && detection.pose_valid) {
    const cv::Matx33d target_rotation = guideRotation(
      guide.roll_x_deg[i], guide.pitch_y_deg[i], guide.yaw_z_deg[i]);
    const auto & r = detection.pose_rotation;
    cv::Matx33d actual_rotation(r[0], r[1], r[2],
      r[3], r[4], r[5], r[6], r[7], r[8]);
    // 无方向标记的棋盘/对称圆阵，角点可从对角开始编号；板面姿态相差
    // 局部 Z 轴半周时是同一物理目标，取与目标较近的等价旋转。
    if (halfTurnSymmetry(board)) {
      const cv::Matx33d half_turn(-1, 0, 0, 0, -1, 0, 0, 0, 1);
      const cv::Matx33d equivalent = actual_rotation * half_turn;
      if (guideRotationError(target_rotation, equivalent) <
        guideRotationError(target_rotation, actual_rotation)) {
        actual_rotation = equivalent;
      }
    }
    decision.orientation_error_deg = guideRotationError(
      target_rotation, actual_rotation);
    const cv::Vec3d correction = guideCameraRotationCorrectionDeg(
      target_rotation, actual_rotation);
    for (int axis = 0; axis < 3; ++axis) {
      decision.camera_rotation_correction_deg[axis] = correction[axis];
    }
    decision.orientation_ready =
      decision.orientation_error_deg <= guide.max_orientation_error_deg;
  }
  if (detection.frame_size.isEmpty() || decision.target.size() != 4 ||
    detection.board_polygon.size() != 4) {return decision;}
  decision.observed = normalized({
    {detection.board_polygon[0].x(), detection.board_polygon[0].y()},
    {detection.board_polygon[1].x(), detection.board_polygon[1].y()},
    {detection.board_polygon[2].x(), detection.board_polygon[2].y()},
    {detection.board_polygon[3].x(), detection.board_polygon[3].y()}}, detection.frame_size);
  if (decision.shape_mode && halfTurnSymmetry(board)) {
    const auto corner_distance = [&decision](int offset) {
        double total = 0.0;
        for (int corner = 0; corner < 4; ++corner) {
          const QPointF & actual = decision.observed[(corner + offset) % 4];
          const QPointF & desired = decision.target[corner];
          total += std::hypot(actual.x() - desired.x(), actual.y() - desired.y());
        }
        return total;
      };
    if (corner_distance(2) < corner_distance(0)) {
      QPolygonF relabeled;
      for (int corner = 0; corner < 4; ++corner) {
        relabeled << decision.observed[(corner + 2) % 4];
      }
      decision.observed = relabeled;
    }
  }
  const double target_diagonal = std::hypot(
    decision.target[2].x() - decision.target[0].x(),
    decision.target[2].y() - decision.target[0].y());
  if (decision.shape_mode && target_diagonal > 0 && area(decision.observed) > 0) {
    double corner_error = 0;
    for (int corner = 0; corner < 4; ++corner) {
      corner_error += std::hypot(
        decision.target[corner].x() - decision.observed[corner].x(),
        decision.target[corner].y() - decision.observed[corner].y());
    }
    decision.shape_error_ratio = corner_error / (4 * target_diagonal);
    decision.orientation_ready =
      decision.shape_error_ratio <= guide.max_shape_corner_error_ratio;
    const auto edge = [](const QPolygonF & polygon, int a, int b) {
        return std::hypot(polygon[b].x() - polygon[a].x(),
          polygon[b].y() - polygon[a].y());
      };
    const double top_target = edge(decision.target, 0, 1);
    const double bottom_target = edge(decision.target, 3, 2);
    const double top_actual = edge(decision.observed, 0, 1);
    const double bottom_actual = edge(decision.observed, 3, 2);
    const double left_target = edge(decision.target, 0, 3);
    const double right_target = edge(decision.target, 1, 2);
    const double left_actual = edge(decision.observed, 0, 3);
    const double right_actual = edge(decision.observed, 1, 2);
    if (std::min({top_target, bottom_target, top_actual, bottom_actual,
      left_target, right_target, left_actual, right_actual}) > 1e-8) {
      const double x_error = std::log(top_target / bottom_target) -
        std::log(top_actual / bottom_actual);
      const double y_error = std::log(left_target / right_target) -
        std::log(left_actual / right_actual);
      const auto yaw = [](const QPolygonF & polygon) {
          return std::atan2(polygon[1].y() - polygon[0].y(),
            polygon[1].x() - polygon[0].x());
        };
      const double z_error = std::atan2(std::sin(yaw(decision.target) -
        yaw(decision.observed)), std::cos(yaw(decision.target) -
        yaw(decision.observed)));
      if (std::abs(z_error) > 0.08 && std::abs(z_error) >
        std::max(std::abs(x_error), std::abs(y_error))) {
        decision.shape_direction = z_error > 0 ?
          QStringLiteral("请在画面中顺时针旋转标定板") :
          QStringLiteral("请在画面中逆时针旋转标定板");
      } else if (std::abs(x_error) > std::abs(y_error)) {
        decision.shape_direction = x_error > 0 ?
          QStringLiteral("请调整上下倾斜，使板上边在画面中相对更宽") :
          QStringLiteral("请调整上下倾斜，使板下边在画面中相对更宽");
      } else {
        decision.shape_direction = y_error > 0 ?
          QStringLiteral("请调整左右倾斜，使板左边在画面中相对更长") :
          QStringLiteral("请调整左右倾斜，使板右边在画面中相对更长");
      }
    }
  }
  const double target_area = area(decision.target);
  decision.coverage = target_area > 0 ?
    area(intersectConvex(decision.observed, decision.target)) / target_area : 0;
  decision.inside = true;
  const double margin = guide.inside_margin_ratio;
  const double frame_margin = guide.frame_edge_margin_ratio;
  for (const QPointF & corner : decision.observed) {
    if (corner.x() < frame_margin - 1e-6 ||
      corner.x() > 1 - frame_margin + 1e-6 ||
      corner.y() < frame_margin - 1e-6 ||
      corner.y() > 1 - frame_margin + 1e-6) {decision.inside = false; break;}
    for (int edge = 0; edge < decision.target.size(); ++edge) {
      const QPointF a = decision.target[edge];
      const QPointF b = decision.target[(edge + 1) % decision.target.size()];
      const double length = std::hypot(b.x() - a.x(), b.y() - a.y());
      if (cross(a, b, corner) < -margin * length) {decision.inside = false; break;}
    }
    if (!decision.inside) {break;}
  }
  decision.ready = detection.complete && decision.inside &&
    decision.coverage >= guide.min_guide_coverage_ratio &&
    decision.board_frame_ratio >= guide.min_board_frame_ratio &&
    decision.orientation_ready;
  return decision;
}
}  // namespace danmucamera
