#include "intrinsic_guided_pose.hpp"
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace danmucamera
{
namespace
{
constexpr double kRad = 3.14159265358979323846 / 180.0;
std::vector<cv::Point3f> boardCorners(const GuideBoard & board)
{
  const float x = static_cast<float>(board.width / 2);
  const float y = static_cast<float>(board.height / 2);
  return {{-x, -y, 0}, {x, -y, 0}, {x, y, 0}, {-x, y, 0}};
}
std::vector<cv::Point2d> project(const GuideBoard & board, const GuideCamera & camera,
  const cv::Matx33d & R, const cv::Vec3d & t)
{
  cv::Vec3d rvec;
  cv::Rodrigues(cv::Mat(R), rvec);
  std::vector<cv::Point2f> projected;
  const auto corners = boardCorners(board);
  if (camera.fisheye) {
    cv::fisheye::projectPoints(corners, projected, rvec, t, cv::Mat(camera.K),
      cv::Mat(camera.distortion));
  } else {
    cv::projectPoints(corners, rvec, t, cv::Mat(camera.K),
      cv::Mat(camera.distortion), projected);
  }
  return {projected.begin(), projected.end()};
}
double signedArea(const std::vector<cv::Point2d> & points)
{
  double sum = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % points.size()];
    sum += a.x * b.y - a.y * b.x;
  }
  return sum / 2;
}
cv::Point2d mean(const std::vector<cv::Point2d> & points)
{
  cv::Point2d result;
  for (const auto & p : points) {result += p;}
  return result * (1.0 / points.size());
}
bool validPoints(const std::vector<cv::Point2d> & points)
{
  if (points.size() != 4) {return false;}
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return false;}
  }
  return true;
}
bool centerAt(const GuideBoard & board, const GuideCamera & camera,
  const cv::Matx33d & R, double z, cv::Point2d desired, cv::Vec3d & t,
  std::vector<cv::Point2d> & corners)
{
  t = {(desired.x - camera.K(0, 2)) * z / camera.K(0, 0),
    (desired.y - camera.K(1, 2)) * z / camera.K(1, 1), z};
  for (int i = 0; i < 12; ++i) {
    corners = project(board, camera, R, t);
    if (!validPoints(corners)) {return false;}
    const cv::Point2d delta = desired - mean(corners);
    if (cv::norm(delta) < 0.01) {return true;}
    const double h = std::max(z * 1e-4, 1e-7);
    const auto px = project(board, camera, R, t + cv::Vec3d(h, 0, 0));
    const auto py = project(board, camera, R, t + cv::Vec3d(0, h, 0));
    const cv::Point2d dx = (mean(px) - mean(corners)) * (1.0 / h);
    const cv::Point2d dy = (mean(py) - mean(corners)) * (1.0 / h);
    const double determinant = dx.x * dy.y - dx.y * dy.x;
    if (!std::isfinite(determinant) || std::abs(determinant) < 1e-9) {return false;}
    t[0] += (delta.x * dy.y - delta.y * dy.x) / determinant;
    t[1] += (dx.x * delta.y - dx.y * delta.x) / determinant;
  }
  corners = project(board, camera, R, t);
  return validPoints(corners) && cv::norm(desired - mean(corners)) < 0.5;
}
}  // namespace

GuideBoard guideBoard(const BoardSpec & board, double physical_width, double physical_height)
{
  GuideBoard result;
  if (board.columns < 2 || board.rows < 2 || !std::isfinite(board.spacing) ||
    board.spacing <= 0) {return result;}
  double left = 0, top = 0, right = (board.columns - 1) * board.spacing;
  double bottom = (board.rows - 1) * board.spacing;
  if (board.type == "circles_asymmetric") {
    right = (2 * (board.columns - 1) + (board.rows > 1 ? 1 : 0)) * board.spacing;
  } else if (board.type == "charuco") {
    if (board.columns < 3 || board.rows < 3) {return result;}
    left = top = board.spacing;
    right = (board.columns - 1) * board.spacing;
    bottom = (board.rows - 1) * board.spacing;
  } else if (board.type == "aprilgrid") {
    if (!std::isfinite(board.marker_length) || board.marker_length <= 0 ||
      board.marker_length >= board.spacing) {return result;}
    left = top = -board.marker_length / 2;
    right += board.marker_length / 2;
    bottom += board.marker_length / 2;
  } else if (board.type != "chessboard" && board.type != "circles_symmetric") {
    return result;
  }
  result.model_center = {(left + right) / 2, (top + bottom) / 2};
  result.width = physical_width > 0 ? physical_width : right - left;
  result.height = physical_height > 0 ? physical_height : bottom - top;
  return result;
}
cv::Matx33d guideRotation(double roll, double pitch, double yaw)
{
  const double x = roll * kRad, y = pitch * kRad, z = yaw * kRad;
  const double cx = std::cos(x), sx = std::sin(x);
  const double cy = std::cos(y), sy = std::sin(y);
  const double cz = std::cos(z), sz = std::sin(z);
  const cv::Matx33d Rx(1, 0, 0, 0, cx, -sx, 0, sx, cx);
  const cv::Matx33d Ry(cy, 0, sy, 0, 1, 0, -sy, 0, cy);
  const cv::Matx33d Rz(cz, -sz, 0, sz, cz, 0, 0, 0, 1);
  // 固定相机轴：先 X，接着 Y，最后 Z；每步作用于前一步结果。
  cv::Matx33d rotation = cv::Matx33d::eye();
  rotation = Rx * rotation;
  rotation = Ry * rotation;
  rotation = Rz * rotation;
  return rotation;
}
cv::Vec3d guideCameraRotationCorrectionDeg(
  const cv::Matx33d & target, const cv::Matx33d & actual)
{
  // 左乘的相对旋转作用于相机光学坐标，方向指向目标姿态。
  cv::Vec3d rvec;
  cv::Rodrigues(cv::Mat(target * actual.t()), rvec);
  return rvec * (1.0 / kRad);
}
double guideRotationError(const cv::Matx33d & target, const cv::Matx33d & actual)
{
  const cv::Matx33d relative = target.t() * actual;
  const double cosine = std::clamp((cv::trace(relative) - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) / kRad;
}
GuideCamera visualGuideCamera(cv::Size frame_size, double perspective_strength)
{
  GuideCamera camera;
  if (frame_size.width <= 0 || frame_size.height <= 0 ||
    !std::isfinite(perspective_strength) || perspective_strength <= 0) {return camera;}
  // 无量纲视觉透视强度：仅控制绿框收敛程度，不估计真实焦距或视场角。
  const double scale = static_cast<double>(frame_size.width) / perspective_strength;
  camera.K = cv::Matx33d(scale, 0, frame_size.width / 2.0,
    0, scale, frame_size.height / 2.0, 0, 0, 1);
  camera.distortion.assign(5, 0.0);
  return camera;
}
GuideCamera scaledGuideCamera(const GuideCamera & camera, cv::Size calibration_size,
  cv::Size frame_size)
{
  GuideCamera result = camera;
  const double sx = static_cast<double>(frame_size.width) / calibration_size.width;
  const double sy = static_cast<double>(frame_size.height) / calibration_size.height;
  for (int c = 0; c < 3; ++c) {result.K(0, c) *= sx; result.K(1, c) *= sy;}
  return result;
}
bool guideTarget(const GuideBoard & board, const GuideCamera & camera, cv::Size frame,
  cv::Point2d center_ratio, double area_ratio, const std::array<double, 3> & xyz_deg,
  GuidePose & pose, std::string & error)
{
  if (!(board.width > 0 && board.height > 0) || frame.width <= 0 || frame.height <= 0 ||
    !(area_ratio > 0 && area_ratio < 1) || !(camera.K(0, 0) > 0 && camera.K(1, 1) > 0)) {
    error = "标定板物理尺寸、图像大小、内参或目标面积无效"; return false;
  }
  try {
  pose.R = guideRotation(xyz_deg[0], xyz_deg[1], xyz_deg[2]);
  const double desired_area = area_ratio * frame.area();
  const cv::Point2d desired(center_ratio.x * frame.width, center_ratio.y * frame.height);
  const cv::Vec3d normal = pose.R * cv::Vec3d(0, 0, 1);
  const double initial_z = std::sqrt(camera.K(0, 0) * camera.K(1, 1) *
    board.width * board.height * std::abs(normal[2]) / desired_area);
  if (!std::isfinite(initial_z) || initial_z <= 0) {
    error = "目标板面姿态或面积无法投影"; return false;
  }
  auto at = [&](double z, cv::Vec3d & t, std::vector<cv::Point2d> & corners) {
      for (const auto & corner : boardCorners(board)) {
        if ((pose.R * cv::Vec3d(corner.x, corner.y, 0))[2] + z <= 1e-6) {return false;}
      }
      return centerAt(board, camera, pose.R, z, desired, t, corners);
    };
  double low = initial_z * 0.05, high = initial_z * 20;
  cv::Vec3d t;
  std::vector<cv::Point2d> corners;
  for (int i = 0; i < 60; ++i) {
    const double mid = (low + high) / 2;
    if (!at(mid, t, corners)) {low = mid; continue;}
    if (std::abs(signedArea(corners)) > desired_area) {low = mid;} else {high = mid;}
  }
  if (!at((low + high) / 2, t, corners) ||
    std::abs(std::abs(signedArea(corners)) / desired_area - 1.0) > 0.01) {
    error = "无法求得目标面积对应的工作距离"; return false;
  }
  pose.t = t;
  pose.corners = std::move(corners);
  return true;
  } catch (const cv::Exception & e) {
    error = std::string("目标三维投影失败: ") + e.what();
    return false;
  }
}
bool guideObserved(const BoardObservation & observation, const GuideBoard & board,
  const GuideCamera & camera, GuidePose & pose, std::string & error)
{
  if (!observation.valid || observation.object_points.size() < 4 ||
    observation.object_points.size() != observation.image_points.size() ||
    !(board.width > 0 && board.height > 0)) {
    error = "特征点不足或标定板物点无效"; return false;
  }
  try {
  std::vector<cv::Point3f> centered = observation.object_points;
  for (auto & point : centered) {
    point.x -= static_cast<float>(board.model_center.x);
    point.y -= static_cast<float>(board.model_center.y);
  }
  std::vector<cv::Point2f> image_points = observation.image_points;
  cv::Mat distortion(camera.distortion);
  if (camera.fisheye) {
    cv::fisheye::undistortPoints(image_points, image_points,
      cv::Mat(camera.K), distortion, cv::Mat(), cv::Mat(camera.K));
    distortion = cv::Mat();
  }
  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(centered, image_points, cv::Mat(camera.K), distortion,
    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE) || tvec[2] <= 0) {
    error = "PnP 姿态估计失败"; return false;
  }
  cv::Rodrigues(rvec, pose.R);
  pose.t = tvec;
  pose.corners = project(board, camera, pose.R, pose.t);
  if (!validPoints(pose.corners) || signedArea(pose.corners) <= 0) {
    error = "PnP 投影四角无效"; return false;
  }
  return true;
  } catch (const cv::Exception & e) {
    error = std::string("PnP 姿态估计异常: ") + e.what();
    return false;
  }
}
bool guideObservedOutline(const BoardObservation & observation, const GuideBoard & board,
  std::vector<cv::Point2d> & corners, std::string & error)
{
  if (!observation.valid || observation.object_points.size() < 4 ||
    observation.object_points.size() != observation.image_points.size() ||
    !(board.width > 0 && board.height > 0)) {
    error = "特征点不足，无法拟合画面四边形"; return false;
  }
  try {
    std::vector<cv::Point2f> object_plane;
    object_plane.reserve(observation.object_points.size());
    for (const auto & point : observation.object_points) {
      object_plane.emplace_back(point.x - board.model_center.x,
        point.y - board.model_center.y);
    }
    const cv::Mat homography = cv::findHomography(object_plane,
      observation.image_points, cv::RANSAC, 3.0);
    if (homography.empty()) {error = "特征点不能形成稳定的四边形"; return false;}
    const float x = static_cast<float>(board.width / 2);
    const float y = static_cast<float>(board.height / 2);
    const std::vector<cv::Point2f> model = {
      {-x,-y}, {x,-y}, {x,y}, {-x,y}};
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(model, projected, homography);
    corners.assign(projected.begin(), projected.end());
    if (!validPoints(corners) || signedArea(corners) <= 0) {
      error = "画面四边形无效"; return false;
    }
    return true;
  } catch (const cv::Exception & e) {
    error = std::string("画面四边形拟合失败: ") + e.what();
    return false;
  }
}
}  // namespace danmucamera
