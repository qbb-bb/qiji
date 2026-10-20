#pragma once

#include "board_detector.hpp"
#include <opencv2/calib3d.hpp>
#include <array>
#include <string>
#include <vector>

namespace danmucamera
{
struct GuideCamera
{
  cv::Matx33d K{cv::Matx33d::eye()};
  std::vector<double> distortion;
  bool fisheye{false};
};
struct GuideBoard
{
  cv::Point2d model_center;
  double width{0};
  double height{0};
};
struct GuidePose
{
  cv::Matx33d R{cv::Matx33d::eye()};
  cv::Vec3d t{0, 0, 0};
  std::vector<cv::Point2d> corners;
};
GuideBoard guideBoard(const BoardSpec & board, double physical_width = 0,
  double physical_height = 0);
cv::Matx33d guideRotation(double roll, double pitch, double yaw);
cv::Vec3d guideCameraRotationCorrectionDeg(
  const cv::Matx33d & target, const cv::Matx33d & actual);
double guideRotationError(const cv::Matx33d & target, const cv::Matx33d & actual);
GuideCamera visualGuideCamera(cv::Size frame_size, double perspective_strength);
GuideCamera scaledGuideCamera(const GuideCamera & camera, cv::Size calibration_size,
  cv::Size frame_size);
bool guideTarget(const GuideBoard & board, const GuideCamera & camera, cv::Size frame,
  cv::Point2d center_ratio, double area_ratio, const std::array<double, 3> & xyz_deg,
  GuidePose & pose, std::string & error);
bool guideObserved(const BoardObservation & observation, const GuideBoard & board,
  const GuideCamera & camera, GuidePose & pose, std::string & error);
bool guideObservedOutline(const BoardObservation & observation, const GuideBoard & board,
  std::vector<cv::Point2d> & corners, std::string & error);
}  // namespace danmucamera
