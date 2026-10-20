#include "board_detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <stdexcept>

namespace danmucamera
{
BoardObservation detectBoard(const cv::Mat & image, const BoardSpec & spec)
{
  BoardObservation result;
  if (image.empty()) {result.message = "图像无法读取"; return result;}
  if (spec.columns < 2 || spec.rows < 2 || spec.spacing <= 0) {
    result.message = "标定板参数无效";
    return result;
  }
  cv::Mat gray;
  if (image.channels() == 1) {gray = image; cv::cvtColor(image, result.preview, cv::COLOR_GRAY2BGR);}
  else {cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); result.preview = image.clone();}
  const cv::Size board_size(spec.columns, spec.rows);

  if (spec.type == "chessboard") {
    result.valid = cv::findChessboardCornersSB(gray, board_size, result.image_points,
      cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
    if (result.valid) {
      for (int row = 0; row < spec.rows; ++row) {
        for (int col = 0; col < spec.columns; ++col) {
          result.object_points.emplace_back(col * spec.spacing, row * spec.spacing, 0.0);
        }
      }
      cv::drawChessboardCorners(result.preview, board_size, result.image_points, true);
    }
  } else if (spec.type == "circles_symmetric" || spec.type == "circles_asymmetric") {
    const bool asymmetric = spec.type == "circles_asymmetric";
    const int flags = (asymmetric ? cv::CALIB_CB_ASYMMETRIC_GRID :
      cv::CALIB_CB_SYMMETRIC_GRID) | cv::CALIB_CB_CLUSTERING;
    result.valid = cv::findCirclesGrid(gray, board_size, result.image_points, flags);
    if (!result.valid) {
      result.image_points.clear();
      result.valid = cv::findCirclesGrid(gray, board_size, result.image_points,
        asymmetric ? cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID);
    }
    if (result.valid) {
      for (int row = 0; row < spec.rows; ++row) {
        for (int col = 0; col < spec.columns; ++col) {
          const double x = asymmetric ? (2 * col + (row % 2)) * spec.spacing : col * spec.spacing;
          result.object_points.emplace_back(x, row * spec.spacing, 0.0);
        }
      }
      cv::drawChessboardCorners(result.preview, board_size, result.image_points, true);
    }
  } else if (spec.type == "aprilgrid") {
    if (spec.marker_length <= 0 || spec.marker_length >= spec.spacing) {
      result.message = "AprilGrid 标签边长必须小于中心间距";
      return result;
    }
    const auto dictionary = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(spec.dictionary_id));
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids);
    if (!marker_ids.empty()) {
      cv::aruco::drawDetectedMarkers(result.preview, marker_corners, marker_ids);
      const float half = static_cast<float>(spec.marker_length / 2.0);
      for (size_t i = 0; i < marker_ids.size(); ++i) {
        const int id = marker_ids[i];
        if (id < 0 || id >= spec.columns * spec.rows) {continue;}
        const float x = static_cast<float>((id % spec.columns) * spec.spacing);
        const float y = static_cast<float>((id / spec.columns) * spec.spacing);
        const std::array<cv::Point3f, 4> corners = {
          cv::Point3f(x - half, y - half, 0), cv::Point3f(x + half, y - half, 0),
          cv::Point3f(x + half, y + half, 0), cv::Point3f(x - half, y + half, 0)};
        for (int point = 0; point < 4; ++point) {
          result.object_points.push_back(corners[point]);
          result.image_points.push_back(marker_corners[i][point]);
        }
      }
      result.valid = result.image_points.size() >= 8;
    }
  } else if (spec.type == "charuco") {
    if (spec.columns < 3 || spec.rows < 3 ||
      spec.marker_length <= 0 || spec.marker_length >= spec.spacing) {
      result.message = "ChArUco 需要至少 3×3 方格，且标记边长小于方格边长";
      return result;
    }
    const auto dictionary = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(spec.dictionary_id));
    const auto board = cv::aruco::CharucoBoard::create(
      spec.columns, spec.rows, static_cast<float>(spec.spacing),
      static_cast<float>(spec.marker_length), dictionary);
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids);
    if (!marker_ids.empty()) {
      cv::aruco::drawDetectedMarkers(result.preview, marker_corners, marker_ids);
      cv::Mat charuco_corners, charuco_ids;
      const int count = cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board, charuco_corners, charuco_ids);
      if (count >= 6) {
        result.valid = true;
        for (int i = 0; i < count; ++i) {
          const int id = charuco_ids.at<int>(i);
          result.image_points.push_back(charuco_corners.at<cv::Point2f>(i));
          result.object_points.push_back(board->chessboardCorners.at(id));
        }
        cv::aruco::drawDetectedCornersCharuco(result.preview, charuco_corners, charuco_ids);
      }
    }
  } else {
    throw std::invalid_argument("未知标定板类型: " + spec.type);
  }
  result.message = result.valid ? "识别到 " + std::to_string(result.image_points.size()) + " 个特征点" :
    (result.message.empty() ? "未完整识别标定板特征点" : result.message);
  return result;
}
}  // namespace danmucamera
