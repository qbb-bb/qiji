#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace danmucamera
{
struct BoardSpec
{
  std::string type{"chessboard"};
  int columns{7};
  int rows{6};
  double spacing{0.06};
  double marker_length{0.04};
  int dictionary_id{0};
};

struct BoardObservation
{
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  cv::Mat preview;
  std::string message;
  bool valid{false};
};

BoardObservation detectBoard(const cv::Mat & image, const BoardSpec & spec);
}  // namespace danmucamera
