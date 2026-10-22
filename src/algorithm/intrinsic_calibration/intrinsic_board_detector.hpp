#pragma once

#include "board_spec.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace danmucamera
{
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
