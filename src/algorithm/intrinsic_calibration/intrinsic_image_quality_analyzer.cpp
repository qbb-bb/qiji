#include "intrinsic_image_quality_analyzer.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace danmucamera
{
namespace
{

// 这些是统计分区，不是报警阈值；与当前 UI 的显示定义保持一致。
constexpr int kDarkLast = 63;
constexpr int kBrightFirst = 192;
constexpr int kBlackClipLast = 3;
constexpr int kWhiteClipFirst = 252;

bool validRatio(double value)
{
  return value >= 0.0 && value <= 1.0;
}

}  // namespace

ImageQualityAnalyzer::ImageQualityAnalyzer(ImageQualityConfig config)
: config_(std::move(config))
{
  validateConfig(config_);
}

void ImageQualityAnalyzer::validateConfig(const ImageQualityConfig & config)
{
  if (config.analysis_max_width <= 0 || config.analysis_max_height <= 0) {
    throw std::invalid_argument("quality.analysis_max_width/height 必须大于 0");
  }
  if (config.roi_margin_ratio < 0.0 || config.roi_margin_ratio > 1.0) {
    throw std::invalid_argument("quality.roi_margin_ratio 必须位于 [0, 1]");
  }
  if (config.roi_min_points < 3) {
    throw std::invalid_argument("quality.roi_min_points 必须至少为 3");
  }
  if (config.roi_min_pixels <= 0) {
    throw std::invalid_argument("quality.roi_min_pixels 必须大于 0");
  }
  if (!validRatio(config.min_board_area_ratio) ||
    !validRatio(config.max_board_area_ratio) ||
    config.min_board_area_ratio >= config.max_board_area_ratio)
  {
    throw std::invalid_argument(
      "quality 标定板占比阈值必须满足 0 <= min_board_area_ratio < max_board_area_ratio <= 1");
  }
  if (config.min_laplacian_variance < 0.0) {
    throw std::invalid_argument("quality.min_laplacian_variance 必须大于等于 0");
  }
  if (!validRatio(config.max_dark_ratio) || !validRatio(config.max_bright_ratio) ||
    !validRatio(config.max_black_clip_ratio) || !validRatio(config.max_white_clip_ratio))
  {
    throw std::invalid_argument("quality 的比例阈值必须位于 [0, 1]");
  }
  if (config.min_mean_gray < 0.0 || config.max_mean_gray > 255.0 ||
    config.min_mean_gray >= config.max_mean_gray)
  {
    throw std::invalid_argument("quality 平均灰度阈值必须满足 0 <= min < max <= 255");
  }

}

cv::Mat ImageQualityAnalyzer::buildBoardMask(
  const cv::Size & image_size,
  const std::vector<cv::Point2f> & board_points) const
{
  if (image_size.width <= 0 || image_size.height <= 0 ||
    static_cast<int>(board_points.size()) < config_.roi_min_points)
  {
    return {};
  }

  std::vector<cv::Point2f> valid_points;
  valid_points.reserve(board_points.size());
  for (const auto & point : board_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {continue;}
    // 允许少量点落在边界外，但过滤明显异常坐标。
    if (point.x < -image_size.width || point.x > image_size.width * 2.0f ||
      point.y < -image_size.height || point.y > image_size.height * 2.0f)
    {
      continue;
    }
    valid_points.push_back(point);
  }

  if (static_cast<int>(valid_points.size()) < config_.roi_min_points) {return {};}

  std::vector<cv::Point2f> hull;
  cv::convexHull(valid_points, hull, true, true);
  if (hull.size() < 3) {return {};}

  cv::Point2f center(0.0f, 0.0f);
  for (const auto & point : hull) {center += point;}
  center *= 1.0f / static_cast<float>(hull.size());

  // roi_margin_ratio 表示“每一侧”的扩展比例。
  // 例如 0.20 -> 凸包整体线性尺度约扩大到 1.40，能把棋盘格内角点区域
  // 扩到接近实际标定板边缘，同时尽量不带入大面积背景。
  const float expand = static_cast<float>(1.0 + 2.0 * config_.roi_margin_ratio);
  std::vector<cv::Point> polygon;
  polygon.reserve(hull.size());
  for (const auto & point : hull) {
    cv::Point2f expanded = center + (point - center) * expand;
    expanded.x = std::clamp(expanded.x, 0.0f, static_cast<float>(image_size.width - 1));
    expanded.y = std::clamp(expanded.y, 0.0f, static_cast<float>(image_size.height - 1));
    polygon.emplace_back(cvRound(expanded.x), cvRound(expanded.y));
  }

  cv::Mat mask = cv::Mat::zeros(image_size, CV_8UC1);
  cv::fillConvexPoly(mask, polygon, cv::Scalar(255), cv::LINE_8);
  return mask;
}

ImageQualityResult ImageQualityAnalyzer::evaluate(
  const cv::Mat & input,
  const std::vector<cv::Point2f> & board_points) const
{
  ImageQualityResult result;
  if (input.empty()) {
    result.exposure_warning = true;
    result.warning_reason = "unreadable";
    return result;
  }

  result.readable = true;
  result.board_area_min_ratio = config_.min_board_area_ratio;
  result.board_area_max_ratio = config_.max_board_area_ratio;
  for (const auto & point : board_points) {
    if (point.x < 0 || point.y < 0 || point.x >= input.cols || point.y >= input.rows) {continue;}
    const int column = std::min(2, static_cast<int>(3.0 * point.x / input.cols));
    const int row = std::min(2, static_cast<int>(3.0 * point.y / input.rows));
    ++result.corner_grid_counts[row * 3 + column];
  }

  cv::Mat gray;
  if (input.channels() == 1) {
    gray = input;
  } else if (input.channels() == 3) {
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
  } else if (input.channels() == 4) {
    cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::invalid_argument("ImageQualityAnalyzer 仅支持 1/3/4 通道图像");
  }

  cv::Mat analysis_mask = buildBoardMask(gray.size(), board_points);
  if (!analysis_mask.empty()) {
    const int board_pixels = cv::countNonZero(analysis_mask);
    result.board_area_ratio =
      static_cast<double>(board_pixels) / static_cast<double>(gray.total());
    result.board_size_warning =
      result.board_area_ratio < config_.min_board_area_ratio ||
      result.board_area_ratio > config_.max_board_area_ratio;
    if (board_pixels < config_.roi_min_pixels) {analysis_mask = cv::Mat();}
  }
  result.roi_used = !analysis_mask.empty();

  // 统一分析分辨率上限，降低原始分辨率对拉普拉斯方差阈值的影响。
  const double scale = std::min({
    1.0,
    static_cast<double>(config_.analysis_max_width) / gray.cols,
    static_cast<double>(config_.analysis_max_height) / gray.rows});
  if (scale < 1.0) {
    cv::resize(gray, gray, cv::Size(), scale, scale, cv::INTER_AREA);
    if (!analysis_mask.empty()) {
      cv::resize(analysis_mask, analysis_mask, cv::Size(), scale, scale, cv::INTER_NEAREST);
    }
  }

  const int pixel_count = analysis_mask.empty() ?
    static_cast<int>(gray.total()) : cv::countNonZero(analysis_mask);
  if (pixel_count <= 0) {
    result.readable = false;
    result.exposure_warning = true;
    result.warning_reason = "unreadable";
    return result;
  }
  result.analyzed_pixels = pixel_count;

  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);
  cv::Scalar average, deviation;
  cv::meanStdDev(laplacian, average, deviation, analysis_mask);
  result.laplacian_variance = deviation[0] * deviation[0];
  result.blur_warning = result.laplacian_variance < config_.min_laplacian_variance;

  double weighted = 0.0;
  for (int y = 0; y < gray.rows; ++y) {
    const auto * row = gray.ptr<uint8_t>(y);
    const auto * mask_row = analysis_mask.empty() ? nullptr : analysis_mask.ptr<uint8_t>(y);
    for (int x = 0; x < gray.cols; ++x) {
      if (mask_row && mask_row[x] == 0) {continue;}
      const int value = row[x];
      ++result.histogram[value];
      weighted += value;
    }
  }

  const double total = static_cast<double>(pixel_count);
  result.mean_gray = weighted / total;

  const auto count = [&result](int first, int last) {
    int n = 0;
    for (int i = first; i <= last; ++i) {n += result.histogram[i];}
    return n;
  };

  result.dark_ratio = count(0, kDarkLast) / total;
  result.bright_ratio = count(kBrightFirst, 255) / total;
  result.black_clip_ratio = count(0, kBlackClipLast) / total;
  result.white_clip_ratio = count(kWhiteClipFirst, 255) / total;

  // 直方图两侧占比各自判断；平均灰度正常时也能发现暗部或亮部过多。
  const bool dark_distribution_bad = result.dark_ratio >= config_.max_dark_ratio;
  const bool bright_distribution_bad = result.bright_ratio >= config_.max_bright_ratio;

  // 近黑(0~3) / 近白(252~255)属于裁剪/饱和指标。
  // 大量像素压在动态范围两端时，即使平均灰度尚未明显异常，也直接进入质量筛查。
  const bool black_clipping_bad =
    result.black_clip_ratio >= config_.max_black_clip_ratio;

  const bool white_clipping_bad =
    result.white_clip_ratio >= config_.max_white_clip_ratio;

  // 三类条件均可触发：
  // 1) 极端平均灰度；2) 普通暗/亮分布；3) 大量近黑/近白裁剪。
  result.underexposed =
    result.mean_gray < config_.min_mean_gray ||
    dark_distribution_bad ||
    black_clipping_bad;
  result.overexposed =
    result.mean_gray > config_.max_mean_gray ||
    bright_distribution_bad ||
    white_clipping_bad;
  result.exposure_warning = result.underexposed || result.overexposed;

  // warning_reason 只传递后端已经完成的判定原因，前端只负责翻译显示。
  // scope_* 仅说明本次统计范围，不参与报警。
  result.warning_reason = result.roi_used ? "scope_roi," : "scope_full,";
  if (result.blur_warning) {result.warning_reason += "blur,";}
  if (result.board_area_ratio >= 0.0) {
    if (result.board_area_ratio < config_.min_board_area_ratio) {
      result.warning_reason += "board_too_far,";
    } else if (result.board_area_ratio > config_.max_board_area_ratio) {
      result.warning_reason += "board_too_close,";
    }
  }

  if (result.mean_gray < config_.min_mean_gray) {result.warning_reason += "mean_dark,";}
  if (dark_distribution_bad) {result.warning_reason += "dark_distribution,";}
  if (black_clipping_bad) {result.warning_reason += "black_clip,";}
  if (result.underexposed) {result.warning_reason += "under,";}

  if (result.mean_gray > config_.max_mean_gray) {result.warning_reason += "mean_bright,";}
  if (bright_distribution_bad) {result.warning_reason += "bright_distribution,";}
  if (white_clipping_bad) {result.warning_reason += "white_clip,";}
  if (result.overexposed) {result.warning_reason += "over,";}

  if (!result.warning_reason.empty()) {result.warning_reason.pop_back();}

  return result;
}

}  // namespace danmucamera
