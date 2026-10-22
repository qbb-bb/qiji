#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

namespace danmucamera
{

// 图像质量算法参数由 ROS 2 从 calibration.yaml 注入。
// 本模块本身不读取 YAML，也不依赖 ROS/Qt。
struct ImageQualityConfig
{
  int analysis_max_width{0};
  int analysis_max_height{0};

  // ROI：优先使用标定板特征点凸包，并向外扩展一定比例。
  double roi_margin_ratio{-1.0};
  int roi_min_points{0};
  int roi_min_pixels{0};

  // 标定板投影区域占整幅图像的比例；基于特征点凸包及 ROI 外扩估计。
  double min_board_area_ratio{-1.0};
  double max_board_area_ratio{-1.0};

  // 清晰度。
  double min_laplacian_variance{-1.0};

  // 普通暗部/亮部比例阈值：独立判断。
  double max_dark_ratio{-1.0};
  double max_bright_ratio{-1.0};

  // 近黑/近白裁剪比例阈值：超过时可直接触发曝光质量告警。
  double max_black_clip_ratio{-1.0};
  double max_white_clip_ratio{-1.0};

  // 极端平均灰度：单独即可触发报警。
  double min_mean_gray{-1.0};
  double max_mean_gray{-1.0};

};

struct ImageQualityResult
{
  std::array<int, 256> histogram{};

  double laplacian_variance{-1.0};
  double mean_gray{0.0};
  double dark_ratio{0.0};
  double bright_ratio{0.0};
  double black_clip_ratio{0.0};
  double white_clip_ratio{0.0};
  double board_area_ratio{-1.0};
  double board_area_min_ratio{0.0};
  double board_area_max_ratio{1.0};
  std::array<int, 9> corner_grid_counts{};

  bool readable{false};
  bool roi_used{false};
  bool blur_warning{false};
  bool board_size_warning{false};
  bool exposure_warning{false};
  bool underexposed{false};
  bool overexposed{false};

  int analyzed_pixels{0};
  std::string warning_reason;
};

class ImageQualityAnalyzer
{
public:
  explicit ImageQualityAnalyzer(ImageQualityConfig config);

  // board_points 为空或无法形成足够大的 ROI 时，自动回退整图分析。
  ImageQualityResult evaluate(
    const cv::Mat & input,
    const std::vector<cv::Point2f> & board_points = {}) const;

private:
  static void validateConfig(const ImageQualityConfig & config);
  cv::Mat buildBoardMask(
    const cv::Size & image_size,
    const std::vector<cv::Point2f> & board_points) const;

  ImageQualityConfig config_;
};

}  // namespace danmucamera
