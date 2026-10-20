#pragma once

#include "calibration_ros_client.hpp"

#include <QString>
#include <QWidget>

class QLabel;

namespace danmucamera
{

// ============================================================================
// ImageQualityPanel
// ----------------------------------------------------------------------------
// 定位：纯前端“图像质量结果显示控件”。
//
// 负责：
//   1. 显示分析范围（标定板 ROI / 整图回退）；
//   2. 显示质量状态与告警原因；
//   3. 显示清晰度、平均灰度、暗/中/亮、近黑/近白等文字数据；
//   4. 绘制 256-bin 灰度直方图。
//
// 不负责：
//   1. 不读取 calibration.yaml；
//   2. 不保存任何报警阈值；
//   3. 不判断图像是否模糊/欠曝/过曝；
//   4. 不做 ROI 提取；
//   5. 不做 ROS 通信。
//
// 所有质量结论来自后端 image_quality_analyzer。
// 本控件只把 ImageQualityMetrics 转换成人可读的界面内容。
// ============================================================================
class ImageQualityPanel final : public QWidget
{
public:
  explicit ImageQualityPanel(QWidget * parent = nullptr);

  void clear(const QString & message = QStringLiteral("等待图像"));
  void setMetrics(const ImageQualityMetrics & metrics, const QString & source);

private:
  class HistogramWidget;

  QLabel * title_label_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * sharpness_label_{nullptr};
  QLabel * exposure_label_{nullptr};
  QLabel * board_size_label_{nullptr};
  QLabel * distribution_label_{nullptr};
  HistogramWidget * histogram_widget_{nullptr};
};

}  // namespace danmucamera
