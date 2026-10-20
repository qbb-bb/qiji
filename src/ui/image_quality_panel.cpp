#include "image_quality_panel.hpp"

#include <QColor>
#include <QLabel>
#include <QPainter>
#include <QSizePolicy>
#include <QStringList>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <numeric>

namespace danmucamera
{
namespace
{

// ============================================================================
// 以下常量只用于“界面文字展示”。
// ----------------------------------------------------------------------------
// 后端 image_quality_analyzer 才负责真正的报警判断。
// 这里不会拿这些区间去触发 blur / under / over。
// ============================================================================
constexpr int kDisplayDarkLast = 63;
constexpr int kDisplayBrightFirst = 192;
constexpr int kDisplayBlackClipLast = 3;
constexpr int kDisplayWhiteClipFirst = 252;

struct DisplayDistribution
{
  double dark_percent{0.0};
  double mid_percent{0.0};
  double bright_percent{0.0};
  double black_clip_percent{0.0};
  double white_clip_percent{0.0};
};

DisplayDistribution distributionForDisplay(const std::array<int, 256> & bins)
{
  DisplayDistribution result;

  const int total = std::accumulate(bins.begin(), bins.end(), 0);
  if (total <= 0) {
    return result;
  }

  const auto percent = [&bins, total](int first, int last) {
    int count = 0;
    for (int gray = first; gray <= last; ++gray) {
      count += bins[gray];
    }
    return 100.0 * static_cast<double>(count) / static_cast<double>(total);
  };

  result.dark_percent = percent(0, kDisplayDarkLast);
  result.mid_percent = percent(kDisplayDarkLast + 1, kDisplayBrightFirst - 1);
  result.bright_percent = percent(kDisplayBrightFirst, 255);
  result.black_clip_percent = percent(0, kDisplayBlackClipLast);
  result.white_clip_percent = percent(kDisplayWhiteClipFirst, 255);

  return result;
}

// 后端 warning_reason 中的 scope_* 只是告诉前端本次统计使用了哪个区域。
QString scopeText(const QString & reason)
{
  if (reason.contains(QStringLiteral("scope_roi"))) {
    return QStringLiteral("标定板 ROI");
  }
  if (reason.contains(QStringLiteral("scope_full"))) {
    return QStringLiteral("整图回退");
  }
  return QStringLiteral("分析区域");
}

// 只“翻译”后端已经给出的原因标签，不重新判断任何阈值。
QStringList warningTextList(const ImageQualityMetrics & metrics)
{
  QStringList warnings;

  if (metrics.warning_reason.contains(QStringLiteral("blur"))) {
    warnings << QStringLiteral("清晰度不足");
  }

  if (metrics.warning_reason.contains(QStringLiteral("mean_dark"))) {
    warnings << QStringLiteral("整体灰度过低");
  }
  if (metrics.warning_reason.contains(QStringLiteral("dark_distribution"))) {
    warnings << QStringLiteral("暗部偏多且整体偏暗");
  }
  if (metrics.warning_reason.contains(QStringLiteral("black_clip"))) {
    warnings << QStringLiteral("近黑像素过多");
  }

  if (metrics.warning_reason.contains(QStringLiteral("mean_bright"))) {
    warnings << QStringLiteral("整体灰度过高");
  }
  if (metrics.warning_reason.contains(QStringLiteral("bright_distribution"))) {
    warnings << QStringLiteral("亮部偏多且整体偏亮");
  }
  if (metrics.warning_reason.contains(QStringLiteral("white_clip"))) {
    warnings << QStringLiteral("近白像素过多");
  }

  if (metrics.warning_reason.contains(QStringLiteral("board_too_far"))) {
    warnings << QStringLiteral("标定板太远");
  }
  if (metrics.warning_reason.contains(QStringLiteral("board_too_close"))) {
    warnings << QStringLiteral("标定板太近");
  }

  if (metrics.warning_reason.contains(QStringLiteral("unreadable"))) {
    warnings << QStringLiteral("图像无法读取");
  }

  // 兼容只发送 under / over 的旧后端。
  if (warnings.isEmpty()) {
    if (metrics.warning_reason.contains(QStringLiteral("under"))) {
      warnings << QStringLiteral("欠曝风险");
    }
    if (metrics.warning_reason.contains(QStringLiteral("over"))) {
      warnings << QStringLiteral("过曝风险");
    }
  }

  return warnings;
}

QString exposureText(const ImageQualityMetrics & metrics)
{
  const bool under = metrics.warning_reason.contains(QStringLiteral("under"));
  const bool over = metrics.warning_reason.contains(QStringLiteral("over"));

  if (under && over) {
    return QStringLiteral("欠曝 / 过曝风险并存");
  }
  if (under) {
    return QStringLiteral("欠曝风险");
  }
  if (over) {
    return QStringLiteral("过曝风险");
  }
  return QStringLiteral("正常");
}

}  // namespace

// ============================================================================
// HistogramWidget
// ----------------------------------------------------------------------------
// 只负责画图。
// 不判断质量，不接触 YAML，不知道报警阈值。
// ============================================================================
class ImageQualityPanel::HistogramWidget final : public QWidget
{
public:
  explicit HistogramWidget(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    setMinimumHeight(70);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setToolTip(QStringLiteral(
      "灰度直方图：左侧接近黑，右侧接近白。直方图仅用于显示，质量结论来自后端。"));
  }

  void setBins(const std::array<int, 256> & bins)
  {
    bins_ = bins;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.fillRect(rect(), QColor("#0e131b"));

    const QRect plot = rect().adjusted(7, 5, -7, -18);
    if (plot.width() <= 0 || plot.height() <= 0) {
      return;
    }

    painter.setPen(QColor("#313e52"));
    painter.drawRect(plot);

    const int peak = *std::max_element(bins_.begin(), bins_.end());
    if (peak > 0) {
      painter.setPen(QColor("#70a8f9"));

      for (int x = 0; x < plot.width(); ++x) {
        const int first = x * 256 / plot.width();
        const int last = std::min(255, (x + 1) * 256 / plot.width());

        int value = 0;
        for (int i = first; i <= last; ++i) {
          value = std::max(value, bins_[i]);
        }

        const int bar_height = static_cast<int>(
          value * (plot.height() - 2.0) / static_cast<double>(peak));

        painter.drawLine(
          plot.left() + x,
          plot.bottom() - 1,
          plot.left() + x,
          plot.bottom() - 1 - bar_height);
      }
    }

    painter.setPen(QColor("#9eacc2"));
    painter.drawText(
      7, rect().bottom() - 3,
      QStringLiteral("0 · 黑"));

    painter.drawText(
      rect().adjusted(0, 0, -7, -2),
      Qt::AlignRight | Qt::AlignBottom,
      QStringLiteral("白 · 255"));
  }

private:
  std::array<int, 256> bins_{};
};

// ============================================================================
// ImageQualityPanel
// ============================================================================
ImageQualityPanel::ImageQualityPanel(QWidget * parent)
: QWidget(parent)
{
  setObjectName(QStringLiteral("imageQualityPanel"));

  setToolTip(QStringLiteral(
    "这里只显示图像质量结果；质量阈值、ROI 和报警判断均由后端完成。"));

  setStyleSheet(QStringLiteral(
    "QWidget#imageQualityPanel{"
    "background:#222b3a;"
    "border:1px solid #344258;"
    "border-radius:9px;"
    "}"
    "QWidget#imageQualityPanel QLabel{"
    "background:transparent;"
    "border:0;"
    "}"));

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(11, 8, 11, 8);
  layout->setSpacing(3);

  title_label_ = new QLabel(this);
  title_label_->setStyleSheet(
    QStringLiteral("font-weight:700;color:#dce8fb;"));

  status_label_ = new QLabel(this);
  status_label_->setWordWrap(true);

  sharpness_label_ = new QLabel(this);
  exposure_label_ = new QLabel(this);
  board_size_label_ = new QLabel(this);
  distribution_label_ = new QLabel(this);

  exposure_label_->setWordWrap(true);
  board_size_label_->setWordWrap(true);
  distribution_label_->setWordWrap(true);

  histogram_widget_ = new HistogramWidget(this);

  layout->addWidget(title_label_);
  layout->addWidget(status_label_);
  layout->addWidget(sharpness_label_);
  layout->addWidget(exposure_label_);
  layout->addWidget(board_size_label_);
  layout->addWidget(distribution_label_);
  layout->addWidget(histogram_widget_, 1);

  clear();
}

void ImageQualityPanel::clear(const QString & message)
{
  title_label_->setText(
    QStringLiteral("图像质量 · ") + message);

  status_label_->setText(QStringLiteral("状态：—"));
  status_label_->setStyleSheet(QString());

  sharpness_label_->setText(QStringLiteral("清晰度：—"));
  exposure_label_->setText(QStringLiteral("曝光：—"));
  board_size_label_->setText(QStringLiteral("标定板占比：—"));
  distribution_label_->setText(QStringLiteral("灰度分布：—"));

  histogram_widget_->setBins({});
}

void ImageQualityPanel::setMetrics(
  const ImageQualityMetrics & metrics,
  const QString & source)
{
  const int total = std::accumulate(
    metrics.histogram.begin(),
    metrics.histogram.end(),
    0);

  if (total <= 0) {
    clear(QStringLiteral("无法读取原图"));
    return;
  }

  // --------------------------------------------------------------------------
  // 从这一行开始全部是“显示”：
  // - 不比较 YAML 阈值；
  // - 不产生 blur_warning；
  // - 不产生 exposure_warning；
  // - 不改变 warning_reason。
  // --------------------------------------------------------------------------
  const DisplayDistribution distribution =
    distributionForDisplay(metrics.histogram);

  const QStringList warnings = warningTextList(metrics);
  const bool has_warning =
    metrics.blur_warning || metrics.exposure_warning || metrics.board_size_warning;

  title_label_->setText(
    QStringLiteral("图像质量 · %1 · %2")
      .arg(source, scopeText(metrics.warning_reason)));

  if (has_warning) {
    status_label_->setText(
      QStringLiteral("状态：质量告警 · %1")
        .arg(
          warnings.isEmpty()
          ? QStringLiteral("请检查图像")
          : warnings.join(QStringLiteral("、"))));

    status_label_->setStyleSheet(
      QStringLiteral("color:#ffad4a;font-weight:700;"));
  } else {
    status_label_->setText(
      QStringLiteral("状态：通过质量筛查"));

    status_label_->setStyleSheet(
      QStringLiteral("color:#4ddd8b;font-weight:700;"));
  }

  sharpness_label_->setText(
    QStringLiteral("清晰度：拉普拉斯方差 %1%2")
      .arg(metrics.laplacian_variance, 0, 'f', 1)
      .arg(
        metrics.blur_warning
        ? QStringLiteral(" · 偏模糊")
        : QStringLiteral(" · 正常")));

  exposure_label_->setText(
    QStringLiteral("曝光：%1 · 平均灰度 %2 / 255")
      .arg(exposureText(metrics))
      .arg(metrics.mean_gray, 0, 'f', 1));

  board_size_label_->setText(metrics.board_area_ratio < 0.0 ?
    QStringLiteral("标定板占比：无法计算（未识别到完整标定板）") :
    QStringLiteral("标定板占比：%1%（要求 %2%～%3%）· %4")
      .arg(metrics.board_area_ratio * 100.0, 0, 'f', 1)
      .arg(metrics.board_area_min_ratio * 100.0, 0, 'f', 0)
      .arg(metrics.board_area_max_ratio * 100.0, 0, 'f', 0)
      .arg(metrics.board_size_warning ?
        (metrics.warning_reason.contains(QStringLiteral("board_too_far")) ?
          QStringLiteral("不合格：太远") : QStringLiteral("不合格：太近")) :
        QStringLiteral("合格")));

  distribution_label_->setText(
    QStringLiteral(
      "灰度分布：暗部 %1% · 中间调 %2% · 亮部 %3%\n"
      "端点分布：近黑 %4% · 近白 %5%")
      .arg(distribution.dark_percent, 0, 'f', 1)
      .arg(distribution.mid_percent, 0, 'f', 1)
      .arg(distribution.bright_percent, 0, 'f', 1)
      .arg(distribution.black_clip_percent, 0, 'f', 1)
      .arg(distribution.white_clip_percent, 0, 'f', 1));

  histogram_widget_->setBins(metrics.histogram);
}

}  // namespace danmucamera
