#include "point_cloud_view.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace danmucamera
{
PointCloudView::PointCloudView(QWidget * parent) : QWidget(parent)
{
  setMinimumSize(180, 180);
  setStyleSheet(QStringLiteral("background:#111924;"));
}

void PointCloudView::setPoints(const std::vector<QVector3D> & points)
{
  if (points.empty()) {
    frames_.clear();
  } else {
    frames_.push_back(points);
    while (static_cast<int>(frames_.size()) > accumulation_frames_) {frames_.pop_front();}
  }
  rebuildPoints();
}

void PointCloudView::setAccumulationFrames(int count)
{
  accumulation_frames_ = std::clamp(count, 1, 30);
  while (static_cast<int>(frames_.size()) > accumulation_frames_) {frames_.pop_front();}
  rebuildPoints();
}

void PointCloudView::rebuildPoints()
{
  size_t total = 0;
  for (const auto & frame : frames_) {total += frame.size();}
  const size_t stride = std::max<size_t>(1, (total + 47999) / 48000);
  points_.clear();
  points_.reserve(std::min<size_t>(total, 48000));
  for (const auto & frame : frames_) {
    for (size_t i = 0; i < frame.size(); i += stride) {points_.push_back(frame[i]);}
  }
  update();
}

void PointCloudView::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.fillRect(rect(), QColor(17, 25, 36));
  painter.setPen(QColor(161, 181, 204));
  if (points_.empty()) {
    painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("等待点云…"));
    return;
  }
  // 用 5%~95% 分位范围构图；远处离群点不能把近处主体缩成一个小点。
  // 所有原始点仍保存在 points_，仅视图缩放受分位范围影响。
  std::vector<float> xs, ys, zs;
  xs.reserve(points_.size());
  ys.reserve(points_.size());
  zs.reserve(points_.size());
  for (const auto & p : points_) {
    xs.push_back(p.x());
    ys.push_back(p.y());
    zs.push_back(p.z());
  }
  const auto bounds = [](std::vector<float> & values) {
    const size_t low = values.size() * 5 / 100;
    const size_t high = std::min(values.size() - 1, values.size() * 95 / 100);
    std::nth_element(values.begin(), values.begin() + low, values.end());
    const float minimum = values[low];
    std::nth_element(values.begin(), values.begin() + high, values.end());
    return std::pair<float, float>(minimum, values[high]);
  };
  const auto [min_x, max_x] = bounds(xs);
  const auto [min_y, max_y] = bounds(ys);
  const auto [min_z, max_z] = bounds(zs);
  const QVector3D center((min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f,
    (min_z + max_z) * 0.5f);
  const float span = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 0.01f});
  const float scale = zoom_ * 0.78f * std::min(width(), height()) / span;
  const float cy = std::cos(yaw_), sy = std::sin(yaw_);
  const float cp = std::cos(pitch_), sp = std::sin(pitch_);
  painter.setPen(Qt::NoPen);
  for (const auto & p : points_) {
    const auto q = p - center;
    const float x = cy * q.x() + sy * q.z();
    const float z = -sy * q.x() + cy * q.z();
    const float y = cp * q.y() - sp * z;
    const int hue = std::clamp(static_cast<int>(240.0f * (p.z() - min_z) /
      std::max(0.001f, max_z - min_z)), 0, 240);
    painter.setBrush(QColor::fromHsv(240 - hue, 210, 240));
    painter.drawEllipse(QPointF(width() * 0.5f + x * scale,
      height() * 0.5f - y * scale), 1.5, 1.5);
  }
  painter.setPen(QColor(182, 198, 216));
  painter.drawText(10, height() - 10,
    QStringLiteral("拖动旋转 · 滚轮缩放 · %1 点 / %2 帧")
      .arg(points_.size()).arg(frames_.size()));
}

void PointCloudView::mousePressEvent(QMouseEvent * event)
{
  drag_origin_ = event->position().toPoint();
}

void PointCloudView::mouseMoveEvent(QMouseEvent * event)
{
  if (!(event->buttons() & Qt::LeftButton)) {return;}
  const QPoint next = event->position().toPoint();
  const QPoint delta = next - drag_origin_;
  yaw_ += delta.x() * 0.01f;
  pitch_ = std::clamp(pitch_ + delta.y() * 0.01f, -1.55f, 1.55f);
  drag_origin_ = next;
  update();
}

void PointCloudView::wheelEvent(QWheelEvent * event)
{
  zoom_ = std::clamp(zoom_ * (event->angleDelta().y() > 0 ? 1.15f : 1.0f / 1.15f),
    0.1f, 20.0f);
  update();
}
}  // namespace danmucamera
