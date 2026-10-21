#include "point_cloud_view.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace danmucamera
{
PointCloudView::PointCloudView(QWidget * parent) : QWidget(parent)
{
  setMinimumSize(180, 180);
  setStyleSheet(QStringLiteral("background:#111924;"));
}

void PointCloudView::setPoints(const std::vector<QVector3D> & points)
{
  points_ = points;
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
  // 以当前点云的中位深度为观察中心。投影只供交互预览，不改变标定数据。
  float min_x = points_[0].x(), max_x = min_x;
  float min_y = points_[0].y(), max_y = min_y;
  float min_z = points_[0].z(), max_z = min_z;
  for (const auto & p : points_) {
    min_x = std::min(min_x, p.x()); max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y()); max_y = std::max(max_y, p.y());
    min_z = std::min(min_z, p.z()); max_z = std::max(max_z, p.z());
  }
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
  painter.drawText(10, height() - 10, QStringLiteral("拖动旋转 · 滚轮缩放 · %1 点")
    .arg(points_.size()));
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
