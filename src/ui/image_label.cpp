#include "image_label.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QSizePolicy>

#include <algorithm>
#include <cmath>

namespace danmucamera
{

ImageLabel::ImageLabel(QWidget * parent)
: QLabel(parent)
{
  setAlignment(Qt::AlignCenter);
  setMinimumSize(180, 150);
  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
}

QString ImageLabel::placeholder() const
{
  return placeholder_;
}

void ImageLabel::setPlaceholder(const QString & text)
{
  placeholder_ = text;
  if (source_.isNull()) {setText(text);}
}

void ImageLabel::setImage(const QImage & image)
{
  source_ = image;
  if (zoom_enabled_) {resetZoom();}
  setText({});
  renderImage();
}

void ImageLabel::clearImage(const QString & text)
{
  source_ = QImage();
  resetZoom();
  clear();
  setText(text.isEmpty() ? placeholder_ : text);
}

void ImageLabel::setZoomEnabled(bool enabled)
{
  if (zoom_enabled_ == enabled) {return;}
  zoom_enabled_ = enabled;
  resetZoom();
  if (enabled) {
    // 结果视图直接从原始 QImage 重绘，不保留缩小过的 QLabel pixmap。
    setText({});
  } else {
    renderImage();
  }
  setCursor(enabled ? Qt::OpenHandCursor : Qt::ArrowCursor);
  update();
}

void ImageLabel::resetZoom()
{
  zoom_ = 1.0;
  pan_ = {};
  update();
}

void ImageLabel::clampPan()
{
  if (source_.isNull()) {pan_ = {}; return;}
  const double fit = std::min(static_cast<double>(width()) / source_.width(),
    static_cast<double>(height()) / source_.height());
  const double scaled_width = source_.width() * fit * zoom_;
  const double scaled_height = source_.height() * fit * zoom_;
  const double max_x = std::max(0.0, (scaled_width - width()) / 2.0);
  const double max_y = std::max(0.0, (scaled_height - height()) / 2.0);
  pan_.setX(std::clamp(pan_.x(), -max_x, max_x));
  pan_.setY(std::clamp(pan_.y(), -max_y, max_y));
}

void ImageLabel::setGuideStyle(const GuideOverlayStyle & style)
{
  guide_style_ = style;
  update();
}

void ImageLabel::setGuideOverlay(
  bool enabled, const QPolygonF & target, const QPolygonF & detected,
  const QString & status, bool ready)
{
  guide_enabled_ = enabled;
  guide_target_ = target;
  guide_detected_ = detected;
  guide_status_ = status;
  guide_ready_ = ready;
  update();
}

void ImageLabel::paintEvent(QPaintEvent * event)
{
  if (!source_.isNull()) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor("#111821"));
    const double fit = std::min(static_cast<double>(width()) / source_.width(),
      static_cast<double>(height()) / source_.height());
    const double scale = fit * (zoom_enabled_ ? zoom_ : 1.0);
    const QSizeF display_size(source_.width() * scale, source_.height() * scale);
    const QPointF offset = zoom_enabled_ ? pan_ : QPointF{};
    const QRectF target(QPointF((width() - display_size.width()) / 2.0 + offset.x(),
      (height() - display_size.height()) / 2.0 + offset.y()), display_size);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, scale < 1.0);
    painter.drawImage(target, source_);
    if (!guide_enabled_) {return;}
    const auto map_polygon = [&target](const QPolygonF & normalized) {
      QPolygonF mapped;
      for (const QPointF & point : normalized) {
        mapped << QPointF(target.x() + point.x() * target.width(),
          target.y() + point.y() * target.height());
      }
      return mapped;
    };
    painter.setRenderHint(QPainter::Antialiasing);
    const QPolygonF guide_target = map_polygon(guide_target_);
    if (guide_target.size() == 4) {
      painter.setPen(QPen(guide_ready_ ? guide_style_.ready_color : guide_style_.guide_color,
        guide_ready_ ? guide_style_.ready_line_width : guide_style_.guide_line_width));
      painter.setBrush(Qt::NoBrush);
      QPainterPath outline;
      for (int i = 0; i < 4; ++i) {
        const QPointF & before = guide_target[(i + 3) % 4];
        const QPointF & corner = guide_target[i];
        const QPointF & after = guide_target[(i + 1) % 4];
        const double incoming = std::hypot(corner.x() - before.x(), corner.y() - before.y());
        const double outgoing = std::hypot(after.x() - corner.x(), after.y() - corner.y());
        const double radius = std::min({guide_style_.corner_radius_px,
          incoming * 0.25, outgoing * 0.25});
        const QPointF first = corner + (before - corner) * (radius / incoming);
        const QPointF last = corner + (after - corner) * (radius / outgoing);
        if (i == 0) {outline.moveTo(first);} else {outline.lineTo(first);}
        outline.quadTo(corner, last);
      }
      outline.closeSubpath();
      painter.drawPath(outline);
    }
    if (guide_detected_.size() == 4) {
      painter.setPen(QPen(guide_ready_ ?
        guide_style_.ready_color : guide_style_.detected_color,
        guide_style_.detected_line_width));
      painter.setBrush(Qt::NoBrush);
      painter.drawPolygon(map_polygon(guide_detected_));
    }
    return;
  }
  QLabel::paintEvent(event);
}

void ImageLabel::resizeEvent(QResizeEvent * event)
{
  QLabel::resizeEvent(event);
  if (zoom_enabled_) {clampPan();}
  update();
}

void ImageLabel::renderImage()
{
  update();
}

void ImageLabel::wheelEvent(QWheelEvent * event)
{
  if (!zoom_enabled_ || source_.isNull() || event->angleDelta().y() == 0) {
    QLabel::wheelEvent(event);
    return;
  }
  const double fit = std::min(static_cast<double>(width()) / source_.width(),
    static_cast<double>(height()) / source_.height());
  const double old_scale = fit * zoom_;
  const QPointF center(width() / 2.0, height() / 2.0);
  const QPointF old_top = center - QPointF(source_.width() * old_scale / 2.0,
    source_.height() * old_scale / 2.0) + pan_;
  const QPointF source_point = (event->position() - old_top) / old_scale;
  zoom_ = std::clamp(zoom_ * std::pow(1.2, event->angleDelta().y() / 120.0),
    1.0, 20.0);
  const double new_scale = fit * zoom_;
  pan_ = event->position() - source_point * new_scale - center +
    QPointF(source_.width() * new_scale / 2.0,
      source_.height() * new_scale / 2.0);
  clampPan();
  update();
  event->accept();
}

void ImageLabel::mousePressEvent(QMouseEvent * event)
{
  if (zoom_enabled_ && event->button() == Qt::LeftButton && !source_.isNull()) {
    drag_start_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QLabel::mousePressEvent(event);
}

void ImageLabel::mouseMoveEvent(QMouseEvent * event)
{
  if (zoom_enabled_ && event->buttons().testFlag(Qt::LeftButton) && !source_.isNull()) {
    pan_ += event->pos() - drag_start_;
    drag_start_ = event->pos();
    clampPan();
    update();
    event->accept();
    return;
  }
  setCursor(zoom_enabled_ ? Qt::OpenHandCursor : Qt::ArrowCursor);
  QLabel::mouseMoveEvent(event);
}

void ImageLabel::mouseDoubleClickEvent(QMouseEvent * event)
{
  if (zoom_enabled_ && event->button() == Qt::LeftButton) {
    resetZoom();
    event->accept();
    return;
  }
  QLabel::mouseDoubleClickEvent(event);
}

}  // namespace danmucamera
