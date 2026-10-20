#pragma once

#include <QColor>
#include <QImage>
#include <QLabel>
#include <QPolygonF>

class QResizeEvent;
class QPaintEvent;
class QMouseEvent;
class QWheelEvent;

namespace danmucamera
{

struct GuideOverlayStyle
{
  QColor guide_color;
  QColor ready_color;
  QColor detected_color;
  int guide_line_width{0};
  int ready_line_width{0};
  int detected_line_width{0};
  double corner_radius_px{0.0};
};

class ImageLabel : public QLabel
{
  Q_OBJECT
  Q_PROPERTY(QString placeholder READ placeholder WRITE setPlaceholder)
  Q_PROPERTY(bool zoomEnabled READ zoomEnabled WRITE setZoomEnabled)

public:
  explicit ImageLabel(QWidget * parent = nullptr);

  QString placeholder() const;
  void setPlaceholder(const QString & text);
  void setImage(const QImage & image);
  bool zoomEnabled() const {return zoom_enabled_;}
  void setZoomEnabled(bool enabled);
  void resetZoom();
  void clearImage(const QString & text = {});
  void setGuideStyle(const GuideOverlayStyle & style);
  void setGuideOverlay(bool enabled, const QPolygonF & target = {},
    const QPolygonF & detected = {}, const QString & status = {}, bool ready = false);

protected:
  void resizeEvent(QResizeEvent * event) override;
  void paintEvent(QPaintEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent * event) override;

private:
  void renderImage();
  void clampPan();

  QImage source_;
  bool zoom_enabled_{false};
  double zoom_{1.0};
  QPointF pan_;
  QPoint drag_start_;
  QString placeholder_;
  GuideOverlayStyle guide_style_;
  bool guide_enabled_{false};
  bool guide_ready_{false};
  QPolygonF guide_target_;
  QPolygonF guide_detected_;
  QString guide_status_;
};

}  // namespace danmucamera
