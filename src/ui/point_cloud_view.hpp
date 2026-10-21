#pragma once

#include <QPoint>
#include <QVector3D>
#include <QWidget>

#include <vector>

namespace danmucamera
{
class PointCloudView : public QWidget
{
public:
  explicit PointCloudView(QWidget * parent = nullptr);
  void setPoints(const std::vector<QVector3D> & points);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  std::vector<QVector3D> points_;
  QPoint drag_origin_;
  float yaw_{0.0f};
  float pitch_{0.0f};
  float zoom_{1.0f};
};
}  // namespace danmucamera
