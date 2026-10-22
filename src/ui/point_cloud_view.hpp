#pragma once

#include <QPoint>
#include <QVector3D>
#include <QWidget>

#include <deque>
#include <vector>

namespace danmucamera
{
class PointCloudView : public QWidget
{
public:
  explicit PointCloudView(QWidget * parent = nullptr);
  void setPoints(const std::vector<QVector3D> & points);
  void setAccumulationFrames(int count);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  void rebuildPoints();
  std::deque<std::vector<QVector3D>> frames_;
  std::vector<QVector3D> points_;
  int accumulation_frames_{1};
  QPoint drag_origin_;
  float yaw_{0.0f};
  float pitch_{0.0f};
  float zoom_{1.0f};
};
}  // namespace danmucamera
