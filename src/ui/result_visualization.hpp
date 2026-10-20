#pragma once

#include <QDialog>
#include <QPointF>
#include <QQuaternion>
#include <QVector3D>
#include <QStringList>
#include <QVector>
#include <QWidget>
#include <opencv2/core.hpp>

namespace danmucamera
{
struct ResultPoint
{
  QPointF projected;
  QPointF observed;
};
struct ResultPose
{
  cv::Matx33d rotation;
  cv::Vec3d translation;
};
struct ResultVisualizationData
{
  int width{0}, height{0};
  double min_x{0}, max_x{0}, min_y{0}, max_y{0};
  QVector<ResultPose> poses;
  QVector<QVector<ResultPoint>> points;
  static bool load(const QString & path, ResultVisualizationData & result, QString & error);
};

class ResultVisualizationCanvas : public QWidget
{
public:
  explicit ResultVisualizationCanvas(QWidget * parent = nullptr);
  void setData(const ResultVisualizationData & data);
  void setMode(int mode);
  void setSample(int sample);
  void setHighlightedPoint(int sample, int point);
  void resetView();

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;
  void wheelEvent(QWheelEvent *) override;

private:
  void paintErrors(QPainter & painter);
  void paintScene(QPainter & painter);
  ResultVisualizationData data_;
  int mode_{0};
  int sample_{0};
  double zoom_{1.0};
  QQuaternion view_rotation_;
  int highlighted_sample_{-1};
  int highlighted_point_{-1};
  QVector3D trackballPoint(const QPointF & position) const;
  QPoint drag_start_;
};

class ResultVisualizationDialog : public QDialog
{
public:
  ResultVisualizationDialog(const QString & data_path, const QStringList & sample_names,
    QWidget * parent = nullptr);
  bool valid() const {return valid_;}
  QString error() const {return error_;}

private:
  bool valid_{false};
  QString error_;
};
}  // namespace danmucamera
