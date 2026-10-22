#include "result_visualization.hpp"
#include "ui_result_visualization_dialog.h"

#include <QComboBox>
#include <QDesktopServices>
#include <QDir>
#include <QPushButton>
#include <QHeaderView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QSplitter>
#include <QSignalBlocker>
#include <opencv2/calib3d.hpp>
#include <QFile>
#include <QFileInfo>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTextStream>
#include <QUrl>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <numeric>
#include <cmath>

namespace danmucamera
{
namespace
{
constexpr double kArrowLengthScreenPx = 22.0;
constexpr double kPi = 3.14159265358979323846;

QColor directionColor(const QPointF & delta)
{
  const double hue = (std::atan2(delta.y(), delta.x()) + kPi) / (2 * kPi);
  return QColor::fromHsvF(hue, 0.88, 0.96);
}

QString errorStatistics(const ResultVisualizationData & data, int sample)
{
  std::array<int, 8> directions{};
  std::array<int, 5> magnitudes{};
  QVector<double> errors;
  double sum = 0;
  for (int i = 0; i < data.points.size(); ++i) {
    if (sample >= 0 && i != sample) {continue;}
    for (const auto & point : data.points[i]) {
      const QPointF delta = point.observed - point.projected;
      const double value = std::hypot(delta.x(), delta.y());
      errors.append(value);
      sum += value;
      if (value > 1e-9) {
        const int sector = static_cast<int>(std::lround(
          std::atan2(delta.y(), delta.x()) * 4.0 / kPi));
        directions[(sector % 8 + 8) % 8]++;
      }
      magnitudes[value < 0.1 ? 0 : value < 0.25 ? 1 :
        value < 0.5 ? 2 : value < 1.0 ? 3 : 4]++;
    }
  }
  if (errors.isEmpty()) {return QStringLiteral("暂无误差数据");}
  std::sort(errors.begin(), errors.end());
  const double count = errors.size();
  const int p95_index = static_cast<int>(std::ceil(count * 0.95)) - 1;
  const QStringList direction_names{
    QStringLiteral("→"), QStringLiteral("↘"), QStringLiteral("↓"),
    QStringLiteral("↙"), QStringLiteral("←"), QStringLiteral("↖"),
    QStringLiteral("↑"), QStringLiteral("↗")};
  QStringList direction_parts;
  QStringList magnitude_parts;
  const int directed_count = std::accumulate(directions.begin(), directions.end(), 0);
  for (int i = 0; i < 8; ++i) {
    direction_parts << QStringLiteral("%1 %2%")
      .arg(direction_names[i]).arg(100.0 * directions[i] / std::max(1, directed_count), 0, 'f', 1);
  }
  const QStringList range_names{QStringLiteral("<0.1"), QStringLiteral("0.1–0.25"),
    QStringLiteral("0.25–0.5"), QStringLiteral("0.5–1.0"), QStringLiteral("≥1.0")};
  for (int i = 0; i < 5; ++i) {
    magnitude_parts << QStringLiteral("%1 px %2%")
      .arg(range_names[i]).arg(100.0 * magnitudes[i] / count, 0, 'f', 1);
  }
  return QStringLiteral("方向占比（%1 点）：%2\n误差分布：%3　|　均值 %4 px　P95 %5 px　最大 %6 px")
    .arg(errors.size()).arg(direction_parts.join(QStringLiteral("　")))
    .arg(magnitude_parts.join(QStringLiteral("　")))
    .arg(sum / count, 0, 'f', 3).arg(errors[p95_index], 0, 'f', 3)
    .arg(errors.last(), 0, 'f', 3);
}

cv::Vec3d transform(const ResultPose & pose, const cv::Vec3d & point)
{
  return pose.rotation * point + pose.translation;
}

cv::Vec3d inverseTransform(const ResultPose & pose, const cv::Vec3d & point)
{
  return pose.rotation.t() * (point - pose.translation);
}
}  // namespace

bool ResultVisualizationData::load(
  const QString & path, ResultVisualizationData & result, QString & error)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    error = QStringLiteral("无法读取位姿与误差数据：%1").arg(path);
    return false;
  }
  ResultVisualizationData parsed;
  QTextStream stream(&file);
  while (!stream.atEnd()) {
    const QStringList fields = stream.readLine().split('\t');
    if (fields.isEmpty()) {continue;}
    bool okay = true;
    auto number = [&](int index) {
      bool valid = false;
      const double value = fields.value(index).toDouble(&valid);
      okay &= valid && std::isfinite(value);
      return value;
    };
    if (fields[0] == QStringLiteral("META") && fields.size() == 7) {
      parsed.width = static_cast<int>(number(1));
      parsed.height = static_cast<int>(number(2));
      parsed.min_x = number(3);
      parsed.max_x = number(4);
      parsed.min_y = number(5);
      parsed.max_y = number(6);
    } else if (fields[0] == QStringLiteral("POSE") && fields.size() == 8) {
      const int index = static_cast<int>(number(1));
      const cv::Vec3d rv(number(2), number(3), number(4));
      const cv::Vec3d translation(number(5), number(6), number(7));
      if (index < 0 || index > 10000) {okay = false;}
      if (okay) {
        cv::Matx33d rotation;
        cv::Rodrigues(rv, rotation);
        if (parsed.poses.size() <= index) {parsed.poses.resize(index + 1);}
        parsed.poses[index] = {rotation, translation};
      }
    } else if (fields[0] == QStringLiteral("POINT") && fields.size() == 6) {
      const int index = static_cast<int>(number(1));
      const ResultPoint point{{number(2), number(3)}, {number(4), number(5)}};
      if (index < 0 || index > 10000) {okay = false;}
      if (okay) {
        if (parsed.points.size() <= index) {parsed.points.resize(index + 1);}
        parsed.points[index].append(point);
      }
    } else {
      okay = false;
    }
    if (!okay) {
      error = QStringLiteral("位姿与误差数据格式错误：%1").arg(path);
      return false;
    }
  }
  if (parsed.width <= 0 || parsed.height <= 0 || parsed.poses.isEmpty() ||
    parsed.poses.size() != parsed.points.size() ||
    parsed.max_x <= parsed.min_x || parsed.max_y <= parsed.min_y) {
    error = QStringLiteral("位姿与误差数据不完整：%1").arg(path);
    return false;
  }
  result = std::move(parsed);
  return true;
}

ResultVisualizationCanvas::ResultVisualizationCanvas(QWidget * parent) : QWidget(parent)
{
  setMinimumSize(300, 250);
  setMouseTracking(true);
}
void ResultVisualizationCanvas::setData(const ResultVisualizationData & data)
{
  data_ = data;
  resetView();
}
void ResultVisualizationCanvas::setMode(int mode)
{
  mode_ = mode;
  setCursor(mode >= 2 ? Qt::OpenHandCursor : Qt::ArrowCursor);
  resetView();
}
void ResultVisualizationCanvas::setSample(int sample) {sample_ = sample; update();}
void ResultVisualizationCanvas::setHighlightedPoint(int sample, int point)
{
  highlighted_sample_ = sample;
  highlighted_point_ = point;
  update();
}
void ResultVisualizationCanvas::resetView()
{
  zoom_ = 1.0;
  view_rotation_ =
    QQuaternion::fromAxisAndAngle(0, 1, 0, -32.0f) *
    QQuaternion::fromAxisAndAngle(1, 0, 0, -26.0f);
  update();
}
void ResultVisualizationCanvas::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor("#101923"));
  if (data_.width <= 0) {return;}
  if (mode_ < 2) {paintErrors(painter);} else {paintScene(painter);}
}

void ResultVisualizationCanvas::paintErrors(QPainter & painter)
{
  const double fit = std::min((width() - 70.0) / data_.width,
    (height() - 70.0) / data_.height);
  const double scale = fit * zoom_;
  const QPointF offset((width() - data_.width * scale) / 2.0,
    (height() - data_.height * scale) / 2.0);
  const auto map = [&](const QPointF & point) {return offset + point * scale;};
  const QRectF canvas(offset, QSizeF(data_.width * scale, data_.height * scale));
  painter.fillRect(canvas, QColor("#172431"));
  painter.setPen(QPen(QColor("#304250"), 1));
  for (int n = 1; n < 10; ++n) {
    const double x = canvas.left() + canvas.width() * n / 10.0;
    const double y = canvas.top() + canvas.height() * n / 10.0;
    painter.drawLine(QPointF(x, canvas.top()), QPointF(x, canvas.bottom()));
    painter.drawLine(QPointF(canvas.left(), y), QPointF(canvas.right(), y));
  }
  painter.setPen(QPen(QColor("#7692a5"), 1));
  painter.drawRect(canvas);
  painter.save();
  painter.setClipRect(canvas);
  QFont label_font = painter.font();
  label_font.setPixelSize(11);
  for (int i = 0; i < data_.points.size(); ++i) {
    if (mode_ == 1 && i != sample_) {continue;}
    for (int j = 0; j < data_.points[i].size(); ++j) {
      const auto & point = data_.points[i][j];
      const QPointF residual = point.observed - point.projected;
      const double error = std::hypot(residual.x(), residual.y());
      const QPointF from = map(point.projected);
      const QPointF unit = error > 1e-9 ? residual / error : QPointF();
      const QPointF to = from + unit * kArrowLengthScreenPx;
      QColor color = error > 1e-9 ? directionColor(residual) : QColor("#aaaaaa");
      if (mode_ == 0) {color.setAlpha(210);}
      if (error > 1e-9) {
        painter.setPen(QPen(color, 2));
        painter.drawLine(from, to);
        const QPointF normal(-unit.y(), unit.x());
        QPainterPath arrow;
        arrow.moveTo(to);
        arrow.lineTo(to - unit * 7.0 + normal * 3.0);
        arrow.lineTo(to - unit * 7.0 - normal * 3.0);
        arrow.closeSubpath();
        painter.fillPath(arrow, color);
      } else {
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(from, 2.5, 2.5);
      }
      if (mode_ == 1) {
        const QString label = QStringLiteral("%1 px").arg(error, 0, 'f', 3);
        QPainterPath text_path;
        text_path.addText(to + QPointF(4, -3), label_font, label);
        painter.setPen(QPen(QColor("#081018"), 3.2, Qt::SolidLine,
          Qt::RoundCap, Qt::RoundJoin));
        painter.drawPath(text_path);
        painter.fillPath(text_path, color);
      }
    }
  }
  if (highlighted_sample_ >= 0 && highlighted_sample_ < data_.points.size() &&
    highlighted_point_ >= 0 && highlighted_point_ < data_.points[highlighted_sample_].size()) {
    const QPointF center = map(data_.points[highlighted_sample_][highlighted_point_].projected);
    painter.setPen(QPen(QColor("#ffffff"), 2.5));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(center, 10, 10);
  }
  painter.restore();
  painter.setPen(QColor("#b9cad8"));
  painter.drawText(QPointF(canvas.left(), std::min(height() - 9.0, canvas.bottom() + 20.0)),
    QStringLiteral("图像坐标：X 向右，Y 向下；等长箭头表示方向；全局统计见上方"));
}

void ResultVisualizationCanvas::paintScene(QPainter & painter)
{
  const double board_width = data_.max_x - data_.min_x;
  const double board_height = data_.max_y - data_.min_y;
  const double base = std::max(board_width, board_height);
  const cv::Vec3d corners[4] = {
    {data_.min_x, data_.min_y, 0}, {data_.max_x, data_.min_y, 0},
    {data_.max_x, data_.max_y, 0}, {data_.min_x, data_.max_y, 0}};
  QVector<cv::Vec3d> locations;
  if (mode_ == 2) {
    locations.append(cv::Vec3d(0, 0, 0));
    for (const auto & pose : data_.poses) {
      for (const auto & corner : corners) {locations.append(transform(pose, corner));}
    }
  } else {
    for (const auto & corner : corners) {locations.append(corner);}
    for (const auto & pose : data_.poses) {
      locations.append(inverseTransform(pose, cv::Vec3d(0, 0, 0)));
    }
  }
  cv::Vec3d center(0, 0, 0);
  for (const auto & point : locations) {center += point;}
  center *= 1.0 / locations.size();
  const auto rotate = [&](const cv::Vec3d & point) {
    const cv::Vec3d d = point - center;
    const QVector3D rotated = view_rotation_.rotatedVector(
      QVector3D(d[0], d[1], d[2]));
    return QPointF(rotated.x(), rotated.y());
  };
  double radius = base * 0.5;
  for (const auto & point : locations) {
    radius = std::max(radius, cv::norm(point - center));
  }
  // 尺度仅随样本数据变化，不随拖动角度变化，避免旋转时画面忽大忽小。
  const double extent = 2.5 * (radius + base * 0.3);
  const double scale = std::min(width() - 100.0, height() - 100.0) / extent * zoom_;
  const auto map = [&](const cv::Vec3d & point) {
    const QPointF p = rotate(point);
    return QPointF(width() / 2.0 + p.x() * scale, height() / 2.0 + p.y() * scale);
  };
  const auto line = [&](const cv::Vec3d & a, const cv::Vec3d & b,
      const QColor & color, double thickness = 1.5) {
    painter.setPen(QPen(color, thickness));
    painter.drawLine(map(a), map(b));
  };
  const auto drawBoard = [&](const ResultPose & pose, int index) {
    QPolygonF polygon;
    for (const auto & corner : corners) {polygon << map(transform(pose, corner));}
    const QColor color = index < 0 ? QColor("#e9f2fc") : QColor::fromHsvF(
      std::fmod(index * 0.61803398875, 1.0), 0.7, 0.95);
    painter.setPen(QPen(color, 2));
    painter.setBrush(QColor(color.red(), color.green(), color.blue(), 35));
    painter.drawPolygon(polygon);
    line(transform(pose, corners[0]), transform(pose, corners[2]), color, 0.8);
    painter.setPen(QColor("#eef6ff"));
    painter.drawText(map(transform(pose, corners[0])) + QPointF(4, -4),
      index < 0 ? QStringLiteral("标定板") : QString::number(index + 1));
  };
  const auto drawCamera = [&](const ResultPose & pose, int index) {
    const cv::Vec3d origin = inverseTransform(pose, cv::Vec3d(0, 0, 0));
    const double side = base * 0.16;
    const double depth = base * 0.28;
    const cv::Vec3d face[4] = {{-side,-side,depth}, {side,-side,depth},
      {side,side,depth}, {-side,side,depth}};
    const QColor color = index < 0 ? QColor("#e9f2fc") : QColor::fromHsvF(
      std::fmod(index * 0.61803398875, 1.0), 0.7, 0.95);
    for (int j = 0; j < 4; ++j) {
      const cv::Vec3d a = inverseTransform(pose, face[j]);
      const cv::Vec3d b = inverseTransform(pose, face[(j + 1) % 4]);
      line(origin, a, color);
      line(a, b, color);
    }
    painter.setPen(QColor("#eef6ff"));
    painter.drawText(map(origin) + QPointF(4, -4),
      index < 0 ? QStringLiteral("相机") : QString::number(index + 1));
  };
  const ResultPose identity{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
  if (mode_ == 2) {
    for (int i = 0; i < data_.poses.size(); ++i) {drawBoard(data_.poses[i], i);}
    drawCamera(identity, -1);
  } else {
    drawBoard(identity, -1);
    for (int i = 0; i < data_.poses.size(); ++i) {drawCamera(data_.poses[i], i);}
  }
  const cv::Vec3d origin(0, 0, 0);
  const double axis = base * 0.55;
  line(origin, {axis, 0, 0}, QColor("#ff6868"), 2.5);
  line(origin, {0, axis, 0}, QColor("#65d987"), 2.5);
  line(origin, {0, 0, axis}, QColor("#5daeff"), 2.5);
  painter.setPen(QColor("#ff6868")); painter.drawText(map({axis,0,0}), "X");
  painter.setPen(QColor("#65d987")); painter.drawText(map({0,axis,0}), "Y");
  painter.setPen(QColor("#5daeff")); painter.drawText(map({0,0,axis}), "Z");
}

QVector3D ResultVisualizationCanvas::trackballPoint(const QPointF & position) const
{
  const double radius = std::max(1.0, std::min(width(), height()) * 0.46);
  double x = (position.x() - width() * 0.5) / radius;
  double y = (position.y() - height() * 0.5) / radius;
  const double squared = x * x + y * y;
  // 球面与双曲面连续衔接；光标越过圆周后径向拖动仍会继续旋转。
  const double z = squared <= 0.5 ? std::sqrt(1.0 - squared) :
    0.5 / std::sqrt(squared);
  return QVector3D(x, y, z).normalized();
}

void ResultVisualizationCanvas::mousePressEvent(QMouseEvent * event)
{
  if (mode_ >= 2 && event->button() == Qt::LeftButton) {
    drag_start_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
  }
}
void ResultVisualizationCanvas::mouseMoveEvent(QMouseEvent * event)
{
  if (mode_ < 2 || !(event->buttons() & Qt::LeftButton)) {return;}
  const QVector3D previous = trackballPoint(drag_start_);
  const QVector3D current = trackballPoint(event->position());
  const QQuaternion delta = QQuaternion::rotationTo(previous, current);
  view_rotation_ = (delta * view_rotation_).normalized();
  drag_start_ = event->pos();
  update();
  event->accept();
}
void ResultVisualizationCanvas::mouseReleaseEvent(QMouseEvent * event)
{
  if (mode_ >= 2 && event->button() == Qt::LeftButton) {
    setCursor(Qt::OpenHandCursor);
    event->accept();
  }
}
void ResultVisualizationCanvas::wheelEvent(QWheelEvent * event)
{
  zoom_ = std::clamp(zoom_ * (event->angleDelta().y() > 0 ? 1.18 : 1.0 / 1.18),
    0.15, 12.0);
  update();
  event->accept();
}

ResultVisualizationDialog::ResultVisualizationDialog(
  const QString & data_path, const QStringList & sample_names, QWidget * parent)
: QDialog(parent)
{
  ResultVisualizationData data;
  if (!ResultVisualizationData::load(data_path, data, error_)) {return;}
  auto * ui = new Ui::ResultVisualizationDialog;
  ui->setupUi(this);
  ui->sample_combo->addItems(sample_names);
  auto * canvas = new ResultVisualizationCanvas(ui->canvas_host);
  canvas->setData(data);
  ui->canvas_layout->addWidget(canvas);
  const QFileInfo info(data_path);
  ui->visual_splitter->setStretchFactor(0, 4);
  ui->visual_splitter->setStretchFactor(1, 1);
  ui->error_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  ui->error_table->setAlternatingRowColors(false);
  const auto populate_table = [=]() {
    const QSignalBlocker blocker(ui->error_table);
    ui->error_table->setUpdatesEnabled(false);
    ui->error_table->clearContents();
    ui->error_table->setRowCount(0);
    canvas->setHighlightedPoint(-1, -1);
    const int mode = ui->view_combo->currentIndex();
    if (mode >= 2) {
      ui->error_table->hide();
      ui->error_table->setUpdatesEnabled(true);
      return;
    }
    ui->error_table->show();
    int count = 0;
    for (int i = 0; i < data.points.size(); ++i) {
      if (mode == 1 && i != ui->sample_combo->currentIndex()) {continue;}
      count += data.points[i].size();
    }
    ui->error_table->setRowCount(count);
    int row = 0;
    for (int i = 0; i < data.points.size(); ++i) {
      if (mode == 1 && i != ui->sample_combo->currentIndex()) {continue;}
      for (int j = 0; j < data.points[i].size(); ++j, ++row) {
        const QPointF delta = data.points[i][j].observed - data.points[i][j].projected;
        const double error = std::hypot(delta.x(), delta.y());
        const QStringList values{
          sample_names.value(i, QString::number(i + 1)),
          QString::number(data.points[i][j].projected.x(), 'f', 1),
          QString::number(data.points[i][j].projected.y(), 'f', 1),
          QString::number(delta.x(), 'f', 4),
          QString::number(delta.y(), 'f', 4),
          QString::number(error, 'f', 4)};
        for (int col = 0; col < values.size(); ++col) {
          auto * item = new QTableWidgetItem(values[col]);
          item->setBackground(QColor(row % 2 == 0 ? "#15212e" : "#223247"));
          item->setForeground(QColor("#e4edf7"));
          if (col == 0) {
            item->setData(Qt::UserRole, i);
            item->setData(Qt::UserRole + 1, j);
          }
          ui->error_table->setItem(row, col, item);
        }
      }
    }
    ui->error_table->setUpdatesEnabled(true);
  };
  const auto refresh = [=]() {
    const int mode = ui->view_combo->currentIndex();
    canvas->setMode(mode);
    canvas->setSample(ui->sample_combo->currentIndex());
    const bool errors = mode < 2;
    ui->sample_combo->setEnabled(mode == 1);
    ui->sample_label->setEnabled(mode == 1);
    ui->svg_button->setEnabled(errors);
    ui->statistics_label->setVisible(errors);
    if (errors) {
      ui->statistics_label->setText(errorStatistics(data,
        mode == 0 ? -1 : ui->sample_combo->currentIndex()));
    }
    ui->summary_label->setText(errors ?
      (mode == 0 ? QStringLiteral("全局图：等长彩色箭头仅表示方向，图中不显示逐点数值；上方是方向与误差分布，下表可查每点数值。滚轮缩放。") :
        QStringLiteral("单张图：等长彩色箭头表示方向，点旁仅标实际误差(px)；下表可查 ΔX、ΔY。滚轮缩放。")) :
      QStringLiteral("同一组标定外参：%1。X 红、Y 绿、Z 蓝；单位米。自由拖动旋转视角，滚轮缩放，编号对应样本顺序。")
      .arg(mode == 2 ? QStringLiteral("相机固定，逐张显示标定板") :
        QStringLiteral("标定板固定，逐张显示相机")));
    populate_table();
  };
  connect(ui->view_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [=](int) {refresh();});
  connect(ui->sample_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [=](int index) {
      canvas->setSample(index);
      ui->statistics_label->setText(errorStatistics(data, index));
      populate_table();
    });
  connect(ui->error_table, &QTableWidget::currentCellChanged, this,
    [=](int row, int, int, int) {
      auto * item = ui->error_table->item(row, 0);
      if (item) {
        canvas->setHighlightedPoint(item->data(Qt::UserRole).toInt(),
          item->data(Qt::UserRole + 1).toInt());
      }
    });
  connect(ui->fit_button, &QPushButton::clicked, canvas,
    &ResultVisualizationCanvas::resetView);
  connect(ui->svg_button, &QPushButton::clicked, this, [=]() {
    const QString name = ui->view_combo->currentIndex() == 0 ?
      QStringLiteral("global_error_vectors.svg") :
      QStringLiteral("error_vectors_%1.svg").arg(ui->sample_combo->currentIndex());
    const QString path = info.dir().filePath(name);
    if (QFileInfo::exists(path)) {QDesktopServices::openUrl(QUrl::fromLocalFile(path));}
  });
  connect(ui->close_button, &QPushButton::clicked, this, &QDialog::accept);
  refresh();
  valid_ = true;
}
}  // namespace danmucamera
