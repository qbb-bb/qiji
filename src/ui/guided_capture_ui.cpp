#include "calibration_window.hpp"

#include "guided_capture_plan.hpp"
#include "image_label.hpp"
#include "image_quality_panel.hpp"
#include "ui_guided_capture_dialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QPushButton>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <array>
#include <cmath>

namespace danmucamera
{
namespace
{
constexpr int kSampleValidRole = Qt::UserRole + 3;
constexpr int kManagedSampleRole = Qt::UserRole + 4;
constexpr int kCornerPreviewRole = Qt::UserRole + 5;
constexpr int kQualityWarningRole = Qt::UserRole + 6;
}

void CalibrationWindow::startGuidedCapture()
{
  if (guided_active_ || guided_start_pending_ || batch_import_active_) {return;}
  if (!camera_connected_ || !selected_signal_calibratable_ ||
    !calibrationKindAvailable() || !validateSettings()) {return;}
  guide_settings_ = transport_.guideSettings();
  const QString guide_error = GuidedCapturePlan::validationError(guide_settings_, currentSettings());
  if (!guide_error.isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("自动标定配置无效"),
      guide_error + QStringLiteral("\n请检查 config/calibration.yaml 的 calibration_ui.guide。"));
    return;
  }

  if (!latest_frame_.isNull() &&
    GuidedCapturePlan::target(0, currentSettings(), guide_settings_, latest_frame_.size()).isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("自动标定"),
      QStringLiteral("引导框生成失败。请检查视频分辨率、板尺寸和目标面积。"));
    return;
  }
  guided_active_ = true;
  guided_backend_ready_ = false;
  guided_waiting_capture_ = false;
  guided_waiting_save_ = false;
  guided_wait_for_clear_ = false;
  guided_step_ = 0;
  guided_stable_count_ = 0;
  guided_paths_.clear();
  guided_preview_cache_.clear();

  guided_dialog_ = new QDialog(this);
  guided_dialog_->setAttribute(Qt::WA_DeleteOnClose);
  auto * guide_ui = new Ui::GuidedCaptureDialog;
  guide_ui->setupUi(guided_dialog_);
  guided_dialog_->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
    Qt::WindowSystemMenuHint | Qt::WindowMinimizeButtonHint |
    Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
  guided_live_view_ = guide_ui->guide_live_view;
  guided_status_label_ = guide_ui->guide_status_label;
  guided_status_label_->setText(GuidedCapturePlan::instruction(0));
  guided_live_view_->setGuideStyle({
    QColor(guide_settings_.guide_color),
    QColor(guide_settings_.ready_color),
    QColor(guide_settings_.detected_color),
    guide_settings_.guide_line_width,
    guide_settings_.ready_line_width,
    guide_settings_.detected_line_width,
    guide_settings_.corner_radius_px});
  guided_live_view_->clearImage(QStringLiteral("等待相机视频…"));
  connect(guide_ui->stop_guide_button, &QPushButton::clicked,
    guided_dialog_, &QDialog::reject);
  connect(guided_dialog_, &QDialog::destroyed, this, [guide_ui]() {delete guide_ui;});
  connect(guided_dialog_, &QDialog::finished, this, [this]() {
    if (guided_active_) {stopGuidedCapture();}
  });
  guided_dialog_->show();
  if (!latest_frame_.isNull()) {showImage(guided_live_view_, latest_frame_);}
  guided_live_view_->setGuideOverlay(true,
    GuidedCapturePlan::target(0, currentSettings(), guide_settings_, latest_frame_.size()),
    {}, GuidedCapturePlan::instruction(0), false);

  guided_button_->setText(QStringLiteral("关闭自动标定窗口"));
  capture_button_->setEnabled(false);
  select_image_button_->setEnabled(false);
  accept_button_->setEnabled(false);
  retake_button_->setEnabled(false);
  sample_list_->setEnabled(false);
  for (QWidget * control : {static_cast<QWidget *>(board_type_combo_),
    static_cast<QWidget *>(board_width_spin_), static_cast<QWidget *>(board_height_spin_),
    static_cast<QWidget *>(square_size_spin_),
    static_cast<QWidget *>(board_physical_width_spin_),
    static_cast<QWidget *>(board_physical_height_spin_),
    static_cast<QWidget *>(marker_size_spin_), static_cast<QWidget *>(dictionary_combo_),
    static_cast<QWidget *>(camera_model_combo_),
    static_cast<QWidget *>(distortion_model_combo_),
    static_cast<QWidget *>(auto_initial_checkbox_),
    static_cast<QWidget *>(guide_use_initial_checkbox_),
    static_cast<QWidget *>(guide_initial_width_spin_),
    static_cast<QWidget *>(guide_initial_height_spin_)})
  {
    control->setEnabled(false);
  }
  for (auto * spin : initial_camera_spins_) {spin->setEnabled(false);}
  for (auto * spin : initial_distortion_spins_) {spin->setEnabled(false);}
  run_button_->setEnabled(false);
  delete_button_->setEnabled(false);
  calibration_status_->setText(GuidedCapturePlan::instruction(0));
  if (!camera_streaming_) {
    guided_start_pending_ = true;
    guided_status_label_->setText(QStringLiteral("正在等待图像话题…"));
    calibration_start_stop_button_->click();
    return;
  }
  beginGuidedDetection();
}

void CalibrationWindow::beginGuidedDetection()
{
  if (!guided_active_ || !camera_streaming_) {return;}
  transport_.startGuidedDetection(currentSettings());
  QTimer::singleShot(10000, this, [this]() {
    if (guided_active_ && !guided_backend_ready_) {
      stopGuidedCapture();
      QMessageBox::warning(this, QStringLiteral("自动标定"),
        QStringLiteral("实时特征点检测启动超时，请检查算法后端。"));
    }
  });
}

void CalibrationWindow::stopGuidedCapture(bool completed)
{
  if (!guided_active_) {return;}
  guided_active_ = false;
  guided_start_pending_ = false;
  guided_backend_ready_ = false;
  guided_waiting_capture_ = false;
  guided_waiting_save_ = false;
  transport_.stopGuidedDetection();
  QDialog * dialog = guided_dialog_;
  guided_dialog_ = nullptr;
  guided_live_view_ = nullptr;
  guided_status_label_ = nullptr;
  if (dialog) {dialog->close();}
  guided_button_->setText(QStringLiteral("自动标定 · 引导拍摄 18 张"));
  sample_list_->setEnabled(true);
  for (QWidget * control : {static_cast<QWidget *>(board_type_combo_),
    static_cast<QWidget *>(board_width_spin_), static_cast<QWidget *>(board_height_spin_),
    static_cast<QWidget *>(square_size_spin_),
    static_cast<QWidget *>(board_physical_width_spin_),
    static_cast<QWidget *>(board_physical_height_spin_),
    static_cast<QWidget *>(marker_size_spin_), static_cast<QWidget *>(dictionary_combo_),
    static_cast<QWidget *>(camera_model_combo_),
    static_cast<QWidget *>(distortion_model_combo_),
    static_cast<QWidget *>(auto_initial_checkbox_),
    static_cast<QWidget *>(guide_use_initial_checkbox_)})
  {
    control->setEnabled(true);
  }
  guide_initial_width_spin_->setEnabled(guide_use_initial_checkbox_->isChecked());
  guide_initial_height_spin_->setEnabled(guide_use_initial_checkbox_->isChecked());
  for (auto * spin : initial_camera_spins_) {spin->setEnabled(true);}
  for (auto * spin : initial_distortion_spins_) {spin->setEnabled(true);}
  capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
  select_image_button_->setEnabled(true);
  accept_button_->setEnabled(false);
  retake_button_->setEnabled(false);
  if (!completed) {
    calibration_status_->setText(QStringLiteral("自动拍摄已停止；已保存 %1/18 张")
      .arg(guided_paths_.size()));
  }
  updateSampleCount();
}

void CalibrationWindow::handleGuideDetection(const GuideDetection & detection)
{
  if (!guided_active_ || !guided_backend_ready_ || guided_step_ >= GuidedCapturePlan::kImageCount) {
    return;
  }
  const auto decision = GuidedCapturePlan::evaluate(
    guided_step_, currentSettings(), guide_settings_, detection);
  if (decision.target.size() != 4) {
    stopGuidedCapture();
    QMessageBox::warning(this, QStringLiteral("自动标定"),
      QStringLiteral("第 %1 张引导框生成失败。请检查视频分辨率、板尺寸和目标面积。")
        .arg(guided_step_ + 1));
    return;
  }
  const bool quality_ok = !detection.blur_warning && !detection.exposure_warning &&
    !detection.board_size_warning;
  const bool ready = decision.ready && quality_ok;
  QString status = QStringLiteral("特征 %1/%2  |  自动采集：%3  |  %4")
    .arg(detection.points).arg(detection.expected_points)
    .arg(ready ? QStringLiteral("就绪") : QStringLiteral("未就绪"))
    .arg(decision.instruction);
  status += decision.shape_mode ?
    QStringLiteral("  |  画面形状引导（无需相机内参）") :
    QStringLiteral("  |  三维姿态引导");
  status += QStringLiteral("  |  目标依次转 X→Y→Z (%1°, %2°, %3°)")
    .arg(decision.target_xyz_deg[0], 0, 'f', 1)
    .arg(decision.target_xyz_deg[1], 0, 'f', 1)
    .arg(decision.target_xyz_deg[2], 0, 'f', 1);
  if (decision.shape_mode) {
    status += decision.shape_error_ratio >= 0 ?
      QStringLiteral("  |  四角形状偏差 %1%")
        .arg(decision.shape_error_ratio * 100.0, 0, 'f', 1) :
      QStringLiteral("  |  四角形状待识别");
  } else if (decision.pose_valid) {
    status += QStringLiteral("  |  姿态误差 %1°；相机轴修正 X %2°、Y %3°、Z %4°")
      .arg(decision.orientation_error_deg, 0, 'f', 1)
      .arg(decision.camera_rotation_correction_deg[0], 0, 'f', 1)
      .arg(decision.camera_rotation_correction_deg[1], 0, 'f', 1)
      .arg(decision.camera_rotation_correction_deg[2], 0, 'f', 1);
  } else {
    status += detection.detected ?
      QStringLiteral("  |  实际姿态待估计：%1").arg(detection.pose_error) :
      QStringLiteral("  |  实际姿态待识别");
  }
  status += QStringLiteral("  |  覆盖 %1%")
    .arg(qRound(decision.coverage * 100));
  if (detection.board_area_ratio >= 0.0) {
    status += QStringLiteral("  |  板面占画面 %1%")
      .arg(detection.board_area_ratio * 100.0, 0, 'f', 1);
  }
  QStringList actions;
  if (!detection.detected || !detection.complete) {
    actions << QStringLiteral("角点未完整识别：请让标定板全部入画、避免遮挡和反光");
  }
  if (detection.blur_warning) {
    actions << QStringLiteral("图像模糊：请稳住标定板、改善照明并缩短曝光");
  }
  if (detection.exposure_warning) {
    const bool dark = detection.warning_reason.contains(QStringLiteral("under"));
    const bool bright = detection.warning_reason.contains(QStringLiteral("over"));
    actions << (dark && bright ? QStringLiteral("暗部和亮部占比都超限：请调整照明，减少强反差") :
      dark ? QStringLiteral("曝光不足或暗部占比过高：请增加照明或曝光") :
      bright ? QStringLiteral("过曝或亮部占比过高：请减少照明或曝光，避免反光") :
      QStringLiteral("曝光分布不合格：请调整照明和曝光"));
  }
  if (detection.board_size_warning ||
    (detection.board_area_ratio >= 0 &&
    detection.board_area_ratio < guide_settings_.min_board_frame_ratio)) {
    actions << (detection.warning_reason.contains(QStringLiteral("board_too_close")) ?
      QStringLiteral("标定板太近：请退后") : QStringLiteral("标定板太远：请靠近"));
  }
  if (detection.complete && !decision.inside) {
    actions << QStringLiteral("请把标定板完整放入绿色透视框和画面内");
  }
  if (detection.complete &&
    decision.coverage < guide_settings_.min_guide_coverage_ratio) {
    actions << QStringLiteral("请移动或调整板面，使绿框覆盖达到 %1%")
      .arg(guide_settings_.min_guide_coverage_ratio * 100.0, 0, 'f', 0);
  }
  if (detection.complete && !decision.orientation_ready) {
    if (decision.shape_mode) {
      actions << (decision.shape_direction.isEmpty() ?
        QStringLiteral("请调整板面，使四个角贴近绿框") : decision.shape_direction);
    } else if (decision.pose_valid) {
      const auto & correction = decision.camera_rotation_correction_deg;
      const std::array<QString, 3> axis = {QStringLiteral("X"),
        QStringLiteral("Y"), QStringLiteral("Z")};
      int largest = 0;
      for (int k = 1; k < 3; ++k) {
        if (std::abs(correction[k]) > std::abs(correction[largest])) {largest = k;}
      }
      actions << QStringLiteral("请沿相机%1轴%2旋转板面")
        .arg(axis[largest], correction[largest] > 0 ?
          QStringLiteral("正向") : QStringLiteral("反向"));
    } else {
      actions << QStringLiteral("姿态无法估计：请稳住并完整显示标定板");
    }
  }
  if (actions.isEmpty() && ready) {actions << QStringLiteral("全部达标，保持稳定准备拍摄");}
  if (!actions.isEmpty()) {status += QStringLiteral("  |  ") + actions.join(QStringLiteral("；"));}
  if (guided_wait_for_clear_) {
    status += QStringLiteral("  |  请先移开标定板再重试");
    if (!decision.ready) {guided_wait_for_clear_ = false;}
  }
  if (guided_waiting_capture_ || guided_waiting_save_) {
    status = QStringLiteral("第 %1/18 张：正在复查并保存…").arg(guided_step_ + 1);
  }
  if (guided_live_view_) {
    guided_live_view_->setGuideOverlay(true, decision.target, decision.observed, status, ready);
  }
  if (guided_status_label_) {guided_status_label_->setText(status);}
  calibration_status_->setText(status);
  if (guided_waiting_capture_ || guided_waiting_save_ || guided_wait_for_clear_) {return;}
  guided_stable_count_ = ready ? guided_stable_count_ + 1 : 0;
  if (guided_stable_count_ < 3) {return;}
  guided_stable_count_ = 0;
  guided_waiting_capture_ = true;
  guided_status_received_ = false;
  guided_preview_received_ = false;
  guided_quality_received_ = false;
  guided_capture_valid_ = false;
  current_corner_preview_ = QImage();
  current_quality_ = ImageQualityMetrics{};
  transport_.captureForCalibration(currentSettings());
  const int captured_step = guided_step_;
  QTimer::singleShot(15000, this, [this, captured_step]() {
    if (guided_active_ && guided_waiting_capture_ && guided_step_ == captured_step) {
      showGuidedProblem(QStringLiteral("拍摄或特征点复查超时，请检查相机和算法后端"));
    }
  });
}

void CalibrationWindow::maybeCompleteGuidedCapture()
{
  if (!guided_active_ || !guided_waiting_capture_ || guided_waiting_save_ ||
    !guided_status_received_ || !guided_preview_received_ || !guided_quality_received_) {
    return;
  }
  if (!guided_capture_valid_ || current_corner_preview_.isNull() ||
    current_quality_.blur_warning || current_quality_.exposure_warning ||
    current_quality_.board_size_warning) {
    QStringList reasons;
    if (!guided_capture_valid_) {reasons << guided_corner_message_;}
    if (current_quality_.blur_warning) {reasons << QStringLiteral("图像模糊");}
    if (current_quality_.exposure_warning) {reasons << QStringLiteral("曝光分布异常");}
    if (current_quality_.board_size_warning) {
      reasons << (current_quality_.warning_reason.contains(QStringLiteral("board_too_far")) ?
        QStringLiteral("标定板太远") : QStringLiteral("标定板太近"));
    }
    if (current_corner_preview_.isNull()) {reasons << QStringLiteral("特征点预览不可用");}
    showGuidedProblem(reasons.join(QStringLiteral("；")));
    return;
  }
  guided_waiting_save_ = true;
  transport_.saveFrame(QStringLiteral("guided:%1").arg(guided_step_));
  const int saved_step = guided_step_;
  QTimer::singleShot(15000, this, [this, saved_step]() {
    if (guided_active_ && guided_waiting_save_ && guided_step_ == saved_step) {
      showGuidedProblem(QStringLiteral("保存超时，请检查保存服务"));
    }
  });
}

void CalibrationWindow::showGuidedProblem(const QString & reason)
{
  if (!guided_active_) {return;}
  guided_waiting_capture_ = false;
  guided_waiting_save_ = false;
  guided_wait_for_clear_ = true;
  guided_stable_count_ = 0;
  appendLog(QStringLiteral("自动拍摄第 %1 张需重拍：%2").arg(guided_step_ + 1).arg(reason));
  QDialog dialog(guided_dialog_ ? static_cast<QWidget *>(guided_dialog_) :
    static_cast<QWidget *>(this));
  dialog.setWindowTitle(QStringLiteral("第 %1 张需重拍").arg(guided_step_ + 1));
  auto * layout = new QVBoxLayout(&dialog);
  auto * explanation = new QLabel(reason, &dialog);
  explanation->setWordWrap(true);
  layout->addWidget(explanation);
  auto * preview = new ImageLabel(&dialog);
  preview->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  if (!current_corner_preview_.isNull()) {preview->setImage(current_corner_preview_);}
  else {preview->clearImage(QStringLiteral("未收到特征点识别图"));}
  layout->addWidget(preview, 1);
  auto * quality = new ImageQualityPanel(&dialog);
  if (guided_quality_received_) {
    quality->setMetrics(current_quality_, QStringLiteral("本次拍摄"));
  } else {
    quality->clear(QStringLiteral("未收到图像质量分析"));
  }
  layout->addWidget(quality);
  auto * retry = new QPushButton(QStringLiteral("移开标定板后重拍"), &dialog);
  layout->addWidget(retry);
  connect(retry, &QPushButton::clicked, &dialog, &QDialog::accept);
  dialog.resize(760, 730);
  dialog.exec();
}

void CalibrationWindow::finishGuidedSave(bool success, const QString & path)
{
  if (!guided_active_ || !guided_waiting_save_) {return;}
  guided_waiting_save_ = false;
  guided_waiting_capture_ = false;
  if (!success) {
    showGuidedProblem(path);
    return;
  }
  auto * item = new QListWidgetItem(
    QStringLiteral("✓ 自动 %1/18 — %2").arg(guided_step_ + 1)
      .arg(QFileInfo(path).fileName()), sample_list_);
  item->setData(Qt::UserRole, path);
  item->setData(kSampleValidRole, true);
  item->setData(kManagedSampleRole, true);
  item->setData(kCornerPreviewRole, current_corner_preview_);
  item->setData(kQualityWarningRole, false);
  quality_cache_.insert(path, current_quality_);
  guided_preview_cache_.insert(path, current_corner_preview_);
  guided_paths_ << path;
  ++guided_step_;
  appendLog(QStringLiteral("自动拍摄已保存 %1/18：%2").arg(guided_step_).arg(path));
  if (guided_step_ == GuidedCapturePlan::kImageCount) {
    const QStringList paths = guided_paths_;
    stopGuidedCapture(true);
    calibration_requested_ = true;
    guided_solver_pending_ = true;
    calibration_status_->setText(QStringLiteral("18 张拍摄完成，正在自动标定…"));
    run_button_->setEnabled(false);
    transport_.runCalibration(paths, currentSettings());
    return;
  }
  updateSampleCount();
  calibration_status_->setText(GuidedCapturePlan::instruction(guided_step_));
  if (guided_status_label_) {guided_status_label_->setText(calibration_status_->text());}
  if (guided_live_view_) {
    guided_live_view_->setGuideOverlay(true,
      GuidedCapturePlan::target(guided_step_, currentSettings(), guide_settings_, latest_frame_.size()),
      {}, calibration_status_->text(), false);
  }
}
}  // namespace danmucamera
