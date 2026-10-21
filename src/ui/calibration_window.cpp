#include "calibration_window.hpp"

#include "image_label.hpp"
#include "point_cloud_view.hpp"
#include "image_quality_panel.hpp"
#include "result_visualization.hpp"
#include "calibration_ros_client.hpp"
#include "ui_calibration_window.h"

#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QCloseEvent>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialog>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QLabel>
#include <QLayout>
#include <QListWidget>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QObject>
#include <QPushButton>
#include <QRegularExpression>
#include <QSettings>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSize>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTextEdit>
#include <QTimer>
#include <QToolButton>
#include <QWidget>
#include <QWidgetAction>

#include <algorithm>
#include <csignal>
#include <functional>
#include <memory>
#include <vector>

namespace
{
constexpr int kSampleValidRole = Qt::UserRole + 3;
constexpr int kManagedSampleRole = Qt::UserRole + 4;
constexpr int kCornerPreviewRole = Qt::UserRole + 5;
constexpr int kQualityWarningRole = Qt::UserRole + 6;
constexpr int kQualityOverrideRole = Qt::UserRole + 7;
volatile std::sig_atomic_t g_activate_window_requested = 0;

QString qualityProblemText(const danmucamera::ImageQualityMetrics & quality)
{
  QStringList problems;
  if (quality.warning_reason.contains(QStringLiteral("blur"))) {problems << QStringLiteral("模糊");}
  if (quality.warning_reason.contains(QStringLiteral("under"))) {problems << QStringLiteral("暗部偏多");}
  if (quality.warning_reason.contains(QStringLiteral("over"))) {problems << QStringLiteral("亮部偏多");}
  if (quality.warning_reason.contains(QStringLiteral("board_too_far"))) {problems << QStringLiteral("标定板太远");}
  if (quality.warning_reason.contains(QStringLiteral("board_too_close"))) {problems << QStringLiteral("标定板太近");}
  if (quality.warning_reason.contains(QStringLiteral("unreadable"))) {problems << QStringLiteral("无法读取");}
  return problems.join(QStringLiteral("、"));
}


void requestWindowActivation(int)
{
  g_activate_window_requested = 1;
}

bool cameraUsbPresent(const QString & device_id)
{
  const QStringList fields = device_id.split(':');
  const QString backend = fields.value(0);
  if (backend == QStringLiteral("v4l2")) {
    return QFileInfo::exists(fields.value(1));
  }
  const QByteArray expected_vendor = backend == QStringLiteral("orbbec") ? "2bc5" :
    backend == QStringLiteral("realsense") ? "8086" : QByteArray();
  if (expected_vendor.isEmpty()) {return true;}

  const QDir usb_devices(QStringLiteral("/sys/bus/usb/devices"));
  for (const QString & entry : usb_devices.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
    QFile vendor_file(usb_devices.filePath(entry + QStringLiteral("/idVendor")));
    if (vendor_file.open(QIODevice::ReadOnly) &&
      vendor_file.readAll().trimmed().toLower() == expected_vendor)
    {
      return true;
    }
  }
  return false;
}

}  // namespace

namespace danmucamera
{
CalibrationWindow::CalibrationWindow(CalibrationRosClient & transport)
: transport_(transport), ui_(new ::Ui::CalibrationWindow)
{
  buildUi();
  bindTransport();
  transport_timer_ = new QTimer(this);
  connect(transport_timer_, &QTimer::timeout, this, [this]() {
    if (!transport_.spinSome()) {QApplication::quit();}
  });
  transport_timer_->start(100);
  device_presence_timer_ = new QTimer(this);
  connect(device_presence_timer_, &QTimer::timeout, this, [this]() {
    // 只读 sysfs，不调用厂商 SDK，不与启动/停止请求争用 ROS executor。
    if (camera_connected_ && !connected_device_id_.isEmpty() &&
      !cameraUsbPresent(connected_device_id_))
    {
      handleCameraDisconnected(QStringLiteral(
        "USB 总线中已找不到当前相机，已立即恢复为未连接状态"));
    }
  });
  device_presence_timer_->start(250);
  QTimer::singleShot(600, this, [this]() {
    transport_.refreshDevices();
    transport_.refreshTopicInputs();
  });
  QTimer::singleShot(2000, this, [this]() {transport_.refreshTopicInputs();});
}

CalibrationWindow::~CalibrationWindow()
{
  delete ui_;
}

void CalibrationWindow::closeEvent(QCloseEvent * event)
{
  if (guided_active_) {stopGuidedCapture();}
  transport_timer_->stop();
  QMainWindow::closeEvent(event);
}

void CalibrationWindow::buildUi()
{
  ui_->setupUi(this);
  pages_ = ui_->pages;
  home_page_ = ui_->home_page;
  calibration_page_ = ui_->calibration_page;
  calibration_page_->setFocusPolicy(Qt::StrongFocus);
  camera_combo_ = ui_->camera_combo;
  stream_button_ = new QToolButton(home_page_);
  stream_button_->setText(QStringLiteral("信息流 ▾"));
  stream_button_->setToolTip(QStringLiteral("勾选同一相机要显示的信息流，可同时选择多路"));
  stream_button_->setPopupMode(QToolButton::InstantPopup);
  stream_button_->setMenu(new QMenu(stream_button_));
  ui_->deviceActions->insertWidget(
    ui_->deviceActions->indexOf(ui_->start_stop_button) + 1, stream_button_);
  camera_combo_->setToolTip(QStringLiteral("选择物理相机；RGB、红外和深度流在右侧“信息流”中勾选"));
  resolution_combo_ = ui_->resolution_combo;
  calibration_type_combo_ = ui_->calibration_type_combo;
  disable_emitter_checkbox_ = ui_->disable_emitter_checkbox;
  refresh_button_ = ui_->refresh_button;
  connect_button_ = ui_->connect_button;
  start_stop_button_ = ui_->start_stop_button;
  calibration_start_stop_button_ = ui_->calibration_start_stop_button;
  disconnect_button_ = ui_->disconnect_button;
  home_capture_button_ = ui_->home_capture_button;
  home_save_button_ = ui_->home_save_button;
  home_retake_button_ = ui_->home_retake_button;
  enter_calibration_button_ = ui_->enter_calibration_button;
  connection_status_ = ui_->connection_status;
  signal_notice_ = ui_->signal_notice;
  home_live_view_ = ui_->home_live_view;
  home_capture_view_ = ui_->home_capture_view;
  home_image_splitter_ = ui_->home_image_splitter;
  ui_->homeRoot->setContentsMargins(16, 8, 16, 12);
  ui_->homeRoot->setSpacing(7);
  ui_->homeRoot->setStretch(3, 1);
  ui_->cameraControls->setSpacing(5);
  topic_inputs_panel_ = new QWidget(home_page_);
  auto * topic_layout = new QHBoxLayout(topic_inputs_panel_);
  topic_layout->setContentsMargins(0, 2, 0, 2);
  topic_layout->addWidget(new QLabel(QStringLiteral("图像话题"), topic_inputs_panel_));
  image_topic_edit_ = new QLineEdit(topic_inputs_panel_);
  image_topic_edit_->setPlaceholderText(QStringLiteral("/camera/image_raw"));
  topic_layout->addWidget(image_topic_edit_, 1);
  topic_layout->addWidget(new QLabel(QStringLiteral("点云话题"), topic_inputs_panel_));
  cloud_topic_edit_ = new QLineEdit(topic_inputs_panel_);
  cloud_topic_edit_->setPlaceholderText(QStringLiteral("/camera/points"));
  topic_layout->addWidget(cloud_topic_edit_, 1);
  ui_->cameraControls->insertWidget(1, topic_inputs_panel_);
  topic_inputs_panel_->hide();
  ui_->deviceActions->setSpacing(7);
  ui_->captureActions->setSpacing(7);
  signal_notice_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  home_page_->setStyleSheet(QStringLiteral(
    "QWidget#home_page QPushButton { min-height: 32px; padding: 0 12px; }"
    "QWidget#home_page QComboBox { min-height: 30px; }"
    "QWidget#home_page QLabel#homeTitle { font-size: 21px; }"));
  home_primary_panel_ = new QWidget;
  home_primary_panel_->setObjectName(QStringLiteral("homeStreamPanel"));
  home_primary_panel_->setStyleSheet(QStringLiteral(
    "QWidget#homeStreamPanel { border: 1px solid #344258; border-radius: 6px; }"
    "QLabel#homeStreamTitle { border: none; padding: 2px 6px; color: #dce8fb; font-weight: 600; }"));
  auto * primary_layout = new QVBoxLayout(home_primary_panel_);
  primary_layout->setContentsMargins(4, 4, 4, 4);
  primary_layout->setSpacing(4);
  home_primary_title_ = new QLabel(QStringLiteral("RGB 彩色"), home_primary_panel_);
  home_primary_title_->setObjectName(QStringLiteral("homeStreamTitle"));
  primary_layout->addWidget(home_primary_title_);
  home_live_view_->setParent(home_primary_panel_);
  home_live_view_->setMinimumSize(180, 180);
  primary_layout->addWidget(home_live_view_, 1);
  home_capture_view_->setMinimumSize(180, 180);
  home_image_splitter_->insertWidget(0, home_primary_panel_);
  cloud_panel_ = new QWidget(home_image_splitter_);
  cloud_panel_->setObjectName(QStringLiteral("homeStreamPanel"));
  cloud_panel_->setStyleSheet(home_primary_panel_->styleSheet());
  auto * cloud_layout = new QVBoxLayout(cloud_panel_);
  cloud_layout->setContentsMargins(4, 4, 4, 4);
  cloud_layout->setSpacing(4);
  auto * cloud_title = new QLabel(QStringLiteral("3D 点云"), cloud_panel_);
  cloud_title->setObjectName(QStringLiteral("homeStreamTitle"));
  cloud_layout->addWidget(cloud_title);
  cloud_view_ = new PointCloudView(cloud_panel_);
  cloud_layout->addWidget(cloud_view_, 1);
  home_image_splitter_->insertWidget(1, cloud_panel_);
  cloud_panel_->hide();
  board_width_spin_ = ui_->board_width_spin;
  board_height_spin_ = ui_->board_height_spin;
  square_size_spin_ = ui_->square_size_spin;
  board_physical_width_spin_ = ui_->board_physical_width_spin;
  board_physical_height_spin_ = ui_->board_physical_height_spin;
  calibration_status_ = ui_->calibration_status;
  calibration_connection_status_ = ui_->calibration_connection_status;
  resolution_notice_ = ui_->resolution_notice;
  left_view_ = ui_->left_view;
  right_view_ = ui_->right_view;
  image_splitter_ = ui_->image_splitter;
  sample_list_ = ui_->sample_list;
  capture_button_ = ui_->capture_button;
  guided_button_ = ui_->guided_button;
  select_image_button_ = ui_->select_image_button;
  accept_button_ = ui_->accept_button;
  retake_button_ = ui_->retake_button;
  run_button_ = ui_->run_button;
  delete_button_ = ui_->delete_button;
  supplement_button_ = ui_->supplement_button;
  global_error_button_ = ui_->global_error_button;
  sample_count_label_ = ui_->sample_count_label;
  log_view_ = ui_->log_view;
  auto * sidebar = new QWidget(ui_->content_splitter);
  auto * sidebar_layout = new QVBoxLayout(sidebar);
  sidebar_layout->setContentsMargins(0, 0, 0, 0);
  sidebar_layout->setSpacing(9);
  sample_list_->setParent(sidebar);
  sidebar_layout->addWidget(sample_list_, 1);
  corner_coverage_label_ = new QLabel(sidebar);
  corner_coverage_label_->setWordWrap(true);
  corner_coverage_label_->setTextFormat(Qt::RichText);
  corner_coverage_label_->setStyleSheet(QStringLiteral(
    "background:#222b3a;border:1px solid #344258;border-radius:9px;"
    "padding:8px;color:#dce8fb;"));
  sidebar_layout->addWidget(corner_coverage_label_);
  calibration_quality_panel_ = new ImageQualityPanel(sidebar);
  calibration_quality_panel_->setFixedHeight(248);
  sidebar_layout->addWidget(calibration_quality_panel_);
  calibration_quality_panel_->hide();
  ui_->content_splitter->addWidget(sidebar);
  ui_->content_splitter->setStretchFactor(0, 1);
  ui_->content_splitter->setStretchFactor(1, 0);

  resolution_combo_->setItemData(0, QSize(640, 480));
  resolution_combo_->setItemData(1, QSize(1280, 720));
  resolution_combo_->setItemData(2, QSize(1280, 800));
  resolution_combo_->setItemData(3, QSize(1920, 1080));
  resolution_combo_->setCurrentIndex(1);
  home_image_splitter_->setStretchFactor(0, 1);
  home_image_splitter_->setStretchFactor(1, 1);
  image_splitter_->setStretchFactor(0, 1);
  image_splitter_->setStretchFactor(1, 1);
  image_splitter_->setSizes({640, 640});
  ui_->content_splitter->setSizes({1080, 260});
  ui_->work_log_splitter->setSizes({590, 150});
  buildSettingsUi();
  buildHomePage();
  buildCalibrationPage();
}

void CalibrationWindow::buildSettingsUi()
{
  auto * root = qobject_cast<QVBoxLayout *>(calibration_page_->layout());
  auto * model_row = new QHBoxLayout;
  model_row->setSpacing(9);
  auto add_choice = [this, model_row](const QString & title, QComboBox *& target) {
    model_row->addWidget(new QLabel(title, calibration_page_));
    target = new QComboBox(calibration_page_);
    model_row->addWidget(target);
  };
  add_choice(QStringLiteral("标定板"), board_type_combo_);
  board_type_combo_->addItem(QStringLiteral("棋盘格"), QStringLiteral("chessboard"));
  board_type_combo_->addItem(QStringLiteral("对称圆点阵列"), QStringLiteral("circles_symmetric"));
  board_type_combo_->addItem(QStringLiteral("非对称圆点阵列"), QStringLiteral("circles_asymmetric"));
  board_type_combo_->addItem(QStringLiteral("ChArUco 标记棋盘"), QStringLiteral("charuco"));
  board_type_combo_->addItem(QStringLiteral("AprilGrid 标签阵列"), QStringLiteral("aprilgrid"));
  add_choice(QStringLiteral("相机模型"), camera_model_combo_);
  camera_model_combo_->addItem(QStringLiteral("针孔"), QStringLiteral("pinhole"));
  camera_model_combo_->addItem(QStringLiteral("鱼眼（等距模型）"), QStringLiteral("fisheye"));
  add_choice(QStringLiteral("畸变类型"), distortion_model_combo_);
  model_row->addStretch(1);
  root->insertLayout(1, model_row);

  auto * settings_row = ui_->calibrationSettings;
  marker_size_label_ = new QLabel(QStringLiteral("标记边长(m)"), calibration_page_);
  marker_size_spin_ = new QDoubleSpinBox(calibration_page_);
  marker_size_spin_->setDecimals(4);
  marker_size_spin_->setRange(0.0001, 10.0);
  marker_size_spin_->setValue(0.04);
  marker_size_spin_->setToolTip(QStringLiteral("ChArUco 标记的物理边长，必须小于方格边长"));
  dictionary_label_ = new QLabel(QStringLiteral("标记字典"), calibration_page_);
  dictionary_combo_ = new QComboBox(calibration_page_);
  dictionary_combo_->addItem(QStringLiteral("ArUco 4×4 / 50"), 0);
  dictionary_combo_->addItem(QStringLiteral("ArUco 5×5 / 100"), 5);
  dictionary_combo_->addItem(QStringLiteral("ArUco 6×6 / 250"), 10);
  dictionary_combo_->addItem(QStringLiteral("AprilTag 36h11"), 20);
  square_size_spin_->setToolTip(QStringLiteral("非对称圆点阵列：输入相邻错位圆心的基准间距；同一行相邻圆心间距为其两倍"));
  settings_row->insertWidget(6, marker_size_label_);
  settings_row->insertWidget(7, marker_size_spin_);
  settings_row->insertWidget(8, dictionary_label_);
  settings_row->insertWidget(9, dictionary_combo_);

  auto_initial_checkbox_ = ui_->auto_initial_checkbox;
  guide_use_initial_checkbox_ = ui_->guide_use_initial_checkbox;
  guide_initial_width_spin_ = ui_->guide_initial_width_spin;
  guide_initial_height_spin_ = ui_->guide_initial_height_spin;
  // 暂停自动引导拍摄时隐藏只服务于引导框的设置。
  ui_->boardPhysicalWidthLabel->hide();
  ui_->boardPhysicalHeightLabel->hide();
  ui_->boardPhysicalNote->hide();
  board_physical_width_spin_->hide();
  board_physical_height_spin_->hide();
  guide_use_initial_checkbox_->hide();
  ui_->guideInitialResolutionLabel->hide();
  ui_->guideInitialResolutionTimes->hide();
  guide_initial_width_spin_->hide();
  guide_initial_height_spin_->hide();
  ui_->initial_intrinsics_group->setTitle(QStringLiteral("初始内参"));
  manual_initial_widget_ = ui_->manual_initial_widget;
  initial_camera_spins_ = {ui_->initial_fx_spin, ui_->initial_fy_spin,
    ui_->initial_cx_spin, ui_->initial_cy_spin};
  initial_distortion_labels_ = {ui_->initial_distortion_label_0,
    ui_->initial_distortion_label_1, ui_->initial_distortion_label_2,
    ui_->initial_distortion_label_3, ui_->initial_distortion_label_4,
    ui_->initial_distortion_label_5, ui_->initial_distortion_label_6,
    ui_->initial_distortion_label_7};
  initial_distortion_spins_ = {ui_->initial_distortion_spin_0,
    ui_->initial_distortion_spin_1, ui_->initial_distortion_spin_2,
    ui_->initial_distortion_spin_3, ui_->initial_distortion_spin_4,
    ui_->initial_distortion_spin_5, ui_->initial_distortion_spin_6,
    ui_->initial_distortion_spin_7};
  manual_initial_widget_->hide();

  result_controls_ = new QWidget(calibration_page_);
  auto * result_row = new QHBoxLayout(result_controls_);
  result_row->setContentsMargins(0, 0, 0, 0);
  result_row->addWidget(new QLabel(QStringLiteral("结果视图："), result_controls_));
  show_original_checkbox_ = new QCheckBox(QStringLiteral("原图"), result_controls_);
  show_error_checkbox_ = new QCheckBox(QStringLiteral("特征点重投影误差"), result_controls_);
  show_undistorted_checkbox_ = new QCheckBox(QStringLiteral("去畸变图"), result_controls_);
  show_error_checkbox_->setChecked(true);
  result_row->addWidget(show_original_checkbox_);
  result_row->addWidget(show_error_checkbox_);
  result_row->addWidget(show_undistorted_checkbox_);
  result_row->addStretch(1);
  root->insertWidget(root->count() - 1, result_controls_);
  result_controls_->hide();
  undistorted_view_ = new ImageLabel(image_splitter_);
  undistorted_view_->setPlaceholder(QStringLiteral("去畸变图"));
  image_splitter_->addWidget(undistorted_view_);
  undistorted_view_->hide();
  result_empty_view_ = new ImageLabel(image_splitter_);
  result_empty_view_->setPlaceholder(QStringLiteral("已关闭全部结果视图，可在上方重新勾选"));
  image_splitter_->addWidget(result_empty_view_);
  result_empty_view_->hide();

  connect(board_type_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {updateBoardControls();});
  connect(camera_model_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {updateCameraModelControls();});
  connect(distortion_model_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {updateCameraModelControls();});
  auto update_initial_inputs = [this]() {
    const bool show = !auto_initial_checkbox_->isChecked() ||
      guide_use_initial_checkbox_->isChecked();
    manual_initial_widget_->setVisible(show);
    guide_initial_width_spin_->setEnabled(guide_use_initial_checkbox_->isChecked());
    guide_initial_height_spin_->setEnabled(guide_use_initial_checkbox_->isChecked());
  };
  connect(auto_initial_checkbox_, &QCheckBox::toggled, this,
    [update_initial_inputs](bool) {update_initial_inputs();});
  connect(guide_use_initial_checkbox_, &QCheckBox::toggled, this,
    [update_initial_inputs](bool) {update_initial_inputs();});
  connect(resolution_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {
      const QSize size = resolution_combo_->currentData().toSize();
      if (size.isValid() && !guide_use_initial_checkbox_->isChecked()) {
        guide_initial_width_spin_->setValue(size.width());
        guide_initial_height_spin_->setValue(size.height());
        initial_camera_spins_[2]->setValue(size.width() / 2.0);
        initial_camera_spins_[3]->setValue(size.height() / 2.0);
      }
    });
  auto on_settings_changed = [this]() {
    const auto settings = currentSettings();
    // 引导专用 K/D 不改变标定求解配置，编辑它们不应清空已采集样本。
    const bool solver_manual = !settings.automatic_initial;
    const QString key = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9|%10|%11|%12|%13|%14")
      .arg(settings.board_type).arg(settings.columns).arg(settings.rows)
      .arg(settings.spacing, 0, 'g', 12).arg(settings.marker_length, 0, 'g', 12)
      .arg(settings.dictionary_id).arg(settings.camera_model).arg(settings.distortion_model)
      .arg(settings.automatic_initial)
      .arg(solver_manual ? settings.fx : 0.0).arg(solver_manual ? settings.fy : 0.0)
      .arg(solver_manual ? settings.cx : 0.0).arg(solver_manual ? settings.cy : 0.0)
      .arg([&settings, solver_manual]() {
        QStringList parts;
        if (solver_manual) {
          for (double value : settings.initial_distortion) {parts << QString::number(value, 'g', 12);}
        }
        return parts.join(',');
      }());
    const bool has_session_data = sample_list_->count() > 0 || batch_import_active_ ||
      imported_frame_pending_ || imported_frame_active_ || capture_frozen_ ||
      calibration_view_ == CalibrationView::Result;
    if (!last_board_settings_key_.isEmpty() && key != last_board_settings_key_ &&
      has_session_data) {
      QMessageBox confirm(this);
      confirm.setIcon(QMessageBox::Warning);
      confirm.setWindowTitle(QStringLiteral("确认更改标定参数"));
      confirm.setText(QStringLiteral("更改标定参数将清空当前标定会话中的图片和结果。"));
      confirm.setInformativeText(QStringLiteral("已保存的原图仍保留在磁盘。"));
      auto * proceed = confirm.addButton(QStringLiteral("清空当前并继续"), QMessageBox::DestructiveRole);
      confirm.addButton(QStringLiteral("取消"), QMessageBox::RejectRole);
      confirm.exec();
      if (confirm.clickedButton() != proceed) {
        const auto & previous = last_confirmed_settings_;
        const QList<QObject *> controls = {
          board_type_combo_, camera_model_combo_, distortion_model_combo_, dictionary_combo_,
          board_width_spin_, board_height_spin_, square_size_spin_, marker_size_spin_,
          auto_initial_checkbox_, initial_camera_spins_[0], initial_camera_spins_[1],
          initial_camera_spins_[2], initial_camera_spins_[3],
          initial_distortion_spins_[0], initial_distortion_spins_[1],
          initial_distortion_spins_[2], initial_distortion_spins_[3],
          initial_distortion_spins_[4], initial_distortion_spins_[5],
          initial_distortion_spins_[6], initial_distortion_spins_[7]};
        std::vector<std::unique_ptr<QSignalBlocker>> blockers;
        for (QObject * control : controls) {
          blockers.emplace_back(std::make_unique<QSignalBlocker>(control));
        }
        board_type_combo_->setCurrentIndex(board_type_combo_->findData(previous.board_type));
        camera_model_combo_->setCurrentIndex(camera_model_combo_->findData(previous.camera_model));
        updateBoardControls();
        updateCameraModelControls();
        distortion_model_combo_->setCurrentIndex(
          distortion_model_combo_->findData(previous.distortion_model));
        updateCameraModelControls();
        dictionary_combo_->setCurrentIndex(dictionary_combo_->findData(previous.dictionary_id));
        board_width_spin_->setValue(previous.columns);
        board_height_spin_->setValue(previous.rows);
        square_size_spin_->setValue(previous.spacing);
        marker_size_spin_->setValue(previous.marker_length);
        auto_initial_checkbox_->setChecked(previous.automatic_initial);
        for (int i = 0; i < 4; ++i) {
          initial_camera_spins_[i]->setValue(last_initial_camera_values_[i]);
        }
        for (int i = 0; i < 8; ++i) {
          initial_distortion_spins_[i]->setValue(last_initial_distortion_values_[i]);
        }
        manual_initial_widget_->setVisible(!previous.automatic_initial ||
          guide_use_initial_checkbox_->isChecked());
        return;
      }
      startNewCalibrationSession();
    }
    last_board_settings_key_ = key;
    last_confirmed_settings_ = settings;
    for (int i = 0; i < 4; ++i) {
      last_initial_camera_values_[i] = initial_camera_spins_[i]->value();
    }
    for (int i = 0; i < 8; ++i) {
      last_initial_distortion_values_[i] = initial_distortion_spins_[i]->value();
    }
  };
  for (auto * combo : {board_type_combo_, camera_model_combo_, distortion_model_combo_,
    dictionary_combo_}) {
    connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [on_settings_changed](int) {on_settings_changed();});
  }
  for (auto * spin : {board_width_spin_, board_height_spin_}) {
    connect(spin, qOverload<int>(&QSpinBox::valueChanged), this,
      [on_settings_changed](int) {on_settings_changed();});
  }
  for (auto * spin : {square_size_spin_, marker_size_spin_}) {
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [on_settings_changed](double) {on_settings_changed();});
  }
  for (auto * spin : initial_camera_spins_) {
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [on_settings_changed](double) {on_settings_changed();});
  }
  for (auto * spin : initial_distortion_spins_) {
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [on_settings_changed](double) {on_settings_changed();});
  }
  connect(auto_initial_checkbox_, &QCheckBox::toggled, this,
    [on_settings_changed](bool) {on_settings_changed();});
  for (auto * check : {show_original_checkbox_, show_error_checkbox_,
    show_undistorted_checkbox_}) {
    connect(check, &QCheckBox::toggled, this, [this](bool) {updateResultViews();});
  }
  updateBoardControls();
  updateCameraModelControls();
  on_settings_changed();
}

void CalibrationWindow::rebuildStreamMenu()
{
  QMenu * menu = stream_button_->menu();
  menu->clear();
  stream_actions_.clear();
  const QString selected_id = camera_combo_->currentData().toString();
  const QString prefix = selected_id.left(selected_id.lastIndexOf(':') + 1);
  const bool is_ros = selected_backend_name_ == QStringLiteral("ros-topic");
  bool checked_any = false;
  for (const auto & device : available_devices_) {
    if (!device.path.startsWith(prefix) || prefix.isEmpty()) {continue;}
    if (is_ros && ((device.signal == QStringLiteral("rgb") &&
      image_topic_edit_->text().trimmed().isEmpty()) ||
      (device.signal == QStringLiteral("depth") &&
      cloud_topic_edit_->text().trimmed().isEmpty()))) {continue;}
    auto * option = new QCheckBox(device.signal == QStringLiteral("rgb") ?
      QStringLiteral("RGB 彩色") : device.signal == QStringLiteral("depth") ?
      (is_ros ? QStringLiteral("3D 点云") : QStringLiteral("深度")) :
      device.signal == QStringLiteral("ir-left") ?
      QStringLiteral("左红外") : QStringLiteral("右红外"), menu);
    option->setMinimumWidth(125);
    const bool checked = device.path == selected_id ||
      (!checked_any && !stream_actions_.contains(selected_signal_name_));
    option->setChecked(checked);
    checked_any = checked_any || checked;
    auto * item = new QWidgetAction(menu);
    item->setDefaultWidget(option);
    menu->addAction(item);
    stream_actions_.insert(device.signal, option);
    connect(option, &QCheckBox::toggled, this, [this, option](bool checked) {
      if (!checked && checkedStreams().isEmpty()) {
        const QSignalBlocker blocker(option);
        option->setChecked(true);
        return;
      }
      applySelectedStreams();
    });
  }
  stream_button_->setEnabled(!stream_actions_.isEmpty());
  if (!stream_actions_.contains(selected_signal_name_) && !stream_actions_.isEmpty() &&
    !camera_connected_) {
    selected_signal_name_ = checkedStreams().value(0);
    selected_signal_calibratable_ = selected_signal_name_ == QStringLiteral("rgb");
    applySelectedStreams();
  }
}

QStringList CalibrationWindow::checkedStreams() const
{
  QStringList result;
  for (const auto & signal : {QStringLiteral("rgb"), QStringLiteral("ir-left"),
    QStringLiteral("ir-right"), QStringLiteral("depth")}) {
    if (auto * action = stream_actions_.value(signal); action && action->isChecked()) {
      result << signal;
    }
  }
  return result;
}

void CalibrationWindow::applySelectedStreams()
{
  const QStringList requested = checkedStreams();
  if (requested.isEmpty()) {return;}
  const QString previous_signal = selected_signal_name_;
  const QString primary = selected_backend_name_ == QStringLiteral("ros-topic") &&
    requested.contains(QStringLiteral("rgb")) ? QStringLiteral("rgb") :
    (requested.contains(previous_signal) ? previous_signal : requested.first());
  if (camera_connected_ && primary != previous_signal &&
    (sample_list_->count() > 0 || calibration_view_ == CalibrationView::Result)) {
    QMessageBox choice(this);
    choice.setIcon(QMessageBox::Warning);
    choice.setWindowTitle(QStringLiteral("确认更换标定信号"));
    choice.setText(QStringLiteral("更换标定信号将清空当前标定会话中的图片和结果。"));
    choice.setInformativeText(QStringLiteral("已保存的原图仍保留在磁盘。"));
    auto * proceed = choice.addButton(QStringLiteral("清空当前并继续"), QMessageBox::DestructiveRole);
    choice.addButton(QStringLiteral("取消"), QMessageBox::RejectRole);
    choice.exec();
    if (choice.clickedButton() != proceed) {
      for (auto it = stream_actions_.begin(); it != stream_actions_.end(); ++it) {
        const QSignalBlocker blocker(it.value());
        it.value()->setChecked(active_streams_.contains(it.key()));
      }
      return;
    }
    startNewCalibrationSession();
  }
  const QString old_id = camera_combo_->currentData().toString();
  const QString new_id = old_id.left(old_id.lastIndexOf(':') + 1) + primary;
  if (primary != previous_signal) {
    selected_signal_name_ = primary;
    selected_signal_calibratable_ = primary == QStringLiteral("rgb") ||
      primary.startsWith(QStringLiteral("ir-"));
    updateCalibrationAvailability();
  }
  for (auto it = stream_panels_.begin(); it != stream_panels_.end(); ++it) {
    it.value()->setVisible(requested.contains(it.key()) && it.key() != primary);
  }
  home_primary_panel_->setVisible(primary != QStringLiteral("depth"));
  cloud_panel_->setVisible(requested.contains(QStringLiteral("depth")));
  for (const QString & signal : requested) {
    if (signal == primary || signal == QStringLiteral("depth")) {continue;}
    if (!stream_views_.contains(signal)) {
      auto * panel = new QWidget(home_image_splitter_);
      panel->setObjectName(QStringLiteral("homeStreamPanel"));
      panel->setStyleSheet(home_primary_panel_->styleSheet());
      auto * layout = new QVBoxLayout(panel);
      layout->setContentsMargins(4, 4, 4, 4);
      layout->setSpacing(4);
      auto * title = new QLabel(signal == QStringLiteral("depth") ?
        QStringLiteral("深度") : signal == QStringLiteral("ir-left") ?
        QStringLiteral("左红外") : signal == QStringLiteral("ir-right") ?
        QStringLiteral("右红外") : QStringLiteral("RGB 彩色"), panel);
      title->setObjectName(QStringLiteral("homeStreamTitle"));
      layout->addWidget(title);
      auto * view = new ImageLabel(panel);
      view->setMinimumSize(180, 180);
      view->setPlaceholder(signal + QStringLiteral("：等待首帧…"));
      layout->addWidget(view, 1);
      home_image_splitter_->addWidget(panel);
      home_image_splitter_->setStretchFactor(home_image_splitter_->indexOf(panel), 1);
      stream_views_.insert(signal, view);
      stream_panels_.insert(signal, panel);
    }
    stream_panels_[signal]->show();
  }
  home_primary_title_->setText(primary == QStringLiteral("depth") ?
    QStringLiteral("深度") : primary == QStringLiteral("ir-left") ?
    QStringLiteral("左红外") : primary == QStringLiteral("ir-right") ?
    QStringLiteral("右红外") : QStringLiteral("RGB 彩色"));
  home_live_view_->setPlaceholder(primary + QStringLiteral(" 实时画面"));
  setEqualImageSizes(home_image_splitter_);
  updateCalibrationAvailability();
  if (!camera_connected_ || requested == active_streams_) {return;}
  stream_reconfigure_pending_ = true;
  stream_restart_pending_ = camera_streaming_;
  camera_operation_pending_ = true;
  stream_button_->setEnabled(false);
  if (camera_streaming_) {transport_.stopCamera();}
  else {
    stream_reconfigure_pending_ = false;
    connected_device_id_ = new_id;
    active_streams_ = requested;
    transport_.connectCamera(new_id, active_width_, active_height_,
      disable_emitter_checkbox_->isChecked(), requested,
      image_topic_edit_->text().trimmed(), cloud_topic_edit_->text().trimmed());
  }
}

void CalibrationWindow::updateTopicInputsVisibility()
{
  const bool is_ros = selected_backend_name_ == QStringLiteral("ros-topic");
  topic_inputs_panel_->setVisible(is_ros);
  resolution_combo_->setEnabled(!camera_connected_ && !is_ros);
  if (!camera_connected_) {
    connect_button_->setEnabled(camera_combo_->count() > 0 &&
      (!is_ros || !checkedStreams().isEmpty()));
  }
}

void CalibrationWindow::buildHomePage()
{
  const auto topic_changed = [this](const QString &) {
    if (selected_backend_name_ == QStringLiteral("ros-topic") && !camera_connected_) {
      rebuildStreamMenu();
      updateTopicInputsVisibility();
    }
  };
  connect(image_topic_edit_, &QLineEdit::textChanged, this, topic_changed);
  connect(cloud_topic_edit_, &QLineEdit::textChanged, this, topic_changed);
  connect(camera_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
    selected_signal_calibratable_ = camera_combo_->itemData(index, Qt::UserRole + 1).toBool();
    selected_signal_name_ = camera_combo_->itemData(index, Qt::UserRole + 2).toString();
    selected_backend_name_ = camera_combo_->itemData(index, Qt::UserRole + 3).toString();
    disable_emitter_checkbox_->setEnabled(
      selected_backend_name_ == QStringLiteral("orbbec") ||
      selected_backend_name_ == QStringLiteral("realsense"));
    rebuildStreamMenu();
    updateTopicInputsVisibility();
    updateCalibrationAvailability();
  });
  connect(calibration_type_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int) {updateCalibrationAvailability();});
  connect(disable_emitter_checkbox_, &QCheckBox::toggled, this, [this]() {
    if (camera_connected_) {
      emitter_change_pending_ = true;
      disable_emitter_checkbox_->setEnabled(false);
      disable_emitter_checkbox_->setText(QStringLiteral("设置投射器中…"));
      transport_.setIrEmitterDisabled(disable_emitter_checkbox_->isChecked());
    } else {
      confirmed_emitter_disabled_ = disable_emitter_checkbox_->isChecked();
    }
    updateCalibrationAvailability();
  });

  connect(refresh_button_, &QPushButton::clicked, this, [this]() {
    refresh_button_->setEnabled(false);
    transport_.refreshDevices();
  });
  connect(connect_button_, &QPushButton::clicked, this, [this]() {
    camera_operation_pending_ = true;
    connect_button_->setEnabled(false);
    const QString base_id = camera_combo_->currentData().toString();
    connected_device_id_ = base_id.left(base_id.lastIndexOf(':') + 1) + selected_signal_name_;
    active_streams_ = checkedStreams();
    const QSize resolution = resolution_combo_->currentData().toSize();
    active_width_ = resolution.width();
    active_height_ = resolution.height();
    transport_.connectCamera(
      connected_device_id_, active_width_, active_height_,
      disable_emitter_checkbox_->isChecked(), active_streams_,
      image_topic_edit_->text().trimmed(), cloud_topic_edit_->text().trimmed());
  });
  connect(disconnect_button_, &QPushButton::clicked, this, [this]() {
    camera_operation_pending_ = true;
    disconnect_button_->setEnabled(false);
    start_stop_button_->setEnabled(false);
    transport_.disconnectCamera();
  });
  const auto toggle_camera_stream = [this]() {
    if (camera_operation_pending_) {return;}
    if (pages_->currentWidget() == calibration_page_) {
      QTimer::singleShot(0, this, [this]() {calibration_page_->setFocus(Qt::OtherFocusReason);});
    }
    camera_operation_pending_ = true;
    start_stop_button_->setEnabled(false);
    calibration_start_stop_button_->setEnabled(false);
    start_stop_button_->setText(
      camera_streaming_ ? QStringLiteral("关闭中…") : QStringLiteral("启动中…"));
    calibration_start_stop_button_->setText(start_stop_button_->text());
    disconnect_button_->setEnabled(false);
    if (camera_streaming_) {transport_.stopCamera();}
    else {transport_.startCamera();}
  };
  connect(start_stop_button_, &QPushButton::clicked, this, toggle_camera_stream);
  connect(calibration_start_stop_button_, &QPushButton::clicked, this, toggle_camera_stream);
  connect(home_capture_button_, &QPushButton::clicked, this, [this]() {
    home_capture_button_->setEnabled(false);
    enter_calibration_button_->setEnabled(false);
    home_save_pending_ = false;
    transport_.captureFrame();
  });
  connect(home_save_button_, &QPushButton::clicked, this, [this]() {
    home_save_button_->setEnabled(false);
    home_save_pending_ = true;
    transport_.saveFrame();
  });
  connect(home_retake_button_, &QPushButton::clicked, this, [this]() {
    home_capture_frozen_ = false;
    home_save_pending_ = false;
    home_save_button_->setEnabled(false);
    home_retake_button_->setEnabled(false);
    home_capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
    updateCalibrationAvailability();
    home_capture_view_->hide();
    home_capture_view_->clearImage();
    setEqualImageSizes(home_image_splitter_);
    if (!latest_frame_.isNull()) {
      showImage(home_live_view_, latest_frame_);
    }
  });
  connect(enter_calibration_button_, &QPushButton::clicked, this, [this]() {
    if (!enter_calibration_button_->isEnabled() || !calibrationKindAvailable()) {return;}
    if (sample_list_->count() > 0) {
      QMessageBox choice(this);
      choice.setWindowTitle(QStringLiteral("已有标定会话"));
      choice.setIcon(QMessageBox::Question);
      choice.setText(QStringLiteral("检测到上一次标定仍有 %1 张记录。").arg(sample_list_->count()));
      choice.setInformativeText(QStringLiteral(
        "继续：保留右侧图片和当前结果。\n重新开始：清空界面中的本次列表，但不会删除磁盘图片和日志。"));
      auto * continue_button = choice.addButton(QStringLiteral("继续上一次标定"), QMessageBox::AcceptRole);
      auto * restart_button = choice.addButton(QStringLiteral("重新开始"), QMessageBox::DestructiveRole);
      choice.addButton(QStringLiteral("取消"), QMessageBox::RejectRole);
      choice.exec();
      if (choice.clickedButton() == restart_button) {
        startNewCalibrationSession();
      } else if (choice.clickedButton() != continue_button) {
        return;
      } else if (calibration_view_ == CalibrationView::Capture) {
        resetCaptureView();
      }
    } else {
      startNewCalibrationSession();
    }
    pages_->setCurrentWidget(calibration_page_);
  });
}

void CalibrationWindow::buildCalibrationPage()
{
  connect(ui_->back_button, &QPushButton::clicked, this, [this]() {
    if (guided_active_) {stopGuidedCapture();}
    pages_->setCurrentWidget(home_page_);
  });
  // 自动引导拍摄暂时停用；保留实现，后续需要时再恢复入口。
  guided_button_->hide();
#if 0
  connect(guided_button_, &QPushButton::clicked, this, [this]() {
    if (guided_active_) {stopGuidedCapture();}
    else {startGuidedCapture();}
  });
#endif
  connect(capture_button_, &QPushButton::clicked, this, [this]() {
    if (!validateSettings()) {return;}
    manual_capture_preview_pending_ = true;
    imported_frame_pending_ = false;
    imported_frame_active_ = false;
    capture_button_->setEnabled(false);
    calibration_status_->setText(QStringLiteral("正在识别特征点…"));
    current_quality_received_ = false;
    corners_valid_ = false;
    accept_button_->setEnabled(false);
    calibration_quality_panel_->hide();
    transport_.captureForCalibration(currentSettings());
  });
  connect(select_image_button_, &QPushButton::clicked, this, [this]() {
    if (!validateSettings()) {return;}
    QSettings chooser_settings(QStringLiteral("DanmuCamera"), QStringLiteral("Calibration"));
    const QString samples = QDir::current().filePath(QStringLiteral("samples"));
    const QString fallback_directory = QDir(samples).exists() ? samples : QDir::homePath();
    QString initial_directory = chooser_settings.value(
      QStringLiteral("lastImportDirectory"), fallback_directory).toString();
    if (!QDir(initial_directory).exists()) {initial_directory = fallback_directory;}
    const QStringList paths = QFileDialog::getOpenFileNames(
      this, QStringLiteral("批量选择标定图片"), initial_directory,
      QStringLiteral("图像文件 (*.jpg *.jpeg *.png *.bmp *.tif *.tiff "
        "*.JPG *.JPEG *.PNG *.BMP *.TIF *.TIFF);;所有文件 (*)"));
    if (paths.isEmpty()) {return;}
    manual_capture_preview_pending_ = false;
    updateCornerCoverage();
    chooser_settings.setValue(QStringLiteral("lastImportDirectory"),
      QFileInfo(paths.first()).absolutePath());
    batch_import_queue_ = paths;
    batch_total_ = paths.size();
    batch_processed_ = 0;
    batch_import_active_ = true;
    batch_aborted_ = false;
    batch_waiting_save_ = false;
    ui_->back_button->setEnabled(false);
    board_width_spin_->setEnabled(false);
    board_height_spin_->setEnabled(false);
    square_size_spin_->setEnabled(false);
    board_physical_width_spin_->setEnabled(false);
    board_physical_height_spin_->setEnabled(false);
    capture_button_->setEnabled(false);
    select_image_button_->setEnabled(false);
    accept_button_->setEnabled(false);
    retake_button_->setEnabled(false);
    run_button_->setEnabled(false);
    processNextBatchImage();
  });
  connect(retake_button_, &QPushButton::clicked, this, [this]() {resetCaptureView();});
  connect(accept_button_, &QPushButton::clicked, this, [this]() {
    if (!corners_valid_ || !current_quality_received_ ||
      current_quality_.board_size_warning) {return;}
    accept_button_->setEnabled(false);
    transport_.saveFrame();
  });
  connect(run_button_, &QPushButton::clicked, this, [this]() {
    if (!validateSettings()) {return;}
    run_button_->setEnabled(false);
    calibration_requested_ = true;
    calibration_status_->setText(QStringLiteral("正在标定，请稍候…"));
    QStringList session_samples;
    for (int row = 0; row < sample_list_->count(); ++row) {
      auto * item = sample_list_->item(row);
      const QVariant validity = item->data(kSampleValidRole);
      const QString path = item->data(Qt::UserRole).toString();
      const bool quality_ok = !item->data(kQualityWarningRole).toBool() ||
        item->data(kQualityOverrideRole).toBool();
      if ((!validity.isValid() || validity.toBool()) && quality_ok &&
        !quality_cache_.value(path).board_size_warning) {
        session_samples.push_back(path);
      }
    }
    transport_.runCalibration(session_samples, currentSettings());
  });
  connect(sample_list_, &QListWidget::currentRowChanged, this, [this]() {selectSample();});
  sample_list_->setToolTip(QStringLiteral("红色：特征点失败或标定板占比不合格，不能纳入标定；橙色：其他质量告警，可右键确认。Ctrl/Shift 多选，Ctrl+A 全选。"));
  sample_list_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(sample_list_, &QListWidget::customContextMenuRequested, this,
    [this](const QPoint & pos) {
      auto * item = sample_list_->itemAt(pos);
      if (!item || !item->data(kSampleValidRole).toBool() ||
        !item->data(kQualityWarningRole).toBool()) {return;}
      const bool confirmed = item->data(kQualityOverrideRole).toBool();
      QMenu menu(this);
      auto * toggle = menu.addAction(confirmed ? QStringLiteral("撤销质量确认") :
        QStringLiteral("确认质量可用并纳入标定"));
      if (menu.exec(sample_list_->viewport()->mapToGlobal(pos)) != toggle) {return;}
      item->setData(kQualityOverrideRole, !confirmed);
      const QString suffix = QStringLiteral("（已确认）");
      if (confirmed) {item->setText(item->text().remove(suffix));}
      else {item->setText(item->text() + suffix);}
      updateSampleCount();
    });
  connect(delete_button_, &QPushButton::clicked, this, [this]() {
    const QList<QListWidgetItem *> selected = sample_list_->selectedItems();
    if (selected.isEmpty()) {return;}
    if (selected.size() > 1) {
      const auto answer = QMessageBox::question(
        this, QStringLiteral("批量删除图片"),
        QStringLiteral("确定删除选中的 %1 张图片吗？\n"
          "程序管理的样本会同时从磁盘删除；外部导入的问题原图不会被删除。")
        .arg(selected.size()), QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
      if (answer != QMessageBox::Yes) {return;}
    }

    int next_row = sample_list_->count();
    {
      // 批量删除期间不逐条触发图片读取，删完后只刷新一次。
      const QSignalBlocker blocker(sample_list_);
      for (auto * item : selected) {
        const int row = sample_list_->row(item);
        if (row < 0) {continue;}
        next_row = std::min(next_row, row);
        const QVariant managed = item->data(kManagedSampleRole);
        if (!managed.isValid() || managed.toBool()) {
          transport_.deleteSample(item->data(Qt::UserRole).toString());
        }
        quality_cache_.remove(item->data(Qt::UserRole).toString());
        delete sample_list_->takeItem(row);
      }
    }
    updateSampleCount();
    if (calibration_view_ == CalibrationView::Result) {
      left_view_->clearImage();
      global_error_image_path_.clear();
      global_error_button_->hide();
    }
    right_view_->clearImage();
    undistorted_view_->clearImage();
    if (sample_list_->count() > 0) {
      sample_list_->setCurrentRow(std::min(next_row, sample_list_->count() - 1));
    }
  });
  connect(supplement_button_, &QPushButton::clicked, this, [this]() {
    calibration_view_ = CalibrationView::Capture;
    global_error_image_path_.clear();
    global_error_button_->hide();
    supplement_button_->hide();
    capture_button_->show();
    accept_button_->show();
    retake_button_->show();
    resetCaptureView();
  });
  connect(global_error_button_, &QPushButton::clicked, this, [this]() {
    if (global_error_image_path_.isEmpty()) {return;}
    QStringList names;
    for (int i = 0; i < sample_list_->count(); ++i) {
      names << QFileInfo(sample_list_->item(i)->data(Qt::UserRole).toString()).fileName();
    }
    const QString data_path = QFileInfo(global_error_image_path_).dir().filePath(
      QStringLiteral("visualization_data.tsv"));
    ResultVisualizationDialog dialog(data_path, names, this);
    if (!dialog.valid()) {
      QMessageBox::warning(this, QStringLiteral("标定结果可视化"), dialog.error());
      return;
    }
    dialog.exec();
  });
}

void CalibrationWindow::bindTransport()
{
  transport_.onFrame = [this](const QImage & frame) {
    latest_frame_ = frame;
    if (pages_->currentWidget() == home_page_) {
      showImage(home_live_view_, frame);
    }
    if (guided_active_ && guided_live_view_) {
      showImage(guided_live_view_, frame);
    }
    if (pages_->currentWidget() == calibration_page_ &&
      calibration_view_ == CalibrationView::Capture && !guided_active_)
    {
      showImage(left_view_, frame);
    }
  };
  transport_.onStreamFrame = [this](const QString & signal, const QImage & frame) {
    if (!camera_streaming_) {return;}
    if (signal == selected_signal_name_) {return;}
    if (auto * view = stream_views_.value(signal);
      view && stream_panels_.value(signal)->isVisible()) {
      showImage(view, frame);
    }
  };
  transport_.onStreamCloud = [this](const std::vector<QVector3D> & points) {
    if (camera_streaming_ && cloud_panel_->isVisible()) {cloud_view_->setPoints(points);}
  };
  transport_.onTopicDefaults = [this](const QString & image, const QString & cloud) {
    if (!image_topic_edit_->isModified()) {image_topic_edit_->setText(image);}
    if (!cloud_topic_edit_->isModified()) {cloud_topic_edit_->setText(cloud);}
  };
  transport_.onCapturedFrame = [this](const QImage & frame) {
    if (pages_->currentWidget() == home_page_) {
      home_capture_frozen_ = true;
      home_capture_view_->show();
      setEqualImageSizes(home_image_splitter_);
      showImage(home_capture_view_, frame);
      home_save_button_->setEnabled(true);
      home_retake_button_->setEnabled(true);
      return;
    }
    capture_frozen_ = true;
    if (imported_frame_pending_) {
      imported_frame_pending_ = false;
      imported_frame_active_ = false;
    }
    // 标定页左侧始终保持实时视频；冻结原图只送算法，不覆盖左侧。
  };
  transport_.onCornerPreview = [this](const QImage & frame) {
    if ((pages_->currentWidget() != calibration_page_ ||
      calibration_view_ != CalibrationView::Capture) && !batch_import_active_) {return;}
    if (guided_active_) {
      if (guided_waiting_capture_ && !guided_waiting_save_) {
        current_corner_preview_ = frame;
        guided_preview_received_ = true;
        maybeCompleteGuidedCapture();
      }
      return;
    }
    right_view_->show();
    setEqualImageSizes(image_splitter_);
    current_corner_preview_ = frame;
    showImage(right_view_, frame);
    if (batch_import_active_) {
      batch_preview_received_ = true;
      maybeSaveCurrentBatchImage();
    }
  };
  transport_.onImageQuality = [this](
    const QString & path, const ImageQualityMetrics & quality) {
    if (path.isEmpty()) {
      if (guided_active_) {
        if (guided_waiting_capture_ && !guided_waiting_save_) {
          current_quality_ = quality;
          guided_quality_received_ = true;
          maybeCompleteGuidedCapture();
        }
        return;
      }
      if (pages_->currentWidget() != calibration_page_ && !batch_import_active_) {return;}
      current_quality_ = quality;
      current_quality_received_ = true;
      updateCornerCoverage();
      if (batch_import_active_) {
        maybeSaveCurrentBatchImage();
      } else {
        calibration_quality_panel_->setMetrics(quality, QStringLiteral("本次拍摄"));
        calibration_quality_panel_->show();
        if (quality.board_size_warning) {
          calibration_status_->setText(QStringLiteral(
            "标定板占比不合格：请调整距离后重拍"));
          calibration_status_->setStyleSheet("color:#ff6573;");
        }
        accept_button_->setEnabled(corners_valid_ && !quality.board_size_warning);
      }
      return;
    }
    quality_cache_.insert(path, quality);
    auto * current = sample_list_->currentItem();
    if (!current || current->data(Qt::UserRole).toString() != path) {
      updateSampleCount();
      return;
    }
    calibration_quality_panel_->setMetrics(quality, QFileInfo(path).fileName());
    calibration_quality_panel_->show();
    const bool warning = quality.blur_warning || quality.exposure_warning ||
      quality.board_size_warning;
    current->setData(kQualityWarningRole, warning);
    if (quality.board_size_warning) {
      current->setData(kSampleValidRole, false);
      if (!current->text().contains(QStringLiteral("标定板太"))) {
        current->setText(QStringLiteral("⚠ ") + current->text() +
          QStringLiteral(" — ") + qualityProblemText(quality));
      }
      current->setForeground(QColor("#ff4f5e"));
    }
    if (warning && current->data(kSampleValidRole).toBool()) {
      if (!current->text().startsWith(QStringLiteral("◆ "))) {
        current->setText(QStringLiteral("◆ ") + current->text() +
          QStringLiteral(" — ") + qualityProblemText(quality));
      }
      if (!quality.board_size_warning) {current->setForeground(QColor("#ffad4a"));}
    }
    updateSampleCount();
  };
  transport_.onCornerStatus = [this](bool valid, const QString & message) {
    if (pages_->currentWidget() != calibration_page_) {return;}
    if (guided_active_) {
      if (guided_waiting_capture_ && !guided_waiting_save_) {
        guided_capture_valid_ = valid;
        guided_corner_message_ = message;
        guided_status_received_ = true;
        maybeCompleteGuidedCapture();
      }
      return;
    }
    corners_valid_ = valid;
    updateCornerCoverage();
    const bool board_bad = current_quality_received_ && current_quality_.board_size_warning;
    calibration_status_->setText(board_bad ?
      QStringLiteral("标定板占比不合格：%1；请调整距离后重拍").arg(qualityProblemText(current_quality_)) :
      message);
    calibration_status_->setStyleSheet(valid && !board_bad ? "color:#4ddd8b;" :
      "color:#ff6573;");
    if (batch_import_active_) {
      batch_current_valid_ = valid;
      batch_current_message_ = message;
      batch_corner_status_received_ = true;
      accept_button_->setEnabled(false);
      retake_button_->setEnabled(false);
      maybeSaveCurrentBatchImage();
      return;
    }
    accept_button_->setEnabled(valid && current_quality_received_ &&
      !current_quality_.board_size_warning);
    retake_button_->setEnabled(true);
    if (!valid) {appendLog(QStringLiteral("识别失败：") + message);}
  };
  transport_.onViewedOriginal = [this](const QString & path, const QImage & image) {
    auto * item = sample_list_->currentItem();
    if (calibration_view_ == CalibrationView::Result && item &&
      show_original_checkbox_->isChecked() &&
      item->data(Qt::UserRole).toString() == path)
    {
      showImage(left_view_, image);
    }
  };
  transport_.onViewedError = [this](const QString & path, const QImage & image) {
    auto * item = sample_list_->currentItem();
    if (calibration_view_ != CalibrationView::Result || !item ||
      !show_error_checkbox_->isChecked() ||
      item->data(Qt::UserRole + 1).toString() != path) {return;}
    showImage(right_view_, image);
  };
  transport_.onViewedUndistorted = [this](const QString & path, const QImage & image) {
    auto * item = sample_list_->currentItem();
    if (calibration_view_ == CalibrationView::Result && item &&
      show_undistorted_checkbox_->isChecked() &&
      item->data(Qt::UserRole + 8).toString() == path)
    {
      showImage(undistorted_view_, image);
    }
  };
  transport_.onDevices = [this](const std::vector<UiCameraDevice> & devices) {
    if (camera_connected_) {
      const bool still_present = std::any_of(
        devices.cbegin(), devices.cend(), [this](const UiCameraDevice & device) {
          return device.path == connected_device_id_;
        });
      if (!still_present) {
        handleCameraDisconnected(QStringLiteral(
          "设备枚举中已找不到当前相机，已恢复为未连接状态"));
      }
      return;
    }
    camera_combo_->clear();
    available_devices_ = devices;
    QHash<QString, int> device_rows;
    for (const auto & device : devices) {
      const QString key = device.path.left(device.path.lastIndexOf(':'));
      int row = device_rows.value(key, -1);
      if (row < 0) {
        camera_combo_->addItem(QStringLiteral("[%1] %2")
          .arg(device.backend.toUpper(), device.name), device.path);
        row = camera_combo_->count() - 1;
        device_rows.insert(key, row);
      }
      if (device.signal == QStringLiteral("rgb") ||
        !camera_combo_->itemData(row, Qt::UserRole + 2).isValid()) {
        camera_combo_->setItemData(row, device.path);
        camera_combo_->setItemData(row, device.calibratable, Qt::UserRole + 1);
        camera_combo_->setItemData(row, device.signal, Qt::UserRole + 2);
        camera_combo_->setItemData(row, device.backend, Qt::UserRole + 3);
      }
    }
    if (!devices.empty()) {
      selected_signal_calibratable_ = camera_combo_->currentData(Qt::UserRole + 1).toBool();
      selected_signal_name_ = camera_combo_->currentData(Qt::UserRole + 2).toString();
      selected_backend_name_ = camera_combo_->currentData(Qt::UserRole + 3).toString();
      updateTopicInputsVisibility();
      disable_emitter_checkbox_->setEnabled(
        selected_backend_name_ == QStringLiteral("orbbec") ||
        selected_backend_name_ == QStringLiteral("realsense"));
    }
    connect_button_->setEnabled(!devices.empty());
    updateTopicInputsVisibility();
    if (devices.empty()) {
      selected_signal_calibratable_ = false;
      selected_signal_name_.clear();
      selected_backend_name_.clear();
      disable_emitter_checkbox_->setEnabled(false);
      signal_notice_->setText(QStringLiteral(
        "未发现设备。请检查 USB、udev 权限，以及 Orbbec/RealSense SDK 安装状态。"));
    }
    updateCalibrationAvailability();
    rebuildStreamMenu();
    refresh_button_->setEnabled(true);
  };
  transport_.onConnectionChanged = [this](bool connected, const QString & message) {
    if (!connected && guided_active_) {stopGuidedCapture();}
    if (!connected) {
      cloud_view_->setPoints({});
      stream_reconfigure_pending_ = false;
      stream_restart_pending_ = false;
    }
    camera_operation_pending_ = false;
    camera_connected_ = connected;
    if (!connected) {connected_device_id_.clear();}
    if (!connected) {camera_streaming_ = false;}
    connection_status_->setText(connected ? QStringLiteral("● 已连接（未启动）") :
      QStringLiteral("● 未连接"));
    connection_status_->setStyleSheet(connected ? "color:#ffc857;" : "color:#ff6573;");
    connect_button_->setEnabled(!connected);
    disconnect_button_->setEnabled(connected);
    start_stop_button_->setEnabled(connected);
    start_stop_button_->setText(QStringLiteral("启动相机"));
    calibration_start_stop_button_->setEnabled(connected);
    calibration_start_stop_button_->setText(QStringLiteral("启动相机"));
    calibration_connection_status_->setText(connection_status_->text());
    calibration_connection_status_->setStyleSheet(connection_status_->styleSheet());
    camera_combo_->setEnabled(!connected);
    image_topic_edit_->setEnabled(!connected);
    cloud_topic_edit_->setEnabled(!connected);
    stream_button_->setEnabled(connected || camera_combo_->count() > 0);
    resolution_combo_->setEnabled(!connected &&
      selected_backend_name_ != QStringLiteral("ros-topic"));
    disable_emitter_checkbox_->setEnabled(
      selected_backend_name_ == QStringLiteral("orbbec") ||
      selected_backend_name_ == QStringLiteral("realsense"));
    if (!connected) {emitter_change_pending_ = false;}
    confirmed_emitter_disabled_ = disable_emitter_checkbox_->isChecked();
    disable_emitter_checkbox_->setText(QStringLiteral("关闭红外投射器"));
    refresh_button_->setEnabled(!connected);
    home_capture_button_->setEnabled(false);
    updateCalibrationAvailability();
    appendLog(message);
    resolution_notice_->setText(QStringLiteral(
      "⚠ 当前标定分辨率：%1 × %2。连接后无需启动相机即可批量导图标定。")
      .arg(active_width_).arg(active_height_));
    if (!connected) {
      for (auto * view : stream_views_) {
        view->clearImage(QStringLiteral("相机已断开"));
      }
      for (auto * panel : stream_panels_) {
        panel->hide();
      }
      home_capture_frozen_ = false;
      home_save_pending_ = false;
      home_save_button_->setEnabled(false);
      home_retake_button_->setEnabled(false);
      latest_frame_ = QImage();
      home_live_view_->clearImage(QStringLiteral("相机已断开"));
      home_capture_view_->clearImage();
      home_capture_view_->hide();
      setEqualImageSizes(home_image_splitter_);
      left_view_->clearImage(QStringLiteral("相机已断开"));
      right_view_->clearImage();
      right_view_->show();
      right_view_->clearImage(QStringLiteral("特征点识别结果"));
      capture_button_->setEnabled(false);
    } else {
      home_live_view_->clearImage(QStringLiteral("已连接，点击“启动相机”后显示视频"));
      left_view_->clearImage(QStringLiteral("相机未启动；可直接批量导入图片"));
      capture_button_->setEnabled(false);
    }
    if (connected && stream_restart_pending_) {
      stream_restart_pending_ = false;
      camera_operation_pending_ = true;
      start_stop_button_->setEnabled(false);
      calibration_start_stop_button_->setEnabled(false);
      stream_button_->setEnabled(false);
      transport_.startCamera();
    }
  };
  transport_.onStreamingChanged = [this](bool streaming, const QString & message) {
    camera_operation_pending_ = false;
    camera_streaming_ = streaming;
    start_stop_button_->setEnabled(camera_connected_);
    calibration_start_stop_button_->setEnabled(camera_connected_);
    disconnect_button_->setEnabled(camera_connected_);
    start_stop_button_->setText(streaming ? QStringLiteral("关闭相机") : QStringLiteral("启动相机"));
    calibration_start_stop_button_->setText(start_stop_button_->text());
    connection_status_->setText(streaming ? QStringLiteral("● 已启动") :
      (camera_connected_ ? QStringLiteral("● 已连接（未启动）") : QStringLiteral("● 未连接")));
    connection_status_->setStyleSheet(streaming ? "color:#4ddd8b;" :
      (camera_connected_ ? "color:#ffc857;" : "color:#ff6573;"));
    calibration_connection_status_->setText(connection_status_->text());
    calibration_connection_status_->setStyleSheet(connection_status_->styleSheet());
    home_capture_button_->setEnabled(streaming && selected_signal_calibratable_);
    capture_button_->setEnabled(
      streaming && selected_signal_calibratable_ && !batch_import_active_);
    if (guided_start_pending_) {
      guided_start_pending_ = false;
      if (streaming) {beginGuidedDetection();}
      else {
        stopGuidedCapture();
        QMessageBox::warning(this, QStringLiteral("自动标定"), message);
      }
    } else if (guided_active_ && !streaming) {
      stopGuidedCapture();
    }
    // 启停成功信息由 camera/event 统一记录；服务响应只驱动状态，
    // 避免同一条“已启动/已关闭”在日志中出现两次。
    if (streaming) {
      stream_button_->setEnabled(true);
      if (calibration_view_ == CalibrationView::Capture) {
        left_view_->clearImage(QStringLiteral("等待相机首帧…"));
      }
      const QRegularExpressionMatch match = QRegularExpression(
        QStringLiteral("实际分辨率\\s+(\\d+)x(\\d+)")).match(message);
      if (match.hasMatch()) {
        active_width_ = match.captured(1).toInt();
        active_height_ = match.captured(2).toInt();
      }
      resolution_notice_->setText(QStringLiteral(
        "⚠ 当前标定分辨率：%1 × %2。标定结果只适用于相同分辨率的图像。")
        .arg(active_width_).arg(active_height_));
    } else if (camera_connected_) {
      cloud_view_->setPoints({});
      latest_frame_ = QImage();
      home_live_view_->clearImage(QStringLiteral("相机已关闭；仍保持连接"));
      for (auto * view : stream_views_) {
        view->clearImage(QStringLiteral("相机已关闭"));
      }
      // 结果页左栏显示的是选中样本原图，与实时视频流无关。
      // 关闭相机时只清理拍摄视图，不覆盖已经显示的标定结果。
      if (calibration_view_ == CalibrationView::Capture) {
        left_view_->clearImage(QStringLiteral("相机未启动；可继续批量导入图片"));
      }
    }
    if (!streaming && stream_reconfigure_pending_ && camera_connected_) {
      stream_reconfigure_pending_ = false;
      const QString base_id = camera_combo_->currentData().toString();
      const QString new_id = base_id.left(base_id.lastIndexOf(':') + 1) +
        selected_signal_name_;
      connected_device_id_ = new_id;
      active_streams_ = checkedStreams();
      transport_.connectCamera(new_id, active_width_, active_height_,
        disable_emitter_checkbox_->isChecked(), active_streams_,
        image_topic_edit_->text().trimmed(), cloud_topic_edit_->text().trimmed());
    }
  };
  transport_.onCameraCommandFailed = [this](const QString & message) {
    camera_operation_pending_ = false;
    stream_reconfigure_pending_ = false;
    stream_restart_pending_ = false;
    stream_button_->setEnabled(true);
    start_stop_button_->setEnabled(camera_connected_);
    calibration_start_stop_button_->setEnabled(camera_connected_);
    disconnect_button_->setEnabled(camera_connected_);
    connect_button_->setEnabled(!camera_connected_ && camera_combo_->count() > 0);
    start_stop_button_->setText(
      camera_streaming_ ? QStringLiteral("关闭相机") : QStringLiteral("启动相机"));
    calibration_start_stop_button_->setText(start_stop_button_->text());
    if (guided_start_pending_) {stopGuidedCapture();}
    guided_start_pending_ = false;
    appendLog(QStringLiteral("相机操作失败：") + message);
    QMessageBox::warning(this, QStringLiteral("相机操作失败"), message);
  };
  transport_.onIrEmitterChanged = [this](bool success, const QString & message) {
    emitter_change_pending_ = false;
    if (success) {
      confirmed_emitter_disabled_ = disable_emitter_checkbox_->isChecked();
    } else {
      const QSignalBlocker blocker(disable_emitter_checkbox_);
      disable_emitter_checkbox_->setChecked(confirmed_emitter_disabled_);
    }
    disable_emitter_checkbox_->setText(QStringLiteral("关闭红外投射器"));
    disable_emitter_checkbox_->setEnabled(camera_connected_ &&
      (selected_backend_name_ == QStringLiteral("orbbec") ||
      selected_backend_name_ == QStringLiteral("realsense")));
    updateCalibrationAvailability();
    if (!success) {appendLog(message);}
  };
  transport_.onCameraState = [this](const QString & state, const QString & detail) {
    if (state != QStringLiteral("disconnected")) {return;}
    handleCameraDisconnected(detail);
  };
  transport_.onCaptureFinished = [this](bool success, const QString & message) {
    appendLog(message);
    if (guided_active_ && guided_waiting_capture_) {
      if (!success) {showGuidedProblem(message);}
      return;
    }
    if (!success) {
      manual_capture_preview_pending_ = false;
      updateCornerCoverage();
      if (pages_->currentWidget() == home_page_) {
        home_capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
        updateCalibrationAvailability();
      } else {
        capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
      }
    }
  };
  transport_.onSaveFinished = [this](bool success, const QString & path) {
    if (home_save_pending_) {
      home_save_pending_ = false;
      appendLog(success ? QStringLiteral("首页图片已保存：") + path : path);
      home_save_button_->setEnabled(!success && home_capture_frozen_);
      if (success) {
        home_capture_frozen_ = false;
        home_retake_button_->setEnabled(false);
        home_capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
        updateCalibrationAvailability();
        home_capture_view_->clearImage();
        home_capture_view_->hide();
        setEqualImageSizes(home_image_splitter_);
      }
      return;
    }
    if (!success) {
      appendLog(path);
      accept_button_->setEnabled(corners_valid_ && !current_quality_.board_size_warning);
      return;
    }
    manual_capture_preview_pending_ = false;
    const bool quality_warning = current_quality_.blur_warning ||
      current_quality_.exposure_warning || current_quality_.board_size_warning;
    auto * item = new QListWidgetItem(
      (quality_warning ? QStringLiteral("◆ ") : QString()) + QFileInfo(path).fileName() +
      (quality_warning ? QStringLiteral(" — ") + qualityProblemText(current_quality_) : QString()),
      sample_list_);
    item->setData(Qt::UserRole, path);
    item->setData(kSampleValidRole, !current_quality_.board_size_warning);
    item->setData(kManagedSampleRole, true);
    item->setData(kCornerPreviewRole, current_corner_preview_);
    item->setData(kQualityWarningRole, quality_warning);
    quality_cache_.insert(path, current_quality_);
    if (current_quality_.board_size_warning) {item->setForeground(QColor("#ff4f5e"));}
    else if (quality_warning) {item->setForeground(QColor("#ffad4a"));}
    appendLog(QStringLiteral("已接受并保存原图：") + path);
    updateSampleCount();
    resetCaptureView();
  };
  transport_.onBatchSaveFinished = [this](
    const QString & source, bool success, const QString & path) {
    if (source.startsWith(QStringLiteral("guided:"))) {
      if (guided_active_ && guided_waiting_save_ &&
        source == QStringLiteral("guided:%1").arg(guided_step_)) {
        finishGuidedSave(success, path);
      }
      return;
    }
    if (!batch_import_active_ || !batch_waiting_save_ ||
      source != batch_current_source_) {return;}
    {
      batch_waiting_save_ = false;
      if (!success) {
        addBatchProblem(batch_current_source_, path);
        return;
      }
      const bool quality_warning = current_quality_.blur_warning ||
        current_quality_.exposure_warning || current_quality_.board_size_warning;
      const QString state = !batch_current_valid_ ? QStringLiteral("⚠ ") :
        quality_warning ? QStringLiteral("◆ ") : QStringLiteral("✓ ");
      const QString problem = !batch_current_valid_ ? QStringLiteral("特征点识别失败") :
        quality_warning ? qualityProblemText(current_quality_) : QString();
      auto * item = new QListWidgetItem(
        state + QFileInfo(batch_current_source_).fileName() +
        (problem.isEmpty() ? QString() : QStringLiteral(" — ") + problem), sample_list_);
      item->setData(Qt::UserRole, path);
      item->setData(kSampleValidRole,
        batch_current_valid_ && !current_quality_.board_size_warning);
      item->setData(kManagedSampleRole, true);
      item->setData(kCornerPreviewRole, current_corner_preview_);
      item->setData(kQualityWarningRole, quality_warning);
      quality_cache_.insert(path, current_quality_);
      if (!batch_current_valid_ || current_quality_.board_size_warning) {
        item->setForeground(QColor("#ff4f5e"));
      }
      else if (quality_warning) {item->setForeground(QColor("#ffad4a"));}
      ++batch_processed_;
      appendLog(QStringLiteral("批量选图 %1/%2：%3 — %4")
        .arg(batch_processed_).arg(batch_total_)
        .arg(QFileInfo(batch_current_source_).fileName(), batch_current_message_));
      updateSampleCount();
      QTimer::singleShot(0, this, [this]() {processNextBatchImage();});
      return;
    }
  };
  transport_.onGuideDetection = [this](const GuideDetection & detection) {
    handleGuideDetection(detection);
  };
  transport_.onGuideStart = [this](bool success, const QString & message) {
    if (!guided_active_) {return;}
    guided_backend_ready_ = success;
    if (!success) {
      stopGuidedCapture();
      QMessageBox::warning(this, QStringLiteral("自动标定"), message);
    }
  };
  transport_.onStatus = [this](const QString & status) {
    if (status.startsWith("failed:")) {
      calibration_requested_ = false;
      guided_solver_pending_ = false;
      calibration_status_->setText(status);
      updateSampleCount();
    }
  };
  transport_.onResult = [this](const QString & result) {appendLog(result);};
  transport_.onReport = [this](
    const std::vector<CalibrationReportItem> & report, const QString & result_file,
    const QString & global_error_image, const QString & camera_matrix_file,
    const QString & distortion_matrix_file) {
    // transient_local 可能在 UI 单独重启时送来上次报告；只有本窗口主动运行后才接纳。
    if (!calibration_requested_ || report.empty()) {return;}
    calibration_requested_ = false;
    const bool guided_report = guided_solver_pending_;
    guided_solver_pending_ = false;
    calibration_view_ = CalibrationView::Result;
    global_error_image_path_ = global_error_image;
    global_error_button_->setVisible(true);
    global_error_button_->setEnabled(!global_error_image_path_.isEmpty());
    sample_list_->clear();
    for (const auto & entry : report) {
      auto * item = new QListWidgetItem(
        QStringLiteral("%1   %2 px").arg(QFileInfo(entry.original_path).fileName())
        .arg(entry.mean_error, 0, 'f', 3), sample_list_);
      item->setData(Qt::UserRole, entry.original_path);
      item->setData(Qt::UserRole + 1, entry.error_image_path);
      item->setData(Qt::UserRole + 8, entry.undistorted_image_path);
      item->setData(Qt::UserRole + 2, entry.mean_error);
      item->setData(kSampleValidRole, true);
      item->setData(kQualityWarningRole, false);
      item->setData(kManagedSampleRole, true);
      if (entry.mean_error > 0.5) {item->setForeground(QColor("#ff4f5e"));}
    }
    calibration_status_->setText(QStringLiteral("标定完成：结果图滚轮缩放、拖动平移、双击还原"));
    calibration_status_->setStyleSheet("color:#4ddd8b;");
    capture_button_->hide();
    guided_button_->hide();
    select_image_button_->hide();
    accept_button_->hide();
    retake_button_->hide();
    supplement_button_->show();
    delete_button_->setEnabled(true);
    result_controls_->show();
    {
      const QSignalBlocker original_blocker(show_original_checkbox_);
      const QSignalBlocker error_blocker(show_error_checkbox_);
      const QSignalBlocker undistorted_blocker(show_undistorted_checkbox_);
      show_original_checkbox_->setChecked(false);
      show_error_checkbox_->setChecked(true);
      show_undistorted_checkbox_->setChecked(false);
    }
    left_view_->clearImage(QStringLiteral("原图"));
    right_view_->clearImage(QStringLiteral("特征点重投影误差图"));
    undistorted_view_->clearImage(QStringLiteral("去畸变图"));
    updateResultViews();
    if (sample_list_->count() > 0) {sample_list_->setCurrentRow(0);}
    updateSampleCount();
    if (guided_report) {
      QStringList high_errors;
      QString worst_path;
      double worst_error = 0.5;
      for (const auto & entry : report) {
        if (entry.mean_error > 0.5) {
          high_errors << QStringLiteral("%1：%2 px")
            .arg(QFileInfo(entry.original_path).fileName())
            .arg(entry.mean_error, 0, 'f', 3);
          if (entry.mean_error > worst_error) {
            worst_error = entry.mean_error;
            worst_path = entry.original_path;
          }
        }
      }
      if (!high_errors.isEmpty()) {
        QDialog dialog(this);
        dialog.setWindowTitle(QStringLiteral("自动标定质量提醒 · 建议重拍"));
        auto * layout = new QVBoxLayout(&dialog);
        auto * summary = new QLabel(
          QStringLiteral("以下图像的平均重投影误差超过 0.5 px：\n%1\n"
            "建议查看右侧误差图，并重拍问题图像。")
            .arg(high_errors.join(QStringLiteral("\n"))), &dialog);
        summary->setWordWrap(true);
        layout->addWidget(summary);
        auto * preview = new ImageLabel(&dialog);
        preview->setZoomEnabled(true);
        preview->setToolTip(QStringLiteral("滚轮缩放、拖动平移、双击还原"));
        preview->setMinimumSize(600, 280);
        preview->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        const QImage image = guided_preview_cache_.value(worst_path);
        if (!image.isNull()) {preview->setImage(image);}
        else {preview->clearImage(QStringLiteral("特征点识别图不可用"));}
        layout->addWidget(preview, 1);
        auto * quality = new ImageQualityPanel(&dialog);
        if (quality_cache_.contains(worst_path)) {
          quality->setMetrics(quality_cache_.value(worst_path),
            QFileInfo(worst_path).fileName());
        } else {
          quality->clear(QStringLiteral("图像质量信息不可用"));
        }
        layout->addWidget(quality);
        auto * close = new QPushButton(QStringLiteral("查看标定结果"), &dialog);
        layout->addWidget(close);
        connect(close, &QPushButton::clicked, &dialog, &QDialog::accept);
        dialog.resize(760, 730);
        dialog.exec();
      }
    }
    QMessageBox::information(this, QStringLiteral("标定完成"),
      QStringLiteral("标定已完成，共处理 %1 张图片。\n\n"
        "完整结果：\n%2\n\n内参矩阵：\n%3\n\n畸变矩阵：\n%4\n\n"
        "右侧可选择样本；结果视图可独立开关原图、特征点误差图和去畸变图。")
      .arg(report.size()).arg(result_file, camera_matrix_file, distortion_matrix_file));
  };
  transport_.onCalibrationAccepted = [this](bool success, const QString & message) {
    appendLog(message);
    if (!success) {
      calibration_requested_ = false;
      updateSampleCount();
    }
  };
  transport_.onImportStatus = [this](bool loaded, const QString & path, const QString & detail) {
    if (!batch_import_active_ || path != batch_current_source_ || loaded) {return;}
    imported_frame_pending_ = false;
    imported_frame_active_ = false;
    addBatchProblem(path, detail);
  };
  transport_.onLog = [this](const QString & text) {appendLog(text);};
}

void CalibrationWindow::showImage(ImageLabel * label, const QImage & image)
{
  if (image.isNull()) {return;}
  label->setImage(image);
}

void CalibrationWindow::setEqualImageSizes(QSplitter * splitter)
{
  // show() 后等下一轮布局再设尺寸，避免隐藏控件刚恢复时仍沿用旧宽度。
  QTimer::singleShot(0, splitter, [splitter]() {
    int visible = 0;
    for (int i = 0; i < splitter->count(); ++i) {
      if (splitter->widget(i)->isVisible()) {++visible;}
    }
    if (visible == 0) {return;}
    QList<int> sizes;
    const int each = std::max(1, splitter->width() / visible);
    for (int i = 0; i < splitter->count(); ++i) {
      sizes.push_back(splitter->widget(i)->isVisible() ? each : 0);
    }
    splitter->setSizes(sizes);
  });
}

void CalibrationWindow::startNewCalibrationSession()
{
  if (guided_active_) {stopGuidedCapture();}
  sample_list_->clear();
  calibration_quality_panel_->hide();
  quality_cache_.clear();
  global_error_image_path_.clear();
  global_error_button_->hide();
  calibration_requested_ = false;
  batch_import_active_ = false;
  batch_aborted_ = false;
  batch_waiting_save_ = false;
  batch_import_queue_.clear();
  calibration_view_ = CalibrationView::Capture;
  resetCaptureView();
  appendLog(QStringLiteral("已开始新的标定会话；旧图片仍保留在磁盘，但不会参与本次标定"));
}

void CalibrationWindow::processNextBatchImage()
{
  if (!batch_import_active_) {return;}
  if (batch_import_queue_.isEmpty()) {
    batch_import_active_ = false;
    imported_frame_pending_ = false;
    imported_frame_active_ = false;
    capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
    select_image_button_->setEnabled(true);
    ui_->back_button->setEnabled(true);
    board_width_spin_->setEnabled(true);
    board_height_spin_->setEnabled(true);
    square_size_spin_->setEnabled(true);
    board_physical_width_spin_->setEnabled(true);
    board_physical_height_spin_->setEnabled(true);
    retake_button_->setEnabled(false);
    calibration_status_->setText(batch_aborted_ ?
      QStringLiteral("批量导入已停止：处理 %1/%2 张；超时项已标红，可检查日志后重试。")
        .arg(batch_processed_).arg(batch_total_) :
      QStringLiteral("批量选图完成：共 %1 张；红色为特征点问题，橙色为质量告警，请检查后删除。")
        .arg(batch_total_));
    calibration_status_->setStyleSheet(batch_aborted_ ? "color:#ff6573;" : "color:#ffc857;");
    updateSampleCount();
    return;
  }
  batch_current_source_ = batch_import_queue_.takeFirst();
  const quint64 request_id = ++batch_request_id_;
  QTimer::singleShot(30000, this, [this, request_id]() {
    if (!batch_import_active_ || batch_request_id_ != request_id) {return;}
    const QString stalled = batch_current_source_;
    batch_waiting_save_ = false;
    batch_aborted_ = true;
    batch_import_queue_.clear();
    addBatchProblem(stalled, QStringLiteral("处理超时：已停止本次批量导入，请检查后端状态后重试"));
  });
  batch_current_message_.clear();
  batch_current_valid_ = false;
  batch_corner_status_received_ = false;
  batch_preview_received_ = false;
  current_quality_received_ = false;
  current_quality_ = ImageQualityMetrics{};
  calibration_quality_panel_->hide();
  current_corner_preview_ = QImage();
  imported_frame_pending_ = true;
  imported_frame_active_ = false;
  right_view_->clearImage(QStringLiteral("正在识别特征点…"));
  calibration_status_->setText(QStringLiteral("批量处理中 %1/%2：%3")
    .arg(batch_processed_ + 1).arg(batch_total_)
    .arg(QFileInfo(batch_current_source_).fileName()));
  transport_.importForCalibration(batch_current_source_, currentSettings());
}

void CalibrationWindow::maybeSaveCurrentBatchImage()
{
  if (!batch_import_active_ || batch_waiting_save_ ||
    !batch_corner_status_received_ || !batch_preview_received_ ||
    !current_quality_received_)
  {
    return;
  }
  batch_waiting_save_ = true;
  transport_.saveFrame(batch_current_source_);
}

void CalibrationWindow::addBatchProblem(const QString & source_path, const QString & reason)
{
  auto * item = new QListWidgetItem(
    QStringLiteral("⚠ %1 — %2").arg(QFileInfo(source_path).fileName(), reason), sample_list_);
  item->setData(Qt::UserRole, source_path);
  item->setData(kSampleValidRole, false);
  item->setData(kQualityWarningRole, false);
  item->setData(kManagedSampleRole, false);
  item->setForeground(QColor("#ff4f5e"));
  ++batch_processed_;
  appendLog(QStringLiteral("批量选图 %1/%2：%3 — %4")
    .arg(batch_processed_).arg(batch_total_).arg(QFileInfo(source_path).fileName(), reason));
  updateSampleCount();
  QTimer::singleShot(0, this, [this]() {processNextBatchImage();});
}

void CalibrationWindow::resetCaptureView()
{
  calibration_view_ = CalibrationView::Capture;
  left_view_->setZoomEnabled(false);
  right_view_->setZoomEnabled(false);
  undistorted_view_->setZoomEnabled(false);
  guided_button_->hide();  // 自动引导拍摄暂时停用。
  capture_button_->show();
  select_image_button_->show();
  accept_button_->show();
  retake_button_->show();
  supplement_button_->hide();
  capture_frozen_ = false;
  manual_capture_preview_pending_ = false;
  result_controls_->hide();
  undistorted_view_->hide();
  result_empty_view_->hide();
  left_view_->setMinimumSize(320, 240);
  right_view_->setMinimumSize(320, 240);
  left_view_->show();
  right_view_->show();
  calibration_quality_panel_->hide();
  current_quality_received_ = false;
  current_quality_ = ImageQualityMetrics{};
  current_corner_preview_ = QImage();
  imported_frame_pending_ = false;
  imported_frame_active_ = false;
  corners_valid_ = false;
  right_view_->show();
  right_view_->clearImage(QStringLiteral("特征点识别结果"));
  setEqualImageSizes(image_splitter_);
  accept_button_->setEnabled(false);
  retake_button_->setEnabled(false);
  capture_button_->setEnabled(camera_streaming_ && selected_signal_calibratable_);
  select_image_button_->setEnabled(true);
  calibration_status_->setStyleSheet("");
  calibration_status_->setText(camera_streaming_ ? QStringLiteral("调整标定板姿态后拍摄") :
    QStringLiteral("相机未启动：可批量导入图片，或返回首页启动相机后拍摄"));
  // 进入拍摄态后，左栏的所有权必须立即交还给实时视频。
  // 即使尚未收到新帧，也不能继续留着结果态的选中原图。
  if (!latest_frame_.isNull() && camera_streaming_) {
    showImage(left_view_, latest_frame_);
  } else if (camera_streaming_) {
    left_view_->clearImage(QStringLiteral("等待相机首帧…"));
    calibration_quality_panel_->clear();
  } else {
    left_view_->clearImage(QStringLiteral("相机未启动；可继续批量导入图片"));
    calibration_quality_panel_->clear();
  }
  updateSampleCount();
}

void CalibrationWindow::updateSampleCount()
{
  const int count = sample_list_->count();
  int valid_count = 0;
  for (int row = 0; row < count; ++row) {
    auto * item = sample_list_->item(row);
    const QVariant validity = item->data(kSampleValidRole);
    const bool corners_ok = !validity.isValid() || validity.toBool();
    const bool quality_ok = !item->data(kQualityWarningRole).toBool() ||
      item->data(kQualityOverrideRole).toBool();
    const auto quality = quality_cache_.value(item->data(Qt::UserRole).toString());
    if (corners_ok && quality_ok && !quality.board_size_warning) {++valid_count;}
  }
  const int problem_count = count - valid_count;
  sample_count_label_->setText(QStringLiteral("有效样本 %1 / 13（总计 %2，问题 %3）")
    .arg(valid_count).arg(count).arg(problem_count));
  run_button_->setEnabled(
    valid_count >= 13 && problem_count == 0 && !batch_import_active_ &&
    !calibration_requested_);
  delete_button_->setEnabled(count > 0);
  updateCornerCoverage();
}

void CalibrationWindow::updateCornerCoverage()
{
  const auto thresholds = transport_.cornerCoverageThresholds();
  std::array<int, 9> cells{};
  int total = 0;
  for (int row = 0; row < sample_list_->count(); ++row) {
    auto * item = sample_list_->item(row);
    if (!item->data(kSampleValidRole).toBool() ||
      (item->data(kQualityWarningRole).toBool() &&
      !item->data(kQualityOverrideRole).toBool())) {continue;}
    const QString path = item->data(Qt::UserRole).toString();
    if (!quality_cache_.contains(path)) {continue;}
    const auto & quality = quality_cache_[path];
    if (quality.board_size_warning) {continue;}
    for (int i = 0; i < 9; ++i) {
      cells[i] += quality.corner_grid_counts[i];
      total += quality.corner_grid_counts[i];
    }
  }
  const bool preview = manual_capture_preview_pending_ && capture_frozen_ &&
    current_quality_received_ && corners_valid_ && !current_quality_.board_size_warning;
  if (preview) {
    for (int i = 0; i < 9; ++i) {
      cells[i] += current_quality_.corner_grid_counts[i];
      total += current_quality_.corner_grid_counts[i];
    }
  }
  if (total == 0) {
    corner_coverage_label_->setText(QStringLiteral(
      "<b>角点像素面覆盖</b><br>暂无有效角点；拍摄后显示九宫格统计。"));
    return;
  }
  const std::array<QString, 9> names = {
    QStringLiteral("左上"), QStringLiteral("上中"), QStringLiteral("右上"),
    QStringLiteral("左中"), QStringLiteral("中央"), QStringLiteral("右中"),
    QStringLiteral("左下"), QStringLiteral("下中"), QStringLiteral("右下")};
  QStringList missing;
  QStringList crowded;
  QString html = QStringLiteral("<b>角点像素面覆盖</b> · 共 %1 点%2"
    "<br>补拍 &lt;%3%，集中 &gt;%4%"
    "<table width='100%' cellspacing='3'>")
      .arg(total)
      .arg(preview ? QStringLiteral("（含本次未保存拍摄）") : QString())
      .arg(thresholds.min_ratio * 100.0, 0, 'f', 1)
      .arg(thresholds.max_ratio * 100.0, 0, 'f', 1);
  for (int row = 0; row < 3; ++row) {
    html += QStringLiteral("<tr>");
    for (int column = 0; column < 3; ++column) {
      const int index = row * 3 + column;
      const double percent = 100.0 * cells[index] / total;
      if (percent < thresholds.min_ratio * 100.0) {missing << names[index];}
      if (percent > thresholds.max_ratio * 100.0) {crowded << names[index];}
      const QString color = percent < thresholds.min_ratio * 100.0 ?
        QStringLiteral("#ff6573") :
        percent > thresholds.max_ratio * 100.0 ?
          QStringLiteral("#ffad4a") : QStringLiteral("#4ddd8b");
      html += QStringLiteral("<td align='center' style='color:%1'>%2<br>%3 点 / %4%</td>")
        .arg(color, names[index]).arg(cells[index]).arg(percent, 0, 'f', 1);
    }
    html += QStringLiteral("</tr>");
  }
  html += QStringLiteral("</table>");
  if (!missing.isEmpty()) {
    html += QStringLiteral("<br>缺少：%1；请把标定板移到这些画面区域。")
      .arg(missing.join(QStringLiteral("、")));
  }
  if (!crowded.isEmpty()) {
    html += QStringLiteral("<br>集中：%1；减少在这些区域重复拍摄。")
      .arg(crowded.join(QStringLiteral("、")));
  }
  if (missing.isEmpty() && crowded.isEmpty()) {
    html += QStringLiteral("<br>九宫格分布较均匀。");
  }
  corner_coverage_label_->setText(html);
}

CalibrationWindow::CalibrationKind CalibrationWindow::selectedCalibrationKind() const
{
  switch (calibration_type_combo_->currentIndex()) {
    case 1: return CalibrationKind::Stereo;
    case 2: return CalibrationKind::Extrinsic;
    case 3: return CalibrationKind::HandEye;
    default: return CalibrationKind::Monocular;
  }
}

bool CalibrationWindow::calibrationKindAvailable() const
{
  switch (selectedCalibrationKind()) {
    case CalibrationKind::Monocular: return true;
    case CalibrationKind::Stereo:
    case CalibrationKind::Extrinsic:
    case CalibrationKind::HandEye: return false;
  }
  return false;
}

bool CalibrationWindow::validateSettings()
{
  const auto settings = currentSettings();
  if ((settings.guide_use_initial || !settings.automatic_initial) &&
    (!(settings.fx > 0) || !(settings.fy > 0))) {
    QMessageBox::warning(this, QStringLiteral("初始内参无效"),
      QStringLiteral("使用输入内参时，请填写正确的 fx 和 fy。"));
    return false;
  }
  const bool has_width = settings.board_physical_width > 0.0;
  const bool has_height = settings.board_physical_height > 0.0;
  if (has_width != has_height) {
    QMessageBox::warning(this, QStringLiteral("标定板尺寸无效"),
      QStringLiteral("外形宽度和高度请同时填写，或都设为“自动”。"));
    return false;
  }
  if ((settings.board_type == QStringLiteral("charuco") ||
    settings.board_type == QStringLiteral("aprilgrid")) &&
    settings.marker_length >= settings.spacing) {
    QMessageBox::warning(this, QStringLiteral("标定板参数无效"),
      QStringLiteral("标记边长必须小于方格边长或标签中心间距。"));
    return false;
  }
  return true;
}

CalibrationSettings CalibrationWindow::currentSettings() const
{
  CalibrationSettings settings;
  settings.board_type = board_type_combo_->currentData().toString();
  settings.columns = board_width_spin_->value();
  settings.rows = board_height_spin_->value();
  settings.spacing = square_size_spin_->value();
  settings.board_physical_width = board_physical_width_spin_->value();
  settings.board_physical_height = board_physical_height_spin_->value();
  settings.marker_length = marker_size_spin_->value();
  settings.dictionary_id = dictionary_combo_->currentData().toInt();
  settings.camera_model = camera_model_combo_->currentData().toString();
  settings.distortion_model = distortion_model_combo_->currentData().toString();
  settings.automatic_initial = auto_initial_checkbox_->isChecked();
  settings.guide_use_initial = guide_use_initial_checkbox_->isChecked();
  settings.guide_initial_width = guide_initial_width_spin_->value();
  settings.guide_initial_height = guide_initial_height_spin_->value();
  if (!settings.automatic_initial || settings.guide_use_initial) {
    settings.fx = initial_camera_spins_[0]->value();
    settings.fy = initial_camera_spins_[1]->value();
    settings.cx = initial_camera_spins_[2]->value();
    settings.cy = initial_camera_spins_[3]->value();
    const int coefficients = settings.camera_model == QStringLiteral("fisheye") ?
      (settings.distortion_model == QStringLiteral("fisheye2") ? 2 : 4) :
      settings.distortion_model == QStringLiteral("rational8") ? 8 :
      settings.distortion_model == QStringLiteral("none") ? 0 : 5;
    for (int i = 0; i < coefficients; ++i) {
      settings.initial_distortion[i] = initial_distortion_spins_[i]->value();
    }
  }
  return settings;
}

void CalibrationWindow::updateBoardControls()
{
  const QString type = board_type_combo_->currentData().toString();
  const bool charuco = type == QStringLiteral("charuco");
  const bool aprilgrid = type == QStringLiteral("aprilgrid");
  const bool marker_board = charuco || aprilgrid;
  const bool circles = type.startsWith(QStringLiteral("circles"));
  ui_->boardWidthLabel->setText(charuco ? QStringLiteral("横向方格数") :
    aprilgrid ? QStringLiteral("横向标签数") : circles ? QStringLiteral("横向圆心数") : QStringLiteral("横向内角点"));
  ui_->boardHeightLabel->setText(charuco ? QStringLiteral("纵向方格数") :
    aprilgrid ? QStringLiteral("纵向标签数") : circles ? QStringLiteral("纵向圆心数") : QStringLiteral("纵向内角点"));
  ui_->squareSizeLabel->setText(charuco ? QStringLiteral("方格边长(m)") :
    aprilgrid ? QStringLiteral("标签中心间距(m)") : circles ? QStringLiteral("圆点基准间距(m)") : QStringLiteral("方格边长(m)"));
  board_width_spin_->setMinimum(charuco ? 3 : 2);
  board_height_spin_->setMinimum(charuco ? 3 : 2);
  marker_size_label_->setText(aprilgrid ? QStringLiteral("标签边长(m)") :
    QStringLiteral("标记边长(m)"));
  marker_size_label_->setVisible(marker_board);
  marker_size_spin_->setVisible(marker_board);
  dictionary_label_->setVisible(marker_board);
  dictionary_combo_->setVisible(marker_board);
  if (aprilgrid && dictionary_combo_->currentData().toInt() != 20) {
    dictionary_combo_->setCurrentIndex(dictionary_combo_->findData(20));
  } else if (charuco && dictionary_combo_->currentData().toInt() == 20) {
    dictionary_combo_->setCurrentIndex(0);
  }
}

void CalibrationWindow::updateCameraModelControls()
{
  const bool fisheye = camera_model_combo_->currentData().toString() == QStringLiteral("fisheye");
  const QString selected = distortion_model_combo_->currentData().toString();
  {
    const QSignalBlocker blocker(distortion_model_combo_);
    distortion_model_combo_->clear();
    if (fisheye) {
      distortion_model_combo_->addItem(QStringLiteral("等距 K1–K4（4 参数）"),
        QStringLiteral("fisheye4"));
      distortion_model_combo_->addItem(QStringLiteral("等距 K1–K2（2 参数）"),
        QStringLiteral("fisheye2"));
    } else {
      distortion_model_combo_->addItem(QStringLiteral("Brown 5 参数"),
        QStringLiteral("brown5"));
      distortion_model_combo_->addItem(QStringLiteral("Rational 8 参数"),
        QStringLiteral("rational8"));
      distortion_model_combo_->addItem(QStringLiteral("固定零畸变"),
        QStringLiteral("none"));
    }
    const int index = distortion_model_combo_->findData(selected);
    if (index >= 0) {distortion_model_combo_->setCurrentIndex(index);}
  }
  const QStringList names = fisheye ?
    QStringList{QStringLiteral("k1"), QStringLiteral("k2"), QStringLiteral("k3"),
      QStringLiteral("k4")} :
    QStringList{QStringLiteral("k1"), QStringLiteral("k2"), QStringLiteral("p1"),
      QStringLiteral("p2"), QStringLiteral("k3"), QStringLiteral("k4"),
      QStringLiteral("k5"), QStringLiteral("k6")};
  const QString distortion = distortion_model_combo_->currentData().toString();
  const int visible = fisheye ? (distortion == QStringLiteral("fisheye2") ? 2 : 4) :
    distortion == QStringLiteral("none") ? 0 :
    distortion == QStringLiteral("rational8") ? 8 : 5;
  for (int i = 0; i < 8; ++i) {
    initial_distortion_labels_[i]->setText(i < names.size() ? names[i] : QString());
    initial_distortion_labels_[i]->setVisible(i < visible);
    initial_distortion_spins_[i]->setVisible(i < visible);
  }
}

void CalibrationWindow::updateResultViews()
{
  if (calibration_view_ != CalibrationView::Result) {return;}
  left_view_->setZoomEnabled(true);
  right_view_->setZoomEnabled(true);
  undistorted_view_->setZoomEnabled(true);
  const bool original = show_original_checkbox_->isChecked();
  const bool error = show_error_checkbox_->isChecked();
  const bool undistorted = show_undistorted_checkbox_->isChecked();
  left_view_->setMinimumSize(180, 150);
  right_view_->setMinimumSize(180, 150);
  undistorted_view_->setMinimumSize(180, 150);
  left_view_->setVisible(original);
  right_view_->setVisible(error);
  undistorted_view_->setVisible(undistorted);
  result_empty_view_->setVisible(!original && !error && !undistorted);
  setEqualImageSizes(image_splitter_);
  requestSelectedResultImages();
}

void CalibrationWindow::requestSelectedResultImages()
{
  if (calibration_view_ != CalibrationView::Result) {return;}
  auto * item = sample_list_->currentItem();
  if (!item) {return;}
  transport_.viewImages(
    show_original_checkbox_->isChecked() ? item->data(Qt::UserRole).toString() : QString(),
    show_error_checkbox_->isChecked() ? item->data(Qt::UserRole + 1).toString() : QString(),
    show_undistorted_checkbox_->isChecked() ? item->data(Qt::UserRole + 8).toString() : QString());
}

void CalibrationWindow::updateCalibrationAvailability()
{
  const bool single_signal = checkedStreams().size() == 1;
  const bool enabled = camera_connected_ && single_signal && selected_signal_calibratable_ &&
    !home_capture_frozen_ && !emitter_change_pending_ && calibrationKindAvailable() &&
    (!selected_signal_name_.startsWith(QStringLiteral("ir-")) ||
    confirmed_emitter_disabled_);
  enter_calibration_button_->setEnabled(enabled);
  if (!calibrationKindAvailable()) {
    signal_notice_->setText(QStringLiteral("%1：接口已预留，当前版本尚未提供标定功能。")
      .arg(calibration_type_combo_->currentText()));
    signal_notice_->setStyleSheet("color:#ffc857;");
    return;
  }
  if (!single_signal && checkedStreams().size() > 1) {
    signal_notice_->setText(QStringLiteral("已选择多路信息流；内参标定前请只勾选一条可标定流。"));
    signal_notice_->setStyleSheet("color:#ffc857;");
    return;
  }
  if (selected_signal_name_.isEmpty()) {return;}
  if (selected_signal_name_.startsWith(QStringLiteral("ir-")) &&
    !disable_emitter_checkbox_->isChecked())
  {
    signal_notice_->setText(QStringLiteral(
      "当前信号：%1（红外投射器开启，仅供观察；散斑叠加画面禁止内参标定）")
      .arg(selected_signal_name_));
    signal_notice_->setStyleSheet("color:#ff6573;");
  } else if (selected_signal_calibratable_) {
    signal_notice_->setText(QStringLiteral(
      "当前信号：%1（%2可进行内参标定；结果只对应此成像器与分辨率）")
      .arg(selected_signal_name_, selected_signal_name_.startsWith(QStringLiteral("ir-")) ?
      QStringLiteral("投射器关闭后的纯灰度，") : QString()));
    signal_notice_->setStyleSheet("color:#4ddd8b;");
  } else {
    signal_notice_->setText(QStringLiteral(
      "当前信号：%1（仅供观察；深度/伪彩信号不能用于相机内参标定）")
      .arg(selected_signal_name_));
    signal_notice_->setStyleSheet("color:#ffc857;");
  }
}

void CalibrationWindow::selectSample()
{
  auto * item = sample_list_->currentItem();
  if (!item) {calibration_quality_panel_->hide(); return;}
  const QString original_path = item->data(Qt::UserRole).toString();
  if (quality_cache_.contains(original_path)) {
    calibration_quality_panel_->setMetrics(quality_cache_.value(original_path),
      QFileInfo(original_path).fileName());
    calibration_quality_panel_->show();
  } else {
    calibration_quality_panel_->clear(QStringLiteral("正在分析选中样本"));
    calibration_quality_panel_->show();
    transport_.requestImageQuality(original_path);
  }
  if (calibration_view_ == CalibrationView::Capture) {
    const QImage preview = item->data(kCornerPreviewRole).value<QImage>();
    right_view_->show();
    if (!preview.isNull()) {
      showImage(right_view_, preview);
    } else {
      right_view_->clearImage(item->text());
    }
    return;
  }
  left_view_->clearImage(QStringLiteral("正在读取原图…"));
  right_view_->clearImage(QStringLiteral("正在读取特征点误差图…"));
  undistorted_view_->clearImage(QStringLiteral("正在读取去畸变图…"));
  updateResultViews();
}

void CalibrationWindow::appendLog(const QString & text)
{
  if (!log_view_) {return;}
  log_view_->append(QStringLiteral("[%1] %2").arg(
    QDateTime::currentDateTime().toString("HH:mm:ss"), text));
}

void CalibrationWindow::handleCameraDisconnected(const QString & message)
{
  if (guided_active_) {stopGuidedCapture();}
  guided_start_pending_ = false;
  camera_operation_pending_ = false;
  stream_reconfigure_pending_ = false;
  camera_connected_ = false;
  camera_streaming_ = false;
  manual_capture_preview_pending_ = false;
  connected_device_id_.clear();
  home_capture_frozen_ = false;
  home_save_pending_ = false;
  capture_frozen_ = false;
  imported_frame_pending_ = false;
  imported_frame_active_ = false;
  batch_import_active_ = false;
  batch_aborted_ = false;
  batch_waiting_save_ = false;
  batch_import_queue_.clear();
  latest_frame_ = QImage();
  current_corner_preview_ = QImage();
  calibration_quality_panel_->clear(QStringLiteral("相机已断开"));

  connection_status_->setText(QStringLiteral("● 未连接"));
  connection_status_->setStyleSheet("color:#ff6573;");
  start_stop_button_->setText(QStringLiteral("启动相机"));
  start_stop_button_->setEnabled(false);
  calibration_start_stop_button_->setText(QStringLiteral("启动相机"));
  calibration_start_stop_button_->setEnabled(false);
  calibration_connection_status_->setText(QStringLiteral("● 未连接"));
  calibration_connection_status_->setStyleSheet("color:#ff6573;");
  disconnect_button_->setEnabled(false);
  connect_button_->setEnabled(false);
  camera_combo_->setEnabled(true);
  camera_combo_->clear();
  available_devices_.clear();
  rebuildStreamMenu();
  selected_signal_calibratable_ = false;
  selected_signal_name_.clear();
  selected_backend_name_.clear();
  resolution_combo_->setEnabled(true);
  refresh_button_->setEnabled(true);
  disable_emitter_checkbox_->setEnabled(false);
  home_capture_button_->setEnabled(false);
  home_save_button_->setEnabled(false);
  home_retake_button_->setEnabled(false);
  enter_calibration_button_->setEnabled(false);
  capture_button_->setEnabled(false);
  accept_button_->setEnabled(false);
  retake_button_->setEnabled(false);
  select_image_button_->setEnabled(true);
  ui_->back_button->setEnabled(true);
  board_width_spin_->setEnabled(true);
  board_height_spin_->setEnabled(true);
  square_size_spin_->setEnabled(true);
  board_physical_width_spin_->setEnabled(true);
  board_physical_height_spin_->setEnabled(true);

  home_live_view_->clearImage(QStringLiteral("未连接相机"));
  for (auto * view : stream_views_) {
    view->clearImage(QStringLiteral("未连接相机"));
  }
  for (auto * panel : stream_panels_) {
    panel->hide();
  }
  home_capture_view_->clearImage();
  home_capture_view_->hide();
  setEqualImageSizes(home_image_splitter_);
  left_view_->clearImage(QStringLiteral("未连接相机"));
  right_view_->clearImage(QStringLiteral("特征点识别结果"));
  right_view_->show();
  pages_->setCurrentWidget(home_page_);
  calibration_status_->setText(QStringLiteral("相机已断开，请重新连接或批量导入图片"));
  appendLog(message);

  // 拔线瞬间不立即重新进入厂商 SDK 枚举，避免枚举与 SDK 的断线清理互相等待。
  // 用户重新插入后点击“刷新设备”即可；未启动状态的低频枚举也会识别断线。
}

}  // namespace danmucamera

int main(int argc, char ** argv)
{
  // 尽早接管启动器的窗口激活信号，避免 UI 初始化瞬间重复点击的竞态。
  std::signal(SIGUSR1, requestWindowActivation);
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  danmucamera::CalibrationRosClient transport;
  danmucamera::CalibrationWindow window(transport);
  QTimer activation_timer;
  QObject::connect(&activation_timer, &QTimer::timeout, &window, [&window]() {
    if (!g_activate_window_requested) {return;}
    g_activate_window_requested = 0;
    if (window.isMinimized()) {window.showNormal();}
    window.show();
    window.raise();
    window.activateWindow();
  });
  activation_timer.start(100);
  window.show();
  const int result = app.exec();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
