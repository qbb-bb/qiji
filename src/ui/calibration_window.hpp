#pragma once

#include <QImage>
#include <QHash>
#include <array>
#include "calibration_ros_client.hpp"
#include <QMainWindow>
#include <QStringList>

class QCloseEvent;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QDialog;
class QLineEdit;
class QToolButton;
class QGroupBox;
class QLabel;
class QListWidget;
class QPushButton;
class QSpinBox;
class QSplitter;
class QStackedWidget;
class QTextEdit;
class QTimer;
class QWidget;

namespace Ui
{
class CalibrationWindow;
}

namespace danmucamera
{
class CalibrationRosClient;
class ImageLabel;
class ImageQualityPanel;
class PointCloudView;

class CalibrationWindow : public QMainWindow
{
public:
  explicit CalibrationWindow(CalibrationRosClient & transport);
  ~CalibrationWindow() override;

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  enum class CalibrationView {Capture, Result};
  // 扩展标定类型时，在此分发到对应流程；当前仅单目内参有实现。
  enum class CalibrationKind {Monocular, Stereo, Extrinsic, HandEye};
  CalibrationKind selectedCalibrationKind() const;
  bool calibrationKindAvailable() const;

  void buildUi();
  void buildHomePage();
  void buildCalibrationPage();
  void buildSettingsUi();
  void bindTransport();
  void showImage(ImageLabel * label, const QImage & image);
  void setEqualImageSizes(QSplitter * splitter);
  void startNewCalibrationSession();
  void resetCaptureView();
  void updateSampleCount();
  void updateCornerCoverage();
  void updateCalibrationAvailability();
  void updateCalibrationModeUi();
  void rebuildStreamMenu();
  QStringList checkedStreams() const;
  bool canConnectTopics() const;
  void applySelectedStreams();
  void updateHomeStreamPanels(const QStringList & streams, const QString & primary);
  void updateTopicInputsVisibility();
  void updateExtrinsicInputPanel();
  void selectSample();
  void processNextBatchImage();
  void maybeSaveCurrentBatchImage();
  void addBatchProblem(const QString & source_path, const QString & reason);
  void handleCameraDisconnected(const QString & message);
  void appendLog(const QString & text);
  CalibrationSettings currentSettings() const;
  bool validateSettings();
  void updateBoardControls();
  void updateCameraModelControls();
  void updateResultViews();
  void requestSelectedResultImages();
  void startGuidedCapture();
  void beginGuidedDetection();
  void stopGuidedCapture(bool completed = false);
  void handleGuideDetection(const GuideDetection & detection);
  void maybeCompleteGuidedCapture();
  void showGuidedProblem(const QString & reason);
  void finishGuidedSave(bool success, const QString & path);

  CalibrationRosClient & transport_;
  ::Ui::CalibrationWindow * ui_;
  QTimer * transport_timer_;
  QStackedWidget * pages_;
  QWidget * home_page_;
  QWidget * calibration_page_;

  QComboBox * camera_combo_;
  QToolButton * stream_button_{nullptr};
  QHash<QString, QCheckBox *> stream_actions_;
  QHash<QString, ImageLabel *> stream_views_;
  QHash<QString, QWidget *> stream_panels_;
  QWidget * topic_inputs_panel_{nullptr};
  QComboBox * image_topic_edit_{nullptr};
  QComboBox * depth_image_topic_edit_{nullptr};
  QComboBox * cloud_topic_edit_{nullptr};
  QComboBox * topic_control_command_edit_{nullptr};
  QComboBox * topic_control_status_edit_{nullptr};
  QPushButton * apply_topic_inputs_button_{nullptr};
  QGroupBox * extrinsic_input_panel_{nullptr};
  QLabel * extrinsic_source_label_{nullptr};
  QComboBox * extrinsic_camera_info_edit_{nullptr};
  QDoubleSpinBox * extrinsic_pair_delta_spin_{nullptr};
  QPushButton * apply_extrinsic_input_button_{nullptr};
  QWidget * cloud_panel_{nullptr};
  PointCloudView * cloud_view_{nullptr};
  std::vector<UiTopicSource> available_streams_;
  QStringList active_streams_;
  bool stream_reconfigure_pending_{false};
  bool stream_restart_pending_{false};
  bool topic_inputs_dirty_{false};
  bool topic_selection_touched_{false};
  QString default_image_topic_, default_depth_image_topic_, default_cloud_topic_;
  QString default_command_topic_, default_status_topic_;
  QString default_camera_info_topic_;
  bool emitter_change_pending_{false};
  bool confirmed_emitter_disabled_{true};
  QComboBox * resolution_combo_;
  QComboBox * calibration_type_combo_;
  QCheckBox * disable_emitter_checkbox_;
  QPushButton * refresh_button_;
  QPushButton * connect_button_;
  QPushButton * start_stop_button_;
  QPushButton * calibration_start_stop_button_;
  QPushButton * disconnect_button_;
  QPushButton * home_capture_button_;
  QPushButton * home_save_button_;
  QPushButton * home_retake_button_;
  QPushButton * enter_calibration_button_;
  QLabel * connection_status_;
  QLabel * signal_notice_;
  ImageLabel * home_live_view_;
  QWidget * home_primary_panel_{nullptr};
  QLabel * home_primary_title_{nullptr};
  ImageLabel * home_capture_view_;
  QSplitter * home_image_splitter_;
  ImageQualityPanel * calibration_quality_panel_;

  QSpinBox * board_width_spin_;
  QSpinBox * board_height_spin_;
  QDoubleSpinBox * square_size_spin_;
  QDoubleSpinBox * board_physical_width_spin_;
  QDoubleSpinBox * board_physical_height_spin_;
  QComboBox * board_type_combo_;
  QComboBox * camera_model_combo_;
  QComboBox * distortion_model_combo_;
  QComboBox * dictionary_combo_;
  QDoubleSpinBox * marker_size_spin_;
  QLabel * marker_size_label_;
  QLabel * dictionary_label_;
  QCheckBox * auto_initial_checkbox_;
  QCheckBox * guide_use_initial_checkbox_;
  QSpinBox * guide_initial_width_spin_;
  QSpinBox * guide_initial_height_spin_;
  QWidget * manual_initial_widget_;
  std::array<QDoubleSpinBox *, 4> initial_camera_spins_{};
  std::array<QDoubleSpinBox *, 8> initial_distortion_spins_{};
  std::array<QLabel *, 8> initial_distortion_labels_{};
  QWidget * result_controls_;
  QCheckBox * show_original_checkbox_;
  QCheckBox * show_error_checkbox_;
  QCheckBox * show_undistorted_checkbox_;
  QLabel * calibration_status_;
  QLabel * calibration_connection_status_;
  QLabel * resolution_notice_;
  ImageLabel * left_view_;
  ImageLabel * right_view_;
  ImageLabel * undistorted_view_;
  ImageLabel * result_empty_view_;
  QSplitter * image_splitter_;
  QListWidget * sample_list_;
  QPushButton * capture_button_;
  QPushButton * guided_button_{nullptr};
  QDialog * guided_dialog_{nullptr};
  ImageLabel * guided_live_view_{nullptr};
  QLabel * guided_status_label_{nullptr};
  QPushButton * select_image_button_;
  QPushButton * accept_button_;
  QPushButton * retake_button_;
  QPushButton * run_button_;
  QPushButton * delete_button_;
  QPushButton * supplement_button_;
  QPushButton * global_error_button_;
  QLabel * sample_count_label_;
  QLabel * corner_coverage_label_{nullptr};
  QTextEdit * log_view_;

  QImage latest_frame_;
  QImage current_corner_preview_;
  int active_width_{1280};
  int active_height_{720};
  QSize session_image_size_;
  bool camera_connected_{false};
  bool camera_streaming_{false};
  bool camera_operation_pending_{false};
  bool selected_signal_calibratable_{false};
  QString selected_signal_name_;
  QString selected_backend_name_;
  QString connected_device_id_;
  bool home_capture_frozen_{false};
  bool home_save_pending_{false};
  bool capture_frozen_{false};
  bool manual_capture_preview_pending_{false};
  bool imported_frame_pending_{false};
  bool imported_frame_active_{false};
  bool batch_import_active_{false};
  bool batch_aborted_{false};
  bool batch_waiting_save_{false};
  quint64 batch_request_id_{0};
  bool batch_current_valid_{false};
  bool batch_corner_status_received_{false};
  bool batch_preview_received_{false};
  bool current_quality_received_{false};
  ImageQualityMetrics current_quality_;
  QHash<QString, ImageQualityMetrics> quality_cache_;
  QHash<QString, QImage> guided_preview_cache_;
  int batch_total_{0};
  int batch_processed_{0};
  QString batch_current_source_;
  QString batch_current_message_;
  QStringList batch_import_queue_;
  bool corners_valid_{false};
  bool calibration_requested_{false};
  bool guided_active_{false};
  bool guided_start_pending_{false};
  bool guided_backend_ready_{false};
  bool guided_waiting_capture_{false};
  bool guided_waiting_save_{false};
  bool guided_status_received_{false};
  bool guided_preview_received_{false};
  bool guided_quality_received_{false};
  bool guided_capture_valid_{false};
  bool guided_wait_for_clear_{false};
  bool guided_solver_pending_{false};
  int guided_step_{0};
  int guided_stable_count_{0};
  QString guided_corner_message_;
  QStringList guided_paths_;
  GuideSettings guide_settings_;
  QString global_error_image_path_;
  QString last_board_settings_key_;
  CalibrationSettings last_confirmed_settings_;
  std::array<double, 4> last_initial_camera_values_{};
  std::array<double, 8> last_initial_distortion_values_{};
  CalibrationView calibration_view_{CalibrationView::Capture};
};

}  // namespace danmucamera
