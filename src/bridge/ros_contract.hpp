#pragma once

// 本文件是进程之间唯一共享的“通信契约”。界面、相机和算法桥只依赖这些名称，
// 不相互包含实现代码。后续重命名接口时只需修改这里。
namespace danmucamera::contract
{
inline constexpr char kImageTopic[] = "/calibration/camera/image/compressed";
inline constexpr char kStreamImageTopic[] = "/calibration/camera/streams/compressed";
inline constexpr char kStreamCloudTopic[] = "/calibration/camera/streams/points";
inline constexpr char kCapturedImageTopic[] = "/calibration/camera/captured/compressed";
inline constexpr char kCameraEventTopic[] = "/calibration/camera/event";
inline constexpr char kCameraStateTopic[] = "/calibration/camera/state";
inline constexpr char kCornerPreviewTopic[] = "/calibration/algorithm/corner_preview";
inline constexpr char kQualityResultTopic[] = "/calibration/algorithm/quality_result";
inline constexpr char kQualityRequestTopic[] = "/calibration/algorithm/quality_request";
inline constexpr char kCornerStatusTopic[] = "/calibration/algorithm/corner_status";
inline constexpr char kGuideResultTopic[] = "/calibration/algorithm/guide_result";
inline constexpr char kAlgorithmStatusTopic[] = "/calibration/algorithm/status";
inline constexpr char kAlgorithmResultTopic[] = "/calibration/algorithm/result";
inline constexpr char kAlgorithmReportTopic[] = "/calibration/algorithm/report";
inline constexpr char kViewOriginalRequestTopic[] = "/calibration/io/view_original";
inline constexpr char kViewUndistortedRequestTopic[] = "/calibration/io/view_undistorted";
inline constexpr char kViewedUndistortedTopic[] = "/calibration/io/undistorted_image";
inline constexpr char kViewErrorRequestTopic[] = "/calibration/io/view_error";
inline constexpr char kImportImageRequestTopic[] = "/calibration/io/import_image";
inline constexpr char kImportImageStatusTopic[] = "/calibration/io/import_image_status";
inline constexpr char kViewedOriginalTopic[] = "/calibration/io/original_image";
inline constexpr char kViewedErrorTopic[] = "/calibration/io/error_image";
inline constexpr char kDeleteSampleTopic[] = "/calibration/io/delete_sample";
inline constexpr char kStreamingService[] = "/calibration/camera/set_streaming";
inline constexpr char kEmitterService[] = "/calibration/camera/set_ir_emitter_disabled";
inline constexpr char kCaptureService[] = "/calibration/camera/capture";
inline constexpr char kSaveService[] = "/calibration/camera/save";
inline constexpr char kRefreshDevicesService[] = "/calibration/camera/refresh_devices";
inline constexpr char kRunCalibrationService[] = "/calibration/algorithm/run";
inline constexpr char kCameraNode[] = "/calibration_camera";

// 相机-雷达外参输入契约。图像/点云仍由 calibration_camera 的现有
// topic_input 参数和 UI“信息流”选择决定；CameraInfo 与配对阈值属于算法桥。
// 消息格式、时间戳和坐标系约束见 docs/ROS相机雷达外参输入接口.md。
namespace extrinsic_input
{
inline constexpr char kImageParameter[] = "topic_input.name";
inline constexpr char kImageTypeParameter[] = "topic_input.type";
inline constexpr char kCloudParameter[] = "topic_input.pointcloud_name";
inline constexpr char kCommandParameter[] = "topic_input.control_command";
inline constexpr char kStatusParameter[] = "topic_input.control_status";
inline constexpr char kCameraInfoParameter[] = "inputs.extrinsic.camera_info";
inline constexpr char kMaxPairDeltaParameter[] = "inputs.extrinsic.max_pair_delta_ms";
inline constexpr char kRawImageType[] = "sensor_msgs/msg/Image";
inline constexpr char kCompressedImageType[] = "sensor_msgs/msg/CompressedImage";
inline constexpr char kCloudType[] = "sensor_msgs/msg/PointCloud2";
inline constexpr char kCameraInfoType[] = "sensor_msgs/msg/CameraInfo";
}  // namespace extrinsic_input
}  // namespace danmucamera::contract
