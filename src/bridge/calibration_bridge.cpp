#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros_contract.hpp"
#include "board_detector.hpp"
#include "guided_pose.hpp"
#include "image_quality_analyzer.hpp"

namespace danmucamera
{

struct CalibrationResultFiles
{
  std::string full_result;
  std::string camera_matrix;
  std::string distortion_matrix;
};

// 对 shell 参数做最小且完整的单引号转义。路径来自 ROS 参数，因此执行前必须转义。
std::string shellQuote(const std::string & value)
{
  std::string quoted = "'";
  for (const char c : value) {
    quoted += c == '\'' ? "'\\''" : std::string(1, c);
  }
  return quoted + "'";
}

class CalibrationBridge : public rclcpp::Node
{
public:
  CalibrationBridge()
  : Node("calibration_algorithm_bridge"), running_(false)
  {
    algorithm_executable_ = declare_parameter<std::string>("algorithm_executable", "");
    result_directory_ = declare_parameter<std::string>("result_directory",
      "result/calibration");
    report_directory_ = declare_parameter<std::string>("report_directory", "result");
    declare_parameter<std::vector<std::string>>("sample_paths", std::vector<std::string>{});
    declare_parameter<int>("board_width", 7);
    declare_parameter<int>("board_height", 6);
    declare_parameter<double>("square_size", 0.06);
    declare_parameter<std::string>("board_type", "chessboard");
    declare_parameter<bool>("guide_enabled", false);
    declare_parameter<std::vector<double>>("guide_camera_matrix", std::vector<double>(9, 0.0));
    declare_parameter<std::vector<double>>("guide_distortion", std::vector<double>(5, 0.0));
    declare_parameter<int>("guide_camera_width", 0);
    declare_parameter<int>("guide_camera_height", 0);
    declare_parameter<double>("guide_board_physical_width", 0.0);
    declare_parameter<double>("guide_board_physical_height", 0.0);
    declare_parameter<double>("marker_length", 0.04);
    declare_parameter<int>("dictionary_id", 0);
    declare_parameter<std::string>("camera_model", "pinhole");
    declare_parameter<std::string>("distortion_model", "brown5");
    declare_parameter<bool>("automatic_initial", true);
    declare_parameter<double>("initial_fx", 0.0);
    declare_parameter<double>("initial_fy", 0.0);
    declare_parameter<double>("initial_cx", 0.0);
    declare_parameter<double>("initial_cy", 0.0);
    declare_parameter<std::vector<double>>("initial_distortion", std::vector<double>(8, 0.0));

    // 图像质量阈值只来自 calibration.yaml。这里的 0/-1 是缺失配置哨兵，
    // 不作为实际运行默认值；配置缺失或非法时直接在启动阶段报错。
    ImageQualityConfig quality_config;
    quality_config.analysis_max_width =
      declare_parameter<int>("quality.analysis_max_width", 0);
    quality_config.analysis_max_height =
      declare_parameter<int>("quality.analysis_max_height", 0);
    quality_config.roi_margin_ratio =
      declare_parameter<double>("quality.roi_margin_ratio", -1.0);
    quality_config.roi_min_points =
      declare_parameter<int>("quality.roi_min_points", 0);
    quality_config.roi_min_pixels =
      declare_parameter<int>("quality.roi_min_pixels", 0);
    quality_config.min_board_area_ratio =
      declare_parameter<double>("quality.min_board_area_ratio", -1.0);
    quality_config.max_board_area_ratio =
      declare_parameter<double>("quality.max_board_area_ratio", -1.0);
    quality_config.min_laplacian_variance =
      declare_parameter<double>("quality.min_laplacian_variance", -1.0);
    quality_config.max_dark_ratio =
      declare_parameter<double>("quality.max_dark_ratio", -1.0);
    quality_config.max_bright_ratio =
      declare_parameter<double>("quality.max_bright_ratio", -1.0);
    quality_config.max_black_clip_ratio =
      declare_parameter<double>("quality.max_black_clip_ratio", -1.0);
    quality_config.max_white_clip_ratio =
      declare_parameter<double>("quality.max_white_clip_ratio", -1.0);
    quality_config.min_mean_gray =
      declare_parameter<double>("quality.min_mean_gray", -1.0);
    quality_config.max_mean_gray =
      declare_parameter<double>("quality.max_mean_gray", -1.0);
    quality_analyzer_ = std::make_unique<ImageQualityAnalyzer>(quality_config);
    quality_config_ = quality_config;
    declare_parameter<double>("corner_coverage.min_ratio", 0.05);
    declare_parameter<double>("corner_coverage.max_ratio", 0.25);
    guide_min_board_area_ratio_ = quality_config.min_board_area_ratio;
    guide_max_board_area_ratio_ = quality_config.max_board_area_ratio;

    // transient_local 让后启动/重连的 UI 也能立即看到最近一次状态与结果。
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kAlgorithmStatusTopic, latched_qos);
    result_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kAlgorithmResultTopic, latched_qos);
    report_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kAlgorithmReportTopic, latched_qos);
    quality_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kQualityResultTopic, rclcpp::QoS(10).reliable());
    quality_request_sub_ = create_subscription<std_msgs::msg::String>(
      contract::kQualityRequestTopic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr request) {
        const cv::Mat image = cv::imread(request->data, cv::IMREAD_COLOR);
        std::vector<cv::Point2f> board_points;
        if (!image.empty()) {
          try {
            const BoardObservation observation = detectBoard(image, currentBoardSpec());
            if (observation.valid) {board_points = observation.image_points;}
          } catch (const std::exception & error) {
            RCLCPP_WARN(
              get_logger(), "质量检测 ROI 提取失败，将回退整图: %s", error.what());
          }
        }
        publishQuality(image, "file", request->data, board_points);
      });
    corner_status_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kCornerStatusTopic, rclcpp::QoS(1).reliable());
    corner_preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      contract::kCornerPreviewTopic, rclcpp::QoS(1).reliable());
    // 自动引导拍摄暂时停用：不订阅实时帧，也不启动后台引导线程。
#if 0
    guide_result_pub_ = create_publisher<std_msgs::msg::String>(
      contract::kGuideResultTopic, rclcpp::QoS(5).reliable());
    guide_frame_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kImageTopic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        if (!get_parameter("guide_enabled").as_bool()) {return;}
        {
          std::lock_guard<std::mutex> lock(guide_mutex_);
          latest_guide_frame_ = message->data;
        }
        guide_cv_.notify_one();
      });
#endif
    captured_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      contract::kCapturedImageTopic, rclcpp::QoS(1).reliable(),
      std::bind(&CalibrationBridge::checkCorners, this, std::placeholders::_1));
    run_service_ = create_service<std_srvs::srv::Trigger>(
      contract::kRunCalibrationService,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (running_.exchange(true)) {
          response->success = false;
          response->message = "标定正在运行，请勿重复启动";
          return;
        }
        if (worker_.joinable()) {
          worker_.join();
        }
        response->success = true;
        response->message = "已接受标定任务";
        publish(status_pub_, "running: 正在运行多板型相机标定求解器…");
        worker_ = std::thread(&CalibrationBridge::runAlgorithm, this);
      });

#if 0
    guide_worker_ = std::thread(&CalibrationBridge::guideLoop, this);
#endif
    publish(status_pub_, "idle: 等待开始标定");
  }

  ~CalibrationBridge() override
  {
    {
      std::lock_guard<std::mutex> lock(guide_mutex_);
      guide_shutdown_ = true;
    }
    guide_cv_.notify_all();
    if (guide_worker_.joinable()) {guide_worker_.join();}
    if (worker_.joinable()) {worker_.join();}
  }

private:
  void runAlgorithm()
  {
    std::filesystem::path manifest_path;
    try {
      // 默认从当前可执行文件的同级目录寻找新求解器，安装和 build 空间均适用。
      std::string executable = algorithm_executable_;
      if (executable.empty()) {
        executable = (std::filesystem::read_symlink("/proc/self/exe").parent_path() /
          "calibration_solver").string();
      }
      const auto sample_paths = get_parameter("sample_paths").as_string_array();
      if (sample_paths.size() < 3) {
        throw std::runtime_error("本次会话有效样本不足 3 张");
      }
      // 把 UI 本次会话的显式路径写成一次性清单，不扫描历史样本目录。
      // 不向算法传目录，因而历史 JPG 不可能被自动带入本次标定。
      const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
      manifest_path = std::filesystem::temp_directory_path() /
        ("danmucamera_session_" + std::to_string(unique_id) + ".txt");
      std::ofstream manifest(manifest_path);
      if (!manifest) {throw std::runtime_error("无法创建本次标定样本清单");}
      for (const auto & path : sample_paths) {manifest << path << '\n';}
      manifest.close();
      const auto result_dir = std::filesystem::absolute(result_directory_) /
        ("run_" + std::to_string(unique_id));
      const auto initial_distortion = get_parameter("initial_distortion").as_double_array();
      if (initial_distortion.size() != 8) {
        throw std::runtime_error("手动畸变初值必须有 8 个参数");
      }
      const std::vector<std::string> arguments = {
        executable, manifest_path.string(), result_dir.string(),
        get_parameter("board_type").as_string(),
        std::to_string(get_parameter("board_width").as_int()),
        std::to_string(get_parameter("board_height").as_int()),
        std::to_string(get_parameter("square_size").as_double()),
        std::to_string(get_parameter("marker_length").as_double()),
        std::to_string(get_parameter("dictionary_id").as_int()),
        get_parameter("camera_model").as_string(),
        get_parameter("distortion_model").as_string(),
        get_parameter("automatic_initial").as_bool() ? "1" : "0",
        std::to_string(get_parameter("initial_fx").as_double()),
        std::to_string(get_parameter("initial_fy").as_double()),
        std::to_string(get_parameter("initial_cx").as_double()),
        std::to_string(get_parameter("initial_cy").as_double())
      };
      std::string command;
      for (const auto & argument : arguments) {command += shellQuote(argument) + " ";}
      for (const double value : initial_distortion) {
        command += shellQuote(std::to_string(value)) + " ";
      }
      command += "2>&1";
      FILE * pipe = popen(command.c_str(), "r");
      if (!pipe) {
        throw std::runtime_error("无法启动标定求解器进程");
      }

      std::ostringstream output;
      char buffer[512];
      while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output << buffer;
      }
      const int exit_status = pclose(pipe);
      std::filesystem::remove(manifest_path);
      manifest_path.clear();

      const std::string output_text = output.str();
      if (exit_status != 0) {
        publish(result_pub_, output_text);
        publish(status_pub_, "failed: 标定求解器失败，退出状态 " +
          std::to_string(exit_status));
        running_ = false;
        return;
      }
      const CalibrationResultFiles result_files = saveResult(output_text);
      const std::string detailed_report = saveDetailedReport(
        sample_paths, output_text, result_files, result_dir);
      publish(result_pub_, "完整结果已保存: " + result_files.full_result +
        "\n内参矩阵已保存: " + result_files.camera_matrix +
        "\n畸变矩阵已保存: " + result_files.distortion_matrix +
        "\n检测报告已保存: " + detailed_report + "\n" + output_text);
      publishReport(output_text, result_files);
      publish(status_pub_, "success: 标定完成");
    } catch (const std::exception & error) {
      if (!manifest_path.empty()) {std::filesystem::remove(manifest_path);}
      publish(result_pub_, std::string("标定桥接异常: ") + error.what());
      publish(status_pub_, "failed: 无法运行标定算法");
    }
    running_ = false;
  }

  static int expectedGuidePoints(const BoardSpec & board)
  {
    if (board.type == "charuco") {return (board.columns - 1) * (board.rows - 1);}
    if (board.type == "aprilgrid") {return board.columns * board.rows * 4;}
    return board.columns * board.rows;
  }

  // 引导画面里可能同时出现实体标定板和显示器中的缩略棋盘。
  // 对规则棋盘/圆点逐一遮住已找到的候选，再检测下一块，选面积最大的实体板。
  BoardObservation detectGuidedBoard(const cv::Mat & image, const BoardSpec & board)
  {
    if (board.type != "chessboard" && board.type != "circles_symmetric" &&
      board.type != "circles_asymmetric") {return detectBoard(image, board);}
    cv::Mat search = image.clone();
    BoardObservation best;
    double best_area = -1.0;
    for (int candidate_index = 0; candidate_index < 3; ++candidate_index) {
      BoardObservation candidate = detectBoard(search, board);
      if (!candidate.valid || candidate.image_points.size() < 4) {break;}
      std::vector<cv::Point2f> hull;
      cv::convexHull(candidate.image_points, hull);
      const double candidate_area = std::abs(cv::contourArea(hull));
      if (candidate_area > best_area) {
        best_area = candidate_area;
        best = std::move(candidate);
      }
      cv::Rect covered = cv::boundingRect(hull);
      const int padding = std::max(12, std::min(covered.width, covered.height) / 8);
      covered.x -= padding;
      covered.y -= padding;
      covered.width += 2 * padding;
      covered.height += 2 * padding;
      covered &= cv::Rect(0, 0, search.cols, search.rows);
      if (covered.empty()) {break;}
      cv::rectangle(search, covered, cv::Scalar(127, 127, 127), cv::FILLED);
    }
    if (best.valid) {
      best.preview = image.clone();
      cv::drawChessboardCorners(best.preview, cv::Size(board.columns, board.rows),
        best.image_points, true);
    }
    return best;
  }

  void guideLoop()
  {
    while (true) {
      std::vector<uint8_t> compressed;
      {
        std::unique_lock<std::mutex> lock(guide_mutex_);
        guide_cv_.wait(lock, [this]() {
          return guide_shutdown_ || !latest_guide_frame_.empty();
        });
        if (guide_shutdown_) {return;}
        compressed.swap(latest_guide_frame_);
      }
      if (!get_parameter("guide_enabled").as_bool()) {continue;}
      try {
        const cv::Mat encoded(1, static_cast<int>(compressed.size()), CV_8U,
          compressed.data());
        const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (image.empty()) {continue;}
        const BoardSpec board = currentBoardSpec();
        const BoardObservation observation = detectGuidedBoard(image, board);
        const ImageQualityResult quality = quality_analyzer_->evaluate(
          image, observation.valid ? observation.image_points : std::vector<cv::Point2f>{});
        const int expected = expectedGuidePoints(board);
        const int count = static_cast<int>(observation.image_points.size());
        const bool complete = observation.valid && count == expected;
        const cv::Rect bounds = observation.image_points.empty() ? cv::Rect() :
          cv::boundingRect(observation.image_points);
        GuideCamera camera;
        const auto matrix = get_parameter("guide_camera_matrix").as_double_array();
        const auto distortion = get_parameter("guide_distortion").as_double_array();
        camera.fisheye = get_parameter("camera_model").as_string() == "fisheye";
        camera.distortion.assign(distortion.begin(), distortion.end());
        if (matrix.size() == 9) {
          for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {camera.K(r, c) = matrix[3 * r + c];}
          }
        }
        const bool has_intrinsics = camera.K(0, 0) > 0 && camera.K(1, 1) > 0;
        const int source_width = get_parameter("guide_camera_width").as_int();
        const int source_height = get_parameter("guide_camera_height").as_int();
        if (has_intrinsics && source_width > 0 && source_height > 0) {
          camera = scaledGuideCamera(camera, cv::Size(source_width, source_height), image.size());
        }
        const GuideBoard physical_board = guideBoard(board,
          get_parameter("guide_board_physical_width").as_double(),
          get_parameter("guide_board_physical_height").as_double());
        GuidePose observed_pose;
        std::string pose_error;
        const bool pose_valid = observation.valid && has_intrinsics &&
          guideObserved(observation, physical_board, camera, observed_pose, pose_error);
        std::vector<cv::Point2d> observed_corners;
        bool outline_valid = pose_valid;
        if (pose_valid) {observed_corners = observed_pose.corners;}
        else if (observation.valid) {
          outline_valid = guideObservedOutline(observation, physical_board,
            observed_corners, pose_error);
        } else {
          pose_error = "标定板特征点未完整识别";
        }
        // 引导框面积以物理板面四角为准；不能拿角点凸包外扩 ROI 的面积
        // 与 guide.area_ratio 的四角投影面积比较，两者定义不同。
        double guide_board_area_ratio = quality.board_area_ratio;
        bool guide_board_size_warning = quality.board_size_warning;
        if (outline_valid) {
          const std::vector<cv::Point2f> physical_corners = {
            cv::Point2f(observed_corners[0]), cv::Point2f(observed_corners[1]),
            cv::Point2f(observed_corners[2]), cv::Point2f(observed_corners[3])};
          guide_board_area_ratio = std::abs(cv::contourArea(physical_corners)) /
            static_cast<double>(image.total());
          guide_board_size_warning =
            guide_board_area_ratio < guide_min_board_area_ratio_ ||
            guide_board_area_ratio > guide_max_board_area_ratio_;
        }
        std::ostringstream data;
        data << std::fixed << std::setprecision(3)
          << image.cols << '\t' << image.rows << '\t'
          << observation.valid << '\t' << complete << '\t'
          << count << '\t' << expected << '\t'
          << bounds.x << '\t' << bounds.y << '\t'
          << bounds.width << '\t' << bounds.height << '\t'
          << quality.blur_warning << '\t' << quality.exposure_warning << '\t'
          << quality.laplacian_variance << '\t' << quality.mean_gray << '\t'
          << quality.warning_reason << '\t'
          << guide_board_area_ratio << '\t' << guide_board_size_warning;
        for (int i = 0; i < 4; ++i) {
          const cv::Point2d corner = outline_valid ? observed_corners[i] : cv::Point2d();
          data << '\t' << corner.x << '\t' << corner.y;
        }
        data << '\t' << pose_valid;
        data << std::setprecision(9);
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            data << '\t' << observed_pose.R(row, column);
          }
        }
        data << '\t' << pose_error;
        if (get_parameter("guide_enabled").as_bool()) {
          publish(guide_result_pub_, data.str());
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "引导识别失败: %s", error.what());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

  BoardSpec currentBoardSpec()
  {
    BoardSpec board;
    board.type = get_parameter("board_type").as_string();
    board.columns = get_parameter("board_width").as_int();
    board.rows = get_parameter("board_height").as_int();
    board.spacing = get_parameter("square_size").as_double();
    board.marker_length = get_parameter("marker_length").as_double();
    board.dictionary_id = get_parameter("dictionary_id").as_int();
    return board;
  }

  // bridge 只负责 ROS 消息编排；清晰度/曝光算法均在 ImageQualityAnalyzer 中。
  void publishQuality(
    const cv::Mat & input, const std::string & mode, const std::string & path,
    const std::vector<cv::Point2f> & board_points = {})
  {
    const ImageQualityResult result = quality_analyzer_->evaluate(input, board_points);

    // 在直方图后附加板面占比和距离告警，由 UI 按相同字段顺序解析。
    std::ostringstream data;
    data << std::fixed << std::setprecision(3)
      << mode << '\t'
      << result.laplacian_variance << '\t'
      << result.mean_gray << '\t'
      << result.blur_warning << '\t'
      << result.exposure_warning << '\t'
      << result.warning_reason;
    for (const int bin : result.histogram) {data << '\t' << bin;}
    data << '\t' << result.board_area_ratio
      << '\t' << result.board_size_warning
      << '\t' << result.board_area_min_ratio
      << '\t' << result.board_area_max_ratio;
    for (const int count : result.corner_grid_counts) {data << '\t' << count;}
    data << '\t' << path;
    publish(quality_pub_, data.str());
  }

  void checkCorners(sensor_msgs::msg::CompressedImage::ConstSharedPtr message)
  {
    const cv::Mat encoded(
      1, static_cast<int>(message->data.size()), CV_8U,
      const_cast<uint8_t *>(message->data.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      // 即使图片无法解码也必须发布质量结果，否则 UI 会一直等待 current_quality_received_。
      publishQuality(cv::Mat(), "captured", "");
      publish(corner_status_pub_, "invalid|无法解码拍摄画面，请重拍");
      return;
    }

    BoardObservation observation;
    try {
      observation = detectBoard(image, currentBoardSpec());
    } catch (const cv::Exception & error) {
      observation.message = std::string("标定板识别异常: ") + error.what();
    } catch (const std::exception & error) {
      observation.message = error.what();
    }

    // 识别成功：用特征点凸包扩展后的标定板 ROI 做质量统计。
    // 识别失败：board_points 为空，ImageQualityAnalyzer 自动回退整图。
    publishQuality(
      image, "captured", "",
      observation.valid ? observation.image_points : std::vector<cv::Point2f>{});

    if (observation.preview.empty()) {observation.preview = image;}
    const bool guide_capture = false;  // 自动拍摄暂时停用。
    const int expected = expectedGuidePoints(currentBoardSpec());
    const bool complete = observation.valid &&
      static_cast<int>(observation.image_points.size()) == expected;
    const bool accepted = observation.valid && (!guide_capture || complete);
    const std::string corner_message = guide_capture && observation.valid && !complete ?
      "角点不完整：识别到 " + std::to_string(observation.image_points.size()) +
      "/" + std::to_string(expected) + "，请完整显示标定板后重拍" : observation.message;
    publish(corner_status_pub_, std::string(accepted ? "valid|" : "invalid|") +
      corner_message);

    sensor_msgs::msg::CompressedImage preview;
    preview.header = message->header;
    preview.format = "jpeg";
    cv::imencode(".jpg", observation.preview, preview.data, {cv::IMWRITE_JPEG_QUALITY, 100});
    corner_preview_pub_->publish(std::move(preview));
  }

  static std::vector<double> extractMatrix(
    const std::string & output, const std::string & marker, size_t expected_values)
  {
    const size_t marker_position = output.find(marker);
    if (marker_position == std::string::npos) {
      throw std::runtime_error("算法输出中缺少矩阵: " + marker);
    }
    const std::regex number_pattern(
      R"([-+]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)");
    size_t search_from = marker_position + marker.size();
    while (true) {
      const size_t open = output.find('[', search_from);
      const size_t close = open == std::string::npos ? std::string::npos : output.find(']', open);
      if (open == std::string::npos || close == std::string::npos) {break;}
      const std::string block = output.substr(open + 1, close - open - 1);
      const bool contains_non_exponent_letter = std::any_of(
        block.begin(), block.end(), [](unsigned char c) {
          return std::isalpha(c) && c != 'e' && c != 'E';
        });
      std::vector<double> values;
      if (!contains_non_exponent_letter) {
        for (std::sregex_iterator it(block.begin(), block.end(), number_pattern), end;
          it != end; ++it)
        {
          values.push_back(std::stod(it->str()));
        }
      }
      if (values.size() == expected_values) {return values;}
      search_from = close + 1;
    }
    throw std::runtime_error(
            marker + " 后未找到包含 " + std::to_string(expected_values) +
            " 个数值的矩阵");
  }

  std::string saveDetailedReport(
    const std::vector<std::string> & samples, const std::string & output,
    const CalibrationResultFiles & files, const std::filesystem::path & run_directory)
  {
    const auto result_path = std::filesystem::path(files.full_result);
    const std::string prefix = "calibration_result_";
    const std::string suffix = result_path.stem().string().substr(prefix.size());
    const auto report_dir = std::filesystem::absolute(report_directory_);
    std::filesystem::create_directories(report_dir);
    const auto report_path = report_dir / ("calibration_report_" + suffix + ".md");
    const std::regex error_pattern(
      R"(图片 \[([0-9]+)\] \((.*)\) - 平均误差: ([0-9.]+) 像素)");
    std::map<std::string, std::pair<int, double>> errors;
    std::istringstream lines(output);
    std::string line;
    std::smatch match;
    while (std::getline(lines, line)) {
      if (std::regex_search(line, match, error_pattern)) {
        errors[match[2].str()] = {std::stoi(match[1].str()), std::stod(match[3].str())};
      }
    }
    std::ofstream report(report_path);
    if (!report) {throw std::runtime_error("无法创建标定报告: " + report_path.string());}
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);
    const BoardSpec board = currentBoardSpec();
    const double coverage_min = get_parameter("corner_coverage.min_ratio").as_double();
    const double coverage_max = get_parameter("corner_coverage.max_ratio").as_double();
    report << std::fixed << std::setprecision(4)
      << "# 相机标定检测报告\n\n"
      << "生成时间：" << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << "\n\n"
      << "## 本次标定设置与检测阈值\n\n"
      << "- 标定板：" << board.type << "，" << board.columns << " 列 × "
      << board.rows << " 行；间距 " << board.spacing << "；标记长度 "
      << get_parameter("marker_length").as_double() << "；字典 ID "
      << get_parameter("dictionary_id").as_int() << "\n"
      << "- 相机模型：" << get_parameter("camera_model").as_string()
      << "；畸变模型：" << get_parameter("distortion_model").as_string() << "\n"
      << "- 内参初值：" << (get_parameter("automatic_initial").as_bool() ? "自动" : "手动") << "\n"
      << "- 板面像素占比范围：" << quality_config_.min_board_area_ratio * 100.0
      << "%～" << quality_config_.max_board_area_ratio * 100.0 << "%\n"
      << "- 拉普拉斯方差下限：" << quality_config_.min_laplacian_variance << "\n"
      << "- 平均灰度范围：" << quality_config_.min_mean_gray << "～"
      << quality_config_.max_mean_gray << "\n"
      << "- 暗部 / 亮部占比上限：" << quality_config_.max_dark_ratio * 100.0
      << "% / " << quality_config_.max_bright_ratio * 100.0 << "%\n"
      << "- 近黑 / 近白裁剪占比上限：" << quality_config_.max_black_clip_ratio * 100.0
      << "% / " << quality_config_.max_white_clip_ratio * 100.0 << "%\n"
      << "- 九宫格补拍 / 集中阈值：低于 " << coverage_min * 100.0
      << "% / 高于 " << coverage_max * 100.0 << "%\n"
      << "- 质量分析最大尺寸：" << quality_config_.analysis_max_width << " × "
      << quality_config_.analysis_max_height << "；ROI 外扩比例 "
      << quality_config_.roi_margin_ratio << "；ROI 最少特征点 "
      << quality_config_.roi_min_points << "；ROI 最少像素 "
      << quality_config_.roi_min_pixels << "\n\n";
    if (!get_parameter("automatic_initial").as_bool()) {
      report << "- 手动初始内参 fx / fy / cx / cy："
        << get_parameter("initial_fx").as_double() << " / "
        << get_parameter("initial_fy").as_double() << " / "
        << get_parameter("initial_cx").as_double() << " / "
        << get_parameter("initial_cy").as_double() << "\n";
      report << "- 手动初始畸变系数：";
      for (const double coefficient : get_parameter("initial_distortion").as_double_array()) {
        report << coefficient << ' ';
      }
      report << "\n\n";
    }

    std::array<int, 9> grid{};
    int blur_count = 0, exposure_count = 0, board_count = 0;
    std::ostringstream per_image;
    per_image << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < samples.size(); ++i) {
      const auto & path = samples[i];
      per_image << "### " << i + 1 << ". " << std::filesystem::path(path).filename().string()
        << "\n\n- 原图：`" << path << "`\n";
      const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
      if (image.empty()) {
        per_image << "- 图片无法读取；未参与求解\n\n";
        continue;
      }
      per_image << "- 分辨率：" << image.cols << " × " << image.rows << "\n";
      try {
        const BoardObservation observation = detectBoard(image, board);
        const ImageQualityResult quality = quality_analyzer_->evaluate(
          image, observation.valid ? observation.image_points : std::vector<cv::Point2f>{});
        const auto used = errors.find(path);
        if (quality.blur_warning) {++blur_count;}
        if (quality.exposure_warning) {++exposure_count;}
        if (quality.board_size_warning) {++board_count;}
        std::vector<std::string> quality_warnings;
        if (!quality.readable) {quality_warnings.emplace_back("图像无法分析");}
        if (quality.blur_warning) {quality_warnings.emplace_back("清晰度不足");}
        if (quality.board_size_warning) {
          quality_warnings.emplace_back(
            quality.board_area_ratio < quality_config_.min_board_area_ratio ?
            "标定板太远" : "标定板太近");
        }
        if (quality.underexposed) {quality_warnings.emplace_back("欠曝风险");}
        if (quality.overexposed) {quality_warnings.emplace_back("过曝风险");}
        per_image << "- 求解状态：" << (used == errors.end() ? "未参与" : "已参与")
          << "；特征点：" << (observation.valid ? "识别成功" : "识别失败")
          << "，" << observation.image_points.size() << " 点\n"
          << "- 板面像素占比：";
        if (quality.board_area_ratio < 0) {per_image << "无法计算";} else {
          per_image << quality.board_area_ratio * 100.0 << "%";
        }
        per_image << "；" << (quality.board_size_warning ? "超出范围" : "未超出范围")
          << "\n- 清晰度拉普拉斯方差：" << quality.laplacian_variance
          << "；" << (quality.blur_warning ? "模糊告警" : "通过")
          << "\n- 平均灰度：" << quality.mean_gray
          << "；暗部 / 亮部占比：" << quality.dark_ratio * 100.0 << "% / "
          << quality.bright_ratio * 100.0 << "%"
          << "；近黑 / 近白裁剪占比：" << quality.black_clip_ratio * 100.0 << "% / "
          << quality.white_clip_ratio * 100.0 << "%\n"
          << "- 曝光检测：" << (quality.exposure_warning ? "告警" : "通过")
          << "；分析区域：" << (quality.roi_used ? "标定板附近区域" : "整张图片")
          << "；分析像素数：" << quality.analyzed_pixels
          << "\n- 画质告警：";
        if (quality_warnings.empty()) {
          per_image << "无";
        } else {
          for (size_t i = 0; i < quality_warnings.size(); ++i) {
            if (i) {per_image << "、";}
            per_image << quality_warnings[i];
          }
        }
        per_image << "\n- 九宫格角点数：";
        for (size_t cell = 0; cell < 9; ++cell) {
          if (cell) {per_image << ", ";}
          per_image << quality.corner_grid_counts[cell];
          if (used != errors.end()) {grid[cell] += quality.corner_grid_counts[cell];}
        }
        per_image << "\n";
        if (used != errors.end()) {
          const std::string index = std::to_string(used->second.first);
          per_image << "- 平均重投影误差：" << used->second.second << " px\n"
            << "- 误差图：`" << (run_directory / ("error_vis_" + index + ".png")).string()
            << "`；去畸变图：`" << (run_directory / ("undistorted_" + index + ".png")).string()
            << "`\n";
        }
      } catch (const std::exception & error) {
        per_image << "- 报告阶段复检失败：" << error.what() << "\n";
      }
      per_image << "\n";
    }
    const int total = std::accumulate(grid.begin(), grid.end(), 0);
    const std::array<const char *, 9> names = {
      "左上", "上中", "右上", "左中", "中央", "右中", "左下", "下中", "右下"};
    report << "## 检测汇总\n\n"
      << "- 提交样本：" << samples.size() << " 张；实际参与求解：" << errors.size()
      << " 张；未参与求解：" << samples.size() - errors.size() << " 张\n"
      << "- 模糊 / 曝光 / 板面占比告警：" << blur_count << " / "
      << exposure_count << " / " << board_count << " 张\n"
      << "- 参与求解的角点总数：" << total << "\n\n"
      << "## 九宫格角点覆盖（仅参与求解的图片）\n\n"
      << "| 方位 | 角点数 | 占比 | 提示 |\n| --- | ---: | ---: | --- |\n";
    for (size_t cell = 0; cell < 9; ++cell) {
      const double ratio = total > 0 ? static_cast<double>(grid[cell]) / total : 0.0;
      report << "| " << names[cell] << " | " << grid[cell] << " | "
        << ratio * 100.0 << "% | "
        << (total == 0 ? "无角点" : ratio < coverage_min ? "建议补拍" :
        ratio > coverage_max ? "过于集中" : "正常") << " |\n";
    }
    report << "\n九宫格只用于拍摄指导。\n\n"
      << "## 逐张图片检测结果\n\n" << per_image.str()
      << "## 标定结果文件\n\n"
      << "- 求解器完整结果：`" << files.full_result << "`\n"
      << "- 相机内参矩阵：`" << files.camera_matrix << "`\n"
      << "- 畸变系数：`" << files.distortion_matrix << "`\n"
      << "- 全局误差图：`" << (run_directory / "global_error_distribution.png").string() << "`\n"
      << "- 矢量误差图：`" << (run_directory / "global_error_vectors.svg").string() << "`\n"
      << "- 位姿及逐点误差：`" << (run_directory / "visualization_data.tsv").string() << "`\n\n"
      << "### 求解器完整输出（含 RMS、内参、畸变和逐图误差）\n\n```text\n"
      << output << "\n```\n";
    report.close();
    if (!report) {throw std::runtime_error("写入标定报告失败: " + report_path.string());}
    return report_path.string();
  }

  CalibrationResultFiles saveResult(const std::string & output)
  {
    std::error_code error;
    std::filesystem::create_directories(result_directory_, error);
    if (error) {throw std::runtime_error("无法创建标定结果目录: " + error.message());}
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto directory = std::filesystem::absolute(result_directory_);
    const std::string suffix = std::to_string(timestamp) + ".txt";
    const auto result_path = directory / ("calibration_result_" + suffix);
    const auto camera_path = directory / ("camera_matrix_" + suffix);
    const auto distortion_path = directory / ("distortion_matrix_" + suffix);
    std::ofstream file(result_path);
    if (!file) {throw std::runtime_error("无法创建标定结果文件");}
    file << output;
    file.close();

    const auto camera = extractMatrix(output, "相机内参矩阵 K (Camera Matrix):", 9);
    const auto distortion = extractMatrix(output, "镜头畸变系数 (Distortion Coefficients):",
      get_parameter("camera_model").as_string() == "fisheye" ? 4 :
      get_parameter("distortion_model").as_string() == "rational8" ? 8 : 5);
    std::ofstream camera_file(camera_path);
    std::ofstream distortion_file(distortion_path);
    if (!camera_file || !distortion_file) {
      throw std::runtime_error("无法创建独立内参/畸变矩阵文件");
    }
    camera_file << "# Camera intrinsic matrix K (3x3)\n" << std::setprecision(17);
    for (size_t row = 0; row < 3; ++row) {
      camera_file << camera[row * 3] << ' ' << camera[row * 3 + 1] << ' ' <<
        camera[row * 3 + 2] << '\n';
    }
    distortion_file << "# " << get_parameter("camera_model").as_string() << " / "
      << get_parameter("distortion_model").as_string() << " distortion coefficients ("
      << distortion.size() << ")\n" << std::setprecision(17);
    for (size_t i = 0; i < distortion.size(); ++i) {
      if (i > 0) {distortion_file << ' ';}
      distortion_file << distortion[i];
    }
    distortion_file << '\n';
    return {result_path.string(), camera_path.string(), distortion_path.string()};
  }

  void publishReport(const std::string & output, const CalibrationResultFiles & result_files)
  {
    // 将求解器的逐图误差、误差图、去畸变图转为 UI 清单。
    const std::regex line_pattern(
      R"(图片 \[([0-9]+)\] \((.*)\) - 平均误差: ([0-9.]+) 像素)");
    std::istringstream lines(output);
    std::ostringstream report;
    report << "@result_file\t" << result_files.full_result << '\n';
    report << "@camera_matrix_file\t" << result_files.camera_matrix << '\n';
    report << "@distortion_matrix_file\t" << result_files.distortion_matrix << '\n';
    const std::regex global_pattern(R"(全局重投影误差分布图: (.+))");
    std::smatch global_match;
    if (std::regex_search(output, global_match, global_pattern)) {
      const std::string global_image = std::filesystem::absolute(global_match[1].str()).string();
      report << "@global_error_image\t" << global_image << '\n';
    }
    std::string line;
    std::smatch match;
    while (std::getline(lines, line)) {
      if (!std::regex_search(line, match, line_pattern)) {continue;}
      const std::string suffix = line.substr(match.position() + match.length());
      std::istringstream paths(suffix);
      std::string error_image, undistorted_image;
      if (suffix.empty() || suffix[0] != '\t' ||
        !std::getline(paths, error_image, '\t') ||
        !std::getline(paths, error_image, '\t') ||
        !std::getline(paths, undistorted_image, '\t')) {continue;}
      report << match[2].str() << '\t' << match[3].str() << '\t'
        << error_image << '\t' << undistorted_image << '\n';
    }
    publish(report_pub_, report.str());
  }

  void publish(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
    const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    publisher->publish(message);
  }

  std::string algorithm_executable_;
  std::string result_directory_;
  std::string report_directory_;
  std::atomic_bool running_;
  std::thread worker_;
  std::unique_ptr<ImageQualityAnalyzer> quality_analyzer_;
  ImageQualityConfig quality_config_;
  double guide_min_board_area_ratio_{0.0};
  double guide_max_board_area_ratio_{1.0};
  std::mutex guide_mutex_;
  std::condition_variable guide_cv_;
  std::vector<uint8_t> latest_guide_frame_;
  bool guide_shutdown_{false};
  std::thread guide_worker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guide_result_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr guide_frame_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr quality_request_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr corner_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr corner_preview_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr captured_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
};

}  // namespace danmucamera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<danmucamera::CalibrationBridge>());
  rclcpp::shutdown();
  return 0;
}
