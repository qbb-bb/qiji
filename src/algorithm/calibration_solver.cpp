#include "board_detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Settings
{
  danmucamera::BoardSpec board;
  std::string camera_model;
  std::string distortion_model;
  bool automatic_initial{true};
  double fx{0}, fy{0}, cx{0}, cy{0};
  std::vector<double> initial_distortion;
};

cv::Mat initialCamera(const cv::Size & size, const Settings & settings,
  const std::vector<std::vector<cv::Point3f>> & objects,
  const std::vector<std::vector<cv::Point2f>> & images)
{
  if (!settings.automatic_initial) {
    if (settings.fx <= 0 || settings.fy <= 0) {
      throw std::runtime_error("手动初值的 fx、fy 必须大于零");
    }
    return (cv::Mat_<double>(3, 3) << settings.fx, 0, settings.cx,
      0, settings.fy, settings.cy, 0, 0, 1);
  }
  if (settings.camera_model == "fisheye") {
    // initCameraMatrix2D assumes a pinhole camera. On a fisheye image it can
    // return very large, unequal focal lengths and make InitExtrinsics degenerate.
    // Match OpenCV fisheye::calibrate's own no-guess initialization for the log.
    const double focal = std::max(size.width, size.height) / CV_PI;
    return (cv::Mat_<double>(3, 3) << focal, 0, (size.width - 1) / 2.0,
      0, focal, (size.height - 1) / 2.0, 0, 0, 1);
  }
  return cv::initCameraMatrix2D(objects, images, size, 0);
}

std::string pathString(const std::filesystem::path & path)
{
  return std::filesystem::absolute(path).string();
}

constexpr double kArrowLengthPx = 18.0;

cv::Scalar directionColor(const cv::Point2f & residual)
{
  const double angle = std::atan2(residual.y, residual.x);
  const unsigned char hue = static_cast<unsigned char>(
    std::lround((angle + CV_PI) * 179.0 / (2.0 * CV_PI)));
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 225, 245));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto color = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(color[0], color[1], color[2]);
}

std::string svgColor(const cv::Scalar & bgr)
{
  std::ostringstream out;
  out << '#' << std::hex << std::setfill('0')
      << std::setw(2) << static_cast<int>(bgr[2])
      << std::setw(2) << static_cast<int>(bgr[1])
      << std::setw(2) << static_cast<int>(bgr[0]);
  return out.str();
}

void drawArrow(cv::Mat & image, const cv::Point2f & origin,
  const cv::Point2f & residual, double image_scale, const std::string & label)
{
  const cv::Point2f start = origin * image_scale;
  const double length = cv::norm(residual);
  const cv::Scalar color = length > 1e-9 ? directionColor(residual) :
    cv::Scalar(175, 175, 175);
  const cv::Point2f end = length > 1e-9 ?
    start + residual * static_cast<float>(kArrowLengthPx * image_scale / length) : start;
  if (length > 1e-9) {
    cv::arrowedLine(image, start, end, color, std::max(2, cvRound(image_scale)),
      cv::LINE_AA, 0, 0.32);
  } else {
    cv::circle(image, start, std::max(2, cvRound(image_scale * 2)), color, -1, cv::LINE_AA);
  }
  if (!label.empty()) {
    const cv::Point label_origin(cvRound(end.x + 3 * image_scale),
      cvRound(end.y - 3 * image_scale));
    const double font_scale = std::max(0.38, 0.38 * image_scale);
    cv::putText(image, label, label_origin, cv::FONT_HERSHEY_SIMPLEX,
      font_scale, cv::Scalar(0, 0, 0), std::max(2, cvRound(3 * image_scale)), cv::LINE_AA);
    cv::putText(image, label, label_origin, cv::FONT_HERSHEY_SIMPLEX,
      font_scale, color, std::max(1, cvRound(image_scale)), cv::LINE_AA);
  }
}

void svgHeader(std::ostream & out, const cv::Size & size)
{
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "
      << size.width << ' ' << size.height << "\">\n"
      << "<rect width=\"100%\" height=\"100%\" fill=\"#101923\"/>\n";
  for (int step = 1; step < 10; ++step) {
    const double x = size.width * step / 10.0;
    const double y = size.height * step / 10.0;
    out << "<path d=\"M" << x << " 0V" << size.height << " M0 " << y << 'H'
        << size.width << "\" stroke=\"#263340\" stroke-width=\"0.6\"/>\n";
  }
}

void svgArrow(std::ostream & out, const cv::Point2f & origin,
  const cv::Point2f & residual, double opacity, const std::string & label)
{
  const double length = cv::norm(residual);
  const cv::Point2f end = length > 1e-9 ?
    origin + residual * static_cast<float>(kArrowLengthPx / length) : origin;
  const std::string color = length > 1e-9 ?
    svgColor(directionColor(residual)) : "#aaaaaa";
  out << "<g stroke=\"" << color << "\" fill=\"" << color
      << "\" opacity=\"" << opacity << "\">";
  if (length > 1e-9) {
    const cv::Point2f unit = residual * static_cast<float>(1.0 / length);
    const cv::Point2f normal(-unit.y, unit.x);
    const cv::Point2f a = end - unit * 6.0f + normal * 2.7f;
    const cv::Point2f b = end - unit * 6.0f - normal * 2.7f;
    out << "<path d=\"M" << origin.x << ' ' << origin.y << 'L' << end.x << ' '
        << end.y << "\" stroke-width=\"1.5\"/>"
        << "<path d=\"M" << end.x << ' ' << end.y << 'L' << a.x << ' ' << a.y
        << 'L' << b.x << ' ' << b.y << "Z\"/>";
  } else {
    out << "<circle cx=\"" << origin.x << "\" cy=\"" << origin.y
        << "\" r=\"2\"/>";
  }
  out << "</g>\n";
  if (!label.empty()) {
    out << "<text x=\"" << end.x + 3 << "\" y=\"" << end.y - 3
        << "\" font-size=\"10\" font-family=\"sans-serif\" fill=\""
        << color << "\" stroke=\"#101923\" stroke-width=\"2.2\""
        << " paint-order=\"stroke\">" << label << "</text>\n";
  }
}

void printMatrix(const cv::Mat & matrix)
{
  std::cout << "[";
  for (int row = 0; row < matrix.rows; ++row) {
    if (row) {std::cout << "; ";}
    for (int col = 0; col < matrix.cols; ++col) {
      if (col) {std::cout << ", ";}
      std::cout << std::setprecision(12) << matrix.at<double>(row, col);
    }
  }
  std::cout << "]\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    // 清单、输出目录、板型/尺寸/间距、ChArUco 设置、相机/畸变模型、初值模式、K(4)、D(8)。
    if (argc != 24) {throw std::runtime_error("标定求解器参数数量不正确");}
    const std::filesystem::path manifest_path(argv[1]);
    const std::filesystem::path output_dir(argv[2]);
    Settings settings;
    settings.board.type = argv[3];
    settings.board.columns = std::stoi(argv[4]);
    settings.board.rows = std::stoi(argv[5]);
    settings.board.spacing = std::stod(argv[6]);
    settings.board.marker_length = std::stod(argv[7]);
    settings.board.dictionary_id = std::stoi(argv[8]);
    settings.camera_model = argv[9];
    settings.distortion_model = argv[10];
    settings.automatic_initial = std::stoi(argv[11]) != 0;
    settings.fx = std::stod(argv[12]);
    settings.fy = std::stod(argv[13]);
    settings.cx = std::stod(argv[14]);
    settings.cy = std::stod(argv[15]);
    for (int i = 16; i < 24; ++i) {settings.initial_distortion.push_back(std::stod(argv[i]));}
    if (settings.camera_model != "pinhole" && settings.camera_model != "fisheye") {
      throw std::runtime_error("未知相机模型");
    }
    if (settings.camera_model == "fisheye" &&
      settings.distortion_model != "fisheye4" && settings.distortion_model != "fisheye2") {
      throw std::runtime_error("鱼眼模型仅支持等距 4/2 参数畸变");
    }
    if (settings.camera_model == "pinhole" &&
      settings.distortion_model != "brown5" && settings.distortion_model != "rational8" &&
      settings.distortion_model != "none") {
      throw std::runtime_error("针孔模型畸变类型无效");
    }

    std::ifstream manifest(manifest_path);
    if (!manifest) {throw std::runtime_error("无法读取本次标定样本清单");}
    std::vector<std::string> paths;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;
    std::string path;
    while (std::getline(manifest, path)) {
      if (path.empty()) {continue;}
      const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
      if (image.empty()) {std::cerr << "跳过无法读取的图片: " << path << '\n'; continue;}
      if (image_size.empty()) {image_size = image.size();}
      if (image.size() != image_size) {std::cerr << "跳过尺寸不一致的图片: " << path << '\n'; continue;}
      const auto found = danmucamera::detectBoard(image, settings.board);
      if (!found.valid) {std::cerr << "跳过未识别特征点的图片: " << path << '\n'; continue;}
      paths.push_back(path);
      object_points.push_back(found.object_points);
      image_points.push_back(found.image_points);
    }
    if (paths.size() < 3) {throw std::runtime_error("有效标定板图像不足 3 张");}
    std::filesystem::create_directories(output_dir);
    cv::Mat K = initialCamera(image_size, settings, object_points, image_points);
    const int coefficient_count = settings.camera_model == "fisheye" ? 4 :
      settings.distortion_model == "rational8" ? 8 : 5;
    cv::Mat D = cv::Mat::zeros(coefficient_count, 1, CV_64F);
    if (!settings.automatic_initial && settings.distortion_model != "none") {
      for (int i = 0; i < coefficient_count; ++i) {
        D.at<double>(i) = settings.initial_distortion[i];
      }
    }
    if (settings.distortion_model == "fisheye2") {
      D.at<double>(2) = 0.0;
      D.at<double>(3) = 0.0;
    }
    std::cout << "标定板: " << settings.board.type << "，相机模型: " << settings.camera_model
      << "，畸变模型: " << settings.distortion_model << '\n';
    std::cout << "初值方式: " << (settings.automatic_initial ? "自动" : "手动") << '\n';
    std::cout << "初始相机矩阵 K:\n"; printMatrix(K);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms;
    if (settings.camera_model == "fisheye") {
      int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
      if (!settings.automatic_initial) {flags |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;}
      if (settings.distortion_model == "fisheye2") {
        flags |= cv::fisheye::CALIB_FIX_K3 | cv::fisheye::CALIB_FIX_K4;
      }
      rms = cv::fisheye::calibrate(object_points, image_points, image_size,
        K, D, rvecs, tvecs, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-8));
    } else {
      int flags = cv::CALIB_USE_INTRINSIC_GUESS;
      if (settings.distortion_model == "rational8") {flags |= cv::CALIB_RATIONAL_MODEL;}
      if (settings.distortion_model == "none") {
        flags |= cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 |
          cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
      }
      rms = cv::calibrateCamera(object_points, image_points, image_size,
        K, D, rvecs, tvecs, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-8));
    }
    std::cout << "RMS: " << std::setprecision(8) << rms << " px\n";
    std::cout << "相机内参矩阵 K (Camera Matrix):\n"; printMatrix(K);
    std::cout << "镜头畸变系数 (Distortion Coefficients):\n";
    cv::Mat D_row = D.reshape(1, 1);
    printMatrix(D_row);

    const double scale = 3.0;
    cv::Mat global(static_cast<int>(image_size.height * scale),
      static_cast<int>(image_size.width * scale), CV_8UC3, cv::Scalar(17, 25, 16));
    const auto data_path = output_dir / "visualization_data.tsv";
    const auto global_svg_path = output_dir / "global_error_vectors.svg";
    std::ofstream data(data_path);
    std::ofstream global_svg(global_svg_path);
    if (!data || !global_svg) {throw std::runtime_error("无法创建矢量误差数据文件");}
    data << std::setprecision(17);
    global_svg << std::setprecision(8);
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto & points : object_points) {
      for (const auto & point : points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
      }
    }
    data << "META\t" << image_size.width << '\t' << image_size.height << '\t'
      << min_x << '\t' << max_x << '\t' << min_y << '\t' << max_y << '\n';
    svgHeader(global_svg, image_size);
    double all_sum = 0;
    int all_count = 0;
    for (size_t i = 0; i < paths.size(); ++i) {
      std::vector<cv::Point2f> projected;
      if (settings.camera_model == "fisheye") {
        cv::fisheye::projectPoints(object_points[i], projected, rvecs[i], tvecs[i], K, D);
      } else {
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], K, D, projected);
      }
      const cv::Mat rv = rvecs[i].reshape(1, 1);
      const cv::Mat tv = tvecs[i].reshape(1, 1);
      data << "POSE\t" << i;
      for (int k = 0; k < 3; ++k) {data << '\t' << rv.at<double>(k);}
      for (int k = 0; k < 3; ++k) {data << '\t' << tv.at<double>(k);}
      data << '\n';
      cv::Mat original = cv::imread(paths[i], cv::IMREAD_COLOR);
      cv::Mat error_view = original.clone();
      cv::Mat undistorted;
      if (settings.camera_model == "fisheye") {
        cv::fisheye::undistortImage(original, undistorted, K, D, K, original.size());
      } else {
        cv::undistort(original, undistorted, K, D);
      }
      const auto svg_path = output_dir / ("error_vectors_" + std::to_string(i) + ".svg");
      std::ofstream sample_svg(svg_path);
      if (!sample_svg) {throw std::runtime_error("无法创建单张矢量误差图");}
      sample_svg << std::setprecision(8);
      svgHeader(sample_svg, image_size);
      double image_sum = 0;
      for (size_t j = 0; j < projected.size(); ++j) {
        const cv::Point2f actual = image_points[i][j];
        const cv::Point2f estimate = projected[j];
        const cv::Point2f residual = actual - estimate;
        const double error = cv::norm(residual);
        image_sum += error;
        all_sum += error;
        ++all_count;
        data << "POINT\t" << i << '\t' << estimate.x << '\t' << estimate.y
          << '\t' << actual.x << '\t' << actual.y << '\n';
        std::ostringstream value;
        value << std::fixed << std::setprecision(3) << error;
        const std::string sample_label = value.str();
        drawArrow(error_view, estimate, residual, 1.0, sample_label);
        drawArrow(global, estimate, residual, scale, "");
        svgArrow(sample_svg, estimate, residual, 1.0, sample_label);
        svgArrow(global_svg, estimate, residual, 0.85, "");
      }
      sample_svg << "</svg>\n";
      const double mean = image_sum / projected.size();
      const auto error_path = output_dir / ("error_vis_" + std::to_string(i) + ".png");
      const auto undistort_path = output_dir / ("undistorted_" + std::to_string(i) + ".png");
      if (!cv::imwrite(error_path.string(), error_view) ||
        !cv::imwrite(undistort_path.string(), undistorted)) {
        throw std::runtime_error("无法保存误差图或去畸变图");
      }
      std::cout << "图片 [" << i << "] (" << paths[i] << ") - 平均误差: "
        << std::fixed << std::setprecision(4) << mean << " 像素\t"
        << pathString(error_path) << '\t' << pathString(undistort_path) << '\n';
    }
    global_svg << "</svg>\n";
    data.close();
    global_svg.close();
    const auto global_path = output_dir / "global_error_distribution.png";
    if (!cv::imwrite(global_path.string(), global)) {
      throw std::runtime_error("无法保存全局误差分布图");
    }
    std::cout << "全局重投影误差分布图: " << pathString(global_path) << '\n';
    std::cout << "矢量误差图: " << pathString(global_svg_path) << '\n';
    std::cout << "位姿与误差数据: " << pathString(data_path) << '\n';
    std::cout << "平均单点误差: " << all_sum / all_count << " px\n";
    return 0;
  } catch (const cv::Exception & error) {
    std::cerr << "OpenCV 标定失败: " << error.what() << '\n';
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "标定失败: " << error.what() << '\n';
    return 2;
  }
}
