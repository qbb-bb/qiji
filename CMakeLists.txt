cmake_minimum_required(VERSION 3.10)
project(danmucamera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 寻找 ROS 2 构建工具 和 OpenCV
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/forms)

# 两个厂商 SDK 都是可选后端：安装后自动启用，缺失时仍可用 V4L2。
find_package(realsense2 CONFIG QUIET)
# CMake searches system prefixes and CMAKE_PREFIX_PATH, including a sourced ROS/SDK environment.
find_path(ORBBEC_INCLUDE_DIR NAMES libobsensor/ObSensor.hpp)
find_library(ORBBEC_LIBRARY NAMES OrbbecSDK)

include_directories(
  src/algorithm
  src/bridge
  src/ui
  ${OpenCV_INCLUDE_DIRS}
)

# 2. 编译可执行文件
add_library(guided_pose STATIC src/algorithm/guided_pose.cpp)
target_link_libraries(guided_pose ${OpenCV_LIBS})

add_library(board_detector src/algorithm/board_detector.cpp)
target_link_libraries(board_detector ${OpenCV_LIBS})
add_executable(calibration_solver src/algorithm/calibration_solver.cpp)
target_link_libraries(calibration_solver board_detector ${OpenCV_LIBS})

# I/O 层集中在一个实现中；阻塞相机读取在其内部工作线程运行。
add_executable(io_node src/bridge/io_node.cpp)
target_link_libraries(io_node ${OpenCV_LIBS})
ament_target_dependencies(io_node rclcpp sensor_msgs std_msgs std_srvs)
if(realsense2_FOUND)
  target_link_libraries(io_node realsense2::realsense2)
  target_compile_definitions(io_node PRIVATE DANMUCAMERA_HAS_REALSENSE=1)
  message(STATUS "RealSense SDK backend enabled")
else()
  message(WARNING "librealsense2-dev not found: RealSense backend disabled")
endif()
if(ORBBEC_INCLUDE_DIR AND ORBBEC_LIBRARY)
  target_include_directories(io_node PRIVATE ${ORBBEC_INCLUDE_DIR})
  target_link_libraries(io_node ${ORBBEC_LIBRARY})
  target_compile_definitions(io_node PRIVATE DANMUCAMERA_HAS_ORBBEC=1)
  message(STATUS "Orbbec SDK v2 backend enabled")
else()
  message(WARNING "Orbbec SDK v2 not found: Gemini backend disabled")
endif()

# 图像质量纯算法模块：不依赖 ROS 2 / Qt，只负责质量计算和报警判断。
add_library(image_quality_analyzer STATIC src/algorithm/image_quality_analyzer.cpp)
target_link_libraries(image_quality_analyzer ${OpenCV_LIBS})

# 标定桥负责 ROS 消息编排；具体图像质量算法位于 image_quality_analyzer。
# 这样保持原有 quality topic 链路不变，同时避免在 bridge 内写报警算法。
add_executable(calibration_bridge src/bridge/calibration_bridge.cpp)
target_link_libraries(calibration_bridge board_detector guided_pose image_quality_analyzer ${OpenCV_LIBS})
ament_target_dependencies(calibration_bridge rclcpp sensor_msgs std_msgs std_srvs)

# UI 复用轻量 3D 引导投影算法；标定求解器仍独立运行。
add_executable(calibration_ui
  src/ui/forms/calibration_window.ui
  src/ui/forms/result_visualization_dialog.ui
  src/ui/forms/guided_capture_dialog.ui
  src/ui/calibration_window.cpp
  src/ui/result_visualization.cpp
  src/ui/result_visualization.hpp
  src/bridge/calibration_ros_client.cpp
  src/ui/image_label.cpp
  src/ui/image_quality_panel.cpp
  src/ui/guided_capture_plan.cpp
  src/ui/guided_capture_ui.cpp
  src/ui/image_label.hpp
  src/ui/image_quality_panel.hpp
)
target_link_libraries(calibration_ui Qt6::Widgets guided_pose ${OpenCV_LIBS})
ament_target_dependencies(calibration_ui rclcpp rcl_interfaces sensor_msgs std_msgs std_srvs)

# 3. 安装到 ROS 2 的路径中
install(TARGETS
  calibration_solver
  io_node
  calibration_bridge
  calibration_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/calibration.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES config/calibration.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# 4. 【最核心】这一句负责生成 local_setup.bash 和资源索引
ament_package()
