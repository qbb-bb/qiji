# 相机内参标定工作台

Linux 上的 ROS 2 + Qt 6 + OpenCV 标定程序。相机采集支持 V4L2 彩色流；编译时检测到 Orbbec SDK v2 或 librealsense2 后，会启用对应的 RGB、红外和深度后端。深度流只用于预览，不能参与内参标定。

## 迁移到另一台电脑

编译故障、换机安装命令和逐步验收见 [换机编译与测试说明](docs/换机编译与测试说明.md)。

Ubuntu 22.04 新电脑可在图形桌面终端运行 `./launch/setup_and_run.sh`，自动安装 Qt 6、ROS 2 Humble 和其他必要依赖，随后编译并启动。首次运行需要联网和 sudo 权限。

只复制本目录的 `CMakeLists.txt`、`package.xml`、`src/`、`config/`、`launch/`、`docs/` 和本 README。`build/`、`install/`、`log/` 是本机产物，不要复制；`samples/` 与 `result/calibration/` 是个人数据，按需备份。新电脑须重新标定新相机及其各个分辨率和成像器。

先安装 ROS 2（含 `ament_cmake`、`rclcpp`、`rcl_interfaces`、`sensor_msgs`、`std_msgs`、`std_srvs`、`launch_ros`）、`colcon`、C++17 编译器、Qt 6 Widgets、OpenCV（含 ArUco）和 `v4l-utils`。按相机厂商提供的说明安装 SDK、运行库和 USB/udev 权限规则。SDK 安装后重新插拔相机。普通 V4L2 相机无需厂商 SDK。

CMake 通过系统搜索路径和 `CMAKE_PREFIX_PATH` 查找 SDK，不写死用户名或 ROS 安装目录。如 SDK 安装在自定义位置，先将其安装前缀加入 `CMAKE_PREFIX_PATH`。新厂商 SDK 仍需在 `src/bridge/io_node.cpp` 增加枚举、开流、取帧和释放的实现。

```bash
cd /你的路径/qiji
./launch/build.sh
./launch/run.sh
```

`build.sh` 和 `run.sh` 优先使用已加载的 ROS 环境；未加载时从 `/opt/ros/${ROS_DISTRO:-humble}` 查找。构建输出位于本目录的 `build/`、`install/`、`log/`。`run.sh` 自动进入本目录，原图保存到 `samples/`，每次标定完成后在 `result/` 自动生成 `calibration_report_时间戳.md`，记录设置与阈值、逐图检测、九宫格覆盖、重投影误差和结果文件；其他标定结果保存在 `result/calibration/`。若只想手动启动，可在本目录加载 ROS 与 `install/setup.bash`，再运行 `ros2 launch danmucamera calibration.launch.py`。

构建输出中有 `RealSense SDK backend enabled` 或 `Orbbec SDK v2 backend enabled`，表示对应后端已编入。若未显示，检查 SDK 开发头文件、动态库和 CMake 搜索路径，然后清理本目录的 `build/danmucamera/` 并重新构建。

## 使用

刷新相机，选择设备与分辨率，点击“连接相机”。程序只有在 SDK 打开视频流并收到第一帧后才显示已连接、已启动；五秒无有效帧会报错并恢复未连接。关闭相机按钮只停流并保留设备选择；断开按钮清除连接。红外标定应关闭投射器，RGB、左右红外分别标定。导入图片仍可在不连接相机时使用。

如果设备出现在列表却无法打开，先检查 SDK 示例程序能否采流、所选分辨率和帧率是否受支持，以及设备是否被其他进程占用。相机采集代码在 `src/bridge/io_node.cpp`，ROS 话题和服务名在 `src/bridge/ros_contract.hpp`。

## 从网络 ROS 2 图像话题采集

在运行本程序的电脑上先用 `ros2 topic list` 确认能看到远端图像话题，再用 `ros2 topic type /实际话题名` 确认类型。在 `config/calibration.yaml` 的 `calibration_camera.ros__parameters.topic_input.name` 填入完整话题名，例如 `/remote_camera/image_raw`。`type: auto` 会在启动采流时识别 `sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage`；也可明确填写 `raw` 或 `compressed`。修改配置后重启程序，在设备列表选择 `[ROS-TOPIC]`，点击连接、启动相机，即可照常预览、拍摄和保存。不需要修改代码中的内部预览话题名。

远端发布端须在启动采流前可被 ROS 2 发现。若 5 秒内没有收到首帧，程序会提示连接失败；采流期间连续 3 秒无帧会断开。检查两端的 `ROS_DOMAIN_ID`、网络发现及 `ros2 topic info -v /实际话题名` 中的 QoS。此输入使用传感器数据 QoS。原始图像支持 `bgr8`、`rgb8`、`bgra8`、`rgba8`、`mono8`、`mono16` 和相应的 `8UC1`、`16UC1`；16 位单通道图像会转成 8 位供标定。收到的实际分辨率会作为当前标定分辨率，不按界面预设值缩放原图。

自动引导拍摄入口暂时关闭，普通拍摄和批量导图仍可用。每张图的标定板像素面积占比必须落在 `config/calibration.yaml` 的 `quality.min_board_area_ratio` 与 `quality.max_board_area_ratio` 之间（当前为 15%～70%）；超出区间会标红，不能人工确认纳入。标定页侧栏将当前有效照片的角点位置按画面九宫格累计，每格显示角点数和占总角点数的比例；`calibration_ui.corner_coverage.min_ratio`（默认 5%）以下提示补拍该方位，`max_ratio`（默认 25%）以上提示该方位过于集中。修改配置后重启程序生效。拍摄预览、保存、批量新增和删除后都会重新统计。九宫格是拍摄指导，不单独阻止求解。
