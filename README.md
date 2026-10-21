# 相机内参标定工作台

Linux 上的 ROS 2 + Qt 6 + OpenCV 标定程序。相机采集支持 V4L2 彩色流；编译时检测到 Orbbec SDK v2 或 librealsense2 后，会启用对应的 RGB、红外和深度后端。深度流只用于预览，不能参与内参标定。

## 迁移到另一台电脑

编译故障、换机安装命令和逐步验收见 [换机编译与测试说明](docs/换机编译与测试说明.md)。

Ubuntu 22.04/24.04 新电脑可在图形桌面终端运行 `./launch/setup_and_run.sh`。脚本优先使用已安装的 ROS 2 Humble/Jazzy；完全没有 ROS 时才按 Ubuntu 22.04→Humble、24.04→Jazzy 安装，不会增加第二套 ROS。它只安装缺失的 Qt 6、OpenCV、ROS 2 和相机 SDK，随后编译并启动，还会在应用菜单和桌面生成“相机标定工作台”图标；已安装软件包不会被升级或卸载。首次补装需要联网和 sudo 权限。相机未插入时可用 `--camera=orbbec` 或 `--camera=realsense` 预装对应 SDK；已有多个 ROS 时可用 `--ros=jazzy` 或 `--ros=humble` 明确选择。

只复制本目录的 `CMakeLists.txt`、`package.xml`、`src/`、`config/`、`launch/`、`docs/` 和本 README。`build/`、`install/`、`log/` 是本机产物，不要复制；`samples/` 与 `result/calibration/` 是个人数据，按需备份。新电脑须重新标定新相机及其各个分辨率和成像器。

如需手动安装，先准备 ROS 2（含 `ament_cmake`、`rclcpp`、`rcl_interfaces`、`sensor_msgs`、`std_msgs`、`std_srvs`、`launch_ros`）、`colcon`、C++17 编译器、Qt 6 Widgets、OpenCV（含 ArUco）和 `v4l-utils`。按相机厂商提供的说明安装 SDK、运行库和 USB/udev 权限规则。SDK 安装后重新插拔相机。普通 V4L2 相机无需厂商 SDK。

CMake 通过系统搜索路径和 `CMAKE_PREFIX_PATH` 查找 SDK，不写死用户名或 ROS 安装目录。如 SDK 安装在自定义位置，先将其安装前缀加入 `CMAKE_PREFIX_PATH`。新厂商 SDK 仍需在 `src/bridge/io_node.cpp` 增加枚举、开流、取帧和释放的实现。

```bash
cd /你的路径/qiji
./launch/build.sh
./launch/run.sh
```

`build.sh` 和 `run.sh` 优先使用 `QIJI_ROS_DISTRO` 或已加载的 ROS 环境；否则检测 `/opt/ros` 中的现有安装。构建输出位于本目录的 `build/`、`install/`、`log/`。`run.sh` 自动进入本目录，原图保存到 `samples/`，每次标定完成后在 `result/` 自动生成 `calibration_report_时间戳.md`，记录设置与阈值、逐图检测、九宫格覆盖、重投影误差和结果文件；其他标定结果保存在 `result/calibration/`。若只想手动启动，可在本目录加载 ROS 与 `install/setup.bash`，再运行 `ros2 launch danmucamera calibration.launch.py`。

构建输出中有 `RealSense SDK backend enabled` 或 `Orbbec SDK v2 backend enabled`，表示对应后端已编入。若未显示，检查 SDK 开发头文件、动态库和 CMake 搜索路径，然后清理本目录的 `build/danmucamera/` 并重新构建。

## 使用

刷新相机，选择设备与分辨率，点击“连接相机”。程序只有在 SDK 打开视频流并收到第一帧后才显示已连接、已启动；五秒无有效帧会报错并恢复未连接。关闭相机按钮只停流并保留设备选择；断开按钮清除连接。红外标定应关闭投射器，RGB、左右红外分别标定。导入图片仍可在不连接相机时使用。

如果设备出现在列表却无法打开，先检查 SDK 示例程序能否采流、所选分辨率和帧率是否受支持，以及设备是否被其他进程占用。相机采集代码在 `src/bridge/io_node.cpp`，ROS 话题和服务名在 `src/bridge/ros_contract.hpp`。

## 从网络 ROS 2 图像话题采集

相机与雷达外参的 Topic 输入角色、CameraInfo、时间戳和坐标系约定见 [ROS 相机雷达外参输入接口](docs/ROS相机雷达外参输入接口.md)。该接口已在配置和界面中声明；外参求解仍处于规划阶段。

在运行本程序的电脑上用 `ros2 topic list` 查看完整话题名，再用 `ros2 topic type /实际话题名` 确认格式。设备列表选择 `[ROS-TOPIC] ROS Camera` 后，在首页“图像话题”和“点云话题”输入完整名称（以 `/` 开头）。图像应为 `sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage`，点云应为 `sensor_msgs/msg/PointCloud2`。在“信息流”可单选一路或同时勾选 RGB 和 3D 点云；多选可在首页同时预览，进入内参标定前必须只勾选一条可标定流。点云视图支持拖动旋转、滚轮缩放。配置文件 `topic_input.name` 和 `topic_input.pointcloud_name` 只提供启动默认值；界面连接时会覆盖它们，无需每次修改 YAML。`topic_input.type: auto` 自动识别图像消息格式，也可指定 `raw` 或 `compressed`。点云需要小端 FLOAT32 的 x/y/z 字段；预览最多抽样约 12000 点。程序不会仅凭话题名称推断两路数据来自同一物理设备，填写成对的话题即表示用户指定它们为同一相机。当前不执行图像与点云的时间同步或投影配准。

图像和点云可只填写一路，也可填写两路而其中一路暂时不可用；至少一路必须可发现并收到数据。远端发布端须在启动采流前可被 ROS 2 发现。若 5 秒内没有收到图像或点云，程序会提示启动失败；采流期间连续 3 秒都无数据会断开。检查两端的 `ROS_DOMAIN_ID`、网络发现及 `ros2 topic info -v /实际话题名` 中的 QoS。此输入使用传感器数据 QoS。原始图像支持 `bgr8`、`rgb8`、`bgra8`、`rgba8`、`mono8`、`mono16` 和相应的 `8UC1`、`16UC1`；16 位单通道图像会转成 8 位供标定。收到的实际分辨率会作为当前标定分辨率，不按界面预设值缩放原图。

Topic 模式的硬件控制是可选协议：界面“驱动启停”同时填写命令和状态话题后，启动时发送 Bool `true`，收到驱动 `streaming` 状态并收到数据才算启动；停止时发送 `false` 并等待 `stopped`。两项留空时，启动/停止只控制本程序订阅，不操作外部相机。具体消息、QoS 和适配节点责任见 [ROS 相机雷达外参输入接口](docs/ROS相机雷达外参输入接口.md)。原有 Orbbec、RealSense 和 V4L2 路径继续由本程序直接控制。

自动引导拍摄入口暂时关闭，普通拍摄和批量导图仍可用。每张图的标定板像素面积占比必须落在 `config/calibration.yaml` 的 `quality.min_board_area_ratio` 与 `quality.max_board_area_ratio` 之间（当前为 15%～70%）；超出区间会标红，不能人工确认纳入。标定页侧栏将当前有效照片的角点位置按画面九宫格累计，每格显示角点数和占总角点数的比例；`calibration_ui.corner_coverage.min_ratio`（默认 5%）以下提示补拍该方位，`max_ratio`（默认 25%）以上提示该方位过于集中。修改配置后重启程序生效。拍摄预览、保存、批量新增和删除后都会重新统计。九宫格是拍摄指导，不单独阻止求解。
