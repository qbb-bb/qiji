# 相机内参标定工作台

Linux ROS 2 + Qt 6 + OpenCV 标定程序。标定进程只接收 ROS 标准话题，不打开相机硬件，也不编译链接厂商 SDK。图像、点云发布端独立运行，可在本机或网络中的 Orin 上。

## 安装与启动

Ubuntu 22.04/24.04 图形桌面可运行：

```bash
./launch/setup_and_run.sh
```

已安装依赖时可分别运行 `./launch/build.sh` 和 `./launch/run.sh`。脚本会清理 Snap 注入的动态库路径；若从 Snap 版编辑器的终端启动失败，使用普通系统终端。新电脑上的构建产物 `build/`、`install/`、`log/` 不要跨机器复制。安装流程只准备 ROS 2、Qt 6 和 OpenCV，不安装相机 SDK。更多说明见 [换机编译与测试说明](docs/换机编译与测试说明.md)。

## 选择图像和点云

在线接收前先运行发布端，再点击界面“刷新 ROS 话题”。图像下拉框只列出有发布者的 `sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage`，点云下拉框只列出有发布者的 `sensor_msgs/msg/PointCloud2`；找不到对应格式就显示空选项。两者可只选一路，但都不可用时“连接”不能按。`config/calibration.yaml` 中的 `topic_input.name` 和 `topic_input.pointcloud_name` 只是启动默认值；只有它们已出现在 ROS 图中才自动选中。选择后在“信息流”勾选 RGB、3D 点云或两者，点击“连接”再点击“开始接收”。多路可同时预览；在线单目内参只允许单选图像流。点云支持鼠标旋转和缩放。

可选的“驱动启停”两个下拉框分别列出 `std_msgs/msg/Bool` 命令话题和 `std_msgs/msg/String` 状态话题。两项同时选择时，“开始接收”发送 `true` 并等待驱动确认；“停止接收”发送 `false`。两项留空时只控制标定程序的订阅，外部驱动自行运行。连接后变更话题可点击“应用话题”。消息、QoS 和外参预留接口见 [ROS 相机雷达外参输入接口](docs/ROS相机雷达外参输入接口.md)。

点云要求小端 `FLOAT32` 的 `x/y/z` 字段，预览最多抽样约 12000 点。图像支持 `bgr8`、`rgb8`、`bgra8`、`rgba8`、`mono8`、`mono16`，以及 H120H 等相机可能发布的 `yuv422`（UYVY）和 `yuv422_yuy2`。不同机器须能互相发现 ROS 2 话题，必要时检查 `ROS_DOMAIN_ID`、网络配置和 `ros2 topic info -v` 的 QoS。图像与点云的组合由用户选择，程序不从名称推断它们属于同一设备。

## 独立测试驱动

单独开一个终端运行本项目附带的简易 V4L2 发布端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run danmucamera topic_camera_driver --ros-args -p device:=/dev/video0 -p autostart:=true
```

它发布 `/qiji_test/image_raw`（`sensor_msgs/msg/Image`）、订阅 `/qiji_test/stream_command`（`std_msgs/msg/Bool`），并在 `/qiji_test/stream_status`（`std_msgs/msg/String`）报告 `stopped`、`streaming` 或 `error:原因`。可用 `-p image_file:=/绝对路径/图片.jpg` 代替真实相机循环发布静态图；不加 `autostart:=true` 时需在界面同时选中命令和状态话题，再按“开始接收”。该驱动只用于本机验证图像链路；实际雷达点云由你的独立发布端提供。可用 `-p image_topic:=/自定义名称` 等参数改名。

若系统使用 ROS 2 Jazzy，把上面命令中的 `humble` 改为 `jazzy`。

离线单目内参无需连接或启动话题：直接进入“内参标定”，仅显示导入图片、标定和结果相关操作。未接收视频时，首张可读取图片确定本次会话分辨率；后续多批导入和拍摄的图片都必须与之相同。接收视频时，导入图片须与当前流分辨率一致。不同尺寸的图片标红拒绝，开始求解前还会复查所有有效样本。重新开始标定会话会解除尺寸锁定。结果在 `result/calibration/`，检测报告在 `result/`。

### 广角镜头与鱼眼标定

对于 GMSL2-H120H（AR0233C、GW5200 ISP、H120H 镜头，标称水平视场 118°）的现有 1920×1080 样本，选择“鱼眼（等距模型）”及 K1–K4 或 K1–K2，勾选自动初值；棋盘格设置填写**内角点** 7×6。近期 18 张样本在 K1–K4 模型下实测 RMS 约 0.24 px。普通针孔 Brown 模型在这组样本上约为 10 px，不应仅因镜头视场小于 180°就改选针孔模型。程序对鱼眼焦距尝试多组自动初值，并比较固定与可优化 skew 的结果；只有可优化 skew 明显降低误差时才采用。误差图箭头按实际残差放大 32 倍，长度随误差大小变化；若 RMS 高于图像对角线的 0.1%（最低 1.5 px），会报错并拒绝保存结果，避免把未收敛的 K/D 当成标定成功。

鱼眼去畸变图默认保留较大的原始视场。把宽视场画面投影到同尺寸的平面图时，四周可能出现黑色无像素区域；这与输入图片被裁切不同。实际画面范围、棋盘格是否完整及直线是否恢复，应同时对照原图和去畸变图。标定时保持分辨率、ISP 设置和对焦状态一致，并尽量在棋盘与相机静止时采集，避免滚动快门运动误差。
