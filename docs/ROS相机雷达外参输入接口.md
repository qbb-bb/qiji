# ROS Topic 相机与雷达外参输入接口

## 当前实现状态

本页定义 **ROS Topic 模式**下相机与雷达外参标定的输入接口。首页已可把用户选定的图像与点云作为同一组两条信息流，单选或同时勾选预览；选择“外参标定（规划中）”后可填写附加输入。**外参采样、配对、求解和结果发布尚未实现，界面的进入标定按钮保持禁用。** 双目和手眼仍是独立的规划项，本接口不替它们定义输入。

## 输入来源与配置

图像和点云由界面自动发现同类型话题并下拉选择，外参沿用同一组选择。配置文件为 `config/calibration.yaml`；界面选择在本次运行中覆盖相应参数，重启后重新读取配置默认值。

“刷新 ROS 话题”调用 `/calibration/camera/refresh_topics`（`std_srvs/srv/Trigger`）。节点扫描当前 ROS 图，响应 `success=true`，`message` 每行按 `完整话题名|界面标签|角色` 编码。角色为 `rgb`（Image/CompressedImage）、`depth_image`（原始 Image）、`depth`（PointCloud2）、`camera_info`（CameraInfo）、`command`（Bool）或 `status`（String）。没有匹配话题时 `message` 为空，界面只显示相应的空选项。已选择但刷新后消失的话题也回到空选项。

RGB、深度图、点云、CameraInfo 和状态话题须有发布者；命令话题须有订阅者。连接按钮只在所选数据话题至少有一路可发现时启用；都找不到时拒绝连接，开始接收时再检查发布者与实际数据。

| 角色 | ROS 参数（所属节点） | 消息类型 | 来源与要求 |
| --- | --- | --- | --- |
| 相机 RGB 图像 | `topic_input.name`（`/calibration_camera`） | `sensor_msgs/msg/Image` 或普通 `sensor_msgs/msg/CompressedImage` | 首页“RGB 图像”下拉框；信息流勾选 RGB。 |
| 深度图预览 | `topic_input.depth_image_name`（`/calibration_camera`） | `sensor_msgs/msg/Image`，`16UC1` 或 `32FC1` | 首页“深度图”下拉框；只用于预览，不作为相机内参或相机-雷达外参的 RGB 输入。 |
| 雷达点云 | `topic_input.pointcloud_name`（`/calibration_camera`） | `sensor_msgs/msg/PointCloud2` | 首页“点云话题”，完整绝对话题名；信息流勾选 3D 点云。 |
| 相机内参 | `inputs.extrinsic.camera_info`（`/calibration_algorithm_bridge`） | `sensor_msgs/msg/CameraInfo` | “相机与雷达外参 · Topic 输入”区域指定，与所选图像来自同一相机、同一分辨率。 |
| 图像/点云配对容差 | `inputs.extrinsic.max_pair_delta_ms`（`/calibration_algorithm_bridge`） | `double`，毫秒 | 正数，默认 50.0；未来配对时按原始 `header.stamp` 计算。 |

`src/bridge/ros_contract.hpp` 的 `extrinsic_input` 命名空间是上述参数名及消息类型的代码声明；`calibration_bridge.cpp` 声明附加参数。当前未为 CameraInfo 创建订阅，也未运行外参求解。

## 界面操作

1. 启动发布端并点击“刷新 ROS 话题”；必要时用 `ros2 topic list -t` 排查图像、点云和 CameraInfo 的消息格式。
2. 在首页选择 RGB 图像与点云，并在“信息流”**同时勾选 RGB 与 3D 点云**。深度图可独立选择并同时预览，但不参与相机-雷达外参输入。点击“连接”“开始接收”后查看画面。
3. 标定类型选“外参标定（规划中）”。从 CameraInfo 下拉框选话题并设配对容差，点击“应用外参输入”，将附加参数写入当前运行的标定节点。该操作不写回 YAML。

在线单目内参只接受单选的图像流；同时选图像和点云用于外参输入预览，不会绕过在线单目入口限制。没有连接且没有可用话题时也可进入离线单目内参，导入图片后标定；离线页面隐藏接收与拍摄控件。

## 原始消息约束

- 图像、点云、CameraInfo 的 `header.stamp` 必须处于同一时间基准。未来求解将使用原始消息时间戳，并按配置阈值配对；时间戳为零或不同机器时钟未同步的数据不可可靠配对。
- 图像 `header.frame_id` 应为相机光学坐标系；点云 `header.frame_id` 应为雷达坐标系。CameraInfo 的 `header.frame_id`、宽高、`K`、`D` 应与图像相机和分辨率一致。点云 `x/y/z` 应为米，采用小端 `FLOAT32` 或 `FLOAT64`，无效点可为 NaN；具体字段布局见 [点云与深度图前端适配](点云与深度图前端适配.md)。
- 外参求解需要原始图像和原始点云。现有 `/calibration/camera/image/compressed` 和 `/calibration/camera/streams/points` 仅供预览：它们会重新打时间戳，图像可能缩放/压缩，点云会抽样，**不能作为未来外参配对与求解的原始输入**。后续实现应订阅上表中的外部话题。
- 当前预览订阅采用 ROS 传感器数据 QoS（best effort、volatile、keep last），CameraInfo 的实际 QoS 接入与同步处理在外参算法实现时完成。

ROS Topic 的图像和点云在界面上归为同一组输入，是用户明确选择的输入组合；程序不会仅靠话题名称推断它们是否真的来自同一套相机与雷达。

## 可选驱动启停协议

Topic 模式默认只启停本程序的数据订阅。若要实际启停外部相机/雷达驱动，可在 `calibration_camera.topic_input` 的 `control_command` 与 `control_status` 设置两条不同的绝对话题名，也可在首页“驱动启停”下拉选择。两项须同时选择或同时留空。标定软件本身不打开相机硬件。

| 方向 | 消息类型与 QoS | 数据约定 |
| --- | --- | --- |
| 本程序 → 驱动适配节点：`control_command` | `std_msgs/msg/Bool`，reliable、volatile、深度 1 | `true` 启动硬件发布，`false` 停止。适配节点应按目标状态处理重复命令。 |
| 驱动适配节点 → 本程序：`control_status` | `std_msgs/msg/String`，reliable、transient_local、深度 1 | 原样发布 `connected`、`streaming`、`stopped`、`disconnected`，失败时 `error:原因`。适配节点应在状态改变后发布，并保留最新状态。 |

启动时本程序检查命令订阅者，发送 `true`，等待**新收到的** `streaming` 确认（最多 3 秒），再等待至少一条有效图像或点云（最多 5 秒）。驱动报告 `disconnected`/`error:原因` 或超时会使启动失败；无数据时会发 `false` 回退。停止时发送 `false`，随后关闭本程序订阅；首页“连接”表示本程序已保存话题配置，真正的驱动和数据确认发生在“开始接收”时。

这个协议由本项目定义，ROS 2 标准相机驱动不会自动实现它。要控制现有硬件驱动，需让独立驱动节点订阅命令并发布状态；本机独立的 `~/桌面/c/driver/realsense/` 和 `~/桌面/c/driver/v4l2/` 提供图像链路示例。控制话题留空即可直接消费已持续发布的图像/点云。
