from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """同时启动相机、算法桥与 UI；三者仅通过 ROS 2 接口连接。"""
    package_share = Path(get_package_share_directory("danmucamera"))
    parameters = str(package_share / "config" / "calibration.yaml")

    io_node = Node(
        package="danmucamera",
        executable="io_node",
        name="calibration_camera",
        parameters=[parameters],
        output="screen",
    )
    bridge_node = Node(
        package="danmucamera",
        executable="calibration_bridge",
        name="calibration_algorithm_bridge",
        parameters=[parameters],
        output="screen",
    )
    ui_node = Node(
        package="danmucamera",
        executable="calibration_ui",
        name="calibration_ui",
        parameters=[parameters],
        output="screen",
    )

    # 桌面图标没有终端可用于 Ctrl+C。用户关闭 Qt 主窗口时，
    # 让 launch 同步停止 I/O 和算法桥，避免留下占用相机的后台进程。
    stop_when_ui_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=ui_node,
            on_exit=[EmitEvent(event=Shutdown(reason="calibration UI closed"))],
        )
    )

    return LaunchDescription([io_node, bridge_node, ui_node, stop_when_ui_closes])
