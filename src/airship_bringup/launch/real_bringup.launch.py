"""实飞启动文件（树莓派5伴飞电脑）。

通过launch参数fcu_url指定飞控连接方式：
  - 以太网UDP（默认）: fcu_url:=udp://@<飞控IP>
  - 串口连接:          fcu_url:=serial:///dev/ttyAMA0:921600

启动节点：
  - mavros_node: 按传入的fcu_url连接飞控
  - airship_bridge (bridge_node): MAVLink <-> AirshipStatus / OffboardSetpoint 桥接
  - airship_safety (safety_monitor_node): 高度/姿态/电量/链路超时安全监控
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取airship_bringup包的share目录，用于定位参数配置文件
    bringup_share_dir = get_package_share_directory('airship_bringup')
    params_file = os.path.join(bringup_share_dir, 'config', 'airship_params.yaml')

    # fcu_url launch参数：默认以太网UDP连接飞控，可覆盖为串口URL
    # 示例: ros2 launch airship_bringup real_bringup.launch.py fcu_url:=serial:///dev/ttyAMA0:921600
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://@192.168.1.10',
        description='飞控MAVLink连接URL，例如 udp://@<飞控IP> 或 serial:///dev/ttyAMA0:921600',
    )

    # MAVROS节点：使用传入的fcu_url连接定制PX4飞控
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                # 目标飞控系统ID
                'tgt_system': 1,
            }
        ],
        output='screen',
    )

    # 桥接节点：将MAVLink遥测聚合为AirshipStatus，转发OffboardSetpoint
    airship_bridge_node = Node(
        package='airship_bridge',
        executable='bridge_node',
        name='airship_bridge',
        parameters=[params_file],
        output='screen',
    )

    # Offboard控制器节点：位置/速度控制，飞艇专属约束处理
    offboard_controller_node = Node(
        package='airship_control',
        executable='offboard_controller',
        name='offboard_controller',
        parameters=[params_file],
        output='screen',
    )

    # 安全监控节点：参数文件按节点名airship_safety_monitor分组加载
    airship_safety_node = Node(
        package='airship_safety',
        executable='safety_monitor_node',
        name='airship_safety_monitor',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node,
        airship_bridge_node,
        offboard_controller_node,
        airship_safety_node,
    ])
