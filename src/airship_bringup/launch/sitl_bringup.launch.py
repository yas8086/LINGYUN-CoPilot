"""SITL仿真启动文件。

启动节点：
  - mavros_node: 与PX4 SITL通过UDP通信，fcu_url=udp://:14540@localhost:14557
  - airship_bridge (bridge_node): MAVLink <-> AirshipStatus / OffboardSetpoint 桥接
  - airship_safety (safety_monitor_node): 高度/姿态/电量/链路超时安全监控
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取airship_bringup包的share目录，用于定位参数配置文件
    bringup_share_dir = get_package_share_directory('airship_bringup')
    params_file = os.path.join(bringup_share_dir, 'config', 'airship_params.yaml')

    # MAVROS节点：SITL仿真默认通过UDP连接本机PX4 SITL实例
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            {
                # SITL默认MAVLink UDP端口
                'fcu_url': 'udp://:14540@localhost:14557',
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
        mavros_node,
        airship_bridge_node,
        offboard_controller_node,
        airship_safety_node,
    ])
