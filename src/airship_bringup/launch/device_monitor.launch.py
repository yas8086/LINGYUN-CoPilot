"""
灵云01号伴飞电脑 设备监控启动文件.

编排节点:
  - bms_node   (airship_bms):   锂电池 BMS 驱动
  - mppt_node  (airship_mppt):  MPPT 光伏控制器驱动
  - dcdc_node  (airship_dcdc):  DCDC 电源模块驱动
  - monitor_node (airship_monitor): 设备监控聚合/告警
  - link_node  (airship_link):  串口数传链路 (下传 Qt 上位机)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 参数文件路径
    bringup_share_dir = get_package_share_directory('airship_bringup')
    params_file = os.path.join(bringup_share_dir, 'config', 'airship_params.yaml')

    bms_node = Node(
        package='airship_bms',
        executable='bms_node',
        name='bms_node',
        parameters=[params_file],
        output='screen',
    )

    mppt_node = Node(
        package='airship_mppt',
        executable='mppt_node',
        name='mppt_node',
        parameters=[params_file],
        output='screen',
    )

    dcdc_node = Node(
        package='airship_dcdc',
        executable='dcdc_node',
        name='dcdc_node',
        parameters=[params_file],
        output='screen',
    )

    monitor_node = Node(
        package='airship_monitor',
        executable='monitor_node',
        name='monitor_node',
        parameters=[params_file],
        output='screen',
    )

    link_node = Node(
        package='airship_link',
        executable='link_node',
        name='link_node',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        bms_node,
        mppt_node,
        dcdc_node,
        monitor_node,
        link_node,
    ])
