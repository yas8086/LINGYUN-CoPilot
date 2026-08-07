"""
灵云01号伴飞电脑 设备监控启动文件.

编排节点:
  - mavros_node (mavros)      : 飞控 MAVLink 驱动 (fcu_url 可配置)
  - fc_monitor_node (airship_fc): 飞控数据聚合 -> /fc/status
  - bms_node   (airship_bms): 锂电池 BMS 驱动
  - mppt_node  (airship_mppt): MPPT 光伏控制器驱动
  - dcdc_node  (airship_dcdc): DCDC 电源模块驱动
  - monitor_node (airship_monitor): 设备监控聚合/告警
  - link_node  (airship_link): 串口数传链路 (下传 Qt 上位机)
  - cloud_node (airship_cloud): 4G MQTT 上云

支持参数:
  - fcu_url: MAVROS 飞控串口地址 (默认 CUAV X25 EVO telem2, 921600)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数文件路径
    bringup_share_dir = get_package_share_directory('airship_bringup')
    params_file = os.path.join(bringup_share_dir, 'config', 'airship_params.yaml')

    # 飞控串口地址 (CUAV X25 EVO telem2, 921600; 按实际接线调整)
    fcu_url = LaunchConfiguration('fcu_url')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyAMA1:921600',
        description='MAVROS 飞控串口地址',
    )

    # MAVROS 飞控驱动
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'px4.launch.py')
        ),
        launch_arguments={'fcu_url': fcu_url}.items(),
    )

    fc_monitor_node = Node(
        package='airship_fc',
        executable='fc_monitor_node',
        name='fc_monitor_node',
        parameters=[params_file],
        output='screen',
    )

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

    cloud_node = Node(
        package='airship_cloud',
        executable='cloud_node',
        name='cloud_node',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_launch,
        fc_monitor_node,
        bms_node,
        mppt_node,
        dcdc_node,
        monitor_node,
        link_node,
        cloud_node,
    ])
