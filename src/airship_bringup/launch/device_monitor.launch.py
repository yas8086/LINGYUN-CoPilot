"""
灵云01号伴飞电脑 设备监控启动文件.

编排节点:
  - mavros_node (mavros)      : 飞控 MAVLink 驱动 (fcu_url 可配置)
  - fc_monitor_node (airship_fc): 飞控数据聚合 -> /fc/status
  - bms_node   (airship_bms): 锂电池 BMS 驱动
  - backup_bms_node (airship_backup_bms): 12S 备用电源 BMS 驱动 (串口)
  - mppt_node  (airship_mppt): MPPT 光伏控制器驱动
  - dcdc_node  (airship_dcdc): DCDC 电源模块驱动
  - lora_node  (airship_lora): LoRa 温度/压力采集 (485 Modbus)
  - bladder_bridge_node (airship_fc): 气囊压差桥 (LoRa -> uXRCE-DDS -> PX4)
  - monitor_node (airship_monitor): 设备监控聚合/告警
  - safety_node (airship_safety): 安全仲裁, 发布 safe_to_control
  - link_node  (airship_link): 串口数传链路 (下传 Qt 上位机)
  - cloud_node (airship_cloud): 4G MQTT 上云

支持参数:
  - fcu_url: MAVROS 飞控串口地址 (默认 CUAV X25 EVO telem2, 921600)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数文件路径
    bringup_share_dir = get_package_share_directory('airship_bringup')
    params_file = os.path.join(bringup_share_dir, 'config', 'airship_params.yaml')

    # 飞控连接地址: 通过网口直连飞控 ETH (UDP 14550)
    # 飞控侧需在 SD 卡 net.cfg 配置静态 IP 10.41.10.2 (BOOTPROTO=static)
    # 树莓派 eth0 静态 IP 10.41.10.100/24, 同网段直连, 不设网关
    fcu_url = LaunchConfiguration('fcu_url')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@10.41.10.2:14550',
        description='MAVROS 飞控连接地址 (网口 UDP)',
    )

    # MAVROS 飞控驱动 (直接启动 mavros_node, 配置插件与 fcu_url)
    mavros_share = get_package_share_directory('mavros')
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        parameters=[
            {'fcu_url': fcu_url},
            {'gcs_url': ''},
            {'tgt_system': 1},
            {'tgt_component': 1},
            {'fcu_protocol': 'v2.0'},
            os.path.join(mavros_share, 'launch', 'px4_pluginlists.yaml'),
            os.path.join(mavros_share, 'launch', 'px4_config.yaml'),
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    fc_monitor_node = Node(
        package='airship_fc',
        executable='fc_monitor_node',
        name='fc_monitor_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    bms_node = Node(
        package='airship_bms',
        executable='bms_node',
        name='bms_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    backup_bms_node = Node(
        package='airship_backup_bms',
        executable='backup_bms_node',
        name='backup_bms_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    mppt_node = Node(
        package='airship_mppt',
        executable='mppt_node',
        name='mppt_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # 第二台 MPPT: 副囊光伏板 (设备地址 0x02), 独立话题 /mppt2/status
    mppt2_node = Node(
        package='airship_mppt',
        executable='mppt_node',
        name='mppt2_node',
        parameters=[
            params_file,
            {'device_addr': 2, 'topic_name': '/mppt2/status'},
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    dcdc_node = Node(
        package='airship_dcdc',
        executable='dcdc_node',
        name='dcdc_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )

    lora_node = Node(
        package='airship_lora',
        executable='lora_node',
        name='lora_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # 气囊压差桥: 订阅 /lora/samples, 发布 px4_msgs/AirshipBladderPressure
    # 到 /fmu/in/airship_bladder_pressure (经 MicroXRCEAgent 进 PX4, 见文档 08)
    bladder_bridge_node = Node(
        package='airship_fc',
        executable='bladder_bridge_node',
        name='bladder_bridge_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    monitor_node = Node(
        package='airship_monitor',
        executable='monitor_node',
        name='monitor_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    safety_node = Node(
        package='airship_safety',
        executable='safety_node',
        name='safety_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    link_node = Node(
        package='airship_link',
        executable='link_node',
        name='link_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    cloud_node = Node(
        package='airship_cloud',
        executable='cloud_node',
        name='cloud_node',
        parameters=[params_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node,
        fc_monitor_node,
        bms_node,
        backup_bms_node,
        mppt_node,
        mppt2_node,
        dcdc_node,
        lora_node,
        bladder_bridge_node,
        monitor_node,
        safety_node,
        link_node,
        cloud_node,
    ])
