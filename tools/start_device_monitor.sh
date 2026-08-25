#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 一键启动所有设备监控节点
#
# 功能:
#   1. 自动 source ROS2 环境与工作区
#   2. 自动配置 CAN 接口 (can0, 默认 250kbps)
#   3. 启动 device_monitor.launch.py 编排的全部监控节点
#
# 用法:
#   ./tools/start_device_monitor.sh            # 默认 can0 / 250kbps
#   CAN_INTERFACE=can1 ./tools/start_device_monitor.sh
#   CAN_BITRATE=500000 ./tools/start_device_monitor.sh
set -euo pipefail

# ===== 路径 =====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# ===== 可配置项 =====
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-250000}"

# ===== 1. source ROS2 环境 =====
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[ERROR] 未找到 ROS2 环境: /opt/ros/$ROS_DISTRO"
  exit 1
fi
source "/opt/ros/$ROS_DISTRO/setup.bash"

# ===== 2. source 工作区 =====
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[ERROR] 未找到 $WS_DIR/install/setup.bash"
  echo "       请先构建工程: colcon build"
  exit 1
fi
source "$WS_DIR/install/setup.bash"

# ===== 3. 配置 CAN 接口 =====
# 生产环境 CAN 初始化由 systemd 的 airship-can.service 负责(单一机制)。
# 此处的 ip link 仅为纯开发手动启动(未运行 systemd 服务)场景的便捷配置,
# 波特率优先读取统一配置源 /etc/default/airship-can, 与 airship-can.service 保持一致。
if [ -f /etc/default/airship-can ]; then
  # shellcheck disable=SC1091
  . /etc/default/airship-can
  CAN_BITRATE="${CAN0_BITRATE:-${CAN_BITRATE:-250000}}"
fi
if command -v ip >/dev/null 2>&1; then
  if ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
    if ! ip link show "$CAN_INTERFACE" | grep -q "state UP"; then
      echo "[INFO] 配置 $CAN_INTERFACE @ ${CAN_BITRATE}bps ..."
      sudo ip link set "$CAN_INTERFACE" up type can bitrate "$CAN_BITRATE"
    else
      echo "[INFO] $CAN_INTERFACE 已启用"
    fi
  else
    echo "[WARN] CAN 接口 $CAN_INTERFACE 不存在, 节点可启动但无设备数据"
    echo "       请确认 USB-CAN 适配器已接入(如 can0)"
  fi
else
  echo "[WARN] 未找到 ip 命令, 跳过 CAN 接口配置"
fi

# ===== 4. 启动监控节点 =====
echo "[INFO] 启动设备监控节点 ... (Ctrl+C 退出)"
ros2 launch airship_bringup device_monitor.launch.py