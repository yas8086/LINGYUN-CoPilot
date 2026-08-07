#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 创建虚拟 CAN 接口 vcan0
# 用于本地无硬件验证 bms/mppt/dcdc 节点解析逻辑。
# 需要 sudo; 重启后 vcan0 会消失, 再次运行本脚本即可。
set -e

IFACE="${CAN_INTERFACE:-vcan0}"

echo "[setup_vcan] 加载 vcan 内核模块..."
sudo modprobe vcan

if ip link show "$IFACE" >/dev/null 2>&1; then
  echo "[setup_vcan] $IFACE 已存在, 跳过创建"
else
  echo "[setup_vcan] 创建虚拟接口 $IFACE ..."
  sudo ip link add dev "$IFACE" type vcan
fi

sudo ip link set "$IFACE" up
echo "[setup_vcan] $IFACE 已就绪:"
ip link show "$IFACE" | head -1
echo
echo "下一步: 启动模拟器并运行 bms_node"
echo "  python3 tools/sim_bms.py --iface $IFACE --rate 5"
echo "  # 另开终端:"
echo "  ros2 run airship_bms bms_node --ros-args -p can_interface:=$IFACE -p cell_count:=102"
echo "  ros2 topic echo /bms/status"