#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 本地模拟 MQTT 服务器, 验证 cloud_node 4G 数据上传逻辑
#
# 流程:
#   1. 确保本地 mosquitto broker 运行 (默认 localhost:1883)
#   2. 配置虚拟 CAN 接口 vcan0
#   3. 启动 BMS 模拟器 (sim_bms.py) 产生 CAN 数据
#   4. 启动 bms_node (解析 vcan0 -> /bms/status) + cloud_node (MQTT 上传)
#   5. 前台订阅 <topic> 展示上传的 JSON 帧
#
# 用法:
#   ./tools/sim_mqtt_verify.sh
#   TOPIC=lingyun01/telemetry BROKER_PORT=1883 ./tools/sim_mqtt_verify.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# ===== 可配置项 =====
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
IFACE="${CAN_INTERFACE:-vcan0}"
BROKER_HOST="${BROKER_HOST:-localhost}"
BROKER_PORT="${BROKER_PORT:-1883}"
TOPIC="${MQTT_TOPIC:-lingyun01/telemetry}"
BMS_RATE="${BMS_RATE:-5}"

PID_FILE="/tmp/sim_mqtt_verify.pid"

# ===== 清理后台进程 =====
cleanup()
{
  echo
  echo "[sim_mqtt] 清理后台进程 ..."
  if [ -f "$PID_FILE" ]; then
    while read -r pid; do
      kill "$pid" 2>/dev/null || true
    done < "$PID_FILE"
    rm -f "$PID_FILE"
  fi
}
trap cleanup EXIT INT TERM

# ===== 1. 确保 mosquitto broker 运行 =====
if ! command -v mosquitto >/dev/null 2>&1; then
  echo "[ERROR] 未找到 mosquitto, 请先安装:"
  echo "  sudo apt install mosquitto mosquitto-clients"
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ":$BROKER_PORT "; then
  echo "[INFO] 启动 mosquitto broker ($BROKER_HOST:$BROKER_PORT) ..."
  sudo systemctl start mosquitto
  sleep 1
fi
echo "[OK]  broker 运行于 $BROKER_HOST:$BROKER_PORT"

# ===== 2. 配置虚拟 CAN 接口 =====
echo "[INFO] 配置虚拟 CAN 接口 $IFACE ..."
sudo bash "$SCRIPT_DIR/setup_vcan.sh" >/dev/null
: > "$PID_FILE"

# ===== 3. 启动 BMS 模拟器 =====
echo "[INFO] 启动 BMS 模拟器 ($IFACE, ${BMS_RATE}Hz) ..."
python3 "$SCRIPT_DIR/sim_bms.py" --iface "$IFACE" --rate "$BMS_RATE" &
echo "$!" >> "$PID_FILE"

# ===== 4. 启动 ROS 节点 =====
# ROS2 setup.bash 在 set -u(strict) 下会因未定义变量报错, 故临时关闭
set +u
. "/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[ERROR] 未找到 $WS_DIR/install/setup.bash, 请先执行 colcon build"
  exit 1
fi
. "$WS_DIR/install/setup.bash"
set -u

echo "[INFO] 启动 bms_node (can_interface=$IFACE) ..."
ros2 run airship_bms bms_node --ros-args \
  -p can_interface:="$IFACE" -p cell_count:=102 &
echo "$!" >> "$PID_FILE"

echo "[INFO] 启动 cloud_node (MQTT -> $BROKER_HOST:$BROKER_PORT, topic=$TOPIC) ..."
ros2 run airship_cloud cloud_node --ros-args \
  -p mqtt_host:="$BROKER_HOST" -p mqtt_port:="$BROKER_PORT" -p mqtt_topic:="$TOPIC" &
echo "$!" >> "$PID_FILE"

sleep 3
echo "============================================================"
echo "[INFO] 订阅 MQTT topic '$TOPIC' 验证上传数据 (Ctrl+C 退出)"
echo "============================================================"
mosquitto_sub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "$TOPIC" -v