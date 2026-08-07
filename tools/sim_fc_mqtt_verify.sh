#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 本地模拟 PX4 telem2 串口, 端到端验证飞控数据上传 MQTT
#
# 链路:
#   socat 虚拟串口
#     ├─ sim_px4.py (模拟 PX4 发 MAVLink)  -> /tmp/ttySimPX4
#     └─ MAVROS      (fcu_url=serial:///tmp/ttyMavros)
#                -> /mavros/* -> fc_monitor_node -> /fc/status -> cloud_node -> MQTT
#
# 用法:
#   ./tools/sim_fc_mqtt_verify.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# ===== 可配置项 =====
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
MAVROS_TTY="${MAVROS_TTY:-/tmp/ttyMavros}"
SIM_TTY="${SIM_TTY:-/tmp/ttySimPX4}"
BROKER_HOST="${BROKER_HOST:-localhost}"
BROKER_PORT="${BROKER_PORT:-1883}"
TOPIC="${MQTT_TOPIC:-lingyun01/telemetry}"
FC_RATE="${FC_RATE:-10}"

PID_FILE="/tmp/sim_fc_mqtt_verify.pid"

cleanup()
{
  echo
  echo "[sim_fc] 清理后台进程 ..."
  if [ -f "$PID_FILE" ]; then
    while read -r pid; do
      kill "$pid" 2>/dev/null || true
    done < "$PID_FILE"
    rm -f "$PID_FILE"
  fi
}
trap cleanup EXIT INT TERM

# ===== 1. 确保 broker 运行 =====
if command -v mosquitto >/dev/null 2>&1 && ! ss -ltn 2>/dev/null | grep -q ":$BROKER_PORT "; then
  echo "[INFO] 启动 mosquitto broker ..."
  sudo systemctl start mosquitto
  sleep 1
fi
echo "[OK]  broker 运行于 $BROKER_HOST:$BROKER_PORT"

# ===== 2. 创建虚拟串口对 =====
echo "[INFO] 创建 socat 虚拟串口对 ..."
bash "$SCRIPT_DIR/setup_sim_px4.sh"

: > "$PID_FILE"

# ===== 3. 启动 PX4 串口模拟器 =====
echo "[INFO] 启动 sim_px4.py (MAVLink@${FC_RATE}Hz) ..."
python3 "$SCRIPT_DIR/sim_px4.py" --device "$SIM_TTY" --rate "$FC_RATE" &
echo "$!" >> "$PID_FILE"

# ===== 4. 启动 ROS 节点(MAVROS -> fc_monitor -> cloud_node) =====
set +u
. "/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[ERROR] 未找到 $WS_DIR/install/setup.bash, 请先 colcon build"
  exit 1
fi
. "$WS_DIR/install/setup.bash"
set -u

echo "[INFO] 启动 MAVROS (fcu_url=serial://$MAVROS_TTY:921600) ..."
ros2 launch mavros px4.launch fcu_url:="serial://$MAVROS_TTY:921600" &
echo "$!" >> "$PID_FILE"

echo "[INFO] 启动 fc_monitor_node ..."
ros2 run airship_fc fc_monitor_node &
echo "$!" >> "$PID_FILE"

echo "[INFO] 启动 cloud_node (MQTT -> $BROKER_HOST:$BROKER_PORT) ..."
ros2 run airship_cloud cloud_node --ros-args \
  -p mqtt_host:="$BROKER_HOST" -p mqtt_port:="$BROKER_PORT" -p mqtt_topic:="$TOPIC" &
echo "$!" >> "$PID_FILE"

sleep 8
echo "============================================================"
echo "[INFO] 订阅 MQTT topic '$TOPIC' 验证飞控数据上传 (Ctrl+C 退出)"
echo "============================================================"
mosquitto_sub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "$TOPIC" -v