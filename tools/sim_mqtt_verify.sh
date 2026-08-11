#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 本地模拟 MQTT 服务器, 验证 cloud_node 4G 数据上传逻辑
#
# 流程:
#   1. (仅本地 broker 时) 确保本地 mosquitto broker 运行
#   2. 配置虚拟 CAN 接口 vcan0
#   3. 启动 BMS 模拟器 (sim_bms.py) 产生 CAN 数据
#   4. 启动 bms_node (解析 vcan0 -> /bms/status) + cloud_node (MQTT 上传)
#   5. 前台订阅 <topic> 展示上传的 JSON 帧
#
# 用法:
#   # 连接本地明文 broker (默认)
#   ./tools/sim_mqtt_verify.sh
#
#   # 连接 EMQX Cloud Serverless (TLS 8883)
#   BROKER_HOST=xxx.aws.emea.emqx.cloud \
#     BROKER_PORT=8883 \
#     BROKER_USE_TLS=1 \
#     BROKER_USERNAME=lingyun01_onboard \
#     BROKER_PASSWORD='your_password' \
#     TOPIC=lingyun01/telemetry \
#     ./tools/sim_mqtt_verify.sh
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
# TLS / 认证
BROKER_USE_TLS="${BROKER_USE_TLS:-0}"              # 1=启用 TLS (EMQX Cloud Serverless 必须)
BROKER_CAFILE="${BROKER_CAFILE:-}"                 # 空=用系统 CA 目录
BROKER_INSECURE="${BROKER_INSECURE:-0}"            # 1=跳过证书主机名校验(调试)
BROKER_USERNAME="${BROKER_USERNAME:-}"             # MQTT 用户名
BROKER_PASSWORD="${BROKER_PASSWORD:-}"             # MQTT 密码

# 是否需要启动本地 broker: 只有 host=localhost 且没有账号密码时才启动
NEED_LOCAL_BROKER=0
if [[ "$BROKER_HOST" == "localhost" || "$BROKER_HOST" == "127.0.0.1" ]]; then
  if [[ -z "$BROKER_USERNAME" && "$BROKER_USE_TLS" != "1" ]]; then
    NEED_LOCAL_BROKER=1
  fi
fi

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

# ===== 组装 mosquitto_sub / cloud_node 公共参数 =====
SUB_ARGS=(-h "$BROKER_HOST" -p "$BROKER_PORT" -t "$TOPIC" -v)
CLOUD_EXTRA_ARGS=()

if [[ "$BROKER_USE_TLS" == "1" ]]; then
  if [[ -n "$BROKER_CAFILE" ]]; then
    SUB_ARGS+=(--cafile "$BROKER_CAFILE")
    CLOUD_EXTRA_ARGS+=(-p mqtt_tls_ca_cert:="$BROKER_CAFILE")
  else
    # 未指定 cafile 时, mosquitto_sub 使用系统 CA 目录
    SUB_ARGS+=(--capath /etc/ssl/certs)
  fi
  if [[ "$BROKER_INSECURE" == "1" ]]; then
    SUB_ARGS+=(--insecure)
    CLOUD_EXTRA_ARGS+=(-p mqtt_tls_insecure:=true)
  fi
  CLOUD_EXTRA_ARGS+=(-p mqtt_tls_enable:=true)
else
  CLOUD_EXTRA_ARGS+=(-p mqtt_tls_enable:=false)
fi

if [[ -n "$BROKER_USERNAME" ]]; then
  SUB_ARGS+=(-u "$BROKER_USERNAME" -P "$BROKER_PASSWORD")
  # 密码经环境变量传给 cloud_node, 避免出现在进程 cmdline (仅传用户名到命令行)
  CLOUD_EXTRA_ARGS+=(-p mqtt_username:="$BROKER_USERNAME")
  if [[ -n "$BROKER_PASSWORD" ]]; then
    export MQTT_PASSWORD="$BROKER_PASSWORD"
  fi
fi

# ===== 1. (仅本地 broker 时) 确保 mosquitto broker 运行 =====
if ! command -v mosquitto >/dev/null 2>&1; then
  echo "[ERROR] 未找到 mosquitto, 请先安装:"
  echo "  sudo apt install mosquitto mosquitto-clients"
  exit 1
fi

if [[ "$NEED_LOCAL_BROKER" == "1" ]]; then
  if ! ss -ltn 2>/dev/null | grep -q ":$BROKER_PORT "; then
    echo "[INFO] 启动 mosquitto broker ($BROKER_HOST:$BROKER_PORT) ..."
    sudo systemctl start mosquitto
    sleep 1
  fi
  echo "[OK]  本地 broker 运行于 $BROKER_HOST:$BROKER_PORT"
else
  echo "[INFO] 目标 broker 为远端 ($BROKER_HOST:$BROKER_PORT), 跳过本地 mosquitto 启动"
fi

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

echo "[INFO] 启动 cloud_node (MQTT -> $BROKER_HOST:$BROKER_PORT, tls=$BROKER_USE_TLS, topic=$TOPIC) ..."
ros2 run airship_cloud cloud_node --ros-args \
  -p mqtt_host:="$BROKER_HOST" -p mqtt_port:="$BROKER_PORT" -p mqtt_topic:="$TOPIC" \
  "${CLOUD_EXTRA_ARGS[@]}" &
echo "$!" >> "$PID_FILE"

sleep 5
echo "============================================================"
echo "[INFO] 订阅 MQTT topic '$TOPIC' 验证上传数据 (Ctrl+C 退出)"
echo "============================================================"
mosquitto_sub "${SUB_ARGS[@]}"