#!/usr/bin/env bash
# 灵云01号伴飞电脑 — LoRa 串口掉线/重连 端到端验证脚本
#
# 链路: sim_concentrator(从机) <-> socat虚拟串口对 <-> lora_node(ROS节点)
#
# 用法:
#   ./tools/verify_lora_reconnect.sh up          # 启动: socat + 从机 + lora_node
#   ./tools/verify_lora_reconnect.sh drop        # 模拟掉线: 杀 socat
#   ./tools/verify_lora_reconnect.sh restore     # 模拟恢复: 重启 socat
#   ./tools/verify_lora_reconnect.sh status      # 查看各进程与 summary
#   ./tools/verify_lora_reconnect.sh down        # 停止全部
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

SIM_TTY=/tmp/ttySimPX4      # lora_node 侧
CONC_TTY=/tmp/ttyConcentrator  # 从机侧
SIM_LOG=/tmp/sim_conc.log
LORA_LOG=/tmp/lora_node.log
SOCAT_LOG=/tmp/socat_lora.log
ROS_LOG=/tmp/roslog

start_socat() {
  rm -f "$SIM_TTY" "$CONC_TTY"
  setsid socat -d -d pty,raw,echo=0,mode=666,link="$SIM_TTY" \
    pty,raw,echo=0,mode=666,link="$CONC_TTY" >"$SOCAT_LOG" 2>&1 < /dev/null &
  disown
  sleep 2
  echo "[socat] 虚拟串口对已创建: $SIM_TTY <-> $CONC_TTY"
}

stop_all() {
  # 用 PID 文件精确清理, 避免误杀
  [ -f "$ROS_LOG/lora.pid" ] && kill "$(cat "$ROS_LOG/lora.pid")" 2>/dev/null || true
  [ -f "$ROS_LOG/conc.pid" ] && kill "$(cat "$ROS_LOG/conc.pid")" 2>/dev/null || true
  [ -f "$ROS_LOG/socat.pid" ] && kill "$(cat "$ROS_LOG/socat.pid")" 2>/dev/null || true
  sleep 1
  rm -f "$ROS_LOG"/*.pid "$SIM_TTY" "$CONC_TTY"
  echo "[stop] 已停止全部进程"
}

up() {
  stop_all
  start_socat
  echo "$(pgrep -x socat | head -1)" > "$ROS_LOG/socat.pid"

  set +u
  . "/opt/ros/$ROS_DISTRO/setup.bash"
  . "$WS_DIR/install/setup.bash"
  set -u
  export ROS_LOG_DIR="$ROS_LOG"
  mkdir -p "$ROS_LOG"

  echo "[up] 启动从机模拟器..."
  setsid python3 "$SCRIPT_DIR/sim_concentrator.py" --device "$CONC_TTY" > "$SIM_LOG" 2>&1 < /dev/null &
  disown
  sleep 1
  echo "$(pgrep -f sim_concentrator.py | head -1)" > "$ROS_LOG/conc.pid"

  echo "[up] 启动 lora_node (连 $SIM_TTY)..."
  setsid ros2 run airship_lora lora_node --ros-args \
    -p serial_device:="$SIM_TTY" \
    -p node_ids:="[1,2,6]" \
    -p node_types:="[\"temperature\",\"temperature\",\"pressure\"]" \
    -p resp_timeout_ms:=1000 > "$LORA_LOG" 2>&1 < /dev/null &
  disown
  sleep 5
  echo "$(pgrep -x lora_node | head -1)" > "$ROS_LOG/lora.pid"
  echo "[up] 完成。运行 status 查看结果"
}

drop() {
  local spid
  spid="$(cat "$ROS_LOG/socat.pid" 2>/dev/null || pgrep -x socat | head -1)"
  echo "[drop] 杀掉 socat($spid) 模拟掉线..."
  kill "$spid" 2>/dev/null || true
}

restore() {
  echo "[restore] 重启 socat 模拟恢复..."
  start_socat
  echo "$(pgrep -x socat | head -1)" > "$ROS_LOG/socat.pid"
}

status() {
  echo "== 进程状态 =="
  echo "socat:      $(pgrep -x socat >/dev/null && echo RUNNING || echo DOWN)"
  echo "从机:        $(pgrep -f sim_concentrator.py >/dev/null && echo RUNNING || echo DOWN)"
  echo "lora_node:  $(pgrep -x lora_node >/dev/null && echo RUNNING || echo DOWN)"
  echo
  echo "== lora_node 日志(最近5条) =="
  tail -5 "$LORA_LOG" 2>/dev/null || echo "(无)"
  echo
  echo "== summary =="
  set +u
  . "/opt/ros/$ROS_DISTRO/setup.bash"
  . "$WS_DIR/install/setup.bash"
  set -u
  export ROS_LOG_DIR="$ROS_LOG"
  timeout 4 ros2 topic echo /lora/summary --once 2>/dev/null | grep -E "serial_online|node_count|online_count|avg_temp" || echo "(无 summary)"
}

case "${1:-}" in
  up)   up ;;
  drop) drop ;;
  restore) restore ;;
  status) status ;;
  down) stop_all ;;
  *) echo "用法: $0 {up|drop|restore|status|down}"; exit 1 ;;
esac