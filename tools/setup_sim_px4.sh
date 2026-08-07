#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 创建 socat 虚拟串口对, 用于模拟 PX4 telem2 串口
#
# 生成一对互相桥接的虚拟串口:
#   /tmp/ttyMavros  -> MAVROS 连接端(fcu_url)
#   /tmp/ttySimPX4  -> 模拟器写端(sim_px4.py)
#
# 用法:
#   ./tools/setup_sim_px4.sh
set -euo pipefail

MAVROS_TTY="${MAVROS_TTY:-/tmp/ttyMavros}"
SIM_TTY="${SIM_TTY:-/tmp/ttySimPX4}"
SOCAT_PID_FILE="/tmp/sim_px4.socat.pid"

if [ -f "$SOCAT_PID_FILE" ] && kill -0 "$(cat "$SOCAT_PID_FILE")" 2>/dev/null; then
  echo "[setup_sim_px4] socat 虚拟串口已运行"
else
  echo "[setup_sim_px4] 创建虚拟串口对 ..."
  nohup socat -d -d \
    pty,raw,echo=0,link="$MAVROS_TTY" \
    pty,raw,echo=0,link="$SIM_TTY" \
    >/tmp/sim_px4.socat.log 2>&1 &
  echo "$!" > "$SOCAT_PID_FILE"
  sleep 1
fi

echo "[setup_sim_px4] MAVROS 端: $MAVROS_TTY"
echo "[setup_sim_px4] 模拟器端: $SIM_TTY"
ls -l "$MAVROS_TTY" "$SIM_TTY" 2>/dev/null || {
  echo "[setup_sim_px4] 创建失败, 查看 /tmp/sim_px4.socat.log"
  exit 1
}