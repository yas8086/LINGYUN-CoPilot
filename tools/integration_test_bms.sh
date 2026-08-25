#!/usr/bin/env bash
# 灵云01号伴飞电脑 — BMS 链路端到端集成测试
#
# 验证"节点注册 + 话题出数据 + 离线兜底"一条完整链路:
#   online  : 起 bms_node(接 vcan0) + sim_bms 发帧 -> 断言 /bms/status 出现 online: true
#   offline : 仅起 bms_node 不发帧 -> 断言 /bms/status 出现 online: false(超时离线兜底)
#
# 用法: bash tools/integration_test_bms.sh [online|offline]
# 前提: vcan0 已存在(tools/setup_vcan.sh; CI Test 前已建 vcan0)。
#
# 为什么用独立脚本而非 launch_testing: launch_testing 测试进程与本工程 rclpy 存在
# env/rcl-context 兼容问题(重复 init / PYTHONPATH 缺失), 用 ros2 topic echo 的
# 独立 DDS 进程断言最稳、CI/本机通用。
# 注意: 不能加 -u(Jazzy 的 /opt/ros/jazzy/setup.bash 在 set -u 下会报 unbound variable)
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
MODE="${1:-online}"

# ---- source ROS 与工作区 ----
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[integration] ERROR: 未找到 /opt/ros/jazzy/setup.bash"; exit 1
fi
source /opt/ros/jazzy/setup.bash
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[integration] ERROR: 未构建工作区($WS_DIR/install)"; exit 1
fi
source "$WS_DIR/install/setup.bash"

# 使用独立 DDS domain, 避免与正在运行的生产 device-monitor(domain 5)竞争 participant;
# 测试自带 bms_node + ros2 topic echo 同处该域, 内部通信不受影响。
# (可通过环境变量 INTEG_TEST_DOMAIN 覆盖; 生产域默认 5, 测试用 20 与其隔离)
export ROS_DOMAIN_ID="${INTEG_TEST_DOMAIN:-20}"

cleanup() {
  [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null || true
  [ -n "${BMS_PID:-}" ] && kill "$BMS_PID" 2>/dev/null || true
}
trap cleanup EXIT

SIM_PID=""
BMS_PID=""

run_with_sim() {  # online 模式: 后台 sim_bms 发帧
  python3 "$SCRIPT_DIR/sim_bms.py" --iface vcan0 --rate 5 &
  SIM_PID=$!
  sleep 1
}

echo "[integration] 启动 bms_node(vcan0) ..."
ros2 run airship_bms bms_node \
  --ros-args -p can_interface:=vcan0 -p cell_count:=102 \
  -p link_timeout_s:=1.0 -p pub_interval_s:=0.2 &
BMS_PID=$!
sleep 2

if [ "$MODE" = "online" ]; then
  run_with_sim
  echo "[integration] 收 /bms/status 期望 online=true ..."
  # -k 5: 10s 到期后若 echo 未响应 SIGTERM 则再 5s 强制 KILL, 防 DDS 进程挂死不返回
  OUTPUT=$(timeout -k 5 10 ros2 topic echo --once /bms/status 2>&1) || {
    echo "[integration] FAIL: 未收到 /bms/status(节点未注册或未出数据)"; exit 1
  }
  echo "$OUTPUT"
  if echo "$OUTPUT" | grep -q "online: true"; then
    echo "[integration] PASS: 收到 online=true"
  else
    echo "[integration] FAIL: 期望 online=true"; exit 1
  fi
elif [ "$MODE" = "offline" ]; then
  echo "[integration] 不发帧, 收 /bms/status 期望 online=false(离线兜底) ..."
  # -k 5: 10s 到期后若 echo 未响应 SIGTERM 则再 5s 强制 KILL, 防 DDS 进程挂死不返回
  OUTPUT=$(timeout -k 5 10 ros2 topic echo --once /bms/status 2>&1) || {
    echo "[integration] FAIL: 未收到 /bms/status 离线兜底消息"; exit 1
  }
  echo "$OUTPUT"
  if echo "$OUTPUT" | grep -q "online: false"; then
    echo "[integration] PASS: 收到 online=false"
  else
    echo "[integration] FAIL: 期望 online=false"; exit 1
  fi
else
  echo "[integration] 未知模式: $MODE (应为 online|offline)"; exit 1
fi

echo "[integration] PASSED"