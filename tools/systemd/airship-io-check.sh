#!/bin/bash
# 灵云01号 伴飞电脑 —— RS232-RS485-CAN-Board 全接口自检脚本
#
# 用法: sudo ./airship-io-check.sh
# 需在扩展板插上并重启后运行，逐项检查：
#   CAN    can0 (MCP2515) / can1 (MCP2518FD)
#   RS485  /dev/ttySC0, /dev/ttySC1 (SC16IS752)
#   RS232  /dev/ttyAMA0 (板载 UART)
#   SPI    spidev0.0 / spidev0.1
#   GPIO   INT 引脚 (23/24/25)

PASS=0
FAIL=0

ok()   { echo "  [ OK ] $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }

echo "========== RS232-RS485-CAN-Board 接口自检 =========="
echo "时间: $(date)"

# ---------- 1. 内核 dtoverlay 加载 ----------
echo "--- 1. 内核设备树覆盖 ---"
if dmesg | grep -qiE "mcp2515|mcp2518|sc16is752"; then
    ok "内核已加载 CAN/RS485 控制器驱动"
    dmesg | grep -iE "mcp2515|mcp2518|sc16is752" | tail -5
else
    bad "未检测到 CAN/RS485 驱动，请确认扩展板已插好并重启"
fi

# ---------- 2. CAN 接口 ----------
# 只读探测: 不修改接口配置。旧实现曾用 `ip link set ... bitrate 1000000` 强行
# 重配, 在生产系统运行会把正在通信的 CAN 波特率改坏(MPPT/DCDC/BMS 全部失联)。
# 预期波特率与 tools/systemd/airship-can-up.sh 保持一致:
#   can0=250000 (MPPT/DCDC), can1=500000 (主电源 BMS)
echo "--- 2. CAN 接口 ---"
check_can() {
    local ifname=$1 expected=$2 desc=$3
    if [ ! -d "/sys/class/net/$ifname" ]; then
        bad "$ifname 不存在 ($desc)"
        return
    fi
    local actual
    actual=$(ip -details link show "$ifname" 2>/dev/null | grep -oE 'bitrate [0-9]+' | head -1 | grep -oE '[0-9]+')
    if [ -z "$actual" ]; then
        ok "$ifname 存在 ($desc), 尚未配置波特率 (预期 $expected bps, 由 airship-can.service 配置)"
    elif [ "$actual" = "$expected" ]; then
        ok "$ifname 存在 ($desc), 波特率 ${actual}bps 与预期一致"
    else
        bad "$ifname 波特率 ${actual}bps 与预期 ${expected}bps 不一致 (以 tools/systemd/airship-can-up.sh 为准)"
    fi
}
check_can can0 250000 "MCP2515 经典CAN, MPPT/DCDC"
check_can can1 500000 "MCP2518FD CAN FD, 主电源BMS"
echo "  can 状态:"
ip -br link show 2>/dev/null | grep -E "^can" || echo "  (can 未 up)"

# ---------- 3. RS485 串口 ----------
echo "--- 3. RS485 串口 (SC16IS752) ---"
for dev in /dev/ttySC0 /dev/ttySC1; do
    if [ -e "$dev" ]; then
        ok "$dev 存在"
    else
        bad "$dev 不存在"
    fi
done

# ---------- 4. RS232 串口 ----------
echo "--- 4. RS232 串口 (板载UART) ---"
if [ -e /dev/ttyAMA0 ]; then
    ok "/dev/ttyAMA0 存在"
else
    # Ubuntu 上主 UART 可能是 ttyAMA10，做兼容判断
    if ls /dev/ttyAMA* >/dev/null 2>&1; then
        ok "UART 存在: $(ls /dev/ttyAMA* | tr '\n' ' ')"
        echo "  (注: Ubuntu 主串口可能不是 ttyAMA0)"
    else
        bad "无 ttyAMA* 串口"
    fi
fi

# ---------- 5. SPI ----------
echo "--- 5. SPI 设备 ---"
for dev in /dev/spidev0.0 /dev/spidev0.1; do
    if [ -e "$dev" ]; then
        ok "$dev 存在"
    else
        bad "$dev 不存在"
    fi
done

# ---------- 6. 串口权限 ----------
echo "--- 6. 串口权限 ---"
# 用真实登录用户（避免 sudo 下 whoami 变成 root）
REAL_USER="${SUDO_USER:-$(whoami)}"
if groups "$REAL_USER" 2>/dev/null | grep -q dialout; then
    ok "用户 $REAL_USER 在 dialout 组"
else
    bad "用户 $REAL_USER 不在 dialout 组，需执行: sudo usermod -aG dialout $REAL_USER"
fi

# ---------- 7. INT 中断引脚 ----------
echo "--- 7. 中断引脚 (GPIO 23/24/25) ---"
for pin in 23 24 25; do
    if [ -d /sys/class/gpio/gpio$pin ] || grep -q "gpio$pin" /sys/kernel/debug/gpio 2>/dev/null; then
        ok "GPIO$pin 已被驱动申请"
    else
        bad "GPIO$pin 未申请 (INT 引脚，板载配置应为 23=CAN,24=CANFD,25=RS485)"
    fi
done

# ---------- 汇总 ----------
echo "=============================================="
echo "通过: $PASS / 失败: $FAIL"
[ "$FAIL" -eq 0 ] && echo "全部就绪 ✓" || echo "存在未就绪项，请按提示排查"
exit "$FAIL"
