#!/bin/bash
# 灵云01号 伴飞电脑 —— 开机拉起 CAN 接口 (RS232-RS485-CAN-Board)
#
#   can0 : MCP2515   (经典 CAN 2.0, 5k-1Mbps,  SPI0-0)
#   can1 : MCP2518FD (CAN FD,        5k-8Mbps, SPI0-1)
#
# 波特率可通过环境变量或 /etc/default/airship-can 覆盖。
# 默认与部署规范一致: can0=250k(MPPT/DCDC), can1=500k/数据2M(CAN FD, 预留/BMS)。
# 用法: airship-can-up.sh [up|down]

set -e

ACTION="${1:-up}"

# 允许通过 /etc/default/airship-can 注入 CAN0_BITRATE / CAN1_BITRATE / CAN_DBITRATE / CAN_FD_ON
if [ -f /etc/default/airship-can ]; then
    # shellcheck disable=SC1091
    . /etc/default/airship-can
fi

CAN0_BITRATE="${CAN0_BITRATE:-${CAN_BITRATE:-250000}}"
CAN1_BITRATE="${CAN1_BITRATE:-500000}"
DBITRATE="${CAN_DBITRATE:-2000000}"
FD_ON="${CAN_FD_ON:-1}"   # 1 = can1 开启 CAN FD 模式 (默认与 mcp251xfd 一致)

log() { echo "[airship-can] $*"; }

case "$ACTION" in
  up)
    # ---------- can0 : 经典 CAN (MPPT/DCDC @ 250k) ----------
    if ip link show can0 >/dev/null 2>&1; then
        ip link set can0 up type can bitrate "$CAN0_BITRATE"
        ip link set can0 txqueuelen 65536
        log "can0 up (bitrate=$CAN0_BITRATE)"
    else
        log "can0 不存在，跳过"
    fi

    # ---------- can1 : CAN FD (预留/BMS) ----------
    if ip link show can1 >/dev/null 2>&1; then
        if [ "$FD_ON" = "1" ]; then
            ip link set can1 up type can bitrate "$CAN1_BITRATE" dbitrate "$DBITRATE" fd on
            log "can1 up (bitrate=$CAN1_BITRATE dbitrate=$DBITRATE fd=on)"
        else
            ip link set can1 up type can bitrate "$CAN1_BITRATE"
            log "can1 up (bitrate=$CAN1_BITRATE fd=off)"
        fi
        ip link set can1 txqueuelen 65536
    else
        log "can1 不存在，跳过"
    fi
    ;;

  down)
    ip link set can0 down 2>/dev/null || true
    ip link set can1 down 2>/dev/null || true
    log "can0/can1 down"
    ;;

  *)
    echo "用法: $0 [up|down]" >&2
    exit 1
    ;;
esac

exit 0
