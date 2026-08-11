#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 生产环境一键部署脚本
#
# 功能: 安装 udev 设备规则 / systemd 开机自启(设备监控+DCDC保活) /
#       journald 日志上限 / ROS2 logrotate / NTP 时间同步 / CAN 接口自启
#
# 用法:
#   sudo ./tools/install_production.sh            # 全量安装
#   sudo ./tools/install_production.sh --can  250000   # 仅配置 CAN (可指定波特率)
#
# 注意:
#   - 需要 root (sudo)
#   - 安装后建议重启树莓派验证开机自启
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAN_BITRATE="${CAN_BITRATE:-250000}"

echo "=============================================="
echo " 灵云01号 生产环境部署"
echo "=============================================="

# ============ 1. udev 设备规则(固定串口名) ============
install_udev() {
  echo "[1/4] 安装 udev 设备规则..."
  install -m 644 "$SCRIPT_DIR/70-airship-usb.rules" /etc/udev/rules.d/70-airship-usb.rules
  udevadm control --reload-rules
  udevadm trigger
  echo "      已安装, 触发重载。符号链接: /dev/airship_485, /dev/airship_4g_*"
}

# ============ 1b. 敏感凭据 env 文件 ============
install_secrets() {
  echo "[SECRET] 配置敏感凭据 env 文件(/etc/airship/airship.env)..."
  mkdir -p /etc/airship
  if [ -n "${MQTT_PASSWORD:-}" ]; then
    umask 177
    printf 'MQTT_PASSWORD=%s\n' "$MQTT_PASSWORD" > /etc/airship/airship.env
    chown root:root /etc/airship/airship.env
    chmod 600 /etc/airship/airship.env
    echo "      已写入 MQTT_PASSWORD 到 /etc/airship/airship.env (权限600, 仅 root 可读)"
  else
    echo "      未检测到 MQTT_PASSWORD 环境变量; 若需上云请手动创建 env 文件(见 service 注释)"
  fi
}

# ============ 2. systemd 开机自启 ============
install_systemd() {
  echo "[2/7] 安装 systemd 开机自启服务..."
  install -m 644 "$SCRIPT_DIR/airship-device-monitor.service" /etc/systemd/system/airship-device-monitor.service
  systemctl daemon-reload
  systemctl enable airship-device-monitor
  echo "      已启用设备监控开机自启 (可 systemctl start 立即启动)"
}

# ============ 2b. DCDC 硬件保活服务 ============
install_dcdc_hold() {
  echo "[2b/7] 安装 DCDC 硬件保活服务..."
  install -m 644 "$SCRIPT_DIR/airship-dcdc-hold.service" /etc/systemd/system/airship-dcdc-hold.service
  systemctl daemon-reload
  systemctl enable airship-dcdc-hold
  echo "      已启用 DCDC 硬件保活 (独立于 ROS2, 崩溃自愈)"
}

# ============ 3. journald 日志上限 ============
install_journald() {
  echo "[3/7] 配置 journald 日志上限(50MB)..."
  mkdir -p /etc/systemd/journald.conf.d
  install -m 644 "$SCRIPT_DIR/90-airship-journald.conf" /etc/systemd/journald.conf.d/90-airship-journald.conf
  systemctl restart systemd-journald
  echo "      已配置 journal 日志上限"
}

# ============ 4. ROS2 日志滚动清理 (logrotate) ============
install_logrotate() {
  echo "[4/7] 安装 ROS2 日志滚动清理配置..."
  install -m 644 "$SCRIPT_DIR/ros_logrotate" /etc/logrotate.d/ros
  chown root:root /etc/logrotate.d/ros
  echo "      已安装 /etc/logrotate.d/ros (单文件 10MB 轮转, 保留 5 份)"
}

# ============ 5. NTP 时间同步 ============
install_ntp() {
  echo "[5/7] 配置 NTP 时间同步..."
  if command -v timedatectl >/dev/null 2>&1; then
    timedatectl set-ntp true && echo "      已开启 NTP 自动同步 (timedatectl)"
  else
    echo "      未找到 timedatectl, 跳过(可手动安装 chrony/systemd-timesyncd)"
  fi
}

# ============ 6. CAN 接口自启(可选) ============
install_can() {
  echo "[6/7] 配置 CAN 接口开机自启 (can0 @ ${CAN_BITRATE}bps)..."
  # 使用 /etc/network 或 systemd-networkd 方式保证开机即配置
  if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files | grep -q can0; then
    systemctl enable can0 2>/dev/null || true
  fi
  # 主服务 ExecStartPre 已含 CAN 初始化兜底, 这里仅提示
  echo "      CAN 初始化已由 airship-device-monitor 的 ExecStartPre 兜底"
}

# ============ 执行 ============
# 参数解析: 默认全量; 支持单独指定
# 注意: --can 后可选跟波特率, 如 `--can 250000`
if [ "$#" -gt 0 ]; then
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --udev) install_udev; shift ;;
      --systemd) install_systemd; shift ;;
      --dcdc-hold) install_dcdc_hold; shift ;;
      --journald) install_journald; shift ;;
      --logrotate) install_logrotate; shift ;;
      --ntp) install_ntp; shift ;;
      --can)
        # 消费可选波特率参数(紧跟的数字), 否则使用环境变量/默认值
        case "${2:-}" in
          ''|--*)
            install_can
            shift
            ;;
          *)
            CAN_BITRATE="$2"
            install_can
            shift 2
            ;;
        esac
        ;;
      *) echo "未知参数: $1"; exit 1 ;;
    esac
  done
else
  install_udev
  install_secrets
  install_systemd
  install_dcdc_hold
  install_journald
  install_logrotate
  install_ntp
  install_can
fi

echo ""
echo "=============================================="
echo " 部署完成! 建议重启树莓派验证开机自启:"
echo "   sudo reboot"
echo " 重启后查看: systemctl status airship-device-monitor"
echo "=============================================="