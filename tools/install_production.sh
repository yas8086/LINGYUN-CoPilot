#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 生产环境一键部署脚本
#
# 功能: 安装 udev 设备规则 / systemd 开机自启(设备监控+DCDC保活) /
#       journald 日志上限 / ROS2 logrotate / NTP 时间同步 / CAN 接口自启
#
# 用法:
#   sudo ./tools/install_production.sh                    # 全量安装
#   sudo ./tools/install_production.sh --can0 250000      # 仅配置 CAN, 指定 can0 波特率
#   sudo ./tools/install_production.sh --can0 250000 --can1 500000
#   (--can 为 --can0 的旧名别名, 兼容保留)
#
# 注意:
#   - 需要 root (sudo)
#   - 安装后建议重启树莓派验证开机自启
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 波特率写入 /etc/default/airship-can(airship-can-up.sh 开机 source 该文件)
# 拓扑: can0=MPPT/DCDC@250k, can1=主电源BMS@500k(经典帧, CAN FD 接口)
CAN0_BITRATE="${CAN0_BITRATE:-${CAN_BITRATE:-250000}}"
CAN1_BITRATE="${CAN1_BITRATE:-500000}"
CAN_DBITRATE="${CAN_DBITRATE:-2000000}"
CAN_FD_ON="${CAN_FD_ON:-1}"

echo "=============================================="
echo " 灵云01号 生产环境部署"
echo "=============================================="

# ============ 1. udev 设备规则(固定串口名) ============
install_udev() {
  echo "[1/4] 安装 udev 设备规则..."
  # 70: USB 串口规范化(数传/4G); 71: 微雪工业扩展板(485/BMS 串口符号链接)
  install -m 644 "$SCRIPT_DIR/70-airship-usb.rules" /etc/udev/rules.d/70-airship-usb.rules
  install -m 644 "$SCRIPT_DIR/71-airship-extboards.rules" /etc/udev/rules.d/71-airship-extboards.rules
  udevadm control --reload-rules
  udevadm trigger
  echo "      已安装, 触发重载。符号链接: /dev/airship_485, /dev/airship_backup_bms, /dev/airship_4g_*"
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
  echo "[6/7] 配置 CAN 接口开机自启 (can0@${CAN0_BITRATE} / can1@${CAN1_BITRATE})..."
  # 落地 airship-can 服务: airship-can-up.sh 提供 can0/can1 开机配置
  install -m 755 "$SCRIPT_DIR/systemd/airship-can-up.sh" /usr/local/bin/airship-can-up.sh
  install -m 644 "$SCRIPT_DIR/systemd/airship-can.service" /etc/systemd/system/airship-can.service
  # 波特率写入 /etc/default/airship-can(airship-can-up.sh 开机时 source 该文件)。
  # 旧实现中 --can 指定的波特率仅被赋值从未被消费(静默无效), 现真正落地。
  mkdir -p /etc/default
  cat > /etc/default/airship-can <<EOF
# 由 install_production.sh 生成; /usr/local/bin/airship-can-up.sh 开机时 source 本文件
# 拓扑: can0=MPPT/DCDC, can1=主电源BMS(经典帧, CAN FD 接口)
CAN0_BITRATE=${CAN0_BITRATE}
CAN1_BITRATE=${CAN1_BITRATE}
CAN_DBITRATE=${CAN_DBITRATE}
CAN_FD_ON=${CAN_FD_ON}
EOF
  chmod 644 /etc/default/airship-can
  systemctl daemon-reload
  systemctl enable airship-can >/dev/null 2>&1 || true
  echo "      已启用 airship-can 开机自启 (can0@${CAN0_BITRATE}, can1@${CAN1_BITRATE})"
}

# ============ 6b. 系统看门狗(systemd + 硬件) ============
install_watchdog() {
  echo "[6b] 配置 systemd 看门狗(配合硬件 watchdog)..."
  mkdir -p /etc/systemd/system.conf.d
  install -m 644 "$SCRIPT_DIR/systemd/90-airship-watchdog.conf" /etc/systemd/system.conf.d/90-airship-watchdog.conf
  systemctl daemon-reload
  echo "      已安装看门狗配置 Runtime/ RebootWatchdogSec=20/30"
  echo "      注意: 请确认 /boot/firmware/config.txt 已含 'dtparam=watchdog=on'(见 rpi5_ubuntu2404_config.txt)"
}

# ============ 6c. rosbag 数据记录 ============
install_bag_record() {
  local bag_dir
  bag_dir="${BAG_DIR:-/home/lingyun01/bags}"
  echo "[6c] 配置 rosbag 后台记录(目录: $bag_dir)..."
  install -m 644 "$SCRIPT_DIR/airship-bag-record.service" /etc/systemd/system/airship-bag-record.service
  systemctl daemon-reload
  # 旧版 bag-clean.service/timer 已被 airship-disk-guard 取代(见 [6d]), 统一退役
  systemctl disable --now airship-bag-clean.timer >/dev/null 2>&1 || true
  systemctl enable airship-bag-record >/dev/null 2>&1 || true
  echo "      已启用 rosbag 全量记录 (清理/水位由 airship-disk-guard 负责, 见 [6d])"
}

# ============ 6d. 磁盘守护(时间清理+水位删除+红线停录) ============
install_disk_guard() {
  echo "[6d] 配置磁盘守护 airship-disk-guard..."
  install -m 755 "$SCRIPT_DIR/airship-disk-guard.sh" /usr/local/bin/airship-disk-guard.sh
  install -m 644 "$SCRIPT_DIR/systemd/airship-disk-guard.service" /etc/systemd/system/airship-disk-guard.service
  install -m 644 "$SCRIPT_DIR/systemd/airship-disk-guard.timer" /etc/systemd/system/airship-disk-guard.timer
  systemctl daemon-reload
  systemctl enable --now airship-disk-guard.timer
  echo "      已启用磁盘守护: bags>3天/fc_csv>7天/roslog>7天时间清理,"
  echo "            根分区 >=85% 删最旧bag, >=92% 红线停录, 回落 <=75% 自动恢复"
}

# ============ 6e. TF 卡 TRIM 维护(fstrim) ============
install_trim() {
  echo "[6e] 启用 fstrim 定时裁剪(TF 卡写放大维护)..."
  if systemctl list-unit-files fstrim.timer >/dev/null 2>&1; then
    systemctl enable --now fstrim.timer && echo "      fstrim.timer 已启用(每周裁剪)"
  else
    echo "      本机无 fstrim.timer(Utility 不存在), 跳过"
  fi
}

# ============ 执行 ============
# 参数解析: 默认全量; 支持单独指定
# 注意: --can0/--can1/--can(旧名) 后可选跟波特率数字
if [ "$#" -gt 0 ]; then
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --udev) install_udev; shift ;;
      --systemd) install_systemd; shift ;;
      --dcdc-hold) install_dcdc_hold; shift ;;
      --journald) install_journald; shift ;;
      --logrotate) install_logrotate; shift ;;
      --ntp) install_ntp; shift ;;
      --watchdog) install_watchdog; shift ;;
      --bag-record) install_bag_record; shift ;;
      --disk-guard) install_disk_guard; shift ;;
      --trim) install_trim; shift ;;
      --can|--can0)
        # 消费可选波特率参数(紧跟的数字), 否则使用环境变量/默认值
        case "${2:-}" in
          ''|--*)
            install_can
            shift
            ;;
          *)
            CAN0_BITRATE="$2"
            install_can
            shift 2
            ;;
        esac
        ;;
      --can1)
        case "${2:-}" in
          ''|--*)
            install_can
            shift
            ;;
          *)
            CAN1_BITRATE="$2"
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
  install_watchdog
  install_bag_record
  install_disk_guard
  install_trim
fi

echo ""
echo "=============================================="
echo " 部署完成! 建议重启树莓派验证开机自启:"
echo "   sudo reboot"
echo " 重启后查看: systemctl status airship-device-monitor"
echo "=============================================="