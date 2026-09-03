#!/bin/bash
# 灵云01号伴飞电脑 —— 磁盘守护 (airship-disk-guard)
#
# 职责(每次执行幂等, 由 airship-disk-guard.timer 每 10 分钟触发):
#   [时间清理]
#     a) bags 目录: 删除 mtime > RETAIN_DAYS 的录制目录
#     b) fc CSV:    删除 mtime > FC_RETAIN_DAYS 的 fc_status_*.csv
#     c) ROS 日志:  删除 ~/.ros/log 下 mtime > ROS_LOG_RETAIN_DAYS 的文件并清理空目录
#   [水位控制] (根分区使用率)
#     d) >= WARN_PCT% : 按最旧优先逐个删除 bag 目录, 直到 <= SAFE_PCT%
#     e) >= CRIT_PCT% : systemctl stop airship-bag-record (红线停录保护系统)
#     f) 已停录且回落 <= SAFE_PCT% : 自动恢复 airship-bag-record
#
# 所有动作均通过 logger -t airship-disk-guard 写入 journald 留痕。
set -u

BAG_DIR="${BAG_DIR:-/home/lingyun01/bags}"
FC_LOG_DIR="${FC_LOG_DIR:-/home/lingyun01/airship_fc_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/home/lingyun01/.ros/log}"

RETAIN_DAYS="${RETAIN_DAYS:-3}"          # bag 保留天数(与原 airship-bag-clean 一致)
FC_RETAIN_DAYS="${FC_RETAIN_DAYS:-7}"    # fc CSV 原始保留天数(gz 另有 30 天)
GZ_RETAIN_DAYS="${GZ_RETAIN_DAYS:-30}"   # gz 压缩件保留天数
ROS_LOG_RETAIN_DAYS="${ROS_LOG_RETAIN_DAYS:-7}"

WARN_PCT="${WARN_PCT:-85}"               # 水位警戒线
CRIT_PCT="${CRIT_PCT:-92}"               # 红线(停录)
SAFE_PCT="${SAFE_PCT:-75}"               # 回落安全线
STOP_FLAG="${STOP_FLAG:-/tmp/airship-bag-stopped-by-guard}"
RECORD_UNIT="${RECORD_UNIT:-airship-bag-record.service}"

log() { logger -t airship-disk-guard "$*"; }

usage_pct() {
  df --output=pcent / | tail -1 | tr -dc '0-9'
}

bag_dirs_oldest_first() {
  # 目录名升序 == 时间戳命名下最旧优先
  find "$BAG_DIR" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | sort | sed "s|^$BAG_DIR/||"
}

del_dir() {
  rm -rf "${BAG_DIR:?}/$1" && log "WARN 已删除旧录制目录: $1"
}

# ===== [时间清理] =====
if [ -d "$BAG_DIR" ]; then
  find "$BAG_DIR" -maxdepth 1 -mindepth 1 -type d -mtime +"$RETAIN_DAYS" \
    -exec sh -c 'logger -t airship-disk-guard "定时清理录制目录: $1"; rm -rf "$1"' _ {} \;
fi

if [ -d "$FC_LOG_DIR" ]; then
  find "$FC_LOG_DIR" -name 'fc_status_*.csv' -mtime +"$FC_RETAIN_DAYS" -delete 2>/dev/null || true
  find "$FC_LOG_DIR" -name 'fc_status_*.csv.gz' -mtime +"$GZ_RETAIN_DAYS" -delete 2>/dev/null || true
  # 旧版无日期后缀的固定名文件同样纳入清理, 避免历史遗留无限增长
  find "$FC_LOG_DIR" -maxdepth 1 -name 'fc_status.csv*' -mtime +"$FC_RETAIN_DAYS" -delete 2>/dev/null || true
fi

if [ -d "$ROS_LOG_DIR" ]; then
  find "$ROS_LOG_DIR" -mindepth 2 -type f -mtime +"$ROS_LOG_RETAIN_DAYS" -delete 2>/dev/null || true
  find "$ROS_LOG_DIR" -mindepth 1 -maxdepth 1 -type d -empty -delete 2>/dev/null || true
fi

# ===== [每日一次维护] 系统包自动清理 (2026-09-03) =====
# apt-get autoremove: 清除旧内核与孤儿依赖(Ubuntu 保留 1 个旧内核, unattended-upgrades
# 的 Remove-Unused 只在升级动作时触发, 时机不可控, 此处每日补刀)。
# 当日标记防重复(每次 timer 触发 10min 一轮); --purge 连配置一起清; 秒级超时保护。
APT_MAINT_FLAG="/tmp/.airship-apt-maint-$(date +%Y%m%d)"
if [ ! -f "$APT_MAINT_FLAG" ] && command -v apt-get >/dev/null 2>&1; then
  if timeout 120 apt-get autoremove --purge -y >/dev/null 2>&1; then
    touch "$APT_MAINT_FLAG"
    log "INFO 每日维护: apt autoremove --purge 已执行"
  fi
fi

# ===== [水位控制] =====
P=$(usage_pct)

# d) 警戒线: 删最旧 bag 直到回到 SAFE_PCT 以下
if [ "$P" -ge "$WARN_PCT" ]; then
  while [ "$(usage_pct)" -gt "$SAFE_PCT" ]; do
    OLDEST=$(bag_dirs_oldest_first | head -1)
    if [ -z "$OLDEST" ]; then
      break   # 无可删目录, 交给 e) 红线逻辑处理
    fi
    del_dir "$OLDEST"
  done
fi
P=$(usage_pct)

# e) 红线: 停录保护系统
if [ "$P" -ge "$CRIT_PCT" ] &&
   ! [ -f "$STOP_FLAG" ] &&
   systemctl is-active --quiet "$RECORD_UNIT"; then
  systemctl stop "$RECORD_UNIT" && touch "$STOP_FLAG" \
    && log "ERR 磁盘使用率 ${P}% 达到红线(${CRIT_PCT}%), 已自动停止 ${RECORD_UNIT}"
fi

# f) 回落: 自动恢复录制
if [ -f "$STOP_FLAG" ] && [ "$P" -le "$SAFE_PCT" ]; then
  if systemctl start "$RECORD_UNIT"; then
    rm -f "$STOP_FLAG"
    log "INFO 磁盘使用率回落至 ${P}%(<=${SAFE_PCT}%), 已自动恢复 ${RECORD_UNIT}"
  else
    log "ERR 尝试恢复 ${RECORD_UNIT} 失败, 保留标记待下轮重试"
  fi
fi

exit 0