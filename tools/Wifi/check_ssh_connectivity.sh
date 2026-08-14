#!/usr/bin/env bash
# 灵云01号伴飞电脑 — 检查笔记本 ↔ 树莓派 SSH 连接前提条件
#
# 覆盖 4 个前提：
#   1. 树莓派 mDNS 服务(avahi-daemon)运行 + 主机名正确
#   2. 笔记本支持 mDNS 解析（.local）
#   3. 两者同一局域网 + 热点未开 AP 隔离
#   4. SSH 免密公钥已部署到树莓派
#
# 用法：
#   bash tools/check_ssh_connectivity.sh            # 使用 ~/.ssh/config 中的别名 pi
#   bash tools/check_ssh_connectivity.sh -a pi      # 指定其他别名
#   bash tools/check_ssh_connectivity.sh -h 主机名 -u 用户 -p 端口   # 不依赖 ssh config
#
# 退出码：0=全部通过；1=存在 FAIL；2=存在 WARN（不影响连接）

set -uo pipefail

ALIAS=""
HOSTNAME_OVERRIDE=""
USER_OVERRIDE=""
PORT_OVERRIDE=""

usage() {
    sed -n 's/^# \{0,1\}//p' "$0" | sed -n '1,20p'
    exit 0
}

while getopts ":a:h:u:p:" opt; do
    case "$opt" in
        a) ALIAS="$OPTARG" ;;
        h) HOSTNAME_OVERRIDE="$OPTARG" ;;
        u) USER_OVERRIDE="$OPTARG" ;;
        p) PORT_OVERRIDE="$OPTARG" ;;
        *) usage ;;
    esac
done

# ---------- 输出辅助 ----------
RC=0
NC=""; RED=""; GREEN=""; YELLOW=""; BOLD=""
if [ -t 1 ]; then
    NC="\033[0m"; RED="\033[31m"; GREEN="\033[32m"; YELLOW="\033[33m"; BOLD="\033[1m"
fi

section() { printf "\n${BOLD}== %s ==${NC}\n" "$1"; }
pass()     { printf "  ${GREEN}[PASS]${NC} %s\n" "$1"; }
warn()     { printf "  ${YELLOW}[WARN]${NC} %s\n" "$1"; RC=2; }
fail()     { printf "  ${RED}[FAIL]${NC} %s\n" "$1"; RC=1; }
info()     { printf "  ${NC}%s${NC}\n" "$1"; }

# ---------- 1. 从 ssh config 解析目标 ----------
command -v ssh >/dev/null 2>&1 || { echo "未找到 ssh 命令，请先安装 openssh-client"; exit 1; }

if [ -z "$HOSTNAME_OVERRIDE" ]; then
    : "${ALIAS:=pi}"
    CFG="$(ssh -G "$ALIAS" 2>/dev/null)"
    TARGET_HOST="$(echo "$CFG" | awk '$1=="hostname"{print $2; exit}')"
    TARGET_USER="$(echo "$CFG" | awk '$1=="user"{print $2; exit}')"
    TARGET_PORT="$(echo "$CFG" | awk '$1=="port"{print $2; exit}')"
    : "${TARGET_HOST:?ssh config 中未找到 hostname，请检查别名或改用 -h}"
    : "${TARGET_USER:?ssh config 中未找到 user，请检查别名或改用 -u}"
    : "${TARGET_PORT:=22}"
else
    TARGET_HOST="$HOSTNAME_OVERRIDE"
    TARGET_USER="${USER_OVERRIDE:?使用 -h 时必须同时用 -u 指定用户名}"
    TARGET_PORT="${PORT_OVERRIDE:-22}"
    ALIAS="(direct)"
fi

echo "目标：${TARGET_USER}@${TARGET_HOST} : ${TARGET_PORT}  别名：${ALIAS}"

# ---------- 2. 前提2：笔记本 mDNS 解析支持 ----------
section "前提2 笔记本 mDNS 解析支持"
MDNS_TOOL=""
if command -v avahi-resolve >/dev/null 2>&1; then MDNS_TOOL="avahi-resolve"; fi
if command -v systemd-resolve >/dev/null 2>&1; then MDNS_TOOL="systemd-resolve"; fi
if command -v resolvectl >/dev/null 2>&1; then MDNS_TOOL="resolvectl"; fi
if systemctl is-active systemd-resolved >/dev/null 2>&1; then MDNS_TOOL="systemd-resolved"; fi
if [ -n "$MDNS_TOOL" ]; then
    pass "mDNS 支持组件已存在（$MDNS_TOOL）"
else
    warn "未检测到 avahi/systemd-resolved。若 .local 解析失败，请安装：sudo apt install avahi-daemon"
fi

# ---------- 3. 前提2：mDNS 解析 .local ----------
section "mDNS 解析 ${TARGET_HOST}"
RESOLVED_IP=""
if getent hosts "$TARGET_HOST" >/dev/null 2>&1; then
    RESOLVED_IP="$(getent hosts "$TARGET_HOST" | awk 'NR==1{print $1}')"
    pass "mDNS 解析成功 -> ${RESOLVED_IP}"
elif ping -c1 -W2 "$TARGET_HOST" >/dev/null 2>&1; then
    RESOLVED_IP="$(ping -c1 -W2 "$TARGET_HOST" 2>/dev/null | grep -oP '\([0-9.]+\)' | head -1 | tr -d '()')"
    pass "mDNS 解析成功（ping 返回 ${RESOLVED_IP:-未知IP}）"
else
    fail "mDNS 解析失败。请检查：①树莓派 avahi-daemon 是否运行(前提1) ②主机名是否正确 ③是否同网段/热点 AP 隔离"
fi

# ---------- 4. 前提3：同局域网 + 可达性（单播 / 端口） ----------
section "前提3 同局域网 + 热点未开 AP 隔离"
PING_OK=0
if [ -n "$RESOLVED_IP" ] && ping -c1 -W2 "$RESOLVED_IP" >/dev/null 2>&1; then
    PING_OK=1
    pass "ping ${RESOLVED_IP} 通（单播可达，AP 隔离大概率未开启）"
else
    fail "ping ${RESOLVED_IP:-${TARGET_HOST}} 不通：若在同热点下，多半是热点开启了「AP 隔离/设备隔离」，请到热点设置里关闭"
fi

if [ "$PING_OK" -eq 1 ]; then
    if timeout 3 bash -c "exec 3<>/dev/tcp/${RESOLVED_IP}/${TARGET_PORT}" >/dev/null 2>&1; then
        pass "TCP ${TARGET_PORT} 端口可达（SSH 服务在监听）"
    else
        warn "ping 通但 TCP ${TARGET_PORT} 连不上：检查树莓派 sshd 是否运行（sudo systemctl status ssh）与防火墙"
    fi
fi

# ---------- 5. 前提4：SSH 免密公钥 ----------
section "前提4 SSH 免密登录"
if [ -n "$ALIAS" ] && [ "$ALIAS" != "(direct)" ]; then
    ssh -o BatchMode=yes -o ConnectTimeout=6 -o StrictHostKeyChecking=accept-new "$ALIAS" true >/dev/null 2>&1
    SSH_OK=$?
else
    ssh -o BatchMode=yes -o ConnectTimeout=6 -o StrictHostKeyChecking=accept-new \
        -p "$TARGET_PORT" "${TARGET_USER}@${TARGET_HOST}" true >/dev/null 2>&1
    SSH_OK=$?
fi
if [ "$SSH_OK" -eq 0 ]; then
    pass "免密登录成功（公钥已部署）"
else
    fail "免密登录失败。请部署公钥：ssh-copy-id -i ~/.ssh/id_ed25519.pub ${TARGET_USER}@${TARGET_HOST}"
fi

# ---------- 6. 前提1：树莓派 hostname + avahi ----------
if [ "$SSH_OK" -eq 0 ]; then
    section "前提1 树莓派 mDNS 服务与主机名"
    if [ -n "$ALIAS" ] && [ "$ALIAS" != "(direct)" ]; then
        R_SSH=("$ALIAS")
    else
        R_SSH=(-p "$TARGET_PORT" "${TARGET_USER}@${TARGET_HOST}")
    fi

    R_HOSTNAME="$(ssh -o BatchMode=yes -o ConnectTimeout=6 "${R_SSH[@]}" 'hostname' 2>/dev/null)"
    R_AVAHI="$(ssh -o BatchMode=yes -o ConnectTimeout=6 "${R_SSH[@]}" 'systemctl is-active avahi-daemon 2>/dev/null || echo inactive' 2>/dev/null)"
    R_HOSTNAME_SHORT="$(echo "$TARGET_HOST" | sed 's/\.local$//')"

    if [ -n "$R_HOSTNAME" ]; then
        pass "树莓派主机名 = ${R_HOSTNAME}"
    else
        fail "无法获取树莓派主机名（远端命令失败）"
    fi

    if [ "$R_AVAHI" = "active" ]; then
        pass "avahi-daemon 运行中"
    else
        fail "avahi-daemon 未运行。请执行：sudo apt install -y avahi-daemon && sudo systemctl enable --now avahi-daemon"
    fi

    if [ -n "$R_HOSTNAME" ] && [ "$R_HOSTNAME" = "$R_HOSTNAME_SHORT" ]; then
        pass "主机名与 .local 前缀一致（${R_HOSTNAME}.local）"
    else
        warn "远端主机名(${R_HOSTNAME}) 与 .local 前缀(${R_HOSTNAME_SHORT})不一致，mDNS 实际应解析 ${R_HOSTNAME}.local"
    fi
fi

# ---------- 汇总 ----------
echo
if [ "$RC" -eq 0 ]; then
    echo -e "${GREEN}全部通过：现在直接 ssh ${ALIAS} 即可连接。${NC}"
elif [ "$RC" -eq 2 ]; then
    echo -e "${YELLOW}存在警告：通常仍可连接，建议按上面提示处理。${NC}"
else
    echo -e "${RED}存在失败项：请按上面提示逐项修复。${NC}"
fi
exit "$RC"
