#!/usr/bin/env bash
# 灵云01号伴飞电脑 — WiFi 连接管理脚本（NetworkManager / nmcli）
#
# 子命令：
#   list                        # 打印所有已保存的 WiFi：SSID / 密码状态 / 自动连接 / 优先级 / 当前状态
#   add  <SSID> [密码] [优先级]  # 新增或更新一个 WiFi（幂等：已存在只更新，不存在则新建）
#   rm   <SSID>                 # 删除一个 WiFi 连接
#
# 优先级说明：
#   - autoconnect-priority：数值越大越优先，默认 0
#   - 多个 WiFi 都可用时，NetworkManager 先连优先级最高的；连不上再降级
#   - 建议：手机热点 100、公司 WiFi 50、其他场地依次递减
#
# 用法示例：
#   sudo bash airship_wifi.sh list
#   sudo bash airship_wifi.sh add "于永强的iPhone" "<你的热点密码>" 100
#   sudo bash airship_wifi.sh add "公司WiFi" "密码" 50
#   sudo bash airship_wifi.sh add "某场地开放热点" "" 30     # 开放网络，密码留空
#   sudo bash airship_wifi.sh rm "公司WiFi"
#
# 说明：
#   - list 不需要 root；add/rm 需要 root
#   - eth0 的 netplan 静态 IP（连飞控 10.41.10.100/24）不受影响
#   - 现场临时连新热点且不保存优先级：nmcli device wifi connect "<SSID>" password "<密码>"

set -uo pipefail

usage() {
    sed -n 's/^# \{0,1\}//p' "$0" | sed -n '1,26p'
    exit 0
}

need_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "错误：add/rm 需要 root 权限，请用：sudo bash $0 $*"
        exit 1
    fi
}

# 返回与指定名字匹配的所有连接 uuid（每行一个）；名字含中文也按原始字符匹配
# 注意：连接名可能含冒号，这里按"行首 NAME 精确等于 ssid"匹配整行前缀
find_uuids() {
    local ssid="$1"
    nmcli --escape no -t -f NAME,UUID connection show 2>/dev/null |
        awk -F: -v n="$ssid" '$1==n {print $2}'
}

# ---------- list：打印所有已保存 WiFi ----------
cmd_list() {
    echo "=== 已保存的 WiFi 连接 ==="
    printf "%-28s %-8s %-4s %-6s %s\n" "SSID" "密码" "自动" "优先级" "状态"
    printf "%-28s %-8s %-4s %-6s %s\n" "----" "----" "----" "------" "----"
    while IFS=':' read -r name uuid type dev state; do
        [ "$type" = "802-11-wireless" ] || continue
        # 用 uuid 查询属性，避免同名连接导致命令失败；-g 只输出值不带字段名
        prio=$(nmcli --escape no -g connection.autoconnect-priority connection show uuid "$uuid" 2>/dev/null)
        ac=$(nmcli --escape no -g connection.autoconnect connection show uuid "$uuid" 2>/dev/null)
        psk=$(nmcli --escape no -s -g 802-11-wireless-security.psk connection show uuid "$uuid" 2>/dev/null)
        if [ -n "$psk" ]; then psk_s="已设置"; else psk_s="开放/无"; fi
        printf "%-28s %-8s %-4s %-6s %s\n" "$name" "$psk_s" "${ac:-?}" "${prio:-0}" "${state:-}"
    done < <(nmcli --escape no -t -f NAME,UUID,TYPE,DEVICE,STATE connection show 2>/dev/null)
    echo "（状态列：activated=当前连接中；deactivated=已保存未连接）"
    echo "提示：想改某个 WiFi 的优先级，用 add 重新指定即可，例如："
    echo "  sudo bash $0 add \"于永强的iPhone\" \"密码\" 100"
}

# ---------- add：新增或更新一个 WiFi ----------
cmd_add() {
    need_root add "$@"
    local ssid="${1:-}" pass="${2:-}" prio="${3:-0}"
    [ -z "$ssid" ] && { echo "用法：$0 add <SSID> [密码] [优先级]"; exit 1; }

    local uuids
    uuids="$(find_uuids "$ssid")"

    if [ -n "$uuids" ]; then
        # 已存在：对所有同名连接按 uuid 更新优先级/密码，不删除、不重连
        local n=0
        while IFS= read -r u; do
            [ -z "$u" ] && continue
            n=$((n + 1))
            nmcli connection modify uuid "$u" \
                connection.autoconnect yes \
                connection.autoconnect-priority "$prio"
            if [ -n "$pass" ]; then
                nmcli connection modify uuid "$u" 802-11-wireless-security.psk "$pass"
            fi
        done <<< "$uuids"
        if [ "$n" -gt 1 ]; then
            echo "警告：存在 ${n} 个同名连接，已全部更新；建议用 rm 清理多余的。"
        fi
        echo "已更新：${ssid}（priority=${prio}）"
    else
        # 不存在：新建（密码为空则视为开放网络）
        if [ -n "$pass" ]; then
            nmcli connection add type wifi con-name "$ssid" ssid "$ssid" \
                wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$pass" \
                connection.autoconnect yes connection.autoconnect-priority "$prio" \
                ipv4.method auto ipv6.method auto >/dev/null
        else
            nmcli connection add type wifi con-name "$ssid" ssid "$ssid" \
                connection.autoconnect yes connection.autoconnect-priority "$prio" \
                ipv4.method auto ipv6.method auto >/dev/null
        fi
        echo "已新增：${ssid}（priority=${prio}）"
    fi
}

# ---------- rm：删除一个 WiFi ----------
cmd_rm() {
    need_root rm "$@"
    local ssid="${1:-}"
    [ -z "$ssid" ] && { echo "用法：$0 rm <SSID>"; exit 1; }

    local uuids
    uuids="$(find_uuids "$ssid")"
    if [ -z "$uuids" ]; then
        echo "未找到连接：${ssid}"
        exit 1
    fi
    local n=0
    while IFS= read -r u; do
        [ -z "$u" ] && continue
        n=$((n + 1))
        nmcli connection delete uuid "$u" >/dev/null 2>&1
    done <<< "$uuids"
    echo "已删除：${ssid}（${n} 个连接）"
}

case "${1:-}" in
    list) shift; cmd_list "$@" ;;
    add)  shift; cmd_add  "$@" ;;
    rm)   shift; cmd_rm   "$@" ;;
    *)    usage ;;
esac
