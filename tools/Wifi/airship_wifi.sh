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
#   sudo bash airship_wifi.sh add "于永强的iPhone" "qwertyuiop" 100
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

# ---------- list：打印所有已保存 WiFi ----------
cmd_list() {
    echo "=== 已保存的 WiFi 连接 ==="
    printf "%-28s %-8s %-4s %-6s %s\n" "SSID" "密码" "自动" "优先级" "状态"
    printf "%-28s %-8s %-4s %-6s %s\n" "----" "----" "----" "------" "----"
    while IFS=':' read -r name type dev state; do
        [ "$type" = "802-11-wireless" ] || continue
        prio=$(nmcli --escape no -t -f connection.autoconnect-priority connection show "$name" 2>/dev/null | tr -d '\n')
        ac=$(nmcli --escape no -t -f connection.autoconnect connection show "$name" 2>/dev/null | tr -d '\n')
        psk=$(nmcli --escape no -s -t -f 802-11-wireless-security.psk connection show "$name" 2>/dev/null | tr -d '\n')
        if [ -n "$psk" ]; then psk_s="已设置"; else psk_s="开放/无"; fi
        printf "%-28s %-8s %-4s %-6s %s\n" "$name" "$psk_s" "${ac:-?}" "${prio:-0}" "${state:-}"
    done < <(nmcli --escape no -t -f NAME,TYPE,DEVICE,STATE connection show 2>/dev/null)
    echo "（状态列：activated=当前连接中；deactivated=已保存未连接）"
    echo "提示：想改某个 WiFi 的优先级，用 add 重新指定即可，例如："
    echo "  sudo bash $0 add \"于永强的iPhone\" \"密码\" 100"
}

# ---------- add：新增或更新一个 WiFi ----------
cmd_add() {
    need_root add "$@"
    local ssid="${1:-}" pass="${2:-}" prio="${3:-0}"
    [ -z "$ssid" ] && { echo "用法：$0 add <SSID> [密码] [优先级]"; exit 1; }

    if nmcli --escape no -t -f NAME connection show "$ssid" >/dev/null 2>&1; then
        # 已存在：只更新优先级与密码，不重连、不中断
        nmcli connection modify "$ssid" \
            connection.autoconnect yes \
            connection.autoconnect-priority "$prio"
        if [ -n "$pass" ]; then
            nmcli connection modify "$ssid" 802-11-wireless-security.psk "$pass"
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
    if nmcli --escape no -t -f NAME connection show "$ssid" >/dev/null 2>&1; then
        nmcli connection delete "$ssid"
        echo "已删除：${ssid}"
    else
        echo "未找到连接：${ssid}"
        exit 1
    fi
}

case "${1:-}" in
    list) shift; cmd_list "$@" ;;
    add)  shift; cmd_add  "$@" ;;
    rm)   shift; cmd_rm   "$@" ;;
    *)    usage ;;
esac
