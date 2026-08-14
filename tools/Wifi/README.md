# WiFi / SSH 连接工具

树莓派 WiFi 多连接管理与笔记本→树莓派 SSH 连接自检脚本。

## 文件

| 脚本 | 作用 |
|---|---|
| [airship_wifi.sh](./airship_wifi.sh) | 树莓派上管理多个 WiFi（NetworkManager/nmcli），查看/新增/删除/设优先级 |
| [check_ssh_connectivity.sh](./check_ssh_connectivity.sh) | 笔记本上自检能否 SSH 连上树莓派（mDNS/同网段/AP隔离/免密公钥 4 项检查） |

---

## 1. airship_wifi.sh（在树莓派上运行）

```bash
# 查看已保存的 WiFi（SSID / 密码状态 / 优先级 / 当前状态）
sudo bash airship_wifi.sh list

# 新增 / 更新 WiFi：add <SSID> [密码] [优先级]
sudo bash airship_wifi.sh add "于永强的iPhone" "qwertyuiop" 100   # 热点，最优先
sudo bash airship_wifi.sh add "公司WiFi" "公司密码" 50             # 备用
sudo bash airship_wifi.sh add "开放热点" "" 30                     # 开放网络，密码留空

# 删除
sudo bash airship_wifi.sh rm "公司WiFi"
```

- **优先级**：数值越大越优先（默认 0）。多个 WiFi 可用时先连优先级最高的，连不上自动降级。建议：热点 100、公司 50、其他场地递减。
- **幂等**：对已存在的 WiFi 重新 `add` 只更新优先级/密码，不重连不打断。
- `add`/`rm` 需 root；`list` 不需要。
- 现场临时连热点（不长期保存）：`nmcli device wifi connect "<SSID>" password "<密码>"`

---

## 2. check_ssh_connectivity.sh（在笔记本上运行）

```bash
bash check_ssh_connectivity.sh            # 使用 ~/.ssh/config 别名 pi
bash check_ssh_connectivity.sh -a pi      # 指定别名
bash check_ssh_connectivity.sh -h lingyun01.local -u lingyun01 -p 22   # 直接给参数
```

依次检查 4 项，每项输出 `PASS/FAIL/WARN`，失败会给出修复命令：

1. 笔记本 mDNS 支持（avahi / systemd-resolved）
2. mDNS 解析 `主机名.local`
3. 同一局域网 + 热点未开「AP 隔离」（ping + TCP 22 端口）
4. SSH 免密公钥是否已部署

---

## 常见注意

- 手机热点需**关闭「AP 隔离 / 设备隔离」**，否则同一热点下树莓派与电脑互相访问不到。
- wlan0 走 DHCP，IP 会变；建议用主机名 `ssh <用户>@<主机名>.local`（需树莓派 avahi-daemon 运行）。
- 若 `/etc/netplan/` 下存在旧 wlan0 配置（如 `99-airship-wifi-hotspot.yaml`），手动删除并 `netplan apply`，只保留 eth0 的 `90-NM-*.yaml`。
- eth0 静态 IP `10.41.10.100/24`（连飞控）与本套 WiFi 方案互不影响。
