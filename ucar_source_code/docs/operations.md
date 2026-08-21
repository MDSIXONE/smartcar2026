# 省赛操作文档（现场速查）

**1. 查看小车 IP（有线连接，电脑 PowerShell）**
```powershell
arp -a -N 192.168.137.1
```

**2. 连接 WiFi（SSH 进小车后整块复制粘贴执行，比赛前 15 分钟拿到 WiFi 名/密码后只改前两行）**
```bash
NEW_SSID='新WiFi名'
PSK='WiFi密码'

IFACE=$(ls -d /sys/class/net/*/wireless 2>/dev/null | cut -d/ -f5 | head -1)
echo "【1】网卡: ${IFACE:-无}  | 期望: 有名字(如 wlp2s0)，无=驱动没认出网卡"
if [ -n "$IFACE" ]; then
  NM=$(systemctl is-active NetworkManager)
  ST=$(nmcli -t -f DEVICE,STATE device | awk -F: -v i="$IFACE" '$1==i{print $2}')
  echo "【2】NM: $NM / 网卡: $ST  | 期望: active / 不是 unmanaged"
  sudo rfkill unblock wifi 2>/dev/null; sudo nmcli radio wifi on >/dev/null
  if command -v rfkill >/dev/null; then RK=$(rfkill list wifi | grep -c yes); else RK='无rfkill,跳过'; fi
  echo "【3】rfkill阻断项: $RK  | 期望: 0"
  sudo nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1; sleep 3
  SIG=$(nmcli -t -f SSID,SIGNAL device wifi list ifname "$IFACE" | awk -F: -v s="$NEW_SSID" '$1==s{print $2; exit}')
  echo "【4】$NEW_SSID 信号: ${SIG:-没扫到}  | 期望: 数字且>40，没扫到=SSID打错或是隐藏网络"
  if sudo nmcli -w 45 device wifi connect "$NEW_SSID" password "$PSK" ifname "$IFACE" >/dev/null; then
    unset PSK
    ACT=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: -v i="$IFACE" '$2==i{print $1; exit}')
    [ -n "$ACT" ] && sudo nmcli connection modify "$ACT" connection.autoconnect yes
    nmcli -t -f NAME,TYPE connection show | awk -F: '$2~/wireless|wifi/{print $1}' | grep -Fxv "${ACT:-__none__}" \
      | while read -r c; do sudo nmcli connection modify "$c" connection.autoconnect no; done
    GS=$(nmcli -t -f GENERAL.STATE device show "$IFACE" | cut -d: -f2)
    WIP=$(ip -4 -o addr show dev "$IFACE" scope global | awk '{print $4}' | cut -d/ -f1)
    GW=$(nmcli -t -f IP4.GATEWAY device show "$IFACE" | cut -d: -f2)
    echo "【5】状态: $GS  IP: ${WIP:-无}  | 期望: 100 (connected)+有IP ← 拔线后SSH连这个地址"
    LOSS=$(sudo ping -c3 -W2 -I "$IFACE" "${GW:-192.0.2.1}" 2>/dev/null | grep -o '[0-9]*% packet loss')
    echo "【6】ping无线网关(${GW:-没拿到网关}): ${LOSS:-全丢}  | 期望: 0% packet loss"
    if command -v resolvectl >/dev/null; then
      DNS=$(resolvectl query -i "$IFACE" example.com >/dev/null 2>&1 && echo ok || echo fail); VIA='走无线口'
    else
      DNS=$(getent hosts example.com >/dev/null && echo ok || echo fail); VIA='系统默认,此刻可能走有线'
    fi
    echo "【7】DNS($VIA): $DNS  | 期望: ok"
    NEWAC=$(nmcli -t -f connection.autoconnect connection show "$ACT" 2>/dev/null | cut -d: -f2)
    OLD=$(nmcli -t -f NAME,TYPE,AUTOCONNECT connection show | awk -F: -v a="$ACT" '($2~/wireless|wifi/)&&$1!=a&&$3=="yes"{printf "%s ",$1}')
    echo "【8】新SSID自动连: ${NEWAC:-取不到}/ 仍会自动连的旧wifi: ${OLD:-无}  | 期望: yes / 无（决定重启后连哪个）"
    echo "【9】sshd监听: $(ss -tln | awk '$4~/:22$/{printf "%s ",$4}')  | 期望: 含 0.0.0.0:22（改过端口就看你的端口）"
  else
    echo '连接失败，日志:'; sudo journalctl -u NetworkManager -n 30 --no-pager
  fi
fi
unset PSK
```

**3. 打开仿真（电脑 WSL）**
```bash
cd ~/smartcar2026/simulation && bash scripts/start_simulation_stack.sh
```

**4. 重新随机生成锥桶/物块（电脑 WSL 另开终端）**
```bash
cd ~/smartcar2026/simulation && source /opt/ros/noetic/setup.bash && source devel/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11312 && unset ROS_IP ROS_HOSTNAME && rosrun car3 spawn_cubes.py
```

**5. 省赛主程序（小车终端）**
```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑Windows_IP> mission
```

**6. 省赛备用程序（小车终端）**
```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑Windows_IP> mission_alt1
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑Windows_IP> mission_alt2
```

**7. 停止任务**
```bash
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

**应急：主程序 OCR 发散中止修改（改法 A，三步安全改法）**

任务中止日志出现 `diverged twice` 时，把发散阈值 35% 放宽到 200%。先跳转目录：

```bash
cd ~/ucar_ws/src/ucar_2026/scripts
```

**第 1 步：预览（不加 `-i`，不落盘）**
```bash
sed -n '4365p' production_task_2026.py                          # 看改前
sed '4365s/1.35/3.0/' production_task_2026.py | sed -n '4365p'  # 预览改后
```
两行对比确认只变了 `1.35` → `3.0`。

**第 2 步：确认无误才写入**
```bash
sed -i '4365s/1.35/3.0/' production_task_2026.py
```

**第 3 步：验证（改错立刻暴露）**
```bash
python2 -m py_compile production_task_2026.py && echo OK   # 语法检查，报错就是改坏了
sed -n '4365p' production_task_2026.py                     # 再看最终结果
```

改错恢复：数值错就对调回来 `sed -i '4365s/3.0/1.35/' production_task_2026.py`；结构损坏用本机原版 scp 覆盖。语法错不会带病运行，任务节点启动时会直接报错退出。

不需要编译；改完必须重启任务节点才生效。确认执行权限并重启任务：

```bash
chmod +x ~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑Windows_IP> mission
```

**应急：OCR 停车膨胀调整（0.07 → 0.15）**

OCR 停车阶段局部/全局膨胀临时值想放大时（当前 0.07，必须小于 0.25）：

```bash
cd ~/ucar_ws/src/ucar_2026/launch
sed -i 's/processing_parking_inflation_radius_m" value="0.07"/processing_parking_inflation_radius_m" value="0.15"/' 2026.launch 2026_alt1.launch 2026_alt2.launch
grep -n processing_parking_inflation_radius_m 2026.launch 2026_alt1.launch 2026_alt2.launch
```

grep 输出三行都是 `0.15` 即成功。不需要编译；改完必须重启任务节点才生效。改回 0.07 时把上面 sed 里的 `0.15` 和 `0.07` 对调。
