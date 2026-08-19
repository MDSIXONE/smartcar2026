# 额外 OCR 主流程操作文档：ucar_2026_extra

这是带 OCR 快捷路线模板的额外主流程。基础部署按 [部署文档](deployment.md)；标准
主流程的通用安全门槛与停止原则同样适用。本流程与标准/国赛不能同时运行。

## 1. 额外流程的区别

- 入口包：ucar_2026_extra。
- 默认地图：iflysse_field_walls_national.yaml。
- OCR 模板文件：ucar_ws/src/ucar_2026_extra/config/ocr_route_profile.yaml。
- 当前模板默认为空列表 `[]`，因此默认行为沿用国赛 OCR 路线；该文件通过 launch 的 `param="ocr_route_profile"` 加载，不能再添加同名顶层键。
- 只有在车端明确部署并审核过模板内容后，才会按模板执行指定点、朝向、旋转角度、
  旋转方向和 wall/free 停车模式。

## 2. 仿真端启动

三步启动已合并为一个 WSL 终端命令。脚本会显式使用仿真 Master
`127.0.0.1:11312`，并在 `/map` 就绪后启动 bridge；GUI 启动前后自动执行 COPY MODE 预检：

~~~bash
cd ~/smartcar2026/simulation
bash scripts/start_simulation_stack.sh
~~~

无界面联调：

~~~bash
bash scripts/start_simulation_stack.sh --headless
~~~

确认 /map 可读，且：

~~~bash
curl -sS http://<PC_LAN_IP>:11313/status
~~~

终端单独输出 `OK` 且返回 state=waiting 后，再启动小车。

## 3. 车端启动前检查

先确认网络和车端环境：

~~~bash
bash ~/ucar_ws/src/ucar_2026_extra/scripts/start_2026.sh <PC_LAN_IP> check
~~~

启动静态导航检查：

~~~bash
bash ~/ucar_ws/src/ucar_2026_extra/scripts/start_2026.sh <PC_LAN_IP> manual
~~~

另开车端终端设置环境并检查底盘状态：

~~~bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=<VEHICLE_IP>
export ROS_MASTER_URI="http://$ROS_IP:11311"
rostopic echo -n 3 /odom_raw
timeout 5 rosrun tf tf_echo odom base_link
timeout 5 rosrun tf tf_echo map base_link
rostopic hz /scan
~~~

必须确认 /odom_raw 为有限值、两个 TF 连续、雷达稳定，且没有 TF_NAN_INPUT、
crc16、head_len、sensor not active。如果出现 NaN 或 TF 错误，先零速、停止
manual、重启底盘/定位链路，恢复后再继续。

如需使用 OCR 快捷模板，停止 manual 后在车端只读检查文件：

~~~bash
sed -n '1,160p' ~/ucar_ws/src/ucar_2026_extra/config/ocr_route_profile.yaml
~~~

空列表 `[]` 表示使用默认国赛路线；非空模板必须由现场负责人确认点号、
朝向、旋转方向和停车模式，不能在车辆运动时修改。

## 4. 启动额外主流程

确认 manual 已完全退出、车辆在起点、bridge 仍为 waiting 后执行：

~~~bash
bash ~/ucar_ws/src/ucar_2026_extra/scripts/start_2026.sh <PC_LAN_IP> mission
~~~

起点确认提示输入 yes 后，按提示说“小飞小飞”和两个不同类别。流程完成二维码
分类、生产 OCR、实物/仿真联动，再自动交接常驻 lane_proto。运行中可观察：

~~~bash
rostopic echo /ucar_2026_extra/task_state
rostopic echo /ucar_2026_extra/task_result
rostopic echo /lane_proto/state
~~~

## 5. 停止和清理

~~~bash
bash ~/ucar_ws/src/ucar_2026_extra/scripts/stop_2026_task.sh
~~~

之后在一键启动脚本所在终端按 Ctrl-C。脚本会停止 bridge、Gazebo/RViz 和仿真 Master；
检查本次启动的进程都已退出，不要把车端 roscore、roslaunch、bridge 或 Gazebo 留在后台。

## 6. 额外流程故障处理

| 现象 | 处理 |
| --- | --- |
| WARN:COPY MODE / WORN COPY MODE | 立即关闭 GUI；weston 计数不是 0 或 shared memory 不是 tmpfs 时，先 wsl --terminate Ubuntu-20.04，仍失败再 wsl --shutdown，复检通过才能重启。 |
| 11313 不可达 | 重新获取 Windows 局域网 PC_LAN_IP；确认 bridge、防火墙 TCP 11313、mirrored/portproxy 和小车 curl。不要使用 WSL 172.*。 |
| bridge 不是 waiting 或 /map 不存在 | 仿真三终端统一使用 127.0.0.1:11312；修复仿真后重启 bridge，再启动车端。 |
| /odom_raw 为 NaN 或 TF_NAN_INPUT | 零速并停止；重启底盘/定位链路，确认有限 odom 和两个 TF 恢复后才能继续。 |
| OCR 模板点号/方向异常 | 立即停止，不要边跑边改 YAML。检查 ocr_route_profile.yaml 是否为预期版本；模板置空可恢复默认路线，但必须重启主流程后才生效。 |
| crc16、head_len、sensor not active | 这不是 OCR 参数问题；先排查底盘串口、CP2102、USB Hub 和供电，确认链路稳定后再试。 |
| lane_proto 交接失败或 Python2 cv_bridge 错误 | 使用当前车端 lane_proto，让 launch 通过 Melodic Python2 启动器运行；不要直接以 Python3 执行巡线脚本。 |
