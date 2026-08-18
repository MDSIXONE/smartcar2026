# 标准主流程操作文档：ucar_2026

这是标准 2026 双物品主流程的简明启动手册。新电脑第一次部署、WSL 安装、动态 IP、
防火墙和 COPY MODE 处理，先看 [部署文档](deployment.md)。国赛和额外 OCR 流程分别看
[国赛操作文档](operations-national.md) 和 [额外 OCR 操作文档](operations-extra.md)。
任务运行中的本机 RViz 观察方法、现场无需编译调参清单和文件位置见
[RViz/现场调试文档](debug-rviz-observation.md)。

## 生产地图墙体像素修正

生产编号图和实车运行地图的墙体像素修正由同一工具生成。工具会修正省赛 PGM、按 148-159 → 147-158 生成国赛 PGM，并同步省赛/国赛/额外任务编号 PNG；不启动 ROS、不编译、不连接小车。

在 `ucar_source_code` 目录执行：

~~~powershell
python tools/fix_production_map_pixels.py
~~~

执行后应看到 `updated provincial and national production map pixels`。工具内置像素级断言；另需确认省赛包 PNG 与根目录 PNG 一致、国赛与额外任务 PNG 一致。部署前仍须按对应流程停止旧导航并在车端加载新 PGM，不能在地图文件替换后继续使用旧的 `map_server` 进程。

车端部署只同步运行时 PGM，不在车端创建备份目录或归档文件。先按当前局域网/DHCP 信息动态确认 `$CAR_IP`，再执行：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.pgm "ucar@$CAR_IP`:~/ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.pgm"
scp ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.pgm "ucar@$CAR_IP`:~/ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.pgm"
ssh "ucar@$CAR_IP" 'sha256sum ~/ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.pgm ~/ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.pgm'
~~~

将车端哈希与本地 `Get-FileHash -Algorithm SHA256` 结果逐一比对。替换文件不会让已加载的 `map_server` 自动重新读取地图；若主流程正在运行，须按对应流程重启 `map_server`/导航链路。部署本身不启动 ROS、不编译，也不发送运动命令。

## 膨胀区触发事件式重规划

`ucar_cym_planner_params.yaml` 中点/冲刺模式的 `obstacle_cost_threshold` 为 `1` 时，local costmap 的任意非零 raw cost 都会被视为路径进入膨胀区，并让 CymPlanner 返回失败；在 `planner_frequency: 0.0` 下由 `move_base` 走事件式重规划路径。车体投影模式仍按完整 footprint 和弹性带逻辑处理。修改 YAML 不需要编译，但必须重启对应 `move_base`/2026 主流程后才会加载。

车端同步前先按当前动态网络地址确认目标，再仅替换参数文件；不在车端创建备份目录：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml "ucar@$CAR_IP`:~/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml"
ssh "ucar@$CAR_IP" 'grep -n "obstacle_cost_threshold" ~/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml'
~~~

当前 CymPlanner 判定读取 local costmap；local inflation 已在后续配置中同步为 `0.224 m`，因此重启主流程后可覆盖与全局相同的安全带范围。

## 局部代价地图与前视范围

当前实车配置使用 `1.8×1.8 m` rolling local costmap、`0.02 m` 分辨率、`0.224 m` local inflation 和 `0.8 m` CymPlanner 前视距离。局部窗口约为 `90×90` 格；三者应同时保持：窗口半边至少覆盖前视距离，local inflation 才能在前视点进入安全带前被采样到。修改这些 YAML 后不需要编译，但必须重启对应 `move_base`/2026 主流程；不在运行任务期间强制重启。

## 局部动态代价层在点 3 启用

标准、省赛/国赛和额外任务默认在点 3 前关闭 local costmap 的 `obstacle_layer`、`inflation_layer`，点 3 导航成功返回后再通过 dynamic_reconfigure 同时打开。local costmap 容器和 StaticLayer 始终运行，不能把整个 local costmap 关闭，否则 CymPlanner 会因 `isCurrent()` 不满足而停止输出速度。额外任务使用非空 `ocr_route_profile` 时不经过点 3，该快捷流程不执行这组切换。

同一个点 3 成功回调还会把 `/move_base/global_costmap/inflation_layer` 的 `inflation_radius` 设置为 `0.224m` 并保持到任务结束；断点续跑启动时会重新设置一次。点 3 后 local costmap 的常态 inflation 也为 `0.224m`，两者是独立的 dynamic_reconfigure/启动配置。local 在 OCR 停车阶段临时变为 `0.05m` 时，global 仍保持 `0.224m`，停车结束后 local 恢复 `0.224m`。服务缺失或回读值不一致会中止任务。

源码/launch 改动部署后必须重启任务节点；先动态发现车端 IP，不在车端创建备份目录：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/ucar_2026/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/launch/2026.launch"
scp ucar_ws/src/ucar_2026_national/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/launch/2026.launch"
scp ucar_ws/src/ucar_2026_extra/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/launch/2026.launch"
scp ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml "ucar@$CAR_IP`:~/ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml"
scp ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml "ucar@$CAR_IP`:~/ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml"
ssh "ucar@$CAR_IP" 'grep -n "set_global_costmap_inflation_radius\|global_costmap_inflation_radius_m\|reached_point_3" ~/ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py ~/ucar_ws/src/ucar_2026_national/launch/2026.launch'
~~~

只同步 Python2 源码、launch 和 YAML 时不需要编译；由于新增了 `dynamic_reconfigure` 运行依赖，若车端工作区依赖未安装，先在 Ubuntu 18.04 / ROS Melodic 检查 `rospack find dynamic_reconfigure`，再按部署文档执行白名单构建。启动后只读查看 `/move_base/local_costmap/obstacle_layer/set_parameters`、`/move_base/local_costmap/inflation_layer/set_parameters` 和 `/move_base/global_costmap/inflation_layer/set_parameters` 服务是否存在，并在任务日志中确认点 3 前后时序。

2026-08-18 本轮已将共享 local costmap YAML、三套任务脚本和三套 `2026.launch` 同步到 `ucar-mini`（`192.168.8.231`）；7 个文件的本地/车端 SHA-256 一致。同步未启动 ROS、主流程或车辆运动，参数需下次安全重启对应主流程后加载。

随后 global costmap 目标值调整为 `0.224m`，已再次同步 global costmap 配置、三套任务脚本和三套 `2026.launch`；7 个文件的本地/车端 SHA-256 一致，车端仍未启动 ROS 或主流程。

## OCR 识别后内墙停车坐标

OCR 识别并完成墙面交点测量后，三套 2026 主流程通过 `ocr_stop_offset_m` 计算内墙交点向场内的停车坐标。当前值为 `0.25m`；生产网格的 `square_side_m=0.5` 不变。

该参数位于三个包的 `launch/2026.launch`，只修改参数不需要编译，但必须重启对应主流程后生效。部署前在 `ucar_source_code` 目录执行本地核对：

~~~powershell
@'
import xml.etree.ElementTree as ET
from pathlib import Path
files = [
    Path('ucar_ws/src/ucar_2026/launch/2026.launch'),
    Path('ucar_ws/src/ucar_2026_national/launch/2026.launch'),
    Path('ucar_ws/src/ucar_2026_extra/launch/2026.launch'),
]
for path in files:
    ET.parse(str(path))
    text = path.read_text(encoding='utf-8')
    assert 'ocr_stop_offset_m' in text and 'value="0.25"' in text
print('OCR stop offset: 0.25m')
'@ | python -
~~~

源码/launch 同步到车端后，先按动态 IP 检查车端文件内容，再在 `/odom_raw`、`odom -> base_link`、`map -> base_link` 均正常且车辆零速时重启任务；本轮不在车端创建备份目录，也不在运行任务期间热修改参数。

## 国赛终点雷达角落停车

国赛 `2026.launch` 的常驻 `lane_proto` 已设置 `use_lidar=true`。视觉命中终点线后，节点直接进入 `CORNER_ADJUST`，不调用终点导航；它从 `/scan` 拟合相邻两面墙，目标为车心到两墙均 `0.25m`，容差 `±0.01m`，连续 5 帧稳定后发布 `/lane_proto/result=GOAL` 并退出。任一墙距超过 `1.0m` 时使用原来的 `PAUSE + APPROACH` 兜底；雷达陈旧/拟合失败超时发布 `ABORT`。

现场只读观察：

~~~bash
rostopic echo /lane_proto/state
rostopic echo /lane_proto/result
rostopic echo /scan
~~~

应观察到 `FOLLOW -> CORNER_ADJUST -> STOPPED`，并且只有 `result=GOAL` 才会使国赛主流程发布 `SUCCEEDED`。出现 `ABORT`、`ESTOP` 或 `CONFIG` 时停止任务，不得把 `STOPPED` 当作成功。当前默认墙线阈值为每面至少 8 点、墙段跨度至少 `0.12m`、拟合残差不超过 `0.025m`、夹角误差不超过 `10°`。

源码/launch 同步后不需要本机编译；必须先按动态 IP 同步到车端，再在 `/odom_raw`、两个 TF 和 `/scan` 正常、车辆零速时重启国赛主流程。第一次实车仅做低速静止起步/终点雷达闭环观察，不在运动过程中热改阈值。

本轮终点雷达停车的运行文件只同步以下 5 个文件，不在车端创建备份目录；`$CAR_IP` 必须按当前网络重新确认：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/lane_proto/scripts/lane_common.py "ucar@$CAR_IP`:~/ucar_ws/src/lane_proto/scripts/lane_common.py"
scp ucar_ws/src/lane_proto/scripts/lane_follow.py "ucar@$CAR_IP`:~/ucar_ws/src/lane_proto/scripts/lane_follow.py"
scp ucar_ws/src/lane_proto/launch/lane_proto.launch "ucar@$CAR_IP`:~/ucar_ws/src/lane_proto/launch/lane_proto.launch"
scp ucar_ws/src/ucar_2026_national/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/launch/2026.launch"
scp ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py"
ssh "ucar@$CAR_IP" 'sha256sum ~/ucar_ws/src/lane_proto/scripts/lane_common.py ~/ucar_ws/src/lane_proto/scripts/lane_follow.py ~/ucar_ws/src/lane_proto/launch/lane_proto.launch ~/ucar_ws/src/ucar_2026_national/launch/2026.launch ~/ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py'
~~~

部署后在车端只读检查 `python2` 语法和 `roslaunch --nodes ucar_2026_national 2026.launch task_enabled:=true`；确认车辆零速、`/odom_raw` 和两个 TF 有限、`/scan` 新鲜后，才允许重启国赛主流程。部署本身不自动重启任务。

### OCR 内墙停车分阶段膨胀

正常轨迹规划使用 local costmap 当前的 `0.224m` 膨胀半径。到达点 3 后，任务会将 `/move_base/global_costmap/inflation_layer` 的全局膨胀半径切换为并保持 `0.224m`；断点续跑也会先重新应用该值。该全局值与 local costmap 的分阶段切换相互独立。

OCR 识别完成后，任务进入内墙最终停车前，会将 `/move_base/local_costmap/inflation_layer` 临时切换到 `0.05m`，同时显式保持 CymPlanner 的 `point` 模式；`obstacle_layer` 和 `static_layer` 保持运行。停车动作无论成功、失败还是超时，都会恢复进入前读取到的局部膨胀半径并再次确认 `point` 模式。车端不得在此流程使用 `body_projection`。

三个主流程当前参数均为：

~~~xml
<param name="processing_parking_profile_enabled" value="true"/>
<param name="processing_parking_inflation_radius_m" value="0.05"/>
<param name="ocr_stop_offset_m" value="0.25"/>
<param name="global_costmap_inflation_layer"
       value="/move_base/global_costmap/inflation_layer"/>
<param name="global_costmap_inflation_radius_m" value="0.224"/>
~~~

该切换依赖车端提供两个 inflation layer 的 `set_parameters` 服务，任务会分别校验局部临时半径和点 3 后全局半径确实生效；服务缺失或返回值不一致时会中止，不得继续运动。源码/YAML 同步后必须重启对应 `move_base` 和 2026 主流程；现场复测前仍需先确认动态 IP、`/odom_raw`、两个 TF 和零速度安全门。额外任务使用非空 `ocr_route_profile` 的快捷流程不经过点 3，因此不执行点 3 的全局切换。

## 摄像头稳定设备别名

车端 USB 摄像头的 `/dev/videoN` 编号会随 USB 重新枚举变化。当前已确认的 RHX 摄像头由 udev 规则固定提供 `/dev/ucar_camera`，标准、省赛/国赛和额外任务主流程默认都使用这个别名；其实际目标可以是 `/dev/video0` 或 `/dev/video1`，不再由程序依赖编号。

源码和规则部署到车端时，先按当前网络动态确认 `$CAR_IP`，不在车端创建备份目录：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/startup_scripts/ucar_camera.rules "ucar@$CAR_IP`:~/ucar_ws/src/startup_scripts/ucar_camera.rules"
scp ucar_ws/src/ucar_2026/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/launch/2026.launch"
scp ucar_ws/src/ucar_2026_national/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/launch/2026.launch"
scp ucar_ws/src/ucar_2026_extra/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/launch/2026.launch"
ssh "ucar@$CAR_IP" 'sudo install -m 0644 ~/ucar_ws/src/startup_scripts/ucar_camera.rules /etc/udev/rules.d/99-ucar-camera.rules && sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=video4linux && ls -l /dev/ucar_camera && readlink -f /dev/ucar_camera'
~~~

规则刷新前先停止正在执行的任务节点；不要在运动过程中触发 USB 摄像头重新枚举。仅替换源码不会改变已经启动的 `usb_cam` 节点，必须在安全检查通过后重启对应 2026 主流程。车端验证时确认 `/dev/ucar_camera` 存在、`readlink -f` 指向当前摄像头，再观察 `/usb_cam/start_capture` 是否成功。

本轮同时改了 `usb_cam/src/camera_driver.cpp` 的裸节点默认值。该文件只能在车端 Ubuntu 18.04 / ROS Melodic 编译：

~~~bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="usb_cam"
~~~

只编译 `usb_cam`，不要在任务运行或车辆运动期间执行构建；构建前先完成零速和 `/odom_raw`、TF 安全检查。

## USB Hub 热重连验证

`usb_cam` 现在会把 USB 热断开视为设备生命周期事件：运行中读帧收到 `ENODEV` 等断开错误后，释放旧 fd 和 mmap 缓冲，按 `reconnect_interval=0.5s` 重新打开 `/dev/ucar_camera`；点 52 的 `/usb_cam/start_capture` 请求最多等待 `reconnect_timeout=8.0s`。手动 `/usb_cam/stop_capture` 不会触发自动重连。

验证只能在车辆静止、无任务运动时进行。先启动相机节点并确认 `/usb_cam/image_raw`，再由现场人员断开/恢复 USB Hub，观察：

~~~bash
rostopic hz /usb_cam/image_raw
readlink -f /dev/ucar_camera
grep -E 'USB_CAM_RECONNECT|Video4linux' ~/.ros/log/latest/usb_cam*.log
~~~

出现 `USB_CAM_RECONNECT capture resumed` 后，再验证 QR 流程；不要在车辆运动中人为断开 USB Hub。超过 8 秒未恢复时，本次 start service 会失败，下一次 start 请求仍会重新尝试。

## 国赛与额外任务 70 号点坐标校准

国赛和额外任务配置中的 70 号点当前坐标均为 `(2.32, 1.68)`；这是在原坐标
`(2.25, 1.75)` 基础上执行 `x + 0.07m`、`y - 0.07m` 的结果。修改后只需做本地
JSON 解析和差值核对，不需要在本机编译；正式启动车端前仍须按对应流程的安全检查
确认 `/odom_raw` 与 TF 有限且正常。

上述 `(2.32, 1.68)` 是本地工作区已有的未提交校准记录；本次独立调试部署不把该网格
文件同步到车端，车端国赛共享任务仍保持原坐标 `(2.25, 1.75)`。

本地只读核对命令（在 `ucar_source_code` 目录执行）：

~~~bash
python3 -c 'import json; ps=["ucar_ws/src/ucar_2026_national/config/production_full_grid_all_numbered.json","ucar_ws/src/ucar_2026_extra/config/production_full_grid_all_numbered.json"]; qs=[[x for x in json.load(open(p))["points"] if x["number"]==70] for p in ps]; assert all(len(q)==1 and abs(q[0]["x_m"]-2.25-0.07)<1e-9 and abs(q[0]["y_m"]-1.75+0.07)<1e-9 for q in qs); print(qs)'
~~~

## 国赛冲刺航向环 P 调参

国赛 70→坡顶冲刺使用 `mode3_sprint`。当前航向角度环 P 为
`angular_gain=5.0`（由 `10.0` 减半）；本轮对比将 `linear_x_gain` 由 `12.5` 调为
`13.5`、`max_vel_x` 由 `2.5` 调为 `2.7`。CymPlanner 没有独立的加速度上限字段，
因此 `linear_x_gain` 只作为前向加速响应对比项；本轮任务层 `sprint_yaw_deg=180`。
该配置只在下一次国赛主流程启动时加载，不需要本机编译。

小车端同步后，在动态设置当前车端 `ROS_MASTER_URI` 的终端执行只读核对：

~~~bash
grep -A8 '^  mode3_sprint:' ~/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml
~~~

若本地参数有改动，按当前发现到的车端地址同步单个 YAML（不要在车端创建备份目录）：

~~~bash
CAR_IP=<当前车端IP>
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml \
  ucar@$CAR_IP:~/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml
sha256sum ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml
ssh ucar@$CAR_IP 'sha256sum ~/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml'
~~~

本轮已同步到车端 `192.168.8.231`；参数只在下一次国赛主流程启动时加载，当前运行中的
规划器不会热更新。

## 国赛 70→坡顶独立冲刺速度调试

独立入口位于 `ucar_2026_national/national_sprint_speed_debug.launch`。它只加载底盘、
雷达定位、国赛地图、CymPlanner/move_base 和调试节点，不启动相机、二维码、OCR、
生产任务或巡线。小车必须物理放在国赛 70 号点，车端保持原坐标 `(2.25, 1.75)`；程序把
67 与 290 中点 `(0.875, 1.75)` 作为坡顶目标。默认 `run:=false`，只有显式传
`run:=true` 才会发送目标。调试程序只读取车端现有网格 JSON，不得修改或同步国赛共享
网格文件。

先停止现有标准/国赛/额外流程并发布零速度；按照本次 Wi-Fi 网络发现的地址设置唯一
WSL ROS Master，不能在车端启动 `roscore`：

~~~bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://<本次发现的WSL_MASTER_IP>:11311"
export ROS_IP="$(ip -4 route get <本次发现的WSL_MASTER_IP> | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
~~~

源码、launch 和 `CMakeLists.txt` 先按动态车端地址同步；不在车端创建备份目录或归档：

~~~powershell
$CAR_IP = '<按 rosmaster/NETWORK_CONFIGURATION.md 动态确认的车端IP>'
scp ucar_ws/src/ucar_2026_national/scripts/national_sprint_speed_debug.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/"
scp ucar_ws/src/ucar_2026_national/launch/national_sprint_speed_debug.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/launch/"
scp ucar_ws/src/ucar_2026_national/CMakeLists.txt "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/"
scp ucar_ws/src/ucar_2026_national/test/test_national_sprint_speed_debug.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/test/"
ssh "ucar@$CAR_IP" 'chmod +x ~/ucar_ws/src/ucar_2026_national/scripts/national_sprint_speed_debug.py'
~~~

只在车端 Ubuntu 18.04 / ROS Melodic 编译并执行定向测试；每次 `source` 后重新设置
本次 WSL Master：

~~~bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://<本次发现的WSL_MASTER_IP>:11311"
export ROS_IP="$(ip -4 route get <本次发现的WSL_MASTER_IP> | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
ORIGINAL_WHITELIST="$(grep '^CATKIN_WHITELIST_PACKAGES:' build/CMakeCache.txt | sed 's/.*=//')"
catkin_make --force-cmake "-DCATKIN_WHITELIST_PACKAGES=ucar_2026_national"
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://<本次发现的WSL_MASTER_IP>:11311"
export ROS_IP="$(ip -4 route get <本次发现的WSL_MASTER_IP> | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
python2 -m unittest discover -s src/ucar_2026_national/test -p 'test_national_sprint_speed_debug.py' -v
sed -n '698,712p' src/ucar_2026_national/config/production_full_grid_all_numbered.json
catkin_make --force-cmake "-DCATKIN_WHITELIST_PACKAGES=$ORIGINAL_WHITELIST"
~~~

运动前必须确认 `/odom_raw` 连续有限、`odom -> base_link` 和 `map -> base_link` TF
均恢复且无 `TF_NAN_INPUT`；否则先停止并重启导航/底盘里程计链路。确认通过后，在车端
启动一次试跑：

~~~bash
roslaunch ucar_2026_national national_sprint_speed_debug.launch \
  run:=true sprint_linear_x_gain:=13.5 sprint_max_vel_x:=2.7
~~~

对比速度时只改上述两个参数，例如 `sprint_linear_x_gain:=10.0
sprint_max_vel_x:=2.0`。日志中的 `NATIONAL_SPRINT_DEBUG complete` 会记录本次
`/cmd_vel` 的请求峰值；该值不是轮速计实际车速。调试节点是 `required`，完成或中止后
应确认 launch/底盘终端均退出；结束前再次发布零速度，不得让启动终端残留后台节点。

## 生产任务 Python2 源码部署

每次换 Wi-Fi 先按 [部署文档](deployment.md) 动态确认 `$CAR_IP`，只同步源码和测试文件，
不在车端创建备份目录；Python 脚本必须在车端 Ubuntu 18.04 / ROS Melodic 验证：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/ucar_2026/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026/test/test_production_task_geometry.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/test/test_production_task_geometry.py"
scp ucar_ws/src/ucar_2026_extra/test/test_production_task_geometry.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/test/test_production_task_geometry.py"
ssh "ucar@$CAR_IP" 'cd ~/ucar_ws && source /opt/ros/melodic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;lane_proto;ucar_2026;ucar_2026_national;ucar_2026_extra"'
~~~

部署后只读核对两端 SHA-256，并运行对应 Python2 定向测试；构建和测试不会启动 ROS Master、
主流程或车辆运动。当前 409 兜底版本已部署到 `ucar-mini`，但额外任务全量测试仍有一个既有
`observe()` 测试桩不接受 `stop_mode` 参数的问题，新增 409 用例单独通过。

## WSL 仿真源码同步基准

分享仿真时，以 WSL 当前实际运行的 `/home/car/smartcar2026/simulation` 为唯一源码基准；
Windows 目录 `D:\WORK\ALLCODE\smartcar2026\simulationforreal\simulation` 只是分享镜像。
修改仿真源码后先在 WSL 验证，再从 WSL 同步到 Windows，不要反向覆盖 WSL。

需要重新生成 Windows 分享镜像时，在 PowerShell 执行以下命令。`--delete` 只作用于仿真目录，
并保留两端自行生成的构建产物、日志、临时文件和训练产物：

~~~powershell
wsl.exe -d Ubuntu-20.04 -- rsync -a --delete `
  --exclude=.git/ --exclude=build/ --exclude=devel/ --exclude=logs/ `
  --exclude=tmp/ --exclude=Testing/ --exclude=.vscode/ --exclude=.claude/ `
  --exclude=.agents/ --exclude=__pycache__/ --exclude='task3_run_*.log' `
  --exclude='*.pt' --exclude='*.zip' --exclude='yolov5/yolov5/runs/' `
  --exclude=datasets/ /home/car/smartcar2026/simulation/ `
  /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/simulation/
~~~

同步后的校验只比较内容，不比较 Windows 挂载盘权限位：

~~~powershell
wsl.exe -d Ubuntu-20.04 -- bash -lc 'diff -qr --strip-trailing-cr --exclude=.git --exclude=build --exclude=devel --exclude=logs --exclude=tmp --exclude=Testing --exclude=.vscode --exclude=.claude --exclude=.agents --exclude=__pycache__ --exclude="task3_run_*.log" --exclude="*.pt" --exclude="*.zip" --exclude=runs --exclude=datasets /home/car/smartcar2026/simulation /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/simulation'
~~~

命令无输出才表示分享内容一致；`build/`、`devel/` 等目录不属于分享源码，必须在使用者自己的
WSL 中重新构建。

若 WSL 工作副本存在必须保留的本地改动，只部署本轮仿真运行时文件时，先核对目标命令行和
目标目录，再执行精确覆盖；不要递归同步整个仿真目录：

~~~powershell
wsl.exe -d Ubuntu-20.04 -- bash -lc "install -m 0755 /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/simulation/scripts/start_simulation_stack.sh /home/car/smartcar2026/simulation/scripts/start_simulation_stack.sh; install -m 0644 /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/simulation/bridge/sim_bridge.py /home/car/smartcar2026/simulation/bridge/sim_bridge.py; sha256sum /home/car/smartcar2026/simulation/scripts/start_simulation_stack.sh /home/car/smartcar2026/simulation/bridge/sim_bridge.py"
~~~

部署后必须按哈希结果确认两文件一致；已有 bridge 不会热加载新代码，需先停止已核对的旧 PID，
再启动下一轮仿真。

## 1. 启动前提

- 电脑和小车在同一个可信局域网；PC_LAN_IP 是 Windows WLAN/有线网卡 IPv4。
- 仿真一键启动脚本已在 WSL 运行，并显示 simulation bridge listening on 0.0.0.0:11313 (state=waiting)。
- 小车当前没有其他 2026.launch、独立 lane_follow.py 或相机/底盘串口占用者。
- 车辆已物理放回起点 (-0.25, 2.75, 0)，车头方向正确，急停在手边。

## 2. 仿真端启动

仿真三步已合并为一条命令。脚本会自动 source Noetic 和仿真工作区，设置独立的
`ROS_MASTER_URI=http://127.0.0.1:11312`，依次启动仿真 `roscore`、Gazebo/RViz 和 bridge；
GUI 启动前后会执行 WSLg COPY MODE 预检，并等待 `/map` 后才启动 bridge：

~~~bash
cd ~/smartcar2026/simulation
bash scripts/start_simulation_stack.sh
~~~

无界面联调：

~~~bash
bash scripts/start_simulation_stack.sh --headless
~~~

终端单独出现 `OK` 后，才表示三项仿真服务全部启动成功；看到 SIMULATION_BRIDGE_READY 和
state=waiting 后再操作小车。若这是新一轮任务，
必须在该脚本终端按 Ctrl-C 完整停止后重新运行，不能复用已经返回 done 的 bridge。

## 3. 车端安全检查

先做网络和运行环境检查：

~~~bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <PC_LAN_IP> check
~~~

然后启动无自动目标的导航检查模式：

~~~bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <PC_LAN_IP> manual
~~~

另开车端终端，按启动脚本打印的车端 Master 地址设置环境；下面用到的 VEHICLE_IP
必须是本次到电脑路由所选的车端 IPv4：

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
rostopic info /cmd_vel
~~~

全部满足以下条件才能进入 mission：

- /odom_raw 连续为有限值，不能有 NaN；
- odom -> base_link 和 map -> base_link 都能连续输出，不能有 TF_NAN_INPUT；
- /scan 稳定发布；
- 日志没有 crc16、head_len、sensor not active、No such device；
- cmd_vel 的唯一底盘发布链路符合当前 launch；
- 小车位于真实起点，Gazebo/RViz 可见，bridge 状态仍为 waiting。

检查不通过时，在 manual 终端按 Ctrl-C，不要继续发送目标。需要重启导航或底盘
里程计链路前，先发布零速度：

~~~bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
~~~

## 二维码扫描方向

三套 2026 主流程到达二维码中心点 52 后，固定观察顺序为
`180°→90°→-90°→-135°→135°→45°`，对应点号 `[262, 232, 295, 61, 41, 43]`。
车辆保持在点 52，仅通过 move_base/CymPlanner 调整车头；后续点 3、OCR、仿真联动和终点交接流程不变。

修改该序列后，先停止旧任务，再按当前动态发现的车端地址同步对应入口包的脚本和 launch；三套包若都要保留一致行为则全部同步：

~~~powershell
$CAR_IP = '<当前动态确认的车端IP>'
scp ucar_ws/src/ucar_2026/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026/scripts/production_qr_classifier.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/scripts/production_qr_classifier.py"
scp ucar_ws/src/ucar_2026/scripts/production_task_geometry.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/scripts/production_task_geometry.py"
scp ucar_ws/src/ucar_2026/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026/launch/2026.launch"
scp ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026_national/scripts/production_qr_classifier.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/production_qr_classifier.py"
scp ucar_ws/src/ucar_2026_national/scripts/production_task_geometry.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/scripts/production_task_geometry.py"
scp ucar_ws/src/ucar_2026_national/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_national/launch/2026.launch"
scp ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026_extra/scripts/production_qr_classifier.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/scripts/production_qr_classifier.py"
scp ucar_ws/src/ucar_2026_extra/scripts/production_task_geometry.py "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/scripts/production_task_geometry.py"
scp ucar_ws/src/ucar_2026_extra/launch/2026.launch "ucar@$CAR_IP`:~/ucar_ws/src/ucar_2026_extra/launch/2026.launch"
ssh "ucar@$CAR_IP" 'grep -n "qr_observation_numbers" ~/ucar_ws/src/ucar_2026/launch/2026.launch ~/ucar_ws/src/ucar_2026_national/launch/2026.launch ~/ucar_ws/src/ucar_2026_extra/launch/2026.launch'
~~~

二维码分类请求使用 `request_id` 绑定响应；分类器响应超时为 8 秒，超时会销毁旧 helper，
下一次分类重新启动干净进程，避免迟到响应串给下一个二维码。分类器仍保留车端本地关键词表，
所以远端 Spark 不可用时会在单次有界请求后走本地分类；分类失败的二维码不会被永久标记为已用，
会在当前有限扫描轮次内重试。同步后必须重新启动实际使用的 `2026.launch`，并在日志中确认
`PRODUCTION_SPARK_STALE_RESPONSE`、`PRODUCTION_VOICE_QR_UNCLASSIFIED`（如发生）和后续
`PRODUCTION_SPARK_CLASSIFY` 的 observation/category 对应一致。

同步后必须停止并重新启动实际使用的 `2026.launch`，运行中节点不会重新读取 launch 参数。启动前按本文件安全检查确认 `/odom_raw` 为有限值且 `odom -> base_link`、`map -> base_link` TF 已恢复；出现 NaN 或 `TF_NAN_INPUT` 时先零速并重启底盘/定位链路。启动后观察日志应依次出现六个 `QR_FACE_<编号>` 状态。仅做 Python/launch 静态检查时，在本机执行：

~~~powershell
python -m unittest discover -s ucar_ws/src/ucar_2026/test -p 'test_production_task_geometry.py' -v
python -m unittest discover -s ucar_ws/src/ucar_2026_national/test -p 'test_production_task_geometry.py' -v
python -m unittest discover -s ucar_ws/src/ucar_2026_extra/test -p 'test_production_task_geometry.py' -v
~~~

## 4. 启动标准主流程

确认 manual 已完全退出后，在小车主终端执行：

~~~bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <PC_LAN_IP> mission
~~~

启动脚本会再次预检 PC_LAN_IP:11313，并询问车辆是否已回到起点；只有确认后输入
yes。任务节点启动后：

1. 唤醒词说“**小飞小飞**”；
2. 按语音提示说两个不同类别，例如“日用品”和“食品”；
3. 等待二维码分类、实物停靠和播报；
4. 实物完成后，小车会通过 bridge 启动仿真物品，仿真最多等待约 120 秒；
5. 仿真结束或超时后继续前往终点并自动交接 lane_proto 巡线。

运行中只读观察：

~~~bash
rostopic echo /ucar_2026/task_state
rostopic echo /ucar_2026/task_result
rostopic echo /lane_proto/state
~~~

仿真端可观察：

~~~bash
curl -sS http://<PC_LAN_IP>:11313/status
~~~

## 5. 停止和清理

立即停止真车任务：

~~~bash
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
~~~

然后确认小车任务/导航退出，在仿真一键启动脚本所在终端按一次 Ctrl-C；脚本会依次停止
bridge、task3_prepare.launch（含 Gazebo/RViz）和仿真 roscore；每个子进程等待 5 秒仍未退出时，
会依次升级为 SIGTERM、SIGKILL。

不要用宽泛的 pkill ros* 结束其他 ROS 会话。确认本次没有残留后再结束工作：

~~~bash
pgrep -af 'sim_bridge|task3_prepare|task3_execute|gzserver|gzclient|roscore|rosmaster|roslaunch'
~~~

如果 bridge 因旧终端关闭、脚本被强制结束或上一轮异常退出而残留，先只核对
`11313` 的监听进程确实是本仿真的 `sim_bridge.py`，再按 PID 清理：

~~~bash
BRIDGE_PID="$(ss -H -ltnp 'sport = :11313' | grep -oE 'pid=[0-9]+' | head -n 1 | cut -d= -f2)"
ps -o pid=,ppid=,stat=,args= -p "$BRIDGE_PID"
kill -INT "$BRIDGE_PID"
sleep 2
if kill -0 "$BRIDGE_PID" 2>/dev/null; then kill -TERM "$BRIDGE_PID"; fi
sleep 2
if kill -0 "$BRIDGE_PID" 2>/dev/null; then kill -KILL "$BRIDGE_PID"; fi
ss -H -ltnp 'sport = :11313'
~~~

只对 `ps` 输出中命令为本项目 `simulation/bridge/sim_bridge.py` 的 PID 执行上述清理，
不要对未知监听进程发送信号。新版一键启动脚本会在启动 Gazebo 前检查该端口，并在等待
就绪时核对端口实际归属的 PID；旧 bridge 占用端口时会直接报出 PID，不会再把旧的
`/status` 响应误判为新 bridge 已就绪。

## 6. 最常见故障

| 现象 | 处理 |
| --- | --- |
| 11313 不可达 / No route to host | 重新确认 PC_LAN_IP 是 Windows 局域网 IPv4；确认 bridge 已启动、防火墙放行 TCP 11313、WSL mirrored 或 portproxy 已生效。任务未进入运动前先修复。 |
| bridge 报 `Address already in use` | 说明旧 bridge 仍占用 11313；启动器会直接打印 `ps`、`kill -TERM` 和 `kill -KILL` 命令。先核对命令行确实是本项目 `sim_bridge.py`，执行提示命令清理，确认 `ss -H -ltnp 'sport = :11313'` 无输出后再启动；不会复用旧 bridge 的 `/status`。 |
| `/start` 返回 `409 already running` | 主流程不再因此中止，会转入 `/status` 轮询；120 秒未完成后继续终点流程。赛后仍需停止残留 bridge/roslaunch，并重启 bridge 使下一轮回到 `state=waiting`。 |
| 出现 WARN:COPY MODE 或用户写的 WORN COPY MODE | 关闭 Gazebo/RViz；检查 grep -c 'use_gfxredir = 0' /mnt/wslg/weston.log 必须为 0、findmnt -no FSTYPE /mnt/shared_memory 必须为 tmpfs；失败时先 wsl --terminate Ubuntu-20.04，仍失败再 wsl --shutdown。 |
| bridge 一直 waiting 前启动任务 | 不要启动任务；先确认一键脚本使用 127.0.0.1:11312，并确认仿真 /map 已发布。 |
| /odom_raw 为 NaN 或出现 TF_NAN_INPUT | 立即零速，停止任务，重启底盘/定位链路；在两个 TF 和 odom 恢复有限值之前禁止导航、旋转和再次 mission。 |
| crc16、head_len、sensor not active | 停车排查 CP2102、USB Hub、串口线束和供电；不要把“偶尔恢复一帧”当成可以继续运行。 |
| 仿真等待超过 120 秒 | 主流程会继续车端终点流程，但结果视为未确认。先记录 bridge 和任务日志；下一轮必须停止并重启 bridge，不要在任务中途补启动仿真。 |
| lane_proto 直接 STOPPED 或 Python2 logging 崩溃 | 停止主流程，确认车端是当前 lane_proto，使用 Melodic Python2 启动器；不要用 Python3 直接执行 lane_follow.py。 |
