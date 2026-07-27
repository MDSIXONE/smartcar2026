# SmartCar 2026

SmartCar 2026 真机 ROS 1 工程：包含导航与视觉工作区、CymPlanner 局部规划器、地图资源及部署记录。

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `ucar_ws/` | 实车 ROS Melodic Catkin 工作区；源码位于 `ucar_ws/src/`。 |
| `jie_ware/` | 独立的激光定位包副本；当前部署副本位于 `ucar_ws/src/jie_ware/`。 |
| `docs/operations.md` | 实车部署、导航、二维码与建图操作说明。 |
| `docs/changes/` | 本地改动记录、验证结果和已知限制。 |
| `full_map_grid_0p2m.*` | 当前任务地图的网格导出。 |

## 环境与启动

- 实车工作区使用 Ubuntu 18.04 / ROS Melodic / Python 2。
- 控制电脑的唯一 ROS Master 为 `http://192.168.8.197:11311`；不得在小车端启动 `roscore`。
- 具体的构建、部署和回滚命令以 [docs/operations.md](docs/operations.md) 为准。

首次使用需要在小车的 ROS Melodic 环境下构建所需 Catkin 包。

## 当前导航安全链路

CymPlanner 使用与当前仿真一致的 `main_legacy` 路径跟踪控制律，并直接使用
`/scan_filtered` 激光点沿全局路径前视段投影完整车体。激光缺失、超时或任一激光点
触碰投影车体时，规划器保持零速度并请求 `move_base` 重新规划。真机继续使用实测
footprint、滤波激光接口和安全速度上限，参数位于
`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`。

## 未纳入版本控制的内容

为使仓库可推送且不泄露本机信息，以下内容只保留在本地：备份目录、构建输出、模型权重、厂商运行时二进制、生成的音视频/日志、压缩包和含有设备 App ID 的配置文件。

- YOLO 权重需要从授权来源获取后放回原路径。
- 语音服务的 `appid_params.yaml` 需要按本机/设备账号重新配置。
- 归档文件和 `back/` 用于本地恢复，不会上传到 GitHub。

该仓库以当前工作区的源码快照为准；其中已有的嵌套 Git 元数据和历史远程地址均不会被提交。

## 使用边界

工程可能包含第三方、课程或竞赛资料。仅在已获得相应授权的范围内使用、修改和分发。
