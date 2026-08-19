# 国赛任务脚本依赖模块同步（2026-08-19）

## 症状

国赛主流程启动后报告：

```text
[production_task_2026-18] process has died ... exit code 1
```

车端没有生成对应的独立任务日志。直接用 ROS Melodic Python2 导入任务脚本后，
错误为：

```text
ImportError: cannot import name shortest_yaw_delta
```

## 根因

车端 `production_task_2026.py` 已是新版，引用了
`production_task_geometry.shortest_yaw_delta`；但同目录几何模块仍是旧版，缺少该函数。
这是任务脚本与其同目录依赖文件未成套部署，不是 lane_proto V29 节点故障。

## 修复

- 将本地 `ucar_2026_national/scripts/production_task_geometry.py` 同步到车端。
- 保持任务脚本与几何模块的 SHA-256 成套校验。
- 不改动当前仍在运行的底盘、定位、导航和 lane_follow 进程；等待用户重启整套主流程。

## 车端验证

- `production_task_2026` Python2 导入通过。
- `production_task_geometry.py`、`production_task_perception.py`、
  `production_task_2026.py` Python2 编译通过。
- `shortest_yaw_delta(3.0, -3.0)` 实际调用通过。

重启实际使用的 `2026.launch` 后，需再确认日志出现
`2026 production task node started.`，并继续执行启动前的 odom、TF 和零速检查。
