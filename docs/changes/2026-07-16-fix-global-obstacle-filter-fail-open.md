# 修复全局动态障碍点云错位围堵

## 目的

修复小车旋转时 `map` 与激光坐标变换短暂不可用，`2026.py` 却把原始车体系
激光帧转发到全局代价地图的问题。错误帧会被按 `map` 坐标标记，导致全局障碍
层围住小车并使全局规划失败。

## 改动文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - 增加空扫描构造函数。
  - 静态地图掩膜未就绪或 `map <- laser_frame` TF 缺失时，全局话题发布全无穷
    扫描并限频告警；不再放行原始扫描。
- `ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml`
  - 全局动态障碍观测的 `observation_persistence` 从 `0.6` 改为 `0.0`，不保留
    旋转期间的陈旧帧。
- `docs/operations.md`
  - 补充失败关闭行为及局部代价地图仍使用原始 `/scan` 的说明。

## 验证

- 本机 Python 语法检查通过；车端 Python 2 `py_compile` 通过，远端上传文件
  的 SHA-256 与本地一致。
- 远端 YAML 解析通过；重启使用 `startup_goal_enabled:=false`，没有发送导航目标，
  `/navigation_2026/startup_goal_enabled=false`。
- `/scan` 与 `/scan_global_obstacles` 均约 `11.8 Hz`；8 秒采样分别收到 95 帧，
  有效点数为 28,934 与 5,013，说明静态墙过滤仍在工作。
- 车端强制 TF 异常单元检查返回 `[inf, inf]`，确认异常帧不会再放行到全局层；
  全局参数实测 `observation_persistence=0.0`，观测源仍为
  `/scan_global_obstacles`。

## 已知限制

该修复不会校正静态地图与真实环境的固定几厘米/角度误差；它只阻止 TF 不可用
时的错误全局标记。若 TF 长时间不可用，全局动态障碍不会更新，但局部层仍可
继续避障，需另行检查定位与时间同步。
