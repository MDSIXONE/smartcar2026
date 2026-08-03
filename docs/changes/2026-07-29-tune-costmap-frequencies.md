# 2026-07-29：降低代价地图频率以减轻车端 CPU 压力

## 目的

在不改变当前地图来源和 CymPlanner 碰撞语义的前提下，降低车端代价地图更新负担。

## 涉及文件

- `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_params.yaml`
- `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml`
- `docs/operations.md`

## 调整内容

- 全局代价地图：更新 `5 Hz` 调整为 `3 Hz`，发布保持工作区已有的 `5 Hz`。
- 局部代价地图：更新/发布 `12/5 Hz` 调整为 `8/3 Hz`。
- 当前 CymPlanner `body_projection` 仍使用已发布的全局代价地图，因此暂不降低全局发布频率。

## 验证结果

- 已完成 YAML 和文档修改。
- 尚未在 Ubuntu 18.04 小车端重启导航或测量实际 CPU、话题频率和地图延迟。

## 已知限制

- 运行中的 `move_base` 不会自动重新加载 YAML，必须停止并重新启动导航。
- 本次未修改 CymPlanner 的地图来源；如果后续改为直接使用局部代价地图，可再评估降低全局发布频率。
