# 2026-08-16：全局代价地图膨胀参数调整（0.215→0.22→0.217 / 0.05→0.1→0.2→0.05）

## 目的

按用户要求调整当前 2026 导航使用的全局代价地图膨胀参数：
- `inflation_radius`: `0.215 m` → `0.22 m` → `0.217 m`（最终取 0.217，比初始
  值微增 0.002 m）
- `cost_scaling_factor`: `0.05` → `0.1` → `0.2` → `0.05`（试验 0.1/0.2 后按用户
  要求改回原值 0.05）

## 涉及文件

- `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_common.yaml`
  - `inflation_layer.inflation_radius: 0.217`
  - `inflation_layer.cost_scaling_factor: 0.05`
- `docs/operations.md`
  - 更新全局参数记录（原 `0.205 m/0.05` 已过时，本次统一为 `0.217 m/0.05`）。

## 背景

主流程 `cym_move_base_omni_2026.launch`（被 `ucar_2026`、`ucar_2026_extra`、
`ucar_2026_national` 的 `2026.launch` 引用）加载
`config/testnav20260721/global_costmap_common.yaml`，该文件为当前生效的全局
代价地图配置档。`teb_move_base_omni_2026.launch`（TEB 试验残留）不在主流程中，
本次不改。

实机反馈：`cost_scaling_factor: 0.1` 时全局路径堵死；改为 `0.2` 后按用户要求
将 `inflation_radius` 微调至 `0.217`，随后用户要求将 `cost_scaling_factor`
改回 `0.05`（原值）。

## 验证

- 本地 YAML 校验通过（仅数值变更，结构未动）。
- 已同步到小车 `ucar@192.168.8.231`（ucar-mini），远端确认
  `inflation_radius: 0.217`、`cost_scaling_factor: 0.05`。
- 参数需重启 `move_base` 后生效。

## 已知限制

- 当前组合（0.217 / 0.05）与最初（0.215 / 0.05）几乎一致，仅膨胀半径微增
  2 mm；若路径仍堵死，需另行分析障碍层/地图，而非继续调这两个参数。
- 该参数只影响全局代价地图；局部代价地图参数（`0.07 m/4.0`）未动。
