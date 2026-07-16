# 2026-07-14：全局代价地图膨胀半径增加 0.02 m

## 目的

按要求将当前 2026 导航使用的全局代价地图 `inflation_radius` 从 `0.20 m` 调整为 `0.22 m`，增加全局路径与障碍物的安全间距。

## 涉及文件

- `ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml`
  - `global_costmap.inflation_radius: 0.22`
- `docs/operations.md`
  - 更新实际参数与重启要求。

## 验证

- 已确认 `cym_move_base_omni_2026.launch` 加载该文件。
- 本地 YAML 校验通过。
- 已上传到小车，并确认远端 SHA-256 为 `d64bdde8780904973fcc93e914fcbaf655b7972be42331cf1d3afa88157dbd12`，远端值为 `0.22`。
- 本次未启动车辆。

## 已知限制

- 该参数只在重启 `move_base` 后生效。
- 膨胀半径增加可能使狭窄通道变得不可规划；实际可通行性需在地图上重新验证。
