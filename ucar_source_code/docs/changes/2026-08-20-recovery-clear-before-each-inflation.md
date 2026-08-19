# 恢复阶段前重复清除代价地图

## 变更

将 `ucar_nav/config/testnav20260721/move_base_params.yaml` 的恢复列表改为 18 组固定顺序：

```text
conservative_reset_N -> aggressive_reset_N -> relax_inflation_N
```

其中 `N` 为 `01` 到 `18`。因此每个 `cym_planner/InflationRecovery` 阶段前，都会先分别执行一次
`reset_distance: 1.0` 和 `reset_distance: 0.0` 的 `ClearCostmapRecovery`，总恢复行为数为 54。

膨胀恢复步长保持不变：局部每次降低 `0.02m`，全局每次降低 `0.00395m`，最低半径保持 `0.05m`。

## 原因

此前列表只在整个恢复序列开头清除两次，后续膨胀阶段直接沿用前一阶段的代价地图状态，不满足每次恢复前重新清除的要求。

## 验证

- 本地 YAML 解析通过。
- 本地顺序断言通过：54 个行为、18 组清除双阶段和 18 个膨胀阶段。
- 每组清除参数断言通过：保守清除 `1.0m`，激进清除 `0.0m`。
- 车端 YAML 顺序断言通过，10 个同步文件本地/车端 SHA-256 一致。

## 生效条件

该 YAML 只在 `move_base` 启动时加载。同步文件不会热更新当前运行中的导航节点；本轮不重启主流程、不发送运动指令，需下一次安全重启导航链路后生效。
