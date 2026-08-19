# OCR 停车 profile 同步全局膨胀（2026-08-20）

## 现象

进入处理停车 profile 时日志只显示：

```text
PRODUCTION_PROCESSING_PROFILE entered inflation=0.070 navigation_mode=point
```

实际代码只调用了 local costmap inflation layer。global costmap 仍保持点 3 后的 `0.224m`，因此全局规划路径没有同步放宽。

## 修复

- 三套 2026 主流程进入 profile 前分别读取 local/global 当前 `inflation_radius`。
- 进入 profile 时通过两个 dynamic-reconfigure 服务将 local/global 同步设置为 `processing_parking_inflation_radius_m`，当前值为 `0.07m`。
- 退出 profile 时分别恢复进入前保存的 local/global 半径。
- 进入和退出日志分别输出 `local_inflation` 与 `global_inflation`，避免只看 local 日志误判。
- CymPlanner 继续保持 `point` 模式；不修改 obstacle/static layer。

## 验证要求

- 回归测试必须同时断言两个 namespace 的 `0.224m → 0.07m → 0.224m`。
- 部署后在车辆零速且任务未运行时，车端检查两个 inflation layer 的 dynamic-reconfigure 服务和 Python2 语法。
- 下次安全重启任务后应看到：

```text
PRODUCTION_PROCESSING_PROFILE entered local_inflation=0.070 global_inflation=0.070 navigation_mode=point
PRODUCTION_PROCESSING_PROFILE exited local_inflation=0.224 global_inflation=0.224 navigation_mode=point
```
