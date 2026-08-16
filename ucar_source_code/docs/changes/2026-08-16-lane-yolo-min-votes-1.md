# 巡线认灯改为单帧定案（yolo_min_votes 2 → 1）

## 目的

国赛主流程最后一段的巡线模式（`is_fork:=yolo`）在起跑点认红绿灯时，原逻辑要求某个方向（left/right/straight）**连续攒够 2 票（2 帧）**才定案。现场连续两帧要求太严，灯切向/抖动时容易漏认卡住；改为**一帧认出方向即定案**。

## 涉及文件

- `ucar_ws/src/lane_proto/scripts/lane_follow.py`
  - `self.yolo_min_votes = int(gp("~yolo_min_votes", 2))` → 默认值改为 `1`，并更新注释。
  - 该参数在 `lane_proto.launch`、三个 2026.launch（`ucar_2026` / `ucar_2026_extra` / `ucar_2026_national`）及 `handoff_lane.sh` 中均未显式传参，生效值即此默认值，因此一处修改即覆盖国赛主流程交接（`2026.launch` 自动交接）与手工 `handoff_lane.sh` 两条入口。

## 验证结果

- 静态确认：`step_yolo()` 定案条件 `arrows[0][1] >= self.yolo_min_votes`，票数从 1 起计，`min_votes=1` 时第一帧出方向即返回。
- 测试 `test/test_lane_runtime.py` 仅锁定 `is_fork=yolo` 交接参数，未锁定 `yolo_min_votes`，不受影响。
- 本机无法编译/运行小车端代码（UBUNTU 18.04 车端构建），未做实车验证；误检风险由 `yolo_conf`（默认 0.20）兜底，超时兜底 `yolo_wait_max` / `yolo_fallback` 逻辑不变。

## 已知限制

- 单帧定案降低了对偶发误检的容忍度；若现场出现因误检提前转向，可调高 `yolo_conf` 或恢复 `~yolo_min_votes:=2`（launch 显式传参即可覆盖默认值）。
