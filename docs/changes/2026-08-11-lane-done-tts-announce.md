# 2026-08-11：lane_proto 终点 STOPPED 后语音播报「任务完成」

## 目的

视觉巡线（lane_proto）抵达终点（STOPPED）后，须在 10 秒内开始语音播报
「任务完成」。实现方式：在 `lane_follow.py` 的 `set_phase` 状态入口处，
当进入 STOPPED 且原因不是远程急停/测距超时/运行上限兜底时，立即异步调用
`/home/ucar/wake/tts_say.py` 播报「任务完成」。

## 涉及文件

- `ucar_ws/src/lane_proto/scripts/lane_follow.py`（仅小车端存在，本地仓库无此包）：
  - 新增 `import subprocess`；
  - `__init__` 新增 `self._done_announced = False`（防重复播报）；
  - `set_phase` 新增 STOPPED 分支：`"急停"、"超时"、"兜底" 不在 why 中时`
    置 `_done_announced` 并通过 `subprocess.Popen` 异步播报（不阻塞主循环）。
- 已部署小车 `~/ucar_ws/src/lane_proto/scripts/lane_follow.py`（原文件备份在
  `/tmp/lane_follow.py.bak_*`，小车端不留备份目录）。

## 验证结果

- 本地 `ast.parse` 语法检查通过；小车 `python2 -m py_compile` 通过。
- 本次部署发生在任务 20:15 到达 STOPPED 之后（20:26 部署），故本轮未触发播报；
  下次任务终点 STOPPED 时将生效，需实车复验。

## 已知限制

- 播报异步执行（Popen），不保证播报在 10 秒内**完成**，但会在 STOPPED 瞬间
  立即发起，满足「10 秒内开始播报」的要求。
- 远程急停、测距超时、运行上限兜底三种 STOPPED 不播报（非正常完成任务）。
