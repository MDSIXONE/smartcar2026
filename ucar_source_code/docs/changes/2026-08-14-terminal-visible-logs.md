# mission 终端可见性：仅中文语音日志

## 目的

语音监听器的中文状态此前同时被 Python 2 `rospy.loginfo` 转义为 `\uXXXX`，又以
`[语音]` 打印一次，导致终端重复且含乱码。本次保留直观的中文终端输出，移除会转义的
rosout 转发。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `docs/operations.md`

## 行为

- 非 JSON 的监听状态行（校准、唤醒、ASR 进度等）只显示为 `[语音] <中文原文>`。
- 不再写 `PRODUCTION_VOICE_LISTENER`；`log_safe_text` 保留给其他 Python 2 rospy 日志，
  不在本处删除。
- JSON 识别结果仍由 `PRODUCTION_VOICE_INPUT_ACCEPTED` /
  `PRODUCTION_VOICE_INPUT_REJECTED` 显示。
- 巡线现已采用常驻节点交接，日志直接在主 launch 终端显示；详情见
  `2026-08-14-vehicle-master-persistent-lane-handoff.md`。

## 验证结果

静态语法与车端构建/部署验证记录于同日的常驻交接改动文档。

## 已知限制

监听器若自身输出的字节不是 UTF-8，终端会显示替换字符；应在监听器端修复编码来源，任务端
不会再将其转换成 `\uXXXX`。
