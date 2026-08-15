# 移除二维码识别完成播报（2026-08-16）

## 目的

按用户要求，任务流程中不再播报“二维码识别完成，已获取<实物>和<仿真物品>”
这一句 TTS。QR 阶段完成后仅保留状态发布（`QR_ITEMS_ANNOUNCE`），
仓库归属播报（`announce_item_destinations`）不受影响，继续播报。

## 涉及文件（三个包同步修改）

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
  - `announce_qr_collection()` 中删除 `speak_wait(...)` 播报调用，
    保留 `publish_state("QR_ITEMS_ANNOUNCE")` 状态发布（外部监控仍可感知 QR 完成）。
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026_extra/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026_national/test/test_production_task_geometry.py`
  - 两处事件序列断言中删除对应的 `("announce", u"二维码识别完成，已获取苹果和手机")` 期望。

## 验证

- 三个包本机 `python test/test_production_task_geometry.py`：
  **86 tests OK（71 skipped，ROS 依赖用例按设计跳过）**，含 QR 收集播报序列
  相关的两个用例（`test_run_mission_...` 系列）。
- 全仓 grep：代码中已无该播报文本（仅 `docs/changes/` 历史记录保留原文）。

## 已知限制

- 代码仍需在小车 Ubuntu 18.04 上重新编译部署后生效（脚本类改动随包同步，
  部署命令见 `docs/operations.md` 对应小节）。
- `QR_ITEMS_ANNOUNCE` 状态名未改，虽已无 TTS 播报，但保留状态语义不变，
  避免影响可能的外部状态消费者。
