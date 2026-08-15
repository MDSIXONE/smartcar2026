# 修复国赛新包首次运行 QR 分类失败（2026-08-16）

## 背景与根因

`ucar_2026_national` 首次实车运行（语音双物品模式）时，小车到达物品领取区后
识别到二维码物品却一直转圈收集不齐（`PRODUCTION_VOICE_QR_ROUND ... collected=0/2`）。
日志证据：

```
PRODUCTION_SPARK_CLASSIFY ... category=null source=none
  error="classifier helper unavailable"
PRODUCTION_SPARK_CLASSIFIER_START_FAILED [Errno 2] No such file or directory:
  '/home/ucar/.ros/ucar_2026_national_observations/spark_classifier.log'
```

根因链条：

1. `start_qr_classifier()` 要把日志写在 `result_directory/spark_classifier.log`，
   而 `result_directory`（`$(env HOME)/.ros/ucar_2026_national_observations`）首次运行不存在；
2. `prepare_result_directory()` 原来在 run_mission 的 QR 收集**之后**（OCR 巡航前）才调用，
   且它通过 `os.makedirs(run_directory)` 间接创建 result_directory；
3. 老包 `ucar_2026` 的 `~/.ros/ucar_2026_observations` 因历史运行已存在，掩盖了该顺序问题；
   新包首次运行目录缺失 → spark 分类 helper 起不来 → 每个二维码分类结果为空 →
   语音模式按类别收集永远 0/2 → 转圈识别无法退出。

与"改语音"无关，也与播报修改无关；是包复制后首次运行的环境初始化顺序缺陷。

## 修复内容

- `production_task_2026.py`（三个包一致）：
  - `prepare_result_directory()` 调用提前到 `switch_to_point_mode()` 之后、
    resume/QR 收集分支之前（`run_mission` 内），保证 Spark 分类器启动前
    result_directory 已存在；resume 分支（`classify_qr_text(0, ...)`）同样被覆盖。
  - 删除原 OCR 巡航前的重复调用（函数幂等，`run_directory` 已存在时直接返回）。
- `test/test_production_task_geometry.py`（三个包一致）：
  - `test_point_mode_is_selected_before_staging_navigation`、
    `test_run_mission_aborts_when_qr_codes_not_all_collected`
    补充 `self.task.prepare_result_directory = lambda: None` mock
    （与其他 17 处用例的既有 mock 约定一致；这两个用例此前不会走到该调用，
    提前后需要补齐，避免 AttributeError）。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026_national/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026_extra/test/test_production_task_geometry.py`

## 验证

- 本机三包：86 tests OK（71 skipped = ROS 依赖用例，按设计跳过）。
- 小车端三包全量 python2：86 tests OK（含此前失败的 2 个用例）。
- 部署方式：scp 三个包的 `production_task_2026.py` 与
  `test_production_task_geometry.py` 到小车对应目录；
  本次仅脚本/测试文件改动，无需重新 catkin_make（无 CMake 级变更）。

## 已知限制

- 若某天手动清空了 `~/.ros/ucar_*_observations`，本修复仍会在任务启动时自动重建。
- 车端构建白名单本次扩展为
  `ucar_2026;lane_proto;ucar_2026_national;ucar_2026_extra`
  （原为 `ucar_2026;lane_proto`），见 `docs/operations.md` 对应更新。
