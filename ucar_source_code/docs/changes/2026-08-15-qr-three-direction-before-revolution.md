# 物品领取区 QR 寻找顺序：先三个方向面朝寻找，全部没有才 360 旋转兜底

## 目的

物品领取区 QR 收集流程（`collect_target_qr_codes` /
`collect_target_qr_codes_by_category`）原来在每个观察点"面朝 → 等待
`qr_search_timeout` → 没扫到就立即 360 度旋转一圈"，即每个方向都会在没扫到
时立刻旋转，三个方向全部旋转完才进入下一轮。用户要求改为：**不管扫没扫到，
先完成三个方向的面朝寻找；只有三个方向都找过且全部没有，才进行 360 度旋转
寻找**（旋转兜底由调用方在所有固定方向都寻找过之后统一触发）。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`

## 改动说明

- `scan_observation_point(observation_number, accept_text=None,
  allow_revolution=True)`：
  - 新增 `allow_revolution` 参数；docstring 说明 `allow_revolution=False` 时
    本面只做"面朝 + 等待扫码"，不做 360 旋转兜底，返回 `None` 让收集循环继续
    下一个固定方向；360 旋转兜底由调用方在所有固定方向都寻找过之后统一触发。
  - 在 `if detected is None:` 分支内、`if rejected: ... return None` 之后、
    `publish_state("QR_SEARCH_TURN_...")` 之前插入
    `if not allow_revolution: return None`。
- `collect_target_qr_codes(targets, rounds=2)`：每轮拆为两阶段。
  - 阶段 1：对每个 `qr_observation_numbers` 调用
    `scan_observation_point(observation_number, accept_text,
    allow_revolution=False)`，只做三方向面朝寻找；收集逻辑不变。
  - 阶段 2：阶段 1 结束后若 `len(collected) < len(targets)` 才执行，同样对
    每个观察点：先打日志
    `PRODUCTION_QR_REVOLUTION_FALLBACK observation=%d`，再调用
    `scan_observation_point(..., allow_revolution=True)`；收集逻辑与阶段 1
    相同，中途收齐立即 `break`。
  - 每轮开始处保留 `if len(collected) >= len(targets): break`；docstring 更新
    为描述两阶段顺序。
- `collect_target_qr_codes_by_category(requested_categories, rounds=2)`：同样的
  两阶段拆分。阶段 1 与阶段 2 的后续处理（normalize / seen_items /
  `classify_qr_text` / category 检查 / 收集）完全一致；阶段 2 每点先打日志
  `PRODUCTION_VOICE_QR_REVOLUTION_FALLBACK observation=%d`；docstring 同步更新。
- 测试文件：
  - 5 处 `scan_observation_point` stub 签名增加 `allow_revolution=True`
    （行为不变）。
  - `test_qr_collection_scans_two_full_rounds_when_targets_missing`：断言改为
    每轮 3 次面朝 + 3 次旋转兜底共 6 次、两轮 12 次的完整序列。
  - `test_qr_collection_filters_targets_and_takes_first_only`：预期
    `[262, 232, 295, 262]` 保持不变（阶段 1：262 收苹果、232 拒绝、
    295 重复已收集；阶段 2：262 收手机）。
  - 新增 `test_scan_observation_point_skips_revolution_when_disabled`：
    `allow_revolution=False` 时面朝 + 等待扫码后直接返回 `None`，不调用
    `rotate_full_revolution`。
  - 新增 `test_qr_collection_faces_all_directions_before_revolution`：断言每轮
    前 3 次调用 `allow_revolution` 均为 `False`，之后 3 次均为 `True`（两轮）。

## 验证

- 本机 `python3 -m py_compile` 对两个 Python 文件语法检查通过（python2 的
  `u""` 前缀在 python3 中合法）。
- 已部署小车 `ucar@192.168.8.231`（scp 两个文件，SHA256 与本地一致）：
  - 小车端 `python2 -m py_compile` 通过；
  - `python2 -m unittest discover -s src/ucar_2026/test -p
    'test_production_task_geometry.py' -v`：**86 tests OK**，含新增
    `test_scan_observation_point_skips_revolution_when_disabled` 与
    `test_qr_collection_faces_all_directions_before_revolution`。
- 部署过程中发现并修复：测试文件 `test_run_mission_aborts_when_qr_codes_not_all_collected`
  的 `scan_observation_point` lambda stub 参数名误写为 `_allow_revolution`
  （带下划线前缀），与调用方关键字 `allow_revolution` 不匹配导致 TypeError；
  本机 py_compile 只查语法抓不到该运行时错误，已改为 `allow_revolution=True`
  并重新同步小车后测试全绿。

## 已知限制

- `python2 -m unittest discover -p 'test_*.py'` 全量运行会因
  `test_production_camera_ocr.py`（Python 3 文件，依赖 `importlib.util`）报既有
  ImportError；该文件按 `docs/operations.md` 约定用 `python3` 单独运行，与本次
  改动无关。
- 旋转兜底阶段对每个观察点会先重新面朝再旋转一圈（复用原 `scan_observation_point`
  语义），面朝等待时间按 `qr_search_timeout` 计入。
