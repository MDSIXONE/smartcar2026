# 2026-08-11 语音双物品任务输入

## 目的

将 2026 生产任务开始时在启动终端依次输入“现实物品名、仿真物品名”的流程，改为麦克风阵列唤醒后的单句语音指令。任务只在语音与二维码信息完整匹配后才进入安全检查和任何导航动作。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`

## 行为

`2026.launch` 默认 `item_input_mode=voice`，任务节点在 `WAITING_FOR_ITEM` 时启动：

```bash
python3 -u /home/ucar/wake/micarray/wake_listen.py \
  --loop --asr --set-wake 小飞小飞 --json
```

应在唤醒后完整说出：

> 小飞小飞，前往物品领取区，取得日用品，放置在对应仓库，并领取仿真环境中需要的食品放置在对应仓库。

`wake_listen.py` 返回“取件类别”和“仿真类别”。两者必须是 `食品`、`日用品`、`电子产品` 中不同的两个类别；ASR 失败、槽位缺失、未知类别或重复类别均会留在等待状态，等待下一次唤醒，不会启动导航。

语音结果是类别而非二维码上的商品名。因此二维码阶段会扫描并分类每个首次见到的二维码，将符合两个所说类别的二维码文本分别回填为现实/仿真物品名。后续播报、结果 JSON 和仿真桥接仍传递实际物品名，例如“牙刷”和“蛋糕”。

`item_input_mode=stdin` 保留给无麦克风的开发验证；此模式仍在启动终端读取两个实际物品名。`resume_production_only=true` 不支持语音类别模式，因为它跳过了用于解析真实物品名的二维码阶段。

## 验证

- 已对小车 `/home/ucar/wake/micarray/wake_listen.py` 确认 `--json` 每次输出一行含 `ok` 和 `slots` 的 JSON，且 `--asr` 保持兼容。
- 小车 Ubuntu 18.04 / ROS Melodic：`python2 -m py_compile` 通过；直接运行
  `python2 test/test_production_task_geometry.py` 为 81 tests OK。
- 小车 Ubuntu 18.04 / ROS Melodic：`catkin_make run_tests_ucar_2026 -j2` 后
  `catkin_test_results build/test_results/ucar_2026` 为 93 tests、0 errors、0 failures、0 skipped。

## 已知限制

- 当前语音句式选择的是加工类别，而不是自由口述的任意商品名；实际商品名由现场二维码决定。
- 语音识别依赖麦阵列、讯飞网络识别和 `/home/ucar/wake/micarray/xf_key.conf`。听到 TTS 提示后再说唤醒词，避免扬声器声音被识别为指令。
