# 省赛备用方案三（alt3）：目标不可达跳过 + OCR 逐簇对准

## 目的

审查发现两类“单点异常被放大为整个任务 MissionAbort”的致命路径，按“不改动现行
主流程源码”的要求以备用方案三交付：

- A2：锥桶等动态障碍落在产线目标格中心附近（距四角守卫点均 >0.12 m）时，守卫
  不触发而全局规划被堵死，主路线直达腿在 `wait_for_plan` 30 s 或 `goal_timeout`
  180 s 后直接 MissionAbort，整个任务终止而不是跳过该点。
- A3：OCR helper 把同帧所有文本框并成一个 union bbox；两块牌子同框时对准中心
  落在两牌之间、类别归属不确定，且一块牌子进出画面导致的误差跳变会触发
  “PD 对准连续两次发散 → MissionAbort”。测距/墙点匹配的 MissionAbort 同理。

## 改动

新增 4 个文件，未修改任何既有文件：

- `ucar_2026/scripts/production_task_2026_alt3.py`（自 production_task_2026.py
  派生，5436 行，改动点均带 `alt3` 注释）：
  - A2：`navigate_target_and_scan` 直达腿改为
    `abort_on_navigation_failure=False`；规划失败/超时/未达时不再 raise，而是
    记录 `PRODUCTION_TARGET_NAVIGATION_SKIP` 审计事件（`target_scan_events`
    outcome=`target_navigation_skipped`，状态 `TARGET_NAV_SKIP_xxx`），返回
    `target_navigation_failed`；`cruise_grouped_production_route` 对该结果与守卫
    命中同路处理：先 `try_grouped_target_guard_fallback` 试守卫角点，仍不行则
    `ignored` 继续调度。外圈 fallback 路线的直达腿同样不再终止任务。
    新增参数 `route_goal_timeout`（默认 0 = 沿用 goal_timeout；可调小以更快
    放弃被堵目标）。
  - A3：新增模块级纯函数 `_ocr_item_bbox` / `cluster_ocr_items`（按框高
    0.8 倍外扩后求连通分量的邻近聚类，簇内按行序拼接文本）/
    `production_cluster_candidates` / `select_category_cluster`。
    `handle_candidate` 改为对逐簇候选逐一套用原有四道跳过检查（日志标签
    不变），选中簇的 bbox/文本进入 `turn_detection` 与后续记录；
    `observe_wall` 新增 `target_text_category`，对准环节每帧只取目标类别
    所在簇（该牌不在帧内 = 按空帧处理）；“发散两次”降级为放弃当前候选
    （`PRODUCTION_OCR_ALIGNMENT_DIVERGED`，进入 rejected 继续转圈）；
    测距/墙点匹配失败降级为 `PRODUCTION_OCR_RANGING_REJECTED` 并按未记录
    墙点的观测返回（同样走 rejected）。helper 无 items 时回退旧合并行为。
- `ucar_2026/launch/2026_alt3.launch`：除任务节点换为
  `production_task_2026_alt3.py`（节点名 `production_task_2026_alt3`）与新增
  `route_goal_timeout` 参数外，与 2026.launch 逐行一致。
- `ucar_2026/scripts/start_2026_alt3.sh`：与 start_2026.sh 相同的网络预检/
  孤儿 roscore 清理/Master 托管，模式仅 `check|mission`（默认 mission），
  mission 启动 2026_alt3.launch。
- 本文档。

## 验证

- `python3 -m py_compile` 通过；launch 通过 XML 校验，且与 2026.launch 的
  diff 仅含上述预期改动；start 脚本 `bash -n` 通过。
- 聚类纯函数单测：单牌多行 → 1 簇且行序拼接；双牌同框 → 2 簇、
  `production_cluster_candidates` 给出两个类别候选、`select_category_cluster`
  按类别取对应簇；无 items 回退合并 detection；面积门槛只作用于候选枚举。
- 实车帧实测（镜像“电子产品生产车间”照片，先按 camera_mirror 翻转再 OCR）：
  tesseract chi_sim 逐框输出经 `cluster_ocr_items` 聚为 1 簇，
  `normalize_production_category` 命中“电子产品”；把牌子区域平移合成到画面
  另一侧构造双牌帧后聚为 2 簇，按类别/距中心选择正确簇——旧 union 中心
  落在两牌之间，逐簇中心落在牌内。
- 未在实车运行；生成器对主文件的 14 处锚点逐一断言唯一命中，主文件漂移时
  会显式失败而不是产出错误的 alt3。

## 已知限制

- CMakeLists.txt 未注册 alt3（devel 空间可直接用；install 空间不安装）。
- stop_2026_task.sh 的 pgrep 只匹配 `2026.launch`，停 alt3 请在启动终端
  Ctrl-C；start_2026.sh 未新增 mission_alt3 模式，请使用 start_2026_alt3.sh。
- docs/operations.md 未同步（本次要求不修改既有文件），启动命令见
  start_2026_alt3.sh 头部注释。
- 聚类阈值 gap_scale=0.8（按框高外扩），依据“同牌行距 < 一行高、异牌间距
  远大于一行高”的场地事实；若赛场出现贴得极近的两块牌，可在
  `cluster_ocr_items` 调小该值。
- live_ppocr 的 classify 仍作用于全帧拼接文本（helper 未改动）；alt3 的类别
  判定改为在任务侧对簇文本做关键词归一（normalize_production_category），
  不再依赖 helper 顶层 text。
