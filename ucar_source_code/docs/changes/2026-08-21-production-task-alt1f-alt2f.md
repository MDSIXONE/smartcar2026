# alt1f / alt2f：方案一、方案二各自叠加 OCR 逐簇修复

## 目的

在不修改任何既有文件的前提下，把 alt3 已验证的 OCR 逐簇修复（两牌同框不再
并成一块假牌）分别叠加到省赛备用方案一（alt1）与方案二（alt2，8 方位定点
扫描版）上，得到 alt1f、alt2f 两个平行变体。命名后缀 f = fix（OCR 修复）。
本次**只移植 OCR 部分**：不含 alt3 的 A2 导航跳过改动；alt1/alt2 各自的对准
语义（15s 墙钟预算、发散仅重置 PD 导数、空帧边转边抓、8 方位扫描等）与
测距/墙点匹配失败的处理**原样保留**。

## 改动

新增 7 个文件，未修改任何既有文件：

- `ucar_2026/scripts/production_task_2026_alt1f.py`（← alt1）、
  `production_task_2026_alt2f.py`（← alt2）。每份相对其基线恰好 9 个 diff
  hunk，全部为预期改动：
  1. 文件头 docstring 增加变体说明；
  2. 类定义前注入与 alt3 逐字节相同的模块级纯函数
     `_ocr_item_bbox` / `cluster_ocr_items`（按框高 0.8 倍外扩求连通分量的
     邻近聚类）/ `production_cluster_candidates` / `select_category_cluster`
     （直接从已生成的 alt3 文件切片提取，保证三个变体聚类逻辑完全一致）；
  3. `handle_candidate` 改为对逐簇候选逐一套用原有四道跳过检查（日志标签
     不变），选中簇的 bbox/文本进入 `turn_detection` 与后续记录；
  4. `observe_wall` 新增 `target_text_category` 参数，对准环节每帧只取目标
     类别所在簇（该牌不在帧内 = 走各自原有的空帧路径：alt1f/alt2f 均为
     “边转边抓、计入 15s 预算”）；
  5. 节点名/横幅改为 alt1f / alt2f（顺带修正 alt2 里 `init_node` 误写为
     `production_task_2026_alt1` 的字符串；alt2 原文件未动）。
- `ucar_2026/launch/2026_alt1f.launch`（← 2026_alt1.launch）、
  `2026_alt2f.launch`（← 2026_alt2.launch）：仅头注释 + 任务节点
  type/name 三处差异，其余逐行一致；**无** route_goal_timeout（那是 alt3
  的 A2 参数）。
- `ucar_2026/scripts/start_2026_alt1f.sh` / `start_2026_alt2f.sh`：与
  start_2026_alt3.sh 同款托管启动器（网络预检/孤儿 roscore 清理/Master
  托管，模式 check|mission，默认 mission），分别拉起对应 launch。
- 本文档。

变体家族速查（对准发散行为为例）：

| 变体 | OCR 输入 | 发散两次 | A2 导航跳过 |
| --- | --- | --- | --- |
| 主流程 | 全帧 union | 终止任务 | 无 |
| alt1 / alt2 | 全帧 union | 重置导数继续（15s 预算） | 无 |
| alt3 | 逐簇（目标类别簇） | 放弃该候选 | 有 |
| alt1f / alt2f | 逐簇（目标类别簇） | 同 alt1/alt2：重置导数继续 | 无 |

## 验证

- 两份脚本 `python3 -m py_compile` 通过；两份 launch 通过 XML 解析且与基线
  diff 仅含预期三处；两份 start 脚本 `bash -n` 通过。
- 生成器对 alt1/alt2 的全部锚点逐一断言唯一命中（含“handle_candidate 与
  主流程逐字节一致”“divergence 块仅一处且未被触碰”“源文件不含
  route_goal_timeout / 导航软失败标记”），基线漂移会显式报错。
- 聚类函数从 alt1f/alt2f 各自文件内提取后重跑合成用例（单牌多行 1 簇、
  双牌 2 簇、按类别选簇、缺类别返回 None）全部通过；真实帧验证见
  `2026-08-21-production-task-alt3.md`（逻辑逐字节相同）。
- unified diff 审计：alt1→alt1f、alt2→alt2f 各恰好 9 个 hunk，无任何
  计划外改动；未在实车运行。

## 已知限制

- 与 alt3 相同：CMakeLists.txt 未注册新脚本（devel 空间可用）；
  stop_2026_task.sh 的 pgrep 不匹配这两个 launch 名，请在启动终端 Ctrl-C；
  start_2026.sh 未加对应模式，请用各自的 start_2026_alt1f.sh /
  start_2026_alt2f.sh；docs/operations.md 未同步。
- 锥桶堵死目标点导致的 30s/180s 超时后整任务终止（A2）在 alt1f/alt2f 中
  **依然存在**（按需求仅移植 OCR 修复）；需要该防护请用 alt3。
