# alt3：转圈 OCR 改为完整 360° 全扫描

## 目的

按现场策略调整 alt3 的单点转圈 OCR：即使本圈已经记录到全部所需类别
（如实物与仿真两个目标类别都已命中），也不再提前停转，而是把整圈 360°
转满、把该停靠点周围的所有牌子扫描完再继续任务。这样一次停靠能尽量
多地预记录其余类别的位置，减少后续巡航腿数。

## 改动

仅动 alt3 一族两个文件（alt1f/alt2f 与所有既有文件不受影响）：

- `ucar_2026/scripts/production_task_2026_alt3.py`
  - 新增参数 `ocr_scan_complete_revolution`（默认 true）。
  - `handle_candidate` 内两处“记满即停”判定
    `if all_required_categories_recorded(): self._ocr_turn_stop_flag = True`
    改为仅在该参数为 false 时才置停转标志；参数为 true 时
    `rotate_full_revolution_for_ocr` 自然转满 2π 后按
    `ocr_full_turn_complete` 收尾。
  - 巡航层逻辑不变：整圈结束后若目标类别已记录，照旧结束本轮巡航进入
    停靠/播报；每类别一圈内仍只做一次停车对准（handled_categories），
    已记录类别的重复候选照旧走 SKIP_RECORDED 跳过，不会重复停车。
  - 转圈超时预算不受影响（对准/测距暂停本就补偿进 deadline）。
- `ucar_2026/launch/2026_alt3.launch`：新增
  `<param name="ocr_scan_complete_revolution" value="true"/>`（带注释；
  改为 false 即恢复旧“记满即停”行为）。

## 验证

- 生成器锚点断言全部唯一命中；`python3 -m py_compile` 通过；launch XML
  解析通过。
- git diff 审计：脚本仅 4 处预期变更（docstring、参数块、两处判定），
  launch 仅 +3 行；alt1f/alt2f、主流程、alt1/alt2 零改动。
- 未实车运行。时间成本估算：命中目标后原本可省去的剩余弧段现在照转，
  单点最多多花约一整圈时长（2π/0.35 ≈ 18 s + 候选处理暂停）。

## 已知限制

- 每个类别一圈内仍只停车对准一次；全扫描增加的是覆盖角度与预记录机会，
  不改变“一牌一类一停”的节奏。
- 若赛时时间紧张，可在 launch 把 `ocr_scan_complete_revolution` 设回
  false，行为立即回到 08-21 版 alt3。
