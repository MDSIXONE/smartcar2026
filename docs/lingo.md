# 项目黑话词典（词条正文）

> 本文件是 `.agents/skills/project-lingo/SKILL.md` 的词典正文。AI 在对话中遇到
> 黑话时按 SKILL.md 的规则查阅本文件；新增条目须经用户确认后由 AI 沉淀。
> 本文件只做展开摘要，权威命令细节以各词条「权威来源」指向的文档为准。

---

## 启动主流程

- **等价说法**：跑主流程 / mission 模式 / 正式任务
- **含义**：在小车上以 mission 模式启动 2026 双物品生产主流程
- **前置条件**：WSL Master 已启动且不是 `localhost`；车已放回起点
- **精确动作**：
  1. `bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <MASTER_IP> mission`
  2. 任务节点进入 `WAITING_FOR_ITEM`，等待唤醒词"小飞小飞"+ 双类别语音指令
- **不是**：manual 模式（无二维码/语音/任务）；full 模式（历史 yolo2025 流程）
- **权威来源**：`ucar_source_code/docs/new-computer-gui-simulation-mission.md` §7、`ucar_source_code/docs/operations.md`「2026 双物品主流程」

## 国赛主流程

- **等价说法**：国赛版主流程 / national mission
- **含义**：使用 `ucar_2026_national/launch/2026.launch` 的国赛双物品生产任务主流程；与省赛流程共用任务逻辑，但使用国赛地图墙位。
- **前置条件**：已同步最新 `lane_proto`；小车已具备 `/scan`、`/odom_raw` 和共享 ROS 相机话题。
- **精确动作**：修改 `ucar_source_code/ucar_ws/src/ucar_2026_national/launch/2026.launch` 的常驻 `lane_proto` include；OCR 完成后的交接保持共享相机/单底盘模式，并启用新版板检测与绕板参数。
- **不是**：省赛 `ucar_2026/launch/2026.launch`，也不是国赛现场随机任务副本 `ucar_2026_extra`。
- **权威来源**：`ucar_source_code/docs/operations.md`「2026 主流程三场比赛副本」「常驻 lane_proto 无重启交接」

## 调节音量

- **含义**：设置小车 USB 扬声器（UACDemoV1.0）的 PCM 播放音量
- **前置条件**：无（不启动 ROS、不发声、不发布速度）
- **精确动作**：
  1. `python3 ~/wake/audio_dev.py` 查看当次识别的声卡 idx
  2. `amixer -M -c <AUDIO_CARD> set PCM <百分比>% unmute`
  3. `sudo alsactl store`（保存）
- **不是**：Windows 本机音量、仿真音量、其他声卡音量
- **权威来源**：`ucar_source_code/docs/operations.md`「USB 扬声器音量」

---

## 新增条目模板

```markdown
## <黑话>

- **等价说法**：<用户可能说的其他说法>
- **含义**：<一句话精确含义>
- **前置条件**：<执行前必须满足的条件；无则写"无">
- **精确动作**：
  1. <步骤 1>
  2. <步骤 2>
- **不是**：<容易混淆的其他含义/对象，逐条列出>
- **权威来源**：<指向 docs 下权威文档的具体章节>
```
