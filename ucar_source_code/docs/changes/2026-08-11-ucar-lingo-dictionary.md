# 新增项目黑话词典（ucar-lingo skill）

## 目的

用户经常用口语化短句（黑话，如"启动主流程"、"调节音量"）下达操作指令。这些
短语在项目中有精确的多步骤定义，但分散在 `docs/quickstart.md`、
`docs/operations.md`（约 2400 行）等长文档中，AI 每次对话不会主动全文重读，
导致理解歧义。本次引入 skill 触发机制，让 AI 在用户说出黑话时自动加载词典。

## 涉及文件

- `.opencode/skills/ucar-lingo/SKILL.md`：skill 入口，含使用规则（读）、新增条目流程（写）
- `docs/lingo.md`：词条正文（词典本体），含两个初始词条与新增条目模板

## 设计决策

- 用 skill 而非纯 docs 文档：skill 的 `name + description` 在每次对话开始时注册
  到系统提示，用户说出触发词时 AI 自动加载，实现"第一时间知道"。
- 词条正文与 skill 分离：SKILL.md 只放规则与高频索引，详细词条放 `docs/lingo.md`，
  AI 触发后按需读取，避免词条增多时每次全量加载。
- 词条格式：等价说法 / 含义 / 前置条件 / 精确动作 / 不是 / 权威来源。
  其中「不是」字段用于消除歧义（如"调节音量"不是 Windows 本机音量）。
- 维护机制：AI 主动沉淀 + 用户确认（对话中遇到未收录口语或用户纠正时，询问确认
  后写入词典并登记索引）。
- 初始词条仅收录用户本次举例的两个（启动主流程、调节音量），其余后续自然积累。

## 验证结果

- skill 文件位于 opencode 项目级 skill 目录 `.opencode/skills/`，命名规范。
- `docs/lingo.md` 两个词条的精确动作均与权威来源核对（quickstart.md §5、
  operations.md「USB 扬声器音量」章节）。
- skill 自包含读写：SKILL.md「使用规则」覆盖读（命中按词条执行、未命中先询问），
  「新增条目流程」覆盖写（确认后写入 docs/lingo.md 并登记索引）。

## 已知限制

- skill 触发依赖 description 的触发词覆盖；触发词写不全时可能不自动加载。
- 词典词条需要在使用中持续积累，初始仅两条。

## 后续更新：接入 workspace-init（2026-08-11）

将黑话词典做成通用组件，加入用户全局 `workspace-init` skill 的初始化流程，
使**新项目**初始化时自动安装黑话词典：

- 新增 `C:\Users\10478\.agents\skills\workspace-init\resources\project-lingo\SKILL.md`：
  通用版黑话词典模板（命名 `project-lingo`，不绑定智能车项目；规则、索引、
  新增条目流程、词条模板与 `ucar-lingo` 一致，但不含智能车专属词条）。
- 更新 `workspace-init\SKILL.md`：description 与任务清单加入 project-lingo；
  流程新增 8a 步（复制 `resources/project-lingo/` 到 `.agents/skills/project-lingo/`）。
- 词条正文（`docs/lingo.md`）由该 skill 首次使用时按模板初始化，初始化时不创建。

注意：通用模板只含机制不含词条；新项目的黑话仍需在使用中按维护流程积累。
当前智能车项目的 `ucar-lingo`（`.opencode/skills/`，含真实词条）不受影响。
