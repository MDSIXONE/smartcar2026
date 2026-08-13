# 配置工程诊断技能上下文

## 目的

为工程诊断、测试驱动开发和问题分流技能补充仓库级约定，使后续工具能够一致地找到
问题跟踪系统、标签词汇和领域文档。

## 涉及文件

- `AGENTS.md`
- `docs/agents/issue-tracker.md`
- `docs/agents/triage-labels.md`
- `docs/agents/domain.md`
- `docs/changes/2026-07-28-configure-agent-skills.md`

## 验证结果

- GitHub Issues 已配置为问题跟踪系统。
- 五个默认分流标签已记录。
- 工程已配置为单一领域上下文；`CONTEXT.md` 和 `docs/adr/` 可按需创建。

## 已知限制

- 本次只记录技能消费约定，没有创建 GitHub 标签或 Issue。
- 当前尚无 `CONTEXT.md` 或 ADR；诊断技能会在它们不存在时继续使用现有工程文档。
