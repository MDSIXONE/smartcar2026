# GitHub Commit & Pull Request Guidelines

## Commit Message Format

Use a Chinese Emoji subject in the form `Emoji 范围：简短动作`, for example `🧭 导航：优化目标点路径规划`.

```
Emoji 类型[范围]：简短动作描述

[可选正文]

[可选脚注]
```

Subject line form: `Emoji 范围：简短动作`, for example:

```
🧭 导航：优化目标点路径规划
✨ 界面：新增深色模式切换
🐛 登录：修复令牌过期后跳转异常
```

- Use one coherent change per commit.

## Type Reference (Emoji + Chinese + Semantics)

| Emoji | Chinese type | English equivalent | Usage |
| ----- | ------------ | ----------------- | ----- |
| ✨ | 功能 | `feat` | New feature |
| 🐛 | 修复 | `fix` | Bug fix |
| 📚 | 文档 | `docs` | Documentation only |
| 🎨 | 样式 | `style` | Formatting/code style (no logic change) |
| ♻️ | 重构 | `refactor` | Code refactor (not feature/fix) |
| ⚡ | 性能 | `perf` | Performance improvement |
| ✅ | 测试 | `test` | Add/update tests |
| 📦 | 构建 | `build` | Build system/dependency change |
| 🚀 | 部署 | `ci` | CI/config change |
| 🧹 | 杂务 | `chore` | Maintenance/misc |
| ⏪ | 回滚 | `revert` | Revert commit |

Scope uses Chinese or a short module name, e.g. `导航`, `认证`, `db`.

## Breaking Changes

```
# Add an exclamation mark after the type
✨!: 移除已废弃的接口

# Or use a BREAKING CHANGE footer
✨ 配置：允许配置扩展其他配置

BREAKING CHANGE: `extends` 键行为已变更
```

## Commit Workflow

1. Analyze the diff: `git diff --staged` for staged changes, `git diff` for unstaged, plus `git status --porcelain`.
2. Stage as needed (`git add` or `git add -p` to group). **Never commit secrets** (.env, credentials.json, private keys, etc.).
3. Determine type, scope, and description from the diff (present tense, imperative mood, ≤72 characters).
4. Commit: `git commit -m "✨ 界面：新增深色模式切换"`; for multi-line messages use a heredoc with body/footer (`Closes #123`, `Refs #456`).

## Pull Request Guidelines

Pull requests should explain:

- affected sections
- safety implications
- validation
- linked issues
- layout screenshots when needed

## Git Safety Protocol

- Never modify git config.
- Never run destructive commands (--force, hard reset) unless explicitly requested.
- Never skip hooks (--no-verify) unless the user asks.
- Never force-push to main/master.
- When a commit fails due to hooks, fix the issue and create a **new commit** (do not amend).