# 项目本地规则

1. 本仓库为统一仓库 [MDSIXONE/smartcar2026](https://github.com/MDSIXONE/smartcar2026)（main 分支），同时包含仿真与源代码两部分：
   - `simulation/`：仿真部分（源自 [MDSIXONE/smartcar2026-simulation](https://github.com/MDSIXONE/smartcar2026-simulation) 原 main 分支），规则见 `simulation/AGENTS.md`；
   - `ucar_source_code/`：小车源代码部分（源自 smartcar2026 仓库原 `simulation_real` 分支），规则见 `ucar_source_code/AGENTS.md`。
2. 所有与仿真相关的代码、配置、资源、生成文件和临时文件，统一放在项目根目录的 `simulation/` 目录中。除 `simulation/` 外，不得新建其他用于仿真的目录；如需细分，必须在 `simulation/` 内创建子目录。
3. 用户新拖入项目的文件，仅可在使用期间保留在其拖入位置；使用完成后，必须根据文件类型和用途移动到对应的位置，并更新相关引用或路径。
4. 编写代码时可使用子智能体

# Universal Rules

- Always answer in Chinese unless the context requires otherwise.
- Do not write defensive or fallback code; it does not solve the root problem. Prefer full exposure: let failures surface clearly (explicit errors, exceptions, logs, failing tests) so bugs are visible and can be fixed at the root cause.
- When editing existing code: if you notice unrelated dead code, mention it - don't delete it.
- Do not use Playwright MCP. When browser automation or Playwright testing is needed, use the Playwright CLI instead.