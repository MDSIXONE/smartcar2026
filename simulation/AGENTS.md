# simulation 本地规则

1. 本目录为统一仓库 [MDSIXONE/smartcar2026](https://github.com/MDSIXONE/smartcar2026) 中的仿真部分；历史来源仓库为 [MDSIXONE/smartcar2026-simulation](https://github.com/MDSIXONE/smartcar2026-simulation)（原 main 分支，已并入本仓库 `simulation/`）。
2. 一个 Git 分支对应一个文件夹；不同分支的代码、配置和生成物不得混放。
3. 编写仿真时不能只考虑固定 IP；由于 Wi-Fi 网络环境会变化，必须支持 IP 动态变化或提供相应的配置/发现机制。
4. WSL 中的命令必须在非 root 用户 `car` 下运行（密码为 `car`）；不得使用 root 用户执行仿真相关操作。
5. 文档只保留以下四类：快速开始、常见问题、结构、部署。部署完成后应尽可能通过少量指令启动，避免要求用户反复 `source`，能固定的环境配置应固定下来。
6. 出现过的错误必须记录在 `犯错档案.md` 中；每次修改代码完成后，都必须查看一遍该档案，并根据其中的经验检查本次修改。
7. WSL 是本项目的实际测试与运行环境。每次修改代码、配置或文档后，必须依次完成：本地验证、提交并推送 GitHub、将 `/home/car/smartcar2026-simulation` 快进到同一提交、在 WSL 中重新构建并执行与改动相关的验证。不得在 WSL 尚未更新时把本地或 GitHub 的结果表述为用户可用。
8. 每次任务完成前必须分别核对本地 `HEAD`、GitHub 远端分支 `HEAD` 和 WSL 部署 `HEAD`，三者必须完全一致。WSL 同步或验证失败时，任务不得宣告完成；必须继续处理或明确报告阻塞项。
9. WSL 部署中的本地模型、数据集和工具目录不得擅自删除。确认属于仅供 WSL 使用的未跟踪资产后，应写入该部署仓库的 `.git/info/exclude` 保存原文件并保持部署工作树可安全快进。
10. 不允许修改官方的urdf模型和世界模型(确保和gazebo_ws里的一样)
11. gazebo_ws是官方给的仿真包，如果要判断是否更改了某些文件，可以参考，但是绝对不能改动官方的仿真包
12. 用户确认跑通流程后，参考：比赛仿真硬性要求.txt，对代码进行核验，确保没有违反比赛规则
- 编写代码时参考(MODEL_ROUTING.md)来部署子智能体

## One-way local, WSL, and GitHub synchronization

- Edit source files only in the local repository. Never make source edits in `/home/car/smartcar2026-simulation`.
- After local validation, commit the local changes and push them to GitHub.
- Fast-forward `/home/car/smartcar2026-simulation` from GitHub to the same commit; it is a deployment copy, not a second source workspace.
- WSL is the authoritative runtime and test environment. After every code, configuration, or documentation update, rebuild there and run validation relevant to the change. A local or GitHub-only result is not complete.
- Before reporting completion, verify that the local repository, GitHub branch, and WSL deployment point to the same `HEAD` commit.
- Run every WSL command as the non-root user `car`.
- If the WSL workspace contains tracked changes, stop and ask the user before syncing; never merge or copy WSL source changes back into the local repository. Preserve confirmed WSL-only untracked assets by listing their exact paths in `.git/info/exclude`; never delete them merely to make the deployment clean.
- Do not synchronize generated `build/`, `devel/`, `log/`, `.ros/`, or Gazebo cache files. Build artifacts remain local to the WSL deployment.