# Workspace Rules

## One-way local, WSL, and GitHub synchronization

- Edit source files only in the local repository. Never make source edits in
  `/home/car/smartcar2026-simulation`.
- After local validation, commit the local changes and push them to GitHub.
- Fast-forward `/home/car/smartcar2026-simulation` from GitHub to the same commit;
  it is a deployment copy, not a second source workspace.
- WSL is the authoritative runtime and test environment. After every code,
  configuration, or documentation update, rebuild there and run validation
  relevant to the change. A local or GitHub-only result is not complete.
- Before reporting completion, verify that the local repository, GitHub branch,
  and WSL deployment point to the same `HEAD` commit.
- Run every WSL command as the non-root user `car`.
- If the WSL workspace contains tracked changes, stop and ask the user before
  syncing; never merge or copy WSL source changes back into the local
  repository. Preserve confirmed WSL-only untracked assets by listing their
  exact paths in `.git/info/exclude`; never delete them merely to make the
  deployment clean.
- Do not synchronize generated `build/`, `devel/`, `log/`, `.ros/`, or Gazebo
  cache files. Build artifacts remain local to the WSL deployment.

## Global Rules

- Do not use Playwright MCP. When browser automation or Playwright testing is
  needed, use the Playwright CLI instead.
