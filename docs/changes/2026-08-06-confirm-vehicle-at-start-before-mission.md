# 主任务启动前强制确认车辆已放回起点

## 目的

按用户要求加入规则：每次开启主任务（`mission` 模式）时，必须先询问是否已把车放回
起点，防止在车辆离开起点后重启固定初值定位链路导致错误地图位姿（此前发生过点 3
热恢复反向返回 52 导致碰撞）。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `docs/operations.md`
- `docs/quickstart.md`

## 行为

- `start_2026.sh` 的 `mission` 分支在启动 roslaunch 前打印定位初值说明，并询问
  「是否已把车放回起点？」；输入 `yes`/`Yes`/`YES`/`y`/`Y` 才继续，其他任何输入
  打印取消提示并以退出码 2 退出，不启动任何 ROS 节点。
- `manual`、`check`、`full` 分支行为不变；`full` 为历史任务入口，本轮不修改。

## 验证结果

- 本机语法检查：`bash -n ucar_ws/src/ucar_2026/scripts/start_2026.sh` 通过。
- 规则检查（对照犯错档案）：新增代码位于两次 `source` 之后、`set -u` 已启用区域，
  未使用未定义变量；不涉及编译；不涉及 PowerShell/SSH 跨 shell 引号。
- 车端部署注意事项（沿用档案）：从 Windows 上传后需确认 `git ls-files --eol` 为
  `w/lf`、shebang 无 `0d`，并显式设置 `0755`。

## 已知限制

- 询问只覆盖 `mission` 模式；若未来 `full` 模式恢复使用，需同样加入起点确认。
- 确认依赖人工目测，脚本无法自动校验实车物理位置。
