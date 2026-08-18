# 仿真三步启动合并

## 目的

将仿真专用 `roscore`、Gazebo/RViz 的 `task3_prepare.launch` 和 HTTP bridge 合并为一条
WSL 命令，减少标准、国赛和额外主流程启动前的手工操作，同时保留仿真 Master 隔离、
`/map` 就绪门槛和 WSLg COPY MODE 安全检查。

## 涉及文件

- `simulation/scripts/start_simulation_stack.sh`：一键启动器，默认启动 GUI/RViz；确认
  `/map` 和 bridge `/status` 的 `state=waiting` 后输出单独一行 `OK`；按 Ctrl-C 依次停止 bridge、
  `task3_prepare.launch` 和仿真 `roscore`。
- `simulation/README.md`：增加一键启动与无界面联调说明。
- `simulation/bridge/README.md`：将一键启动设为真车联动推荐入口，保留 bridge-only 调试说明。
- `ucar_source_code/docs/deployment.md`：首次通信验收和故障排查改用一键入口。
- `ucar_source_code/docs/operations.md`：标准主流程改为一键启动命令，并更新停止流程。
- `ucar_source_code/docs/operations-national.md`：国赛仿真启动/停止改为同一入口。
- `ucar_source_code/docs/operations-extra.md`：额外 OCR 仿真启动/停止改为同一入口。
- `ucar_source_code/docs/operations.md`：记录 WSL 源码基准、Windows 分享镜像和内容校验命令。

## 使用命令

```bash
cd ~/smartcar2026/simulation
bash scripts/start_simulation_stack.sh
```

无界面联调使用 `bash scripts/start_simulation_stack.sh --headless`。

## 验证结果

- `bash -n simulation/scripts/start_simulation_stack.sh` 和一键脚本 `--help` 通过。
- 已将脚本同步至 WSL `/home/car/smartcar2026/simulation/scripts/`，设置 `0755`；Windows
  与 WSL 两端 SHA-256 一致，WSL 按实际用户目录命令完成 `bash -n` 和 `--help` 验证。
- 已以 WSL 当前实际运行源码为准，将可分享的仿真源码、配置、模型、测试和文档同步到 Windows
  `simulation/`；排除 `build/`、`devel/`、`logs/`、`tmp/`、训练产物等生成文件。使用
  `diff -qr --strip-trailing-cr` 做内容校验无差异；Windows 挂载盘权限位差异不计入一致性。
- 本次同步操作没有主动启动或终止 ROS/Gazebo；验证时发现 WSL 已有该一键脚本持有的仿真栈，
  `/map` 和 bridge `state=waiting` 均已检查通过。使用结束时应在该启动终端按 Ctrl-C，
  由脚本清理 bridge、Gazebo/RViz 和仿真 roscore。

## 已知限制

- 脚本假设仿真工作区已经构建并存在 `devel/setup.bash`。
- 同一端口已有仿真 Master 或 bridge 时脚本会直接报错，不会替用户终止未知进程。
- 一键入口只负责电脑侧仿真三步；小车仍需按对应主流程执行 `check`、`manual` 和
  `mission`，并动态填写本次电脑 LAN IP。
