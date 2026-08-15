# 修正 new-computer-gui-simulation-mission.md 架构迁移残留

## 目的

`docs/new-computer-gui-simulation-mission.md` 是"新电脑唯一启动顺序"文档（operations.md 引用），
但 2026-08-14 架构迁移（主流程 ROS Master 改到小车本机）后正文未清理干净，并缺少
2026-08-15 的仿真兜底改动，会误导新电脑部署。

## 修改

`docs/new-computer-gui-simulation-mission.md`：

- 删除 1.3 节"安装 WSL 真车 Master 启动器"（`start_ros_master.sh`、`sitecustomize.py`、
  `ros_network.sh`、`ros_network.env` 整套旧架构组件），改为退役说明。
- 2.1 节改名"找到电脑的局域网 IP（即 `SIMULATION_HOST`）"，删除 `ros_network.env` 写入
  与 WSL 地址验证步骤。
- 2.3 节防火墙只保留 bridge TCP 11313 一条规则，删除 11311/TCPROS/HyperV 规则
  （与 `rosmaster/NETWORK_CONFIGURATION.md` 一致）。
- 终端 A（WSL 真车 Master 11311）整节退役，与第 0 节"电脑 WSL 不运行真车 roscore"矛盾消除；
  第 5 节标题"五个终端"改为"四个"。
- 终端 E 与第 6/7/8/9/10 节：`<MASTER_IP>` 全部改为 `<电脑LAN_IP>`；`check` 描述改为
  `start_2026.sh` 实际行为（打印小车本机 Master 与电脑仿真服务地址）；停止说明不再
  提及"WSL Master"；正常结束顺序去掉终端 A 步骤。
- 第 3 节车端构建命令删除指向电脑 11311 的 `ROS_MASTER_URI`/`ROS_IP` 导出，注明测试以
  mock 为主、需要时在本终端临时 `roscore`。
- 超时语义修正：第 7 节"最多等待 150 秒"改为约 120 秒（`simulation_done_timeout=120`）
  且超时/`/start` 失败都不中止任务；第 9 节"超过 150 秒按安全策略中止"改为"120 秒后继续
  任务"。
- 第 7 节补充 mission 启动的 TCP 11313 预检说明；第 9 节新增"No route to host"排查行，
  与 `rosmaster/NETWORK_CONFIGURATION.md` 故障排查小节呼应。

## 验证

- 全文检索确认 `MASTER_IP` 只出现在退役说明/无关键路径中；终端 A 仅剩退役说明；
  无残留 `ros_network.env` 步骤、"150 秒"、"按安全策略中止"等旧语义。
- 与 `NETWORK_CONFIGURATION.md`、`start_2026.sh`、`2026.launch`（`simulation_done_timeout=120`）
  实际行为一致。

## 已知限制

- 第 3 节构建测试是否全部不需要 ROS Master 未在车上实测；已给出"需要时临时 roscore"
  的兜底说明。
