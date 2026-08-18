# AI 变更记录

> 此文件由项目记忆技能维护。仅记录本项目 AI 辅助完成的源代码、配置或资源改动。

## 2026-08-18｜70 点坐标回滚与调试入口隔离

- 状态：改动完成
- 目标：响应“70 坐标不能更新”，撤销独立调试部署对车端国赛共享网格的坐标影响。
- 涉及文件：`ucar_ws/src/ucar_2026_national/launch/national_sprint_speed_debug.launch`、定向测试、`docs/changes/2026-08-18-national-sprint-speed-debug.md`、`docs/operations.md`；车端 `production_full_grid_all_numbered.json`。
- 验证结果：车端 70 点两处记录均为 `(2.25, 1.75)`；车端 Python2 定向测试 2 项通过；`run:=false/true` launch 静态解析通过；未启动运动节点。
- 未解决风险：本地工作区原有未提交网格仍保留 `(2.32, 1.68)`，本次未擅自覆盖；后续部署禁止同步该共享网格文件。

## 2026-08-18｜QR 扫描方向扩展

- 状态：改动完成
- 目标：到达二维码扫描中心后，将固定观察方向从 `180°→90°→-90°` 扩展为 `180°→90°→-90°→-135°→135°→45°`，保持后续任务流程不变。
- 涉及文件：三套 `ucar_2026*` 主流程的二维码默认配置、launch、几何回归测试、`docs/changes/` 与 `docs/operations.md`。
- 验证结果：标准/国赛/额外三套几何单测分别 `88/89/103` 通过；本机六个 Python 文件编译检查通过；三套 launch XML 解析通过；六个编号点均存在且从点 52 的方位角顺序匹配目标序列；已通过动态主机名同步到车端，9 个运行时文件 SHA-256 一致，车端 Python 2 编译检查通过。
- 未解决风险：尚未在车端动态 launch 或实车运动验证；扫码固定面每轮由 3 个增加为 6 个，可能增加扫码耗时。同步后未重启现有主流程，需下次按安全检查重启实际使用的入口。

## 2026-08-18｜70 冲刺速度调试程序

- 状态：改动完成
- 目标：新增独立 ROS 调试入口，加载比赛地图，从 70 冲刺起始点运行到坡顶，用于标定合适速度。
- 涉及文件：`ucar_ws/src/ucar_2026_national/scripts/national_sprint_speed_debug.py`、`launch/national_sprint_speed_debug.launch`、`CMakeLists.txt`、定向测试、`docs/changes/2026-08-18-national-sprint-speed-debug.md`、`docs/operations.md`。
- 验证结果：本机 2 项定向单测通过；车端 Ubuntu 18.04/ROS Melodic 构建成功、Python2 定向测试 2 项通过、launch 静态节点解析通过；4 个新增文件及依赖网格 JSON 已完成 SHA-256 校验；原车端白名单已恢复为 `usb_cam`，无导航残留进程。
- 未解决风险：尚未执行实车运动验证；日志统计的是 `/cmd_vel` 请求速度，不是实际轮速。
