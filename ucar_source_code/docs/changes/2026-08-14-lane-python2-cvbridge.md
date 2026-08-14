# lane_proto 共享相机的 Python2 cv_bridge 修复

## 原因

小车默认 `/usr/bin/python` 是 Python 3.6，但 ROS Melodic 的 `cv_bridge_boost` 为 Python2
扩展。常驻巡线节点经 `#!/usr/bin/env python` 启动后，`imgmsg_to_cv2` 回调报
`PyInit_cv_bridge_boost` 缺失。

## 修改

- `lane_follow.py` shebang 固定为 `#!/usr/bin/env python2`。
- `lane_proto.launch` 为该节点增加 Melodic Python2 启动器。
- `lane_proto` 声明运行时依赖 `ucar_2026`，并新增 shebang/launch-prefix 回归测试。
- Python2 的 `rospy` 日志边界在交给 logging 前先完成 Unicode 格式化：保留 UTF-8 中文
  显示，避免 `unicode` 与 UTF-8 bytes 混合时由 logging 再次 `%` 格式化而崩溃。

## 验证

- 修复前：默认 Python3 可稳定复现该 ImportError；显式 Python2 导入 `getCvType` 成功。
- 修复后：本地回归 3 项通过（无 ROS 的本机跳过 1 项）；车端 `catkin_make ... run_tests`
  后，lane_proto 3 项为 0 errors / 0 failures；此前 ucar_2026 94 项也为 0 errors / 0 failures。
- 日志修复后：`format_ros_log` 回归用例覆盖后端 Unicode 与“ON(翻转)”UTF-8 字节混合的
  首个崩溃模式；车端用 Melodic Python2 执行 lane_proto 回归。

## 生效方式

解释器由进程启动时决定。已运行的 lane_follow 必须随下一次主流程 launch 重启后才会使用修复。
日志格式化也在进程启动时安装，因此同样需要重新启动 lane_follow；无需单独启动或重启主流程以外的节点。
