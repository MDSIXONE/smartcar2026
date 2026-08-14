# lane_proto 共享 ROS 相机帧率属性修复

## 原因

终点交接后巡线已成功订阅共享 ROS 相机并取得图像，但性能输出沿用了
`self.grab.cam_fps`。原 V4L2 `FrameGrabber` 有该属性，新增的 `RosFrameGrabber` 未实现相同
契约，首次打印性能行即以 `AttributeError` 退出。

## 修改

- `RosFrameGrabber` 初始化 `cam_fps`、计数器和统计起始时间。
- 每个成功解码的 ROS 图像回调更新一次统计；每秒计算一次回调帧率。
- 新增定向回归，构造 ROS 采集器后断言 `cam_fps` 可读取。

## 验证

- 修复前，车端 Melodic Python2 定向回归稳定复现
  `AttributeError: 'RosFrameGrabber' object has no attribute 'cam_fps'`。
- 修复后，车端 `python2 -m py_compile` 和定向回归通过；
  `catkin_make -DCATKIN_WHITELIST_PACKAGES=lane_proto run_tests` 为 5 项、
  0 errors / 0 failures。

## 生效方式

该修复已部署到小车。已退出的 lane_follow 不会自行恢复；由下一次主流程启动装载新脚本，
不需要为此终止或重启其它主流程节点。
