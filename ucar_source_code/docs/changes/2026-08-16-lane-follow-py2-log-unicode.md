# lane_follow Python2 日志 UnicodeDecodeError 修复

## 原因

2026-08-16 现场启动 `roslaunch lane_proto lane_proto.launch dry_run:=false linear_speed:=0.3
gain:=1.0 template:=...(red_template_band.png) use_lidar:=false is_fork:=yolo
take_cam_on_start:=true` 时，`lane_follow.py` 在 `ensure_seg()`（加载 TrackSeg 后打印
`backend=... mirror=ON(翻转)`）处崩溃退出（exit code 1）：

```
UnicodeDecodeError: 'ascii' codec can't decode byte 0xe7 in position 3: ordinal not in range(128)
  File ".../logging/__init__.py", line 329, in getMessage
    msg = msg % self.args
```

根因：Melodic 跑的是 Python 2。`trackseg.py:52` 的 `self.backend = self.lib.ts_backend().decode()`
返回 **unicode**；而 `rospy.loginfo` 的格式串是 **str**，且参数含 UTF-8 中文 str
（`"ON(翻转)"`）。Python 2 的 `str % args` 遇到 unicode 参数时会把其余 str 参数按 **ASCII**
解码为 unicode，中文 UTF-8 字节（0xe7...）无法解码 → 抛异常。仓库历史提交 642d3ec 中本有
`format_ros_log` / `lane_loginfo` 等安全包装（`test/test_lane_runtime.py` 的
`test_ros_log_preformats_unicode_before_rospy` 仍保留），但小车当前版本丢失了该实现。

## 修改

`ucar_ws/src/lane_proto/scripts/lane_follow.py`（从小车拉取的最新版本 1812 行上修改）：

- 恢复 `text_type` 兼容定义（Py2 `unicode` / Py3 `str`）。
- 新增 `format_ros_log(message, args)`：把消息与参数先统一为 unicode 完成 `%` 格式化，
  再编码为 UTF-8 字节串返回——rospy/logging 二次格式化时只剩纯 ASCII `%s` 替换，不再炸。
- 新增 `_safe_ros_log` / `lane_loginfo` / `lane_logwarn` / `lane_logerr` /
  `lane_logerr_throttle` 包装，并模块级替换 `rospy.loginfo/logwarn/logerr/
  logerr_throttle`，所有既有调用点写法不变。

## 验证

- 本机：Python3 下模拟 Py2 语义的 `format_ros_log` 单测（混合 unicode + UTF-8 中文字节参数）全部通过。
- 车端（python2.7 + Melodic）：
  - `python2 -m py_compile scripts/lane_follow.py` 通过。
  - `test_lane_runtime.py` 的 `test_ros_log_preformats_unicode_before_rospy` 通过
    （带 ROS 环境运行）。
  - 端到端脚本复现崩溃行：`format_ros_log(u'...mirror=%s', (u'cuda', False, 0.10,
    'ON(翻转)'.encode('utf-8')))` 输出与期望一致；`lane_loginfo` 直接调用无异常；
    `rospy.loginfo is lane_loginfo` 绑定确认。
- 注：`test_lane_runtime.py` 另有 2 项 FAIL（launch 的 `required`/`launch_prefix` 已参数化、
  测试仍断言写死值）与 1 项 ERROR（测试寻找 `ensure_segmentation_model`，当前代码为
  `ensure_seg`）——与本次改动无关，属测试文件与小车端版本不同步，未在本改动内处理。

## 生效方式

修复已部署到小车（`scp` 到 `~/ucar_ws/src/lane_proto/scripts/lane_follow.py`），
无需编译（Python 脚本）。下次 roslaunch 该节点即可生效。
