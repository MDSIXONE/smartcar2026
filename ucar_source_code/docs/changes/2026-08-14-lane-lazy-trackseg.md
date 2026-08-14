# lane_proto 延迟加载 TrackSeg/CUDA

## 原因

主流程以 lane_proto 常驻待命时，`LaneFollow.__init__()` 立即构造 `TrackSeg`，导致启动期就打印
`[trackseg] ... cuda ...` 并占用 Nano GPU；此时巡线尚未被交接控制权。

## 修改

- `STANDBY` 只保存 TrackSeg 动态库路径，`self.seg` 保持为空。
- 新增幂等 `ensure_segmentation_model()`；`/lane_proto/set_active true` 在置为 `FOLLOW` 前调用它。
- 独立巡线模式（`start_enabled=true`）仍在 `run()` 开始时加载，保持原本启动即巡线的语义。

## 验证

- 修复前车端定向回归因缺少延迟加载入口失败。
- 修复后本机静态回归、车端 Python2 语法检查、两项定向回归与 lane_proto 全量 7 项测试均通过，
  0 errors / 0 failures。
- 未启动实际 ROS/CUDA 节点；首次现场交接应观察 TrackSeg 输出仅出现在
  `PRODUCTION_TASK_STATE LANE_ACTIVE` 前的激活服务调用期间。

## 生效方式

修复已部署。下一次主流程启动时 lane_proto 会在待命期跳过模型加载；首次终点交接才加载模型。
