# lane_proto 增加 use_lidar=self 三态入口（历史记录，已由 V29 接管）

## 目的

该记录描述的是上一版工作区实现，随后已由用户提供的 `lane_proto_v29.zip` 覆盖；
当前以 V29 的 `use_lidar` 和 `goal_mode` 语义为准。

## 改动

上一版变更涉及 `lane_proto` 启动文件、巡线节点、测试和两套主流程；这些实现文件
不再是当前运行源，保留本记录仅用于追溯接口迁移。

## 验证

上一版回归结果不能作为当前 V29 版本的哈希依据；当前版本请看
`2026-08-19-lane-proto-v29-integration.md`。

## 已知限制

V29 接入前必须按新文档重新做车端哈希、Python2 和 launch 解析验收；独立测试使用
`take_cam_on_start:=true` 时会独占相机和底盘，不能与任一 2026 主流程同时启动。
