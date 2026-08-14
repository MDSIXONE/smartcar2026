# qrcode_scanner 网址到物品的单进程缓存

## 修改

- 在 `qrcode_scanner` 实例内保存成功 API 查询的“规范化网址 → 物品名”。
- 相同网址再次出现时不创建 HTTP 查询线程，直接发布原有 `/qr_api_result` 格式，并附加
  `cached: true`。
- 只缓存具有非空 `result` 的成功响应；失败响应不会写入缓存。
- 缓存不落盘，节点进程退出即清空。

## 验证

- 新增 Python3 回归：同一网址（含首尾空白差异）连续查询两次，只调用一次 HTTP；第二条
  `/qr_api_result` 含 `cached=true` 且物品为首次 API 结果。
- 本机 Python3 与车端 `/home/ucar/myenv/bin/python3` 均完成语法检查及该回归，结果通过。

## 生效方式

修改已部署至小车。运行中的 `qrcode_scanner` 不会热加载；下一次主流程启动后创建的新节点开始
使用缓存，不需要额外启动或停止其它节点。
