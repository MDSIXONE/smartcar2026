# 仿真状态重连与两分钟继续主流程

## 目的

仿真 bridge 的 HTTP 状态连接临时关闭时，主流程应重连而非立刻 ABORT；等待两分钟仍未确认
完成时，继续小车终点与巡线流程。

## 修改

- `simulation_wait_done()` 将 `urllib2` 网络异常和 `httplib.HTTPException`
  （包括 `BadStatusLine: No status line received`）记录为重连事件；下一次轮询建立新连接。
- `simulation_done_timeout` 默认值与 launch 值均为 120 秒。
- bridge 返回 `failed`、持续 `running` 或持续断连都不会中止小车；到期发布
  `SIMULATION_TIMEOUT_CONTINUE` 并继续任务。只有 `done` 才发布 `SIMULATION_DONE`，但两种路径
  都播报“仿真任务已完成”。

## 验证

- 小车 Melodic Python2 语法检查通过。
- 4 项定向回归通过：done 正常返回、failed 到期继续、running 到期继续、
  `BadStatusLine` 后重连并得到 done。

## 已知限制

120 秒到期继续表示仿真结果未确认；终端会保留 `PRODUCTION_SIMULATION_*` 日志用于赛后核查。
