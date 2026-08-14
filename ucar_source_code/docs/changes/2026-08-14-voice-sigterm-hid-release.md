# 阵列麦 SIGTERM 的 HID 释放

## 原因

主流程在收到有效双类别语音后以 `SIGTERM` 结束持续运行的 `wake_listen.py`。Python 默认处理
SIGTERM 时不保证进入脚本的 `finally`，因此原有 `mic.close()` 不能作为 HID USB 句柄已释放的
保证。

## 修改

- `wake_listen.py` 注册 SIGTERM 处理器，收到主流程终止请求后主动停止录音并调用
  `mic.close()`。
- 正常退出和 SIGTERM 退出统一走同一释放函数。
- `mic_array.py` 的录音停止改为幂等，信号处理器与 `finally` 重复执行时不会重复向设备发送
  finish-record 指令；`hid_close()` 本身仍只在句柄已打开时调用一次。

## 验证

- 本机与小车 `/usr/bin/python3` 均通过两个脚本的语法检查。
- 小车源码结构回归确认存在 SIGTERM 注册、统一释放调用与录音状态幂等保护。
- 未启动或终止现场麦阵列，故本次未进行真实 HID 打开/关闭试验。

## 生效方式

代码已部署；下一次主流程的语音监听实例立即使用。常规停止应保留 SIGTERM 宽限期，不能用
SIGKILL 代替。
