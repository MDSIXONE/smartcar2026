# 星火 Spark 大模型二维码分类

## 目的

二维码阶段识别出的每个二维码文本，调用讯飞星火大模型（OpenAI 兼容 HTTP API）判断其属于
三个加工厂类别中的哪一类（日用品 / 食品 / 电子产品），并把分类结果记录到结果 JSON，供
后续分拣统计使用。分类失败不阻断任务：重试多次后降级到本地关键字映射，本地也无匹配时
只记录 source=none 并在终端输出"无法与模型取得联系"。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_qr_classifier.py`（新增，helper 子进程）
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `docs/operations.md`
- `犯错档案.md`

## 行为

- 模型与端点均可配置；默认使用深度推理 **Spark X2**：`spark_model=spark-x`、
  `https://spark-api-open.xf-yun.com/x2/chat/completions`，`spark_thinking=disabled`
  （分类任务关闭深度思考提速；`enabled` 可开启 X2 深度思考）。X1.5 备选端点
  `/v2/chat/completions`。旧 `lite` 接口（`/v1/chat/completions`）不再默认使用。
- `2026.launch` 默认 `spark_classify_enabled=true`，`spark_password_file`
  `/home/ucar/.ucar/spark_password`；密码文件缺失时 helper 自动跳过远程、纯本地映射，
  不阻断任务。
- 鉴权为 `Authorization: Bearer <APIPassword>`；密码文件（首行即密码）由小车本地保存，
  仓库不入库，launch 只给路径 `spark_password_file`。密码文件缺失时跳过远程调用，
  直接使用本地映射。
- helper 与任务节点以每行一个 JSON 的 stdin/stdout 协议通信（与
  `production_camera_ocr.py` 同模式）；任务线程永不阻塞在网络请求上，helper 崩溃、
  超时或返回异常都只降级不中止任务。
- 每个二维码分类顺序：远程 Spark 重试 `spark_retries + 1` 次 → 本地关键字映射 →
  source=none。本地映射内置 168 个常见物品关键字，可通过 `--local-map-file`
  加载自定义 JSON 覆盖。
- `observations.json` 与 `/ucar_2026/task_result` 新增 `qr_classifications` 数组，
  每条含 `observation`、`qr_text`、`category`、`source`、`attempts`、`model`、`error`。

## 验证结果

- 本机 Python 3.13 下 `py_compile` 两个 Python 文件均通过；helper 协议测试：
  - 本地映射：`旺仔牛奶` → `食品`（local）、`手机充电器` → `电子产品`（local）、
    `抽纸巾` → `日用品`（local）、`SGM-2026-QR001` → `none`（本地无匹配）。
  - 假密码文件触发真实 HTTP 401（`HMAC signature cannot be verified: apikey not found`）
    → 按 `--retries` 重试 → 降级 local 返回 `食品`，`attempts=2`、`error` 保留
    HTTP 错误信息。降级链路完整。
  - **Spark X2 实测（真实凭据）**：`https://spark-api-open.xf-yun.com/x2/chat/completions`
    + model `spark-x`，三个样例全部 `source=spark`：`旺仔牛奶` → `食品`、
    `手机充电器` → `电子产品`；`SGM-2026-QR001`（无语义文本）模型猜测
    （`日用品`/`电子产品` 不稳定，属预期）。`thinking=enabled` 与 `disabled` 均可用；
    `disabled` 响应更快，模型回复可能包裹 ```json 代码块，容错解析已覆盖。
  - **11200 根因**：此前用旧 `lite` 端点（`/v1/chat/completions`）时提示
    `AppIdNoAuthError 11200`（该 appId 没有相关功能的授权）——用户领取的是 X2 授权体验，
    lite 接口未授权；换 X2 端点后即通。
  - 实测每次 classify `attempts=2`（retries=1 时首次请求失败后重试成功），helper 重试
    兜底生效。
- 发现并修复：PowerShell 管道传中文到 Python 时因 Windows locale 乱码导致本地匹配
  误判；helper 启动时强制 stdin/stdout 为 UTF-8（Python 3 `reconfigure` /
  Python 2 `codecs`），同时防御小车端 LANG=C 时 Python 2 按 ASCII 解码中文崩溃。
- Python 2 兼容：日志打印中文 unicode 前先转 UTF-8 字节（复用 2026-08-04 犯错档案
  经验），`json.dumps` 输出保持 `ensure_ascii` 转义，任务侧 `json.loads` 不遇到中文。

## 已知限制

- 尚未在小车 Ubuntu 18.04 / ROS Melodic 部署与实车验证；小车端只能编译/运行
  Python 2 任务代码与 Python 3 helper，本机不能编译后上传。
- 免费 `lite` 模型的识别质量未在真实二维码文本上评估；类别白名单严格限定为
  日用品 / 食品 / 电子产品，模型回复其它文本一律视为无效并重试/降级。
- X2 为授权体验模式，有日/秒级流控（错误码 11201/11202/11203 与 10007 用户流量受限）；
  任务按 `spark_retries` 重试并在连续无结果时降级本地映射。
- 远程调用需要小车能访问公网（`spark-api-open.xf-yun.com:443`），网络不可达时
  依赖本地映射兜底。
