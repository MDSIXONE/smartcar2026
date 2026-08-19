# yolo_tiny_cuda

独立的 yolov4-tiny 推理引擎。网络结构**写死在代码里**，权重从外部 `.weights` 文件读取。
只依赖 CUDA runtime —— 不用 cuDNN、不用 cuBLAS、不用 OpenCV、不用 darknet。

启动后从 stdin 收二进制图像帧，往 stdout 回 JSON 结果。附带一个把它当子进程拉起来的
Python demo。

```
yolo_tiny_cuda/
├── Makefile
├── cfg/
│   ├── yolov4-tiny-tl7-416x256-w75.cfg   # ← 当前网络的唯一事实来源
│   └── obj.names            # 7 个类名
├── include/yolo.h           # 全部对外声明 + 网络常量（常量段是生成的）
├── src/
│   ├── net_def.cpp          # 38 层结构表 —— **生成物，不要手改**
│   ├── weights.cpp          # darknet .weights 解析 + BN 折叠
│   ├── image.cpp            # stb_image 解码 + darknet resize_image 复刻
│   ├── infer_cpu.cpp        # CPU 参考实现
│   ├── kernels.cu           # 手写 CUDA kernel
│   ├── postprocess.cpp      # yolo 解码 + DIoU-NMS
│   └── main.cpp             # 协议 + 主循环
├── third_party/stb_image.h  # public domain
├── detect.py                # Python 子进程 demo
└── tools/
    ├── gen_net_def.py       # cfg -> yolo.h 常量段 + net_def.cpp
    └── compare_darknet.py   # 和 darknet 逐层对拍的脚本
```

---

## ⚠️ 关于目标平台

**Jetson Nano 不是图灵架构。** 两种情况差别很大：

| 型号 | 架构 | arch | 最高 JetPack / CUDA |
|---|---|---|---|
| Jetson Nano（2019 原版） | Maxwell | `sm_53` | JetPack 4.6.x / **CUDA 10.2**，已 EOL |
| Jetson Orin Nano（2023） | Ampere | `sm_87` | JetPack 5/6 / CUDA 11.4+ |

代码只用 CUDA 10.2 就存在的基础特性（没有 cooperative groups、没有 cub、
没有现代 half intrinsics），所以两边都能编。已验证 `sm_53` / `sm_87` / `sm_89` 三个目标
都能通过 nvcc 编译。

**精度目前只做 FP32。** 先保证能和 darknet 逐层对齐、可验证；kernel 的标量类型是集中的，
后面加 FP16 是局部改动。原版 Nano 上 FP32 预计 15~20 FPS，FP16 大约能翻倍。

---

## 当前网络

| | |
|---|---|
| cfg | `cfg/yolov4-tiny-tl7-416x256-w75.cfg` |
| 输入 | **416 × 256** |
| 类别 | **7**：`left right stop straight` + `yellow left/right/straight` |
| 通道 | 基线的 **0.75 倍**（剪枝版，**不能**用 `yolov4-tiny.conv.29` 预训练） |
| 两个 head | 层 30 `mask 3,4,5` → 13×8；层 37 `mask 1,2,3` → 26×16 |
| 权重大小 | `.weights` 应为 **13,292,756 字节**（对不上就是 cfg 和权重不配套） |

> ⚠ **第二个 head 的 mask 是 `1,2,3`，不是通用 yolov4-tiny 的 `0,1,2`。**
> 我们这一系 cfg 都是从 `yolov4-tiny-xf_724.cfg` 改出来的，它就这么写。
> mask 抄错不会报任何错，只会让小目标框的宽高整体缩放一个 anchor 的比例 ——
> 置信度看着还挺高。这也是下面那个生成器存在的理由。

## 换网络（改输入尺寸 / 类别数 / 通道宽度）

**不要手改 `src/net_def.cpp` 和 `include/yolo.h` 里的常量。** 把新的 cfg 和
`obj.names` 放进 `cfg/`，然后：

```bash
make gen CFG=cfg/新的.cfg NAMES=cfg/obj.names   # 重新生成
make clean && make GPU=1 ARCH=sm_53
```

生成器会顺带校验：两个 head 的 anchors/classes/scale_x_y 是否一致、head 前一层
`filters` 是不是 `(classes+5)*3`、mask 有没有越界、route 两路空间尺寸对不对得上，
并算出 `.weights` 应该多大。

`make` 每次都会先跑 `make check-cfg`（代码和 cfg 对不上就直接停），所以不会出现
"cfg 换了、代码忘了重新生成"这种情况。

## 编译

```bash
make                                  # 只编 CPU 版（无 CUDA 也能编，用于验证/开发）
make GPU=1                            # CUDA 版，默认 ARCH=sm_53（原版 Jetson Nano）
make GPU=1 ARCH=sm_87                 # Jetson Orin Nano
make GPU=1 ARCH=sm_89                 # RTX 5000 Ada（你的笔记本）
make GPU=1 CUDA_DIR=/usr/local/cuda   # CUDA 安装路径不在默认位置时
```

## 跑起来

```bash
# Python demo：把可执行文件当子进程拉起来
./detect.py --weights backup/yolov4-tiny-tl7-416x256-w75_final.weights a.jpg b.jpg
{}
{"left":[[0.512,0.377,0.052,0.088,0.9912]]}

# 单次调试模式（不走协议，直接打完整 JSON 后退出）
./yolo_tiny_cuda --weights xxx.weights --oneshot a.jpg
```

每行对应一张输入图，顺序和命令行一致。框是 `[cx, cy, w, h, conf]`，
`cx/cy/w/h` 是相对**原图**的归一化坐标，和你训练用的 YOLO 标签格式完全一致。

同一类有多个框时是二维数组；加 `--flat` 可以在"每类只有一个框"时退化成一维数组
（就是你举例的那个形状）。默认保持二维是为了让类型稳定，下游不用做分支判断。

---

## 通讯协议 `yolo-pipe/1`

stdin / stdout 走二进制定长帧头，**stderr 只用于日志，不参与协议**。

### 帧格式

```
偏移  长度  内容
 0     2    magic = 'Y','V'
 2     1    type
 3     1    flags
 4     4    id       (uint32 小端)
 8     4    length   (uint32 小端)
12   length payload
```

### 帧类型

| type | 名称 | 方向 | payload |
|---|---|---|---|
| `0x01` | HELLO | 子进程 → 调用方 | JSON，能力描述 |
| `0x02` | CONFIG | 调用方 → 子进程 | JSON，运行参数 |
| `0x03` | READY | 子进程 → 调用方 | JSON，生效后的参数 |
| `0x10` | IMAGE | 调用方 → 子进程 | 图像数据，见下 |
| `0x11` | RESULT | 子进程 → 调用方 | JSON，检测结果 |
| `0x1F` | ERROR | 子进程 → 调用方 | JSON，`{"error": "..."}` |
| `0x7F` | BYE | 调用方 → 子进程 | 空，优雅退出 |

### 握手

进程一启动就**主动**发一帧 HELLO，调用方必须先把它读掉：

```json
{"proto":"yolo-pipe/1","impl":"yolo_tiny_cuda","backend":"cuda",
 "input":{"w":416,"h":256,"c":3},
 "encodings":["auto","rgb8","bgr8"],
 "classes":["left","right","stop","straight",
            "yellow left","yellow right","yellow straight"],
 "defaults":{"conf":0.25,"nms":0.45,"box_format":"cxcywh_norm"}}
```

这样做的好处是**类别名和输入尺寸由引擎单方面声明**，Python 侧不需要重复维护一份
`obj.names`，换模型时不会两边对不上。

CONFIG 是可选的，不发就用 defaults。可设的字段：

```json
{"conf": 0.25, "nms": 0.45, "box_format": "cxcywh_norm", "timings": true}
```

`box_format` 可选 `cxcywh_norm`（默认，归一化中心宽高）或 `xyxy_pixel`（原图像素左上右下）。
子进程回一帧 READY 确认实际生效的值。

### 送图

IMAGE 帧的 `flags` 决定 payload 怎么解析：

- `0` — 编码图像的原始文件字节（JPEG / PNG / BMP / TGA / GIF，走 stb_image 自动识别）
- `1` — raw RGB8：`uint32 w`、`uint32 h`，然后 `w*h*3` 字节交错像素
- `2` — raw BGR8：同上，但通道序是 BGR（OpenCV 的默认顺序，省一次 `cvtColor`）

**id 由调用方指定**，RESULT 会原样带回来。调用方可以连续灌多张图再统一收结果做流水线，
子进程按到达顺序处理，靠 id 对上号。

### 结果

```json
{"id":1,"w":1920,"h":1080,"n":1,
 "det":[{"cls":0,"name":"left","conf":0.9912,"box":[0.512,0.377,0.052,0.088]}],
 "ms":{"pre":2.1,"infer":8.4,"post":0.3}}
```

`w`/`h` 是**原图**尺寸。`ms` 里 `pre` 含解码和缩放，`infer` 含 H2D/D2H 拷贝。

一帧 IMAGE 恰好对应一帧 RESULT 或一帧 ERROR，不会出现丢帧或多帧。
解码失败这类错误只影响当前这一帧，进程不退出，可以继续送下一张。

---

## 数值正确性

这不是"看起来能跑"，是和 darknet 对拍过的。方法：本地编译 darknet CPU 版，打补丁让它
把每层输出落盘，用同一份随机权重和同一张图跑两边，逐层比对。

**结果：38 层里 36 个可直接比对的层，最大相对误差 4.6e-6。**

```
层   元素数      最大绝对误差    相对幅值
0    1802240    8.345e-07     4.004e-07
...
28   112640     3.004e-05     4.570e-06     <- 最差的一层
36   23760      1.171e-05     3.762e-06
```

这个量级就是浮点累加顺序不同带来的噪声（darknet 走 im2col+GEMM，本实现走直接卷积），
不是算法差异。第 30 / 37 层没直接比对，因为 darknet 在 yolo 层里就做了 sigmoid 和
`scale_x_y`，本实现放在后处理做，两者语义位置不同、结果等价。

端到端再比一次检测框：同一张图、阈值 0.25，两边都输出 **1601 个框**，
按 (类别, 坐标±2px) 匹配 **1598/1601 = 99.8%**，剩下 3 个是坐标落在量化桶边界上
（宽 24 vs 25）导致的，实际是亚像素差异。平均置信度两边都是 34.39%。

复现：

```bash
python3 tools/compare_darknet.py --darknet /path/to/darknet --cfg xxx.cfg --weights xxx.weights --image a.jpg
```

### 对齐 darknet 的几个细节

这些地方稍有不慎就会和 darknet 差一点，全部逐行核对过源码：

1. **BN 折叠的 eps 在 sqrt 里面**：`sqrt(var + 1e-5)`，不是 `sqrt(var) + 1e-5`
   （`src/blas.c normalize_cpu`）。
2. **缩放用 align_corners**：`scale = (src-1)/(dst-1)`，且先横后纵两趟
   （`src/image.c resize_image`）。这跟 OpenCV 的 `INTER_LINEAR` 不一样，用错会让框整体偏移。
3. **预处理是拉伸不是 letterbox**：cfg 里 `letter_box` 没开，`detector test` 走 `resize_image`。
   所以归一化坐标对原图和网络输入是同一个值，不需要还原。
4. **NMS 用的是 DIoU 不是 IoU**：cfg 里 `nms_kind=greedynms`，darknet 走
   `diounms_sort()` 里的 `box_diou()` 分支 —— IoU 减去 `(中心距²/最小包围盒对角²)^0.6`
   （`src/box.c`）。用普通 IoU 会在密集框时给出不同结果。
5. **x,y 的 scale_x_y 在 sigmoid 之后**：`x = sigmoid(tx) * 1.05 - 0.025`
   （`src/yolo_layer.c forward_yolo_layer`）。
6. **route 的 groups 切的是连续内存块**：`part = total/groups`，偏移 `part*group_id`。
   在 CHW 布局下等价于取后一半通道。
7. **maxpool 的 padding**：darknet 默认 `pad = size-1`，但 `offset = -pad/2` 整数除后是 0，
   所以 2x2/stride2 就是普通无 padding 池化。

---

## 还没做 / 已知限制

- **CUDA 路径没有在真实 GPU 上跑过。** 我这边只有 CPU，能做的是：三个目标架构都过了
  nvcc 编译、CPU 参考实现和 darknet 逐层对齐。CUDA kernel 和 CPU 实现是同一套数学，
  但**第一次上机请先用 `--backend cpu` 和 `--backend cuda` 各跑一张图比对结果**，
  确认一致再上生产。
- **只支持 batch=1。** 协议层预留了 id 做流水线，但引擎内部一次一张。
- **kernel 没做深度调优。** 现在是共享内存分块的直接卷积，1x1 卷积用的是同一套代码，
  对 512 通道的层不是最优。真嫌慢再优化。
- **FP16 没做。**
- 换网络结构必须同时改 `net_def.cpp` 的层表和 `yolo.h` 里的 `NET_W/NET_H/NUM_CLASSES`，
  没有 cfg 解析器 —— 这是"写死结构"的代价。`weights.cpp` 结尾会校验文件大小，
  结构对不上会直接报错而不是静默读错。

---

# 用真权重跑出来的实测成绩（纯 C/C++ CPU 后端）

以下全部用 `--backend cpu`（不含任何 CUDA，纯 C++ + OpenMP）跑出来，
权重是你训练的 `yolov4tinytraffic_final.weights`（header 里 `seen=512000`，
即 8000 iteration × batch 64，确认是跑完的那份）。

## 先确认数值仍与 darknet 一致（这次是真权重）

```
38 层中 36 个可直接比对的层，最大相对误差 3.783e-06
darknet 检出 1 个框，本实现 1 个框
```

在 `bright_stop_front_01.jpg` 上，本实现给出 `stop conf=0.9982`。

**交叉印证**：darknet 自己 `detector map` 报的是 `average IoU = 83.10%`，
本实现在同样 16 张 valid 上算出 **83.12%**。两条完全独立的代码路径，
差 0.02 个百分点。

## 正样本

| 测试集 | n | 命中率 | 平均 IoU | 平均 conf | 多余框 |
|---|---|---|---|---|---|
| valid 16 张原图 | 16 | **100.00%** | 0.8312 | 0.9985 | 0 |
| 全部 80 张原图 | 80 | **100.00%** | 0.8720 | 0.9986 | 0 |
| 合成增强集抽样 | 120 | **100.00%** | 0.9502 | 0.9982 | 0 |

命中 = 最高分框类别正确 **且** 与真值 IoU ≥ 0.5。四类混淆矩阵全对角，零错分。
216 张里一个多余框都没有。

## 负样本（此前完全没测过的场景）

| 测试集 | n | 干净率 | 误检框 |
|---|---|---|---|
| 纯室内背景（MIT Indoor-67） | 120 | **100.00%** | 0 |
| **场地原图移除灯箱** | 80 | **100.00%** | 0 |

第二项是重点：拿你那 80 张原图，把灯箱区域（外扩 40%，连光晕一起）用同一行
偏移的挡板内容补掉，保留整个场地在自然尺度下的样子 —— 包括挡板上那排跟箭头灯
长得很像的印刷图案。**80 张一个误检都没有。** 之前我担心那排图案会成为误检来源，
实测证明没有；copy-paste 增强让模型学会了看灯本身而不是看环境。

（注意第一项不算独立测试，那 120 张正是增强用的背景池，模型见过它们贴了灯的版本。
第二项才是独立的域内负样本。）

## 远距离能力边界 ⚠️ 这才是真问题

把原图整体缩小模拟灯变远（真值框同步缩放），24 张 × 4 档：

| 缩放 | 目标宽(原图) | 目标宽(网络输入) | 命中率 | 平均 IoU | 命中 conf 最小值 |
|---|---|---|---|---|---|
| 1.00 | 92 px | 31 px | **100.0%** | 0.872 | 0.990 |
| 0.50 | 46 px | 15 px | **100.0%** | 0.851 | 0.975 |
| 0.30 | 28 px | 9 px | 79.2% | 0.698 | 0.477 |
| 0.20 | 18 px | 6 px | **0.0%** | — | — |
| 0.13 | 12 px | 4 px | 0.0%（全部漏检） | — | — |

**能力边界在网络输入 ~15 px 处。** 15 px 以上满分，9 px 开始掉，6 px 完全崩。

崩的方式很值得警惕 —— **它不是安全地失败，而是自信地给错答案**：

```
f=0.20 (6px) 的 24 张:
  stop      6 张 -> 全部漏检（红圆在 6px 下彻底消失）
  right     6 张 -> 全部误判成 left，conf 0.369~0.585
  straight  4 张 -> 误判成 left，conf 0.288~0.396
  left      6 张 -> 类别对，但 IoU 只有 0.20~0.31，框回归已失效

f=0.30 (9px) 的 5 个失败全是 straight -> 被判成 left / right，conf 0.358~0.684
```

形状信息一消失，输出就塌缩到 `left`（默认吸引子）。对一个要靠这个决定往哪拐的
小车来说，这是最危险的失效模式。

**但有个干净的分界线可以用**：

```
可靠区 (f>=0.5)  命中样本 conf 最小 0.975
崩溃区 (f<=0.3)  错误预测 conf 最高 0.684
```

所以**部署时把置信度门限设到 0.9**，上面所有错误预测全被挡掉，而正常距离下的
正确检测（conf ≥ 0.975）一个不误杀。建议 ROS 节点里这么写：

```python
det.configure(conf=0.9)      # 而不是默认的 0.25
```

再稳一点可以加一条几何门限：框宽 < 画幅 2.5%（≈ 网络输入 15px）就直接丢弃，
因为实测那个尺度以下的结果不可信。

## CPU 后端性能

单张 640×352 中位 **6.9 秒**（2 核 / OMP_NUM_THREADS=1，直接卷积无优化）。
这条路径是给验证和无 GPU 环境兜底用的，不是给你跑实时的。
已加 `-fopenmp`（`make OPENMP=0` 可关），输出通道维度并行。

## 复现

```bash
# 正样本成功率
python3 tools/eval_cpu.py --weights xxx.weights --images data/obj/*.jpg --jobs 2
# 负样本误检率
python3 tools/eval_neg.py --weights xxx.weights --list bg_list.txt --n 120
```
