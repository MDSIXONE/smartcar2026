# trackseg-small — 固化 TrackSegNet(small) 的 CUDA 工程

把 `seg_best.pth`(small, 4 类)**编译期固化**进一个 `.so`：结构写死在 C 里、
权重变成字节数组编进二进制，运行时零外部文件、零 torch 依赖。
只用 CUDA runtime，不要 cuDNN / cuBLAS / TensorRT / OpenCV。

与 `cuda_seg/`(mnv2 版)是**同一套骨架**：`core_ops.h` 里的逐元素算子、
`trackseg.h` 的 C ABI、`trackseg.py` 的 ctypes 封装都一模一样，
只有 `net_graph.inc`(前向图)和 `export_cuda_weights.py`(权重顺序)是 small 专属的。
两个 `.so` 接口一致，换模型就是换文件。

```
cuda_seg_small/
├── Makefile
├── export_cuda_weights.py   # .pth -> BN折叠 -> weights.bin + gen/*
├── src/
│   ├── ts_half.h            # 精度抽象 fp16/fp32 + 软件 half 转换(CPU 用)
│   ├── core_ops.h           # 逐元素算子, CPU/GPU 共用同一份数学
│   ├── net_graph.inc        # small 的前向图(两个后端 #include 同一份)
│   ├── trackseg.h           # C ABI
│   ├── trackseg_cpu.c       # CPU 参考后端(验证用)
│   └── trackseg_cuda.cu     # CUDA 后端(sm_53 朴素 kernel)
├── gen/                     # 自动生成: weights_embed.c + net_meta.h
├── trackseg.py              # ctypes 封装 (车上真正 import 的东西)
├── seg_torch_infer.py       # 原地替换旧的同名文件, 接口不变、去掉 torch
├── verify_vs_torch.py       # 与 PyTorch 逐帧对拍
└── seg_best.pth             # small 权重(导出用, 上车不需要)
```

## 精度: 默认 FP16 + half2

`make` 现在编出来的是 **fp16 存储 + HFMA2 打包乘加** 的版本(`fp16+h2`)。
三档可选, 车上用 `check_so.py --camera` A/B 完再定:

| 编法 | 语义 | 何时用 |
|---|---|---|
| `make` | fp16 存储, half2 打包, 分块累加(块内 half, 块间 float) | 默认, 最快 |
| `make H2=0` | fp16 存储, 纯 fp32 累加, 标量 | h2 数值/速度存疑时回退 |
| `make FP32=1` | 全 fp32 | 基准 |

half2 的打包方向是**空间**(同通道两个相邻像素), 权重直接广播, 不用重排
权重 blob; 1x1 卷积的 src 沿空间连续, 是对齐的 half2 加载。分块累加
TS_BLK=16: 在 half2 里连乘加 16 项就刷进 float —— stem.1 的 288 项长累加
被切成 18 块, 精度可控。CPU 后端用软件 half 模拟同一个分块模型, 无卡
机器也能验数(很慢, 仅验证)。dw3x3 不打包: 只占 ~3% MACs, 不值得。

实测(CPU 模拟, 真实帧, vs fp32 基准):

| | fp16 标量 | fp16+h2 |
|---|---|---|
| 掩码一致(最差) | 99.9935% | 99.9854% |
| 分歧像素落在类别边界 | 100% | 100% |

h2 的分歧多一倍(块内 half 累加的代价), 但全部仍在"两类打平"的边缘上,
下游吃不出区别。实测 Nano: fp32 27.0ms / fp16 22.5ms / fp16+h2 20.7ms。

### v2 追加的三项(全部零风险或 CPU 侧已验)

1. **4 路展开**: 每线程算 4 个相邻像素(两条独立 HFMA2 链), 权重加载摊薄
   一半、索引开销摊薄 3/4。每个输出各自的累加序列与 2 路版逐位一致。
2. **融合 上采样+argmax**: 平时只要掩码, 不再把 245K 个 logits 写回显存
   再读一遍(省 ~1MB 往返 + 一次启动); `ts_infer_logits` 才走老两步路径。
3. **`__restrict__`**: 只读数据走 Maxwell 的纹理路径(LDG), 白捡带宽。

## 旧版精度说明(标量 fp16)

存储 fp16 / **累加 fp32**(见 `src/ts_half.h`)。累加器用 float 不是保守是必须——
`stem.1` 一个输出要累加 32×9=288 项，half 的 11 位尾数扛不住；而累加器是寄存器
里的标量，用 float 既不占带宽也不占显存。这些 kernel 是一线程一输出、反复从
global memory 拉权重和特征图，**瓶颈在访存**，存 fp16 直接把访存量砍半。

sm_53(Tegra X1) 是少数 fp16 真能提速的 Maxwell——桌面 Maxwell(sm_52) 的 fp16
只有 1/64 速率，别拿那个的经验套。BF16 在 Maxwell 上**根本不存在**(需要 sm_80+，
CUDA 11+ 才有 `cuda_bf16.h`)。

上之前在 PyTorch 侧用 177 张实拍验证帧量过(`seg/fp16_vs_fp32.py`)：

| | fp32 | fp16 |
|---|---|---|
| 验证 loss | 0.0462 | 0.0462 |
| IoU 其他/场地/线/灯 | .987/.973/.696/.921 | .987/.973/.696/.921 |
| argmax 掩码一致率 | — | 99.998% |
| 折叠后权重 max\|w\| | 4.8 | 0 个超 fp16 上限 |

CPU 后端没有硬件 half，用软件转换(已与 `numpy.float16` 逐位对拍：全部 65536 个
half 值 + 50 万个随机 float 全部一致)。只用于验证，慢无所谓。

## 编译

```bash
python3 export_cuda_weights.py seg_best.pth   # 换权重后必须先跑这步
make                    # -> libtrackseg.so      CUDA sm_53, fp16(默认)
make FP32=1             # -> 同上但 fp32 存储(回退用)
make cpu                # -> libtrackseg_cpu.so  无卡验证, fp16
make cpu FP32=1         # -> fp32 版 CPU 库
```

Nano 上 `nvcc` 不在 PATH 时先 `export PATH=/usr/local/cuda/bin:$PATH`。
Orin Nano 用 `make ARCH="-gencode arch=compute_87,code=sm_87"`。
`ts_backend()` 会报 `cuda/fp16` 这种。另外 **`ts_init()` 会往 stderr 刷 10 遍横幅**：

```
[trackseg] ##### fp16 ##### small/cuda  dev=NVIDIA Tegra X1 sm_53  weights=60650  built=Aug 12 2026 07:58:30
```

换 `.so` 是"拷文件"这种最容易出错的操作——编了 fp16 却拷了旧的 fp32、或者压根
没拷成功，从行为上几乎看不出来（掩码只差 0.002%）。所以把精度、权重数、
**编译时刻**、运行时真实的设备型号一起打出来：`built` 那个时间戳对不上，
就是没拷成功。嫌吵就 `make CFLAGS_EXTRA=-DTS_BANNER_TIMES=1`。

导出脚本**不用改**：嵌入的 blob 永远是 fp32，`ts_init` 按编译精度转一次
(fp16 时运行时内存减半，而 .so 里那 0.24MB 无所谓)。

## 验证

```bash
make cpu
python3 verify_vs_torch.py test_frame.jpg
```

判据按 `.so` 的编译精度自动切换：

**fp32**：logits 误差 <1e-2 且掩码 **100%** 一致，差一个像素就是图写错了。
实测 test_frame 4.4e-5 / 12 张实拍帧最大 5.0e-5，全部 100.0000%。

**fp16**：logits 误差会到 1e-1(存储只有 11 位尾数，必然的)，所以判据换成
掩码一致 ≥99.9%，**外加一条关键的**——把 `.so` 的输出同时和"PyTorch fp16 模拟"
比，与模拟的偏差必须小于与 fp32 的偏差。这条才是真正在验 kernel：它证明
kernel 做的正是我们验证过的那套算法，而不是别处写错了。实测：

| 用例 | vs fp32 | vs fp16模拟 | 掩码一致 |
|---|---|---|---|
| test_frame.jpg | 1.3e-1 | 9.1e-2 | 99.9967% |
| 12 张实拍帧 | 1.3e-1 | — | 最差 99.9935% |

白线像素数最大变化 0.06%，下游 `Decider` / `goal_block` 吃不出区别。

`ts_init()` 还会自查权重游标：图和导出脚本对不上就直接返回 -2，
不会带着错位的权重跑起来。

## 数字

| | small | mnv2 |
|---|---|---|
| 权重 | 60,650 个 = **0.12 MB**(fp16) | 547,770 个 = 1.1 MB(fp16) |
| MACs | 102 M | 278 M |
| 卷积层 | 17 | 47 |

最贵的一层是 `stem.1`(48×48×80×32×9 ≈ 53M MAC，占全网一半)——它是**普通**
3×3 卷积不是 depthwise。真要再快，优化这一层的收益最大。

## 改网络的规矩

`net_graph.inc` 的 `TAKE()` 顺序 和 `export_cuda_weights.py` 的 `put()` 顺序
必须**逐字对应**，改一处就得改另一处，否则 `ts_init` 报
`graph/export mismatch`。改完重新导出 + 重编 + 跑 `verify_vs_torch.py`。
