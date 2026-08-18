# trackseg-small 固化 TrackSegNet(small) 分割网
#   make               -> libtrackseg.so      (CUDA sm_53, **fp16**)
#   make FP32=1        -> 同上但 fp32 存储(回退用)
#   make cpu           -> libtrackseg_cpu.so  (无卡验证, fp16)
#   make cpu FP32=1    -> fp32 版 CPU 库
#   make clean
# Nano 上若 nvcc 不在 PATH: export PATH=/usr/local/cuda/bin:$PATH
#
# 精度说明: 存储 fp16 / 累加 fp32(见 src/ts_half.h)。用实拍验证集量过:
# loss 与 fp32 相同, 白线 IoU 变化 -0.0000, argmax 掩码一致率 99.998%。
# sm_53(Tegra X1) 是少数 fp16 真能提速的 Maxwell —— 桌面 Maxwell(sm_52)
# 的 fp16 只有 1/64 速率, 别拿那个的经验套。
#
# 换权重后必须先重新导出再编译:
#   python3 export_cuda_weights.py seg_best.pth && make

NVCC ?= nvcc
ARCH ?= -gencode arch=compute_53,code=sm_53

#   make            fp16 + half2 打包(默认, 最快)
#   make H2=0       fp16 标量版(存fp16/纯fp32累加, 上一版行为, A/B 对比用)
#   make FP32=1     纯 fp32(基准)
ifeq ($(FP32),1)
PREC := -DTS_FP32
else ifeq ($(H2),0)
PREC := -DTS_NO_H2
else
PREC :=
endif

all: libtrackseg.so

libtrackseg.so: src/trackseg_cuda.cu src/core_ops.h src/ts_half.h \
                src/net_graph.inc gen/weights_embed.c gen/net_meta.h
	$(NVCC) -O3 $(ARCH) $(PREC) -Xcompiler -fPIC -shared \
	    src/trackseg_cuda.cu gen/weights_embed.c -o $@

# CPU 参考库故意不开 OpenMP: pip 版 cv2 自带的 OpenMP 与 libgomp 同进程
# 会段错误; 验证用单线程足够。fp16 在 CPU 上是软件转换, 很慢, 只用来验数。
cpu: libtrackseg_cpu.so
libtrackseg_cpu.so: src/trackseg_cpu.c src/core_ops.h src/ts_half.h \
                    src/net_graph.inc gen/weights_embed.c gen/net_meta.h
	$(CC) -O2 -fPIC -shared -Wno-unknown-pragmas $(PREC) \
	    src/trackseg_cpu.c gen/weights_embed.c -o $@ -lm

clean:
	rm -f libtrackseg.so libtrackseg_cpu.so
.PHONY: all cpu clean
