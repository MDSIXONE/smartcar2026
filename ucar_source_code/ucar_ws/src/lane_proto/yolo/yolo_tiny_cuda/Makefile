# yolo_tiny_cuda
#
#   make            -> 只编 CPU 版（无 CUDA 也能编，用于验证/开发）
#   make GPU=1      -> 编 CUDA 版，默认 ARCH=sm_53 (Jetson Nano)
#   make GPU=1 ARCH=sm_87   -> Jetson Orin Nano
#   make GPU=1 ARCH=sm_89   -> RTX 40 / RTX 5000 Ada
#
# 只依赖 CUDA runtime，不需要 cuDNN / cuBLAS / OpenCV。

GPU      ?= 0
ARCH     ?= sm_53
CUDA_DIR ?= /usr/local/cuda
NVCC     ?= $(CUDA_DIR)/bin/nvcc
CXX      ?= g++

TARGET  := yolo_tiny_cuda
BUILD   := build
INC     := -Iinclude -Ithird_party
OPENMP  ?= 1
CXXFLAGS := -std=c++11 -O3 -Wall -Wextra -Wno-unused-parameter $(INC)
LDFLAGS :=
# CPU 路径用 OpenMP 并行输出通道；Jetson 上编 GPU 版时它只影响 CPU 回退分支
ifeq ($(OPENMP),1)
  CXXFLAGS += -fopenmp
  LDFLAGS  += -fopenmp
endif

SRCS := src/net_def.cpp src/weights.cpp src/image.cpp src/postprocess.cpp \
        src/infer_cpu.cpp src/main.cpp
OBJS := $(patsubst src/%.cpp,$(BUILD)/%.o,$(SRCS))

ifeq ($(GPU),1)
  CXXFLAGS += -DUSE_CUDA -I$(CUDA_DIR)/include
  LDFLAGS  += -L$(CUDA_DIR)/lib64 -L$(CUDA_DIR)/lib64/stubs -lcudart
  NVCCFLAGS := -std=c++11 -O2 -DUSE_CUDA $(INC) \
               -gencode arch=compute_$(subst sm_,,$(ARCH)),code=$(ARCH) \
               -Xcompiler "-Wall -Wno-unused-parameter"
  OBJS += $(BUILD)/kernels.o
endif

# 当前网络对应的 cfg —— net_def.cpp / yolo.h 的常量都是从它生成的
CFG   ?= cfg/yolov4-tiny-tl7-640x352-w55.cfg
NAMES ?= cfg/obj.names

.PHONY: all clean run-selftest check-cfg gen
all: check-cfg $(TARGET)

# 编译前先确认代码里那张表和 cfg 一致。手抄 mask/filters 抄错不会有编译错误,
# 只会让框悄悄变歪 —— 所以卡在这里。
check-cfg:
	@python3 tools/gen_net_def.py $(CFG) --names $(NAMES) --check

# 换 cfg 时跑这个重新生成 include/yolo.h 的常量段 + src/net_def.cpp
gen:
	python3 tools/gen_net_def.py $(CFG) --names $(NAMES)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)
	@echo "==> 生成 $(TARGET)  (GPU=$(GPU) ARCH=$(ARCH))"

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/kernels.o: src/kernels.cu | $(BUILD)
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD) $(TARGET)
