#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三类分割小网 (0=其他/墙壁杂物, 1=场地蓝垫, 2=白线)
目标平台: Jetson Nano 4GB (Maxwell 128 CUDA cores), TensorRT FP16
设计原则: 只用 TensorRT 完全支持的算子(conv/bn/relu/双线性上采样/add),
          无 depthwise 大 kernel、无 attention, 保证 FP16 引擎一次转换成功。
参数量 ~0.06M(此场景两类颜色区分度极高, 小容量足够且更抗过拟合),
输入 320x192。CPU onnxruntime 已 ~3ms/帧, Nano TensorRT FP16 预计 >30fps。
若现场发现分割质量不够, 把各层通道数 x2 即可(仍远在 Nano 算力内)。
依赖: pip install torch onnx (训练机); Nano 上只需 tensorrt+pycuda。
结构: Fast-SCNN 风格 —— 下采样干路 + 轻量特征融合 + 小解码头。
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

N_CLASSES = 4  # 0其他 1场地 2白线 3红绿灯
IN_W, IN_H = 320, 192          # 训练/部署统一分辨率 (16 的倍数)


def conv_bn(cin, cout, k=3, s=1):
    return nn.Sequential(
        nn.Conv2d(cin, cout, k, s, k // 2, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True))


class DSConv(nn.Module):
    """depthwise separable conv (3x3 dw + 1x1 pw) —— TensorRT 支持良好"""

    def __init__(self, cin, cout, s=1):
        super().__init__()
        self.dw = nn.Sequential(
            nn.Conv2d(cin, cin, 3, s, 1, groups=cin, bias=False),
            nn.BatchNorm2d(cin), nn.ReLU(inplace=True))
        self.pw = nn.Sequential(
            nn.Conv2d(cin, cout, 1, bias=False),
            nn.BatchNorm2d(cout), nn.ReLU(inplace=True))

    def forward(self, x):
        return self.pw(self.dw(x))


class TrackSegNet(nn.Module):
    def __init__(self, n_classes=N_CLASSES):
        super().__init__()
        # 高分辨率浅层分支 (1/4)
        self.stem = nn.Sequential(
            conv_bn(3, 32, s=2),      # 1/2
            conv_bn(32, 48, s=2))     # 1/4
        # 深层语义分支 (1/16)
        self.deep = nn.Sequential(
            DSConv(48, 64, s=2),      # 1/8
            DSConv(64, 64),
            DSConv(64, 96, s=2),      # 1/16
            DSConv(96, 96),
            DSConv(96, 96))
        # 融合: 深层上采样回 1/4 与浅层相加
        self.fuse_deep = nn.Sequential(
            nn.Conv2d(96, 48, 1, bias=False), nn.BatchNorm2d(48))
        self.fuse_shallow = nn.Sequential(
            nn.Conv2d(48, 48, 1, bias=False), nn.BatchNorm2d(48))
        # 解码头
        self.head = nn.Sequential(
            DSConv(48, 48),
            nn.Conv2d(48, n_classes, 1))

    def forward(self, x):
        s = self.stem(x)                                     # (B,32,H/4,W/4)
        d = self.deep(s)                                     # (B,64,H/16,W/16)
        d = F.interpolate(self.fuse_deep(d), size=s.shape[2:],
                          mode="bilinear", align_corners=False)
        y = F.relu(d + self.fuse_shallow(s))
        y = self.head(y)                                     # (B,C,H/4,W/4)
        return F.interpolate(y, scale_factor=4,
                             mode="bilinear", align_corners=False)


def export_onnx(ckpt_path="seg_best.pth", out="track_seg.onnx"):
    net = TrackSegNet().eval()
    net.load_state_dict(torch.load(ckpt_path, map_location="cpu"))
    dummy = torch.zeros(1, 3, IN_H, IN_W)
    try:                       # 新版 torch 需显式走 legacy 导出器
        torch.onnx.export(net, dummy, out, opset_version=13, dynamo=False,
                          input_names=["image"], output_names=["logits"])
    except TypeError:          # 旧版 torch 无 dynamo 参数
        torch.onnx.export(net, dummy, out, opset_version=13,
                          input_names=["image"], output_names=["logits"])
    print(f"exported -> {out}")
    # Nano 上转 FP16 引擎:
    #   /usr/src/tensorrt/bin/trtexec --onnx=track_seg.onnx --fp16 \
    #       --saveEngine=track_seg.engine


if __name__ == "__main__":
    net = TrackSegNet()
    n = sum(p.numel() for p in net.parameters())
    y = net(torch.zeros(2, 3, IN_H, IN_W))
    print(f"params: {n/1e6:.2f}M  out: {tuple(y.shape)}")
