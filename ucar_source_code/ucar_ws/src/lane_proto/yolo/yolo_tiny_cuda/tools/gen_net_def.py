#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_net_def.py —— 从 darknet .cfg 生成 include/yolo.h 的常量段 + src/net_def.cpp

为什么要有这个脚本
------------------
net_def.cpp 里那张 38 层的表原来是**手抄**的。手抄有两个坑, 都不会在编译期
报错, 只会让识别结果悄悄变歪:

  1. 通道数抄错 -> load_weights 里的"文件大小对不上"能兜住(会直接报错), 还算
     幸运。
  2. **mask 抄错 -> 兜不住。** anchor 选错一档, 框的宽高就整体缩放了一个倍数,
     置信度还挺高, 看日志完全正常。这次就差点踩到: 手写的表第二个 head 是
     mask 0,1,2, 而 xf_724 那一系(我们所有 cfg 的祖宗)第二个 head 是
     **mask 1,2,3**。

所以从现在起 net_def.cpp 是**生成物**, 不要手改。换 cfg 就重新跑一遍。

用法
----
    python3 tools/gen_net_def.py <cfg> --names <obj.names>
    python3 tools/gen_net_def.py cfg/yolov4-tiny-tl7-416x256-w75.cfg \
        --names cfg/obj.names                  # 就地改写 include/yolo.h + src/net_def.cpp
    python3 tools/gen_net_def.py <cfg> --names <n> --check   # 只对拍, 不写

--check 会把 cfg 和当前代码里的表逐项比对, 不一致就退出码 1 —— 可以挂在
CI 或者编译前跑一次。
"""
from __future__ import print_function

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


# ------------------------------------------------------------------ cfg 解析
def parse_cfg(path):
    """-> [(section_name, {k: v}), ...], 保持顺序, 允许同名 section"""
    out = []
    cur = None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                cur = (line[1:-1].strip(), {})
                out.append(cur)
                continue
            if cur is None or "=" not in line:
                continue
            k, v = line.split("=", 1)
            cur[1][k.strip()] = v.strip()
    return out


def ints(s):
    return [int(x) for x in re.split(r"[,\s]+", s.strip()) if x != ""]


class Layer(object):
    def __init__(self, kind):
        self.kind = kind
        self.filters = 0
        self.ksize = 0
        self.stride = 0
        self.pad = 0
        self.bn = 0
        self.act = "ACT_LINEAR"
        self.route = [-1, -1]
        self.groups = 1
        self.group_id = 0
        self.pool_size = 0
        self.pool_stride = 0
        self.up_stride = 0
        self.mask = [0, 0, 0]


def build_layers(cfg):
    """cfg section 列表 -> ([Layer], net_dict, yolo_meta)"""
    net = None
    layers = []
    yolo_meta = []
    for name, d in cfg:
        if name == "net":
            net = d
            continue
        idx = len(layers)          # 这一层的绝对层号(darknet 不把 [net] 算进去)

        if name == "convolutional":
            L = Layer("conv")
            L.filters = int(d["filters"])
            L.ksize = int(d["size"])
            L.stride = int(d.get("stride", 1))
            L.pad = int(d.get("pad", 0))
            L.bn = int(d.get("batch_normalize", 0))
            a = d.get("activation", "linear")
            if a == "leaky":
                L.act = "ACT_LEAKY"
            elif a == "linear":
                L.act = "ACT_LINEAR"
            else:
                die("第 %d 层 activation=%s: 引擎只实现了 leaky/linear" % (idx, a))
            if int(d.get("groups", 1)) != 1:
                die("第 %d 层 convolutional groups!=1: 引擎没实现分组卷积" % idx)

        elif name == "maxpool":
            L = Layer("maxpool")
            L.pool_size = int(d["size"])
            L.pool_stride = int(d.get("stride", L.pool_size))

        elif name == "upsample":
            L = Layer("upsample")
            L.up_stride = int(d.get("stride", 2))

        elif name == "route":
            L = Layer("route")
            src = ints(d["layers"])
            if len(src) > 2:
                die("第 %d 层 route 有 %d 路: 引擎最多支持 2 路" % (idx, len(src)))
            # 负数是相对偏移, 换算成绝对层号
            absidx = [(idx + s if s < 0 else s) for s in src]
            for a in absidx:
                if a < 0 or a >= idx:
                    die("第 %d 层 route 源层 %d 非法" % (idx, a))
            L.route = [absidx[0], absidx[1] if len(absidx) > 1 else -1]
            L.groups = int(d.get("groups", 1))
            L.group_id = int(d.get("group_id", 0))

        elif name == "yolo":
            L = Layer("yolo")
            m = ints(d["mask"])
            if len(m) != 3:
                die("第 %d 层 yolo mask 有 %d 个: 引擎写死 3 个/head" % (idx, len(m)))
            L.mask = m
            yolo_meta.append({
                "idx": idx,
                "mask": m,
                "anchors": ints(d["anchors"]),
                "classes": int(d["classes"]),
                "num": int(d.get("num", len(ints(d["anchors"])) // 2)),
                "scale_x_y": float(d.get("scale_x_y", 1.0)),
                "prev_filters": layers[-1].filters if layers else 0,
            })
        else:
            die("不认识的 section [%s] (第 %d 层)" % (name, idx))
        layers.append(L)
    if net is None:
        die("cfg 里没有 [net]")
    return layers, net, yolo_meta


def die(msg):
    sys.stderr.write("✗ %s\n" % msg)
    sys.exit(2)


# ------------------------------------------------------------------ 一致性检查
def validate(layers, net, ym):
    if len(ym) != 2:
        die("找到 %d 个 [yolo] 层, 引擎写死 2 个" % len(ym))
    a, b = ym
    if a["anchors"] != b["anchors"]:
        die("两个 head 的 anchors 不一样")
    if a["classes"] != b["classes"]:
        die("两个 head 的 classes 不一样")
    if abs(a["scale_x_y"] - b["scale_x_y"]) > 1e-9:
        die("两个 head 的 scale_x_y 不一样")
    if a["num"] * 2 != len(a["anchors"]):
        die("num=%d 和 anchors 个数 %d 对不上" % (a["num"], len(a["anchors"]) // 2))
    nc = a["classes"]
    for h in ym:
        want = (nc + 5) * len(h["mask"])
        if h["prev_filters"] != want:
            die("第 %d 层 [yolo] 前面那个 conv filters=%d, 应该是 (%d+5)*3=%d"
                % (h["idx"], h["prev_filters"], nc, want))
        for m in h["mask"]:
            if m < 0 or m >= a["num"]:
                die("mask 里有 %d, 超出 num=%d" % (m, a["num"]))

    # 空间尺寸整除性: 走一遍前向, 顺便验证 route 两路尺寸一致
    W, H = int(net["width"]), int(net["height"])
    shp = []
    for i, L in enumerate(layers):
        ic, ih, iw = (int(net.get("channels", 3)), H, W) if i == 0 else shp[i - 1]
        if L.kind == "conv":
            p = L.ksize // 2 if L.pad else 0
            # 整数除法是**故意**的: darknet 就是这么算的(src/convolutional_layer.c
            # convolutional_out_height), 引擎里那份也一样。所以这里不能检查
            # "除不尽", 除不尽本身是合法的、两边算出来还一致。
            oh = (ih + 2 * p - L.ksize) // L.stride + 1
            ow = (iw + 2 * p - L.ksize) // L.stride + 1
            shp.append((L.filters, oh, ow))
        elif L.kind == "maxpool":
            oh = (ih + (L.pool_size - 1) - L.pool_size) // L.pool_stride + 1
            ow = (iw + (L.pool_size - 1) - L.pool_size) // L.pool_stride + 1
            shp.append((ic, oh, ow))
        elif L.kind == "upsample":
            shp.append((ic, ih * L.up_stride, iw * L.up_stride))
        elif L.kind == "route":
            s0 = shp[L.route[0]]
            c = s0[0] // L.groups
            if L.route[1] >= 0:
                s1 = shp[L.route[1]]
                if (s1[1], s1[2]) != (s0[1], s0[2]):
                    die("第 %d 层 route 两路空间尺寸 %s vs %s 不一致"
                        % (i, s0[1:], s1[1:]))
                c += s1[0] // L.groups
            shp.append((c, s0[1], s0[2]))
        else:                                        # yolo
            shp.append(shp[i - 1])
    return nc, a["anchors"], a["num"], a["scale_x_y"], shp


# ------------------------------------------------------------------ 代码生成
BANNER = ("// ⚠ 本文件由 tools/gen_net_def.py 从 %s 生成, **不要手改**。\n"
          "//   换 cfg: python3 tools/gen_net_def.py <新cfg> --names <obj.names>\n")


def emit_net_def(cfgname, layers, anchors, names, shp):
    L = []
    L.append(BANNER % cfgname)
    L.append('#include "yolo.h"\n#include <cstdio>\n\nnamespace yolo {\n')
    L.append("const float ANCHORS[NUM_ANCHORS * 2] = {\n    %s\n};\n"
             % ",  ".join("%d, %d" % (anchors[i], anchors[i + 1])
                          for i in range(0, len(anchors), 2)))
    L.append("const char* CLASS_NAMES[NUM_CLASSES] = { %s };\n"
             % ", ".join('"%s"' % n for n in names))
    L.append("""
// 简写宏，纯粹为了让下面这张表能一眼看出对应 cfg 的哪一行
#define CONV(f, k, s, bn, a) { L_CONV, f, k, s, 1, bn, a, {-1,-1}, 1, 0, 0, 0, 0, {0,0,0} }
#define CONVNP(f, k, s, bn, a) { L_CONV, f, k, s, 0, bn, a, {-1,-1}, 1, 0, 0, 0, 0, {0,0,0} }
#define ROUTE1(a)            { L_ROUTE, 0,0,0,0,0, ACT_LINEAR, {a,-1}, 1, 0, 0, 0, 0, {0,0,0} }
#define ROUTE2(a, b)         { L_ROUTE, 0,0,0,0,0, ACT_LINEAR, {a, b}, 1, 0, 0, 0, 0, {0,0,0} }
#define ROUTEG(a, g, gid)    { L_ROUTE, 0,0,0,0,0, ACT_LINEAR, {a,-1}, g, gid, 0, 0, 0, {0,0,0} }
#define MAXPOOL(k, s)        { L_MAXPOOL, 0,0,0,0,0, ACT_LINEAR, {-1,-1}, 1, 0, k, s, 0, {0,0,0} }
#define UPSAMPLE(s)          { L_UPSAMPLE, 0,0,0,0,0, ACT_LINEAR, {-1,-1}, 1, 0, 0, 0, s, {0,0,0} }
#define YOLO(m0, m1, m2)     { L_YOLO, 0,0,0,0,0, ACT_LINEAR, {-1,-1}, 1, 0, 0, 0, 0, {m0,m1,m2} }

const LayerDef LAYERS[NUM_LAYERS] = {""")
    for i, x in enumerate(layers):
        c, h, w = shp[i]
        note = "%dx%dx%d" % (c, h, w)
        if x.kind == "conv":
            mac = "CONV" if x.pad else "CONVNP"
            body = "%s(%d, %d, %d, %d, %s)," % (mac, x.filters, x.ksize,
                                                x.stride, x.bn, x.act)
        elif x.kind == "maxpool":
            body = "MAXPOOL(%d, %d)," % (x.pool_size, x.pool_stride)
        elif x.kind == "upsample":
            body = "UPSAMPLE(%d)," % x.up_stride
        elif x.kind == "route":
            if x.groups != 1:
                body = "ROUTEG(%d, %d, %d)," % (x.route[0], x.groups, x.group_id)
            elif x.route[1] >= 0:
                body = "ROUTE2(%d, %d)," % (x.route[0], x.route[1])
            else:
                body = "ROUTE1(%d)," % x.route[0]
        else:
            body = "YOLO(%d, %d, %d)," % tuple(x.mask)
        L.append("    /*%2d*/ %-38s // -> %s" % (i, body, note))
    L.append("};\n")
    L.append(BUILD_FN)
    return "\n".join(L)


BUILD_FN = r'''
bool build_network(Network& net, std::string* err) {
    net.max_workspace = 0;
    int ny = 0;
    for (int i = 0; i < NUM_LAYERS; ++i) {
        const LayerDef& d = LAYERS[i];
        LayerShape& s = net.shape[i];

        // 输入形状 = 上一层输出（route/yolo 另算）
        if (i == 0) { s.in_c = NET_C; s.in_h = NET_H; s.in_w = NET_W; }
        else        { s.in_c = net.shape[i-1].out_c; s.in_h = net.shape[i-1].out_h; s.in_w = net.shape[i-1].out_w; }

        switch (d.type) {
        case L_CONV: {
            int p = d.pad ? d.ksize / 2 : 0;
            s.out_w = (s.in_w + 2 * p - d.ksize) / d.stride + 1;
            s.out_h = (s.in_h + 2 * p - d.ksize) / d.stride + 1;
            s.out_c = d.filters;
            break;
        }
        case L_MAXPOOL:
            // darknet: padding 默认 = size-1, out = (w + padding - size)/stride + 1
            // 且 w_offset = -padding/2 = 0（整数除），所以对 2x2/2 就是普通无 padding 池化
            s.out_w = (s.in_w + (d.pool_size - 1) - d.pool_size) / d.pool_stride + 1;
            s.out_h = (s.in_h + (d.pool_size - 1) - d.pool_size) / d.pool_stride + 1;
            s.out_c = s.in_c;
            break;
        case L_ROUTE: {
            int a = d.route_from[0], b = d.route_from[1];
            if (a < 0 || a >= i) { if (err) *err = "route 源层非法"; return false; }
            s.in_c = net.shape[a].out_c; s.in_h = net.shape[a].out_h; s.in_w = net.shape[a].out_w;
            s.out_h = net.shape[a].out_h; s.out_w = net.shape[a].out_w;
            s.out_c = net.shape[a].out_c / d.route_groups;
            if (b >= 0) {
                if (net.shape[b].out_h != s.out_h || net.shape[b].out_w != s.out_w) {
                    if (err) *err = "route 两路空间尺寸不一致";
                    return false;
                }
                s.out_c += net.shape[b].out_c / d.route_groups;
            }
            break;
        }
        case L_UPSAMPLE:
            s.out_w = s.in_w * d.up_stride;
            s.out_h = s.in_h * d.up_stride;
            s.out_c = s.in_c;
            break;
        case L_YOLO:
            s.out_c = s.in_c; s.out_h = s.in_h; s.out_w = s.in_w;
            if (ny < 2) net.yolo_layers[ny++] = i;
            break;
        }
        s.out_elems = (size_t)s.out_c * s.out_h * s.out_w;
        if (s.out_elems > net.max_workspace) net.max_workspace = s.out_elems;
    }
    if (ny != 2) { if (err) *err = "未找到 2 个 yolo 层"; return false; }
    return true;
}

} // namespace yolo
'''


CONSTS = r"""// 这些值由 tools/gen_net_def.py 从 %(cfg)s 写入，**不要手改**。
constexpr int NET_W      = %(w)d;
constexpr int NET_H      = %(h)d;
constexpr int NET_C      = %(c)d;
constexpr int NUM_CLASSES = %(nc)d;
constexpr int NUM_ANCHORS = %(na)d;     // [yolo] num=%(na)d
constexpr int ANCHORS_PER_HEAD = 3;
constexpr float SCALE_XY = %(sxy)gf;  // [yolo] scale_x_y
constexpr float BN_EPS   = 1e-5f;  // darknet: sqrt(var + 0.00001f)"""


def patch_header(path, cfgname, net, nc, num, sxy, nlayers, write=True):
    src = open(path, encoding="utf-8").read()
    block = CONSTS % dict(cfg=cfgname, w=int(net["width"]), h=int(net["height"]),
                          c=int(net.get("channels", 3)), nc=nc, na=num, sxy=sxy)
    # ⚠ 用 subn 的替换次数判断"找没找到", 不能用 new == src ——
    #   值恰好没变时字符串相等, 会被误判成"没定位到"。
    new, n1 = re.subn(r"//[^\n]*\n(?:constexpr int NET_W.*?constexpr float BN_EPS[^\n]*)",
                      lambda m: block, src, count=1, flags=re.S)
    if not n1:
        die("没能在 %s 里定位常量段(NET_W .. BN_EPS)" % path)
    new2, n2 = re.subn(r"constexpr int NUM_LAYERS = \d+;",
                       "constexpr int NUM_LAYERS = %d;" % nlayers, new, count=1)
    if not n2:
        die("没能在 %s 里定位 NUM_LAYERS" % path)
    if write:
        open(path, "w", encoding="utf-8").write(new2)
    return new2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cfg")
    ap.add_argument("--names", required=True, help="obj.names, 一行一个类名")
    ap.add_argument("--header", default=os.path.join(ROOT, "include", "yolo.h"))
    ap.add_argument("--out", default=os.path.join(ROOT, "src", "net_def.cpp"))
    ap.add_argument("--check", action="store_true", help="只对拍不写, 不一致退出码 1")
    g = ap.parse_args()

    cfg = parse_cfg(g.cfg)
    layers, net, ym = build_layers(cfg)
    nc, anchors, num, sxy, shp = validate(layers, net, ym)

    names = [l.strip() for l in open(g.names, encoding="utf-8")
             if l.strip() and not l.startswith("#")]
    if len(names) != nc:
        die("%s 里有 %d 个类名, cfg 写的是 classes=%d" % (g.names, len(names), nc))

    cfgname = os.path.basename(g.cfg)
    body = emit_net_def(cfgname, layers, anchors, names, shp)
    hdr = patch_header(g.header, cfgname, net, nc, num, sxy, len(layers),
                       write=not g.check)

    print("cfg      : %s" % g.cfg)
    print("输入     : %sx%s  类别 %d  anchors %d  scale_x_y %g"
          % (net["width"], net["height"], nc, num, sxy))
    print("层数     : %d   两个 head: 层 %d(mask %s) / 层 %d(mask %s)"
          % (len(layers), ym[0]["idx"], ym[0]["mask"], ym[1]["idx"], ym[1]["mask"]))
    print("head 输出: %s  /  %s" % (shp[ym[0]["idx"]], shp[ym[1]["idx"]]))
    print("类名     : %s" % ", ".join(names))

    # 权重文件应该有多大 —— 拿来和 .weights 对一眼
    nw = 0
    for i, L in enumerate(layers):
        if L.kind != "conv":
            continue
        inc = int(net.get("channels", 3)) if i == 0 else shp[i - 1][0]
        nw += L.filters + (3 * L.filters if L.bn else 0)
        nw += L.filters * inc * L.ksize * L.ksize
    print("权重     : %d 个 float -> .weights 应为 %d 字节 (含 20 字节头)"
          % (nw, nw * 4 + 20))

    if g.check:
        old_h = open(g.header, encoding="utf-8").read()
        old_c = open(g.out, encoding="utf-8").read()
        bad = []
        if old_h != hdr:
            bad.append(os.path.relpath(g.header, ROOT))
        if old_c != body:
            bad.append(os.path.relpath(g.out, ROOT))
        if bad:
            print("\n✗ 和 cfg 对不上: %s (跑一遍不带 --check 就能修)" % ", ".join(bad))
            sys.exit(1)
        print("\n✓ 代码和 cfg 一致")
        return

    open(g.out, "w", encoding="utf-8").write(body)
    print("\n-> %s\n-> %s" % (g.header, g.out))
    print("接着: make clean && make GPU=1 ARCH=sm_53")


if __name__ == "__main__":
    main()
