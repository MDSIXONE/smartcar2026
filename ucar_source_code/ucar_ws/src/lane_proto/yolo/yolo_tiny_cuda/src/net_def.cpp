// ⚠ 本文件由 tools/gen_net_def.py 从 yolov4-tiny-tl7-640x352-w55.cfg 生成, **不要手改**。
//   换 cfg: python3 tools/gen_net_def.py <新cfg> --names <obj.names>

#include "yolo.h"
#include <cstdio>

namespace yolo {

const float ANCHORS[NUM_ANCHORS * 2] = {
    10, 14,  23, 27,  37, 58,  81, 82,  135, 169,  344, 319
};

const char* CLASS_NAMES[NUM_CLASSES] = { "left", "right", "stop", "straight", "yellow left", "yellow right", "yellow straight" };


// 简写宏，纯粹为了让下面这张表能一眼看出对应 cfg 的哪一行
#define CONV(f, k, s, bn, a) { L_CONV, f, k, s, 1, bn, a, {-1,-1}, 1, 0, 0, 0, 0, {0,0,0} }
#define CONVNP(f, k, s, bn, a) { L_CONV, f, k, s, 0, bn, a, {-1,-1}, 1, 0, 0, 0, 0, {0,0,0} }
#define ROUTE1(a)            { L_ROUTE, 0,0,0,0,0, ACT_LINEAR, {a,-1}, 1, 0, 0, 0, 0, {0,0,0} }
#define ROUTE2(a, b)         { L_ROUTE, 0,0,0,0,0, ACT_LINEAR, {a, b}, 1, 0, 0, 0, 0, {0,0,0} }
#define ROUTEG(a, g, gid)    { L_ROUTE, 0,0,0,0,0, ACT_LINEAR, {a,-1}, g, gid, 0, 0, 0, {0,0,0} }
#define MAXPOOL(k, s)        { L_MAXPOOL, 0,0,0,0,0, ACT_LINEAR, {-1,-1}, 1, 0, k, s, 0, {0,0,0} }
#define UPSAMPLE(s)          { L_UPSAMPLE, 0,0,0,0,0, ACT_LINEAR, {-1,-1}, 1, 0, 0, 0, s, {0,0,0} }
#define YOLO(m0, m1, m2)     { L_YOLO, 0,0,0,0,0, ACT_LINEAR, {-1,-1}, 1, 0, 0, 0, 0, {m0,m1,m2} }

const LayerDef LAYERS[NUM_LAYERS] = {
    /* 0*/ CONV(16, 3, 2, 1, ACT_LEAKY),          // -> 16x176x320
    /* 1*/ CONV(32, 3, 2, 1, ACT_LEAKY),          // -> 32x88x160
    /* 2*/ CONV(32, 3, 1, 1, ACT_LEAKY),          // -> 32x88x160
    /* 3*/ ROUTEG(2, 2, 1),                       // -> 16x88x160
    /* 4*/ CONV(16, 3, 1, 1, ACT_LEAKY),          // -> 16x88x160
    /* 5*/ CONV(16, 3, 1, 1, ACT_LEAKY),          // -> 16x88x160
    /* 6*/ ROUTE2(5, 4),                          // -> 32x88x160
    /* 7*/ CONV(32, 1, 1, 1, ACT_LEAKY),          // -> 32x88x160
    /* 8*/ ROUTE2(2, 7),                          // -> 64x88x160
    /* 9*/ MAXPOOL(2, 2),                         // -> 64x44x80
    /*10*/ CONV(72, 3, 1, 1, ACT_LEAKY),          // -> 72x44x80
    /*11*/ ROUTEG(10, 2, 1),                      // -> 36x44x80
    /*12*/ CONV(32, 3, 1, 1, ACT_LEAKY),          // -> 32x44x80
    /*13*/ CONV(32, 3, 1, 1, ACT_LEAKY),          // -> 32x44x80
    /*14*/ ROUTE2(13, 12),                        // -> 64x44x80
    /*15*/ CONV(72, 1, 1, 1, ACT_LEAKY),          // -> 72x44x80
    /*16*/ ROUTE2(10, 15),                        // -> 144x44x80
    /*17*/ MAXPOOL(2, 2),                         // -> 144x22x40
    /*18*/ CONV(144, 3, 1, 1, ACT_LEAKY),         // -> 144x22x40
    /*19*/ ROUTEG(18, 2, 1),                      // -> 72x22x40
    /*20*/ CONV(72, 3, 1, 1, ACT_LEAKY),          // -> 72x22x40
    /*21*/ CONV(72, 3, 1, 1, ACT_LEAKY),          // -> 72x22x40
    /*22*/ ROUTE2(21, 20),                        // -> 144x22x40
    /*23*/ CONV(144, 1, 1, 1, ACT_LEAKY),         // -> 144x22x40
    /*24*/ ROUTE2(18, 23),                        // -> 288x22x40
    /*25*/ MAXPOOL(2, 2),                         // -> 288x11x20
    /*26*/ CONV(280, 3, 1, 1, ACT_LEAKY),         // -> 280x11x20
    /*27*/ CONV(144, 1, 1, 1, ACT_LEAKY),         // -> 144x11x20
    /*28*/ CONV(280, 3, 1, 1, ACT_LEAKY),         // -> 280x11x20
    /*29*/ CONV(36, 1, 1, 0, ACT_LINEAR),         // -> 36x11x20
    /*30*/ YOLO(3, 4, 5),                         // -> 36x11x20
    /*31*/ ROUTE1(27),                            // -> 144x11x20
    /*32*/ CONV(72, 1, 1, 1, ACT_LEAKY),          // -> 72x11x20
    /*33*/ UPSAMPLE(2),                           // -> 72x22x40
    /*34*/ ROUTE2(33, 23),                        // -> 216x22x40
    /*35*/ CONV(144, 3, 1, 1, ACT_LEAKY),         // -> 144x22x40
    /*36*/ CONV(36, 1, 1, 0, ACT_LINEAR),         // -> 36x22x40
    /*37*/ YOLO(1, 2, 3),                         // -> 36x22x40
};


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
