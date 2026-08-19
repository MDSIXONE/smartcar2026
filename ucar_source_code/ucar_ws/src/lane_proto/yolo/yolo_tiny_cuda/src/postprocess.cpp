// yolo 层解码 + NMS。语义严格对齐 AlexeyAB darknet：
//   * x,y 走 sigmoid 后再套 scale_x_y（src/yolo_layer.c forward_yolo_layer）
//   * w,h 走 exp * anchor / net_size（get_yolo_box）
//   * nms_kind=greedynms 用的是 **DIoU** 而不是普通 IoU（src/box.c diounms_sort）
//   * 输出 relative=1，即归一化坐标；因为预处理走 resize 而非 letterbox，
//     归一化坐标对原图和网络输入是同一个值，不需要 correct_yolo_boxes 的还原
#include "yolo.h"
#include <cmath>
#include <algorithm>

namespace yolo {

static inline float sigmoidf(float x) { return 1.f / (1.f + std::exp(-x)); }

static inline float overlap(float x1, float w1, float x2, float w2) {
    float l1 = x1 - w1 / 2, l2 = x2 - w2 / 2, left  = l1 > l2 ? l1 : l2;
    float r1 = x1 + w1 / 2, r2 = x2 + w2 / 2, right = r1 < r2 ? r1 : r2;
    return right - left;
}
static float box_iou(const Detection& a, const Detection& b) {
    float w = overlap(a.cx, a.bw, b.cx, b.bw);
    float h = overlap(a.cy, a.bh, b.cy, b.bh);
    if (w < 0 || h < 0) return 0;
    float inter = w * h;
    float uni = a.bw * a.bh + b.bw * b.bh - inter;
    return uni <= 0 ? 0 : inter / uni;
}
// darknet box_diou()：IoU 减去 (中心距² / 最小包围盒对角² ) 的 0.6 次幂
static float box_diou(const Detection& a, const Detection& b) {
    float left  = std::min(a.cx - a.bw / 2, b.cx - b.bw / 2);
    float right = std::max(a.cx + a.bw / 2, b.cx + b.bw / 2);
    float top   = std::min(a.cy - a.bh / 2, b.cy - b.bh / 2);
    float bot   = std::max(a.cy + a.bh / 2, b.cy + b.bh / 2);
    float w = right - left, h = bot - top;
    float c = w * w + h * h;
    float iou = box_iou(a, b);
    if (c == 0) return iou;
    float d = (a.cx - b.cx) * (a.cx - b.cx) + (a.cy - b.cy) * (a.cy - b.cy);
    return iou - std::pow(d / c, 0.6f);
}

void decode_and_nms(const Network& net,
                    const float* const yolo_out[2],
                    float conf_thresh, float nms_thresh,
                    std::vector<Detection>& out)
{
    out.clear();
    // 每个候选保留全部类别分数，NMS 是按类做的（和 darknet 一致）
    struct Cand { Detection d; float prob[NUM_CLASSES]; };
    std::vector<Cand> cands;

    for (int hi = 0; hi < 2; ++hi) {
        const int li = net.yolo_layers[hi];
        const LayerShape& s = net.shape[li];
        const int lw = s.out_w, lh = s.out_h;
        const int area = lw * lh;
        const int step = 4 + 1 + NUM_CLASSES;      // 7 类 -> 12
        const float* p = yolo_out[hi];

        for (int n = 0; n < ANCHORS_PER_HEAD; ++n) {
            const int anchor = LAYERS[li].mask[n];
            const float aw = ANCHORS[2 * anchor], ah = ANCHORS[2 * anchor + 1];
            const float* base = p + (size_t)n * step * area;

            for (int j = 0; j < lh; ++j)
                for (int i = 0; i < lw; ++i) {
                    const int loc = j * lw + i;
                    const float obj = sigmoidf(base[4 * area + loc]);
                    if (obj <= conf_thresh) continue;

                    Cand c{};
                    float x = sigmoidf(base[0 * area + loc]) * SCALE_XY - 0.5f * (SCALE_XY - 1);
                    float y = sigmoidf(base[1 * area + loc]) * SCALE_XY - 0.5f * (SCALE_XY - 1);
                    c.d.cx = (i + x) / lw;
                    c.d.cy = (j + y) / lh;
                    c.d.bw = std::exp(base[2 * area + loc]) * aw / NET_W;
                    c.d.bh = std::exp(base[3 * area + loc]) * ah / NET_H;

                    bool keep = false;
                    for (int k = 0; k < NUM_CLASSES; ++k) {
                        float pr = obj * sigmoidf(base[(5 + k) * area + loc]);
                        c.prob[k] = (pr > conf_thresh) ? pr : 0.f;
                        if (c.prob[k] > 0) keep = true;
                    }
                    if (keep) cands.push_back(c);
                }
        }
    }

    // 逐类 NMS
    for (int k = 0; k < NUM_CLASSES; ++k) {
        std::vector<int> idx;
        idx.reserve(cands.size());
        for (size_t i = 0; i < cands.size(); ++i) if (cands[i].prob[k] > 0) idx.push_back((int)i);
        std::sort(idx.begin(), idx.end(),
                  [&](int a, int b) { return cands[a].prob[k] > cands[b].prob[k]; });

        std::vector<char> dead(idx.size(), 0);
        for (size_t a = 0; a < idx.size(); ++a) {
            if (dead[a]) continue;
            for (size_t b = a + 1; b < idx.size(); ++b) {
                if (dead[b]) continue;
                if (box_diou(cands[idx[a]].d, cands[idx[b]].d) > nms_thresh) dead[b] = 1;
            }
            Detection d = cands[idx[a]].d;
            d.cls = k;
            d.conf = cands[idx[a]].prob[k];
            out.push_back(d);
        }
    }
    std::sort(out.begin(), out.end(),
              [](const Detection& a, const Detection& b) { return a.conf > b.conf; });
}

} // namespace yolo
