// yolov4-tiny 推理引擎 —— 网络结构写死，权重从外部 .weights 文件读取。
// 与 AlexeyAB darknet 数值对齐；CPU 参考实现和 CUDA 实现共用同一套定义。
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>

namespace yolo {

// ------------------------------------------------------------------ 网络常量
// 这些值由 tools/gen_net_def.py 从 yolov4-tiny-tl7-640x352-w55.cfg 写入，**不要手改**。
constexpr int NET_W      = 640;
constexpr int NET_H      = 352;
constexpr int NET_C      = 3;
constexpr int NUM_CLASSES = 7;
constexpr int NUM_ANCHORS = 6;     // [yolo] num=6
constexpr int ANCHORS_PER_HEAD = 3;
constexpr float SCALE_XY = 1.05f;  // [yolo] scale_x_y
constexpr float BN_EPS   = 1e-5f;  // darknet: sqrt(var + 0.00001f)

// anchors=10,14, 23,27, 37,58, 81,82, 135,169, 344,319  （像素，相对网络输入尺寸）
extern const float ANCHORS[NUM_ANCHORS * 2];
extern const char* CLASS_NAMES[NUM_CLASSES];

// ------------------------------------------------------------------ 层定义
enum LayerType { L_CONV, L_MAXPOOL, L_ROUTE, L_UPSAMPLE, L_YOLO };
enum Activation { ACT_LEAKY, ACT_LINEAR };

struct LayerDef {
    LayerType type;
    // --- conv ---
    int filters;       // 输出通道
    int ksize;         // 卷积核
    int stride;
    int pad;           // cfg 的 pad=1 表示 same padding，实际 padding = ksize/2
    int batch_norm;
    Activation act;
    // --- route ---
    int route_from[2]; // 绝对层号（已把 cfg 里的负数偏移解算成绝对值），-1 表示无
    int route_groups;  // groups
    int route_group_id;// group_id
    // --- maxpool / upsample ---
    int pool_size;
    int pool_stride;
    int up_stride;
    // --- yolo ---
    int mask[ANCHORS_PER_HEAD];
};

// 每层的输入/输出张量形状（构建时算出）
struct LayerShape {
    int in_c, in_h, in_w;
    int out_c, out_h, out_w;
    size_t out_elems;
};

constexpr int NUM_LAYERS = 38;
extern const LayerDef LAYERS[NUM_LAYERS];

// ------------------------------------------------------------------ 权重
// 每个卷积层的权重。BN 已在加载时折叠进 weight/bias，推理时不再单独算 BN。
struct ConvWeights {
    std::vector<float> weights;  // [filters][in_c][ky][kx]
    std::vector<float> biases;   // [filters]
};

struct Network {
    LayerShape shape[NUM_LAYERS];
    ConvWeights conv[NUM_LAYERS];   // 只有 L_CONV 的项有效
    size_t max_workspace;           // 最大单层输出元素数
    int yolo_layers[2];             // 两个 [yolo] 层的层号
};

// 构建形状表（不读权重）。失败返回 false。
bool build_network(Network& net, std::string* err);

// 从 darknet .weights 读取并折叠 BN。
bool load_weights(Network& net, const char* path, std::string* err);

// ------------------------------------------------------------------ 图像
struct Image {              // CHW, float, [0,1], RGB —— 与 darknet 内存布局一致
    int w = 0, h = 0, c = 0;
    std::vector<float> data;
};

// 解码 JPEG/PNG/BMP 等（走 stb_image）。失败返回 false。
bool decode_image(const uint8_t* buf, size_t len, Image& out, std::string* err);
// 从已解码的交错 8 位数据构造（RGB8 / BGR8）。
void image_from_raw(const uint8_t* buf, int w, int h, bool bgr, Image& out);
// darknet resize_image 的逐位复刻（align_corners 双线性，先横后纵两趟）。
void resize_darknet(const Image& src, int w, int h, Image& dst);

// ------------------------------------------------------------------ 检测结果
struct Detection {
    float cx, cy, bw, bh;   // 归一化到 [0,1]，相对**原图**
    float conf;             // objectness * class_prob
    int   cls;
};

// 从两个 yolo 层的原始输出解码 + NMS。
// yolo_out[i] 指向第 i 个 yolo 层前面那个 conv 的输出（已 sigmoid 前的原始值）。
void decode_and_nms(const Network& net,
                    const float* const yolo_out[2],
                    float conf_thresh, float nms_thresh,
                    std::vector<Detection>& out);

// ------------------------------------------------------------------ 推理后端
class Backend {
public:
    virtual ~Backend() {}
    // 输入 letterbox/resize 后的 CHW float 图（NET_C*NET_H*NET_W）
    // 输出两个 yolo 层前置 conv 的结果指针（生命周期归 Backend）
    virtual bool forward(const float* input, const float* out[2], std::string* err) = 0;
    virtual const char* name() const = 0;
};

Backend* make_cpu_backend(const Network& net);
#ifdef USE_CUDA
Backend* make_cuda_backend(const Network& net, int device, std::string* err);
#endif

} // namespace yolo
