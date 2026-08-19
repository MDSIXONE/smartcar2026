// 图像解码与缩放。缩放逐位复刻 darknet 的 resize_image()，否则检测框会有系统性偏移。
#include "yolo.h"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO          // 只从内存解码，不碰文件系统
#define STBI_NO_HDR
#define STBI_NO_PIC
#define STBI_NO_PNM
#include "stb_image.h"
#include <cstring>

namespace yolo {

static inline float getpx(const Image& im, int x, int y, int c) {
    return im.data[(size_t)c * im.h * im.w + (size_t)y * im.w + x];
}
static inline void setpx(Image& im, int x, int y, int c, float v) {
    im.data[(size_t)c * im.h * im.w + (size_t)y * im.w + x] = v;
}
static inline void addpx(Image& im, int x, int y, int c, float v) {
    im.data[(size_t)c * im.h * im.w + (size_t)y * im.w + x] += v;
}
static void alloc(Image& im, int w, int h, int c) {
    im.w = w; im.h = h; im.c = c;
    im.data.assign((size_t)w * h * c, 0.f);
}

bool decode_image(const uint8_t* buf, size_t len, Image& out, std::string* err) {
    int w = 0, h = 0, c = 0;
    unsigned char* px = stbi_load_from_memory(buf, (int)len, &w, &h, &c, 3);
    if (!px) {
        if (err) *err = std::string("图像解码失败: ") + (stbi_failure_reason() ? stbi_failure_reason() : "unknown");
        return false;
    }
    // stb 给的是交错 RGB8；darknet 内部是 CHW float [0,1]
    alloc(out, w, h, 3);
    for (int k = 0; k < 3; ++k)
        for (int j = 0; j < h; ++j)
            for (int i = 0; i < w; ++i)
                out.data[(size_t)k * h * w + (size_t)j * w + i] = px[(size_t)3 * (j * (size_t)w + i) + k] / 255.f;
    stbi_image_free(px);
    return true;
}

void image_from_raw(const uint8_t* buf, int w, int h, bool bgr, Image& out) {
    alloc(out, w, h, 3);
    const int r = bgr ? 2 : 0, b = bgr ? 0 : 2;
    const int order[3] = { r, 1, b };
    for (int k = 0; k < 3; ++k) {
        const int src_c = order[k];
        for (int j = 0; j < h; ++j)
            for (int i = 0; i < w; ++i)
                out.data[(size_t)k * h * w + (size_t)j * w + i] =
                    buf[(size_t)3 * (j * (size_t)w + i) + src_c] / 255.f;
    }
}

// darknet src/image.c resize_image()：先横向插值到 (w, im.h)，再纵向插值到 (w, h)。
// 注意坐标映射用的是 (src-1)/(dst-1)，也就是 align_corners=True，和 OpenCV 的 INTER_LINEAR 不同。
void resize_darknet(const Image& src, int w, int h, Image& dst) {
    if (src.w == w && src.h == h) { dst = src; return; }

    alloc(dst, w, h, src.c);
    Image part; alloc(part, w, src.h, src.c);

    const float w_scale = (w == 1) ? 0.f : (float)(src.w - 1) / (w - 1);
    const float h_scale = (h == 1) ? 0.f : (float)(src.h - 1) / (h - 1);

    for (int k = 0; k < src.c; ++k)
        for (int r = 0; r < src.h; ++r)
            for (int c = 0; c < w; ++c) {
                float val;
                if (c == w - 1 || src.w == 1) {
                    val = getpx(src, src.w - 1, r, k);
                } else {
                    float sx = c * w_scale;
                    int   ix = (int)sx;
                    float dx = sx - ix;
                    val = (1 - dx) * getpx(src, ix, r, k) + dx * getpx(src, ix + 1, r, k);
                }
                setpx(part, c, r, k, val);
            }

    for (int k = 0; k < src.c; ++k)
        for (int r = 0; r < h; ++r) {
            float sy = r * h_scale;
            int   iy = (int)sy;
            float dy = sy - iy;
            for (int c = 0; c < w; ++c) setpx(dst, c, r, k, (1 - dy) * getpx(part, c, iy, k));
            if (r == h - 1 || src.h == 1) continue;
            for (int c = 0; c < w; ++c) addpx(dst, c, r, k, dy * getpx(part, c, iy + 1, k));
        }
}

} // namespace yolo
