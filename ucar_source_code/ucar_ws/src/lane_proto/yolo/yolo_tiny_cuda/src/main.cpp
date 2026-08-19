// yolo_tiny_cuda —— stdin 收二进制帧，stdout 回 JSON 结果。
// 协议规范见 README.md。stderr 只用于日志，不参与协议。
#include "yolo.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#ifdef _WIN32
  #include <io.h>
  #include <fcntl.h>
#endif

using namespace yolo;

// ------------------------------------------------------------------ 帧
// 12 字节定长头：'Y','V' | type | flags | id(u32 LE) | length(u32 LE)
static const uint8_t MAGIC0 = 'Y', MAGIC1 = 'V';
enum FrameType : uint8_t {
    T_HELLO = 0x01, T_CONFIG = 0x02, T_READY = 0x03,
    T_IMAGE = 0x10, T_RESULT = 0x11, T_ERROR = 0x1F, T_BYE = 0x7F,
};
enum ImageEnc : uint8_t { ENC_AUTO = 0, ENC_RGB8 = 1, ENC_BGR8 = 2 };

struct Frame { uint8_t type, flags; uint32_t id; std::vector<uint8_t> payload; };

static bool read_exact(void* dst, size_t n) {
    uint8_t* p = (uint8_t*)dst;
    size_t got = 0;
    while (got < n) {
        size_t r = fread(p + got, 1, n - got, stdin);
        if (r == 0) return false;              // EOF 或出错
        got += r;
    }
    return true;
}
static bool read_frame(Frame& f) {
    uint8_t hdr[12];
    if (!read_exact(hdr, 12)) return false;
    if (hdr[0] != MAGIC0 || hdr[1] != MAGIC1) {
        fprintf(stderr, "[yolo] 帧头 magic 错误，协议已失步\n");
        return false;
    }
    f.type = hdr[2]; f.flags = hdr[3];
    memcpy(&f.id, hdr + 4, 4);
    uint32_t len; memcpy(&len, hdr + 8, 4);
    f.payload.resize(len);
    if (len && !read_exact(f.payload.data(), len)) return false;
    return true;
}
static void write_frame(uint8_t type, uint32_t id, const std::string& payload) {
    uint8_t hdr[12] = { MAGIC0, MAGIC1, type, 0 };
    uint32_t len = (uint32_t)payload.size();
    memcpy(hdr + 4, &id, 4);
    memcpy(hdr + 8, &len, 4);
    fwrite(hdr, 1, 12, stdout);
    if (len) fwrite(payload.data(), 1, len, stdout);
    fflush(stdout);
}

// ------------------------------------------------------------------ 迷你 JSON
static std::string jnum(double v, int prec = 6) {
    char b[64];
    snprintf(b, sizeof(b), "%.*f", prec, v);
    // 去掉尾随 0，让输出短一点
    std::string s(b);
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    return s.empty() ? "0" : s;
}
static std::string jstr(const std::string& s) {
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else if ((unsigned char)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
        else o += c;
    }
    return o + "\"";
}

// ------------------------------------------------------------------ 运行时配置
struct Config {
    float conf = 0.25f;
    float nms  = 0.45f;
    std::string box_format = "cxcywh_norm";   // 或 "xyxy_pixel"
    bool timings = true;
};

// 极简 JSON 取值：只支持顶层 "key": number / "key": "string" / true|false
static bool json_get_num(const std::string& s, const char* key, float& out) {
    std::string pat = std::string("\"") + key + "\"";
    size_t p = s.find(pat);
    if (p == std::string::npos) return false;
    p = s.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    out = (float)atof(s.c_str() + p + 1);
    return true;
}
static bool json_get_str(const std::string& s, const char* key, std::string& out) {
    std::string pat = std::string("\"") + key + "\"";
    size_t p = s.find(pat);
    if (p == std::string::npos) return false;
    p = s.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    size_t a = s.find('"', p);
    if (a == std::string::npos) return false;
    size_t b = s.find('"', a + 1);
    if (b == std::string::npos) return false;
    out = s.substr(a + 1, b - a - 1);
    return true;
}
static bool json_get_bool(const std::string& s, const char* key, bool& out) {
    std::string pat = std::string("\"") + key + "\"";
    size_t p = s.find(pat);
    if (p == std::string::npos) return false;
    p = s.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    while (p < s.size() && (s[p] == ':' || s[p] == ' ')) ++p;
    out = (s.compare(p, 4, "true") == 0);
    return true;
}

// ------------------------------------------------------------------ 主流程
static double now_ms() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

static std::string build_result(uint32_t id, const Image& orig,
                                const std::vector<Detection>& dets,
                                const Config& cfg,
                                double t_pre, double t_inf, double t_post)
{
    std::string j = "{\"id\":" + std::to_string(id)
                  + ",\"w\":" + std::to_string(orig.w)
                  + ",\"h\":" + std::to_string(orig.h)
                  + ",\"n\":" + std::to_string(dets.size())
                  + ",\"det\":[";
    for (size_t i = 0; i < dets.size(); ++i) {
        const Detection& d = dets[i];
        if (i) j += ",";
        j += "{\"cls\":" + std::to_string(d.cls)
           + ",\"name\":" + jstr(CLASS_NAMES[d.cls])
           + ",\"conf\":" + jnum(d.conf, 4)
           + ",\"box\":[";
        if (cfg.box_format == "xyxy_pixel") {
            float x1 = (d.cx - d.bw / 2) * orig.w, y1 = (d.cy - d.bh / 2) * orig.h;
            float x2 = (d.cx + d.bw / 2) * orig.w, y2 = (d.cy + d.bh / 2) * orig.h;
            j += jnum(x1, 2) + "," + jnum(y1, 2) + "," + jnum(x2, 2) + "," + jnum(y2, 2);
        } else {
            j += jnum(d.cx) + "," + jnum(d.cy) + "," + jnum(d.bw) + "," + jnum(d.bh);
        }
        j += "]}";
    }
    j += "]";
    if (cfg.timings)
        j += ",\"ms\":{\"pre\":" + jnum(t_pre, 2)
           + ",\"infer\":" + jnum(t_inf, 2)
           + ",\"post\":" + jnum(t_post, 2) + "}";
    return j + "}";
}

int main(int argc, char** argv) {
    const char* weights_path = nullptr;
    const char* want_backend = "auto";
    int device = 0;
    const char* dump_prefix = nullptr;   // 调试：把每层输出写成 .bin，用于和 darknet 对拍
    const char* oneshot = nullptr;       // 调试：直接跑一张本地图然后退出

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--weights") && i + 1 < argc)      weights_path = argv[++i];
        else if (!strcmp(argv[i], "--backend") && i + 1 < argc) want_backend = argv[++i];
        else if (!strcmp(argv[i], "--device") && i + 1 < argc)  device = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dump") && i + 1 < argc)    dump_prefix = argv[++i];
        else if (!strcmp(argv[i], "--oneshot") && i + 1 < argc) oneshot = argv[++i];
        else if (!strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "用法: %s --weights FILE [--backend cuda|cpu|auto] [--device N]\n"
                "调试: [--oneshot IMG] [--dump PREFIX]\n", argv[0]);
            return 0;
        }
    }
    if (!weights_path) { fprintf(stderr, "[yolo] 缺少 --weights\n"); return 2; }

#ifdef _WIN32
    _setmode(_fileno(stdin),  _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    static Network net;
    std::string err;
    if (!build_network(net, &err)) { fprintf(stderr, "[yolo] 构建网络失败: %s\n", err.c_str()); return 3; }
    if (!load_weights(net, weights_path, &err)) { fprintf(stderr, "[yolo] %s\n", err.c_str()); return 4; }

    Backend* be = nullptr;
#ifdef USE_CUDA
    if (strcmp(want_backend, "cpu") != 0) {
        be = make_cuda_backend(net, device, &err);
        if (!be && strcmp(want_backend, "cuda") == 0) {
            fprintf(stderr, "[yolo] CUDA 后端初始化失败: %s\n", err.c_str());
            return 5;
        }
        if (!be) fprintf(stderr, "[yolo] CUDA 不可用(%s)，回退 CPU\n", err.c_str());
    }
#else
    if (strcmp(want_backend, "cuda") == 0) {
        fprintf(stderr, "[yolo] 本二进制未编译 CUDA 支持（make GPU=1 重新编译）\n");
        return 5;
    }
#endif
    if (!be) be = make_cpu_backend(net);
    fprintf(stderr, "[yolo] 就绪，后端=%s，输入=%dx%d，类别数=%d\n",
            be->name(), NET_W, NET_H, NUM_CLASSES);

    Config cfg;
    Image orig, resized;
    std::vector<Detection> dets;
    const float* yout[2] = { nullptr, nullptr };

    // ---------------- 调试用一次性模式 ----------------
    if (oneshot) {
        FILE* f = fopen(oneshot, "rb");
        if (!f) { fprintf(stderr, "[yolo] 打不开 %s\n", oneshot); return 6; }
        fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
        std::vector<uint8_t> buf(n);
        if (fread(buf.data(), 1, n, f) != (size_t)n) { fclose(f); return 6; }
        fclose(f);
        if (!decode_image(buf.data(), buf.size(), orig, &err)) {
            fprintf(stderr, "[yolo] %s\n", err.c_str()); return 6;
        }
        resize_darknet(orig, NET_W, NET_H, resized);
        if (!be->forward(resized.data.data(), yout, &err)) {
            fprintf(stderr, "[yolo] 前向失败: %s\n", err.c_str()); return 7;
        }
        decode_and_nms(net, yout, cfg.conf, cfg.nms, dets);
        printf("%s\n", build_result(0, orig, dets, cfg, 0, 0, 0).c_str());
        if (dump_prefix) {
            for (int i = 0; i < 2; ++i) {
                char p[512];
                snprintf(p, sizeof(p), "%s_yolo%d.bin", dump_prefix, i);
                FILE* o = fopen(p, "wb");
                fwrite(yout[i], sizeof(float), net.shape[net.yolo_layers[i]].out_elems, o);
                fclose(o);
            }
        }
        delete be;
        return 0;
    }

    // ---------------- 握手 ----------------
    {
        std::string hello = "{\"proto\":\"yolo-pipe/1\",\"impl\":\"yolo_tiny_cuda\""
                            ",\"backend\":" + jstr(be->name()) +
                            ",\"input\":{\"w\":" + std::to_string(NET_W) +
                            ",\"h\":" + std::to_string(NET_H) + ",\"c\":3}"
                            ",\"encodings\":[\"auto\",\"rgb8\",\"bgr8\"]"
                            ",\"classes\":[";
        for (int i = 0; i < NUM_CLASSES; ++i) { if (i) hello += ","; hello += jstr(CLASS_NAMES[i]); }
        hello += "],\"defaults\":{\"conf\":" + jnum(cfg.conf) + ",\"nms\":" + jnum(cfg.nms) +
                 ",\"box_format\":\"cxcywh_norm\"}}";
        write_frame(T_HELLO, 0, hello);
    }

    // ---------------- 主循环 ----------------
    Frame f;
    while (read_frame(f)) {
        if (f.type == T_BYE) break;

        if (f.type == T_CONFIG) {
            std::string s((const char*)f.payload.data(), f.payload.size());
            json_get_num(s, "conf", cfg.conf);
            json_get_num(s, "nms", cfg.nms);
            json_get_str(s, "box_format", cfg.box_format);
            json_get_bool(s, "timings", cfg.timings);
            std::string ack = "{\"ok\":true,\"conf\":" + jnum(cfg.conf) +
                              ",\"nms\":" + jnum(cfg.nms) +
                              ",\"box_format\":" + jstr(cfg.box_format) + "}";
            write_frame(T_READY, f.id, ack);
            continue;
        }

        if (f.type != T_IMAGE) {
            write_frame(T_ERROR, f.id, "{\"error\":\"未知帧类型\"}");
            continue;
        }

        double t0 = now_ms();
        bool ok = true;
        if (f.flags == ENC_RGB8 || f.flags == ENC_BGR8) {
            if (f.payload.size() < 8) { ok = false; err = "raw 帧长度不足"; }
            else {
                uint32_t w, h;
                memcpy(&w, f.payload.data(), 4);
                memcpy(&h, f.payload.data() + 4, 4);
                if (f.payload.size() != 8 + (size_t)w * h * 3) { ok = false; err = "raw 帧尺寸与长度不符"; }
                else image_from_raw(f.payload.data() + 8, w, h, f.flags == ENC_BGR8, orig);
            }
        } else {
            ok = decode_image(f.payload.data(), f.payload.size(), orig, &err);
        }
        if (!ok) { write_frame(T_ERROR, f.id, "{\"error\":" + jstr(err) + "}"); continue; }

        resize_darknet(orig, NET_W, NET_H, resized);
        double t1 = now_ms();

        if (!be->forward(resized.data.data(), yout, &err)) {
            write_frame(T_ERROR, f.id, "{\"error\":" + jstr("前向失败: " + err) + "}");
            continue;
        }
        double t2 = now_ms();
        decode_and_nms(net, yout, cfg.conf, cfg.nms, dets);
        double t3 = now_ms();

        write_frame(T_RESULT, f.id, build_result(f.id, orig, dets, cfg, t1 - t0, t2 - t1, t3 - t2));
    }

    delete be;
    return 0;
}
