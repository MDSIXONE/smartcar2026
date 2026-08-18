/* trackseg.h — 固化 small(TrackSegNet) 分割网的 C ABI (ctypes 直接调用)
 * 输入: 192x320x3 uint8 BGR (OpenCV 原生排布)
 * 输出: mask 192*320 uint8 (0其他 1场地 2白线 3红绿灯)
 *       logits 4*192*320 float (CHW)
 * 返回 0 = 成功; 非 0 时用 ts_error() 取错误串 */
#ifndef TRACKSEG_H
#define TRACKSEG_H

#ifdef __cplusplus
extern "C" {
#endif

int ts_init(void);
int ts_infer(const unsigned char *bgr, unsigned char *mask);
int ts_infer_logits(const unsigned char *bgr, float *logits);
const char *ts_error(void);
const char *ts_backend(void);
void ts_destroy(void);

#ifdef __cplusplus
}
#endif
#endif
