#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用纯 C/C++ 的 CPU 后端跑整套数据集，统计成功率。

判定口径（每张图恰好 1 个真值框）：
  命中  = 最高分检测框的类别正确 且 与真值 IoU >= 0.5
  错类  = 检出了但类别错
  漏检  = 一个框都没出
  多检  = 除最高分框外还有其他框（潜在误检）
"""
import argparse, json, os, subprocess, sys, threading, queue, time

# detect.py 在上一级目录 —— tools/ 现在是 yolo_tiny_cuda 的子目录
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detect import Detector                                    # noqa: E402

def _load_names():
    """类名从 cfg/obj.names 读, 别再散着写死 —— 4 类改 7 类时漏改一处
    就会把 'yellow left' 当成越界报错或者显示成别的类。"""
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.join(os.path.dirname(here), 'cfg', 'obj.names')
    with open(p, encoding='utf-8') as f:
        return [l.strip() for l in f if l.strip() and not l.startswith('#')]


CLASSES = _load_names()


def iou(a, b):
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter = (ix1 - ix0) * (iy1 - iy0)
    return inter / (a[2] * a[3] + b[2] * b[3] - inter)


def load_gt(img):
    with open(img[:-4] + '.txt') as f:
        p = f.readline().split()
    return int(p[0]), [float(x) for x in p[1:]]


def worker(exe, weights, conf, nms, q, out, lock, env):
    with Detector(exe, weights, 'cpu', 0, stderr=subprocess.DEVNULL) as det:
        det.configure(conf=conf, nms=nms, timings=True)
        while True:
            try:
                img = q.get_nowait()
            except queue.Empty:
                return
            try:
                r = det.detect_file(img)
            except Exception as e:
                with lock:
                    out.append((img, None, str(e)))
                continue
            with lock:
                out.append((img, r, None))
                n = len(out)
                if n % 10 == 0:
                    print('  ... %d 张' % n, file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exe', default='yolo_tiny_cuda/yolo_tiny_cuda')
    ap.add_argument('--weights', default='real.weights')
    ap.add_argument('--images', nargs='+', required=True)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--nms', type=float, default=0.45)
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--tag', default='')
    ap.add_argument('--save', default='')
    args = ap.parse_args()

    q = queue.Queue()
    for i in args.images:
        q.put(i)
    out, lock = [], threading.Lock()
    env = dict(os.environ, OMP_NUM_THREADS='1')

    t0 = time.time()
    ths = [threading.Thread(target=worker,
                            args=(args.exe, args.weights, args.conf, args.nms, q, out, lock, env))
           for _ in range(args.jobs)]
    for t in ths: t.start()
    for t in ths: t.join()
    dt = time.time() - t0

    hit = wrong = miss = 0
    extra_total = 0
    ious, confs, times = [], [], []
    per_class = {c: {'n': 0, 'hit': 0, 'wrong': 0, 'miss': 0} for c in CLASSES}
    confusion = {c: {d: 0 for d in CLASSES} for c in CLASSES}
    failures = []

    for img, r, err in sorted(out):
        gt_cls, gt_box = load_gt(img)
        gname = CLASSES[gt_cls]
        per_class[gname]['n'] += 1
        if err or r is None:
            miss += 1; per_class[gname]['miss'] += 1
            failures.append((os.path.basename(img), 'ERROR ' + str(err)))
            continue
        times.append(r['ms']['infer'])
        dets = r['det']
        if not dets:
            miss += 1; per_class[gname]['miss'] += 1
            failures.append((os.path.basename(img), '漏检'))
            continue
        best = dets[0]                      # 引擎已按 conf 降序
        extra_total += len(dets) - 1
        v = iou(best['box'], gt_box)
        confusion[gname][best['name']] += 1
        if best['cls'] == gt_cls and v >= 0.5:
            hit += 1; per_class[gname]['hit'] += 1; ious.append(v); confs.append(best['conf'])
        elif best['cls'] != gt_cls:
            wrong += 1; per_class[gname]['wrong'] += 1
            failures.append((os.path.basename(img),
                             '错类 真值=%s 预测=%s conf=%.3f' % (gname, best['name'], best['conf'])))
        else:
            wrong += 1; per_class[gname]['wrong'] += 1
            failures.append((os.path.basename(img), 'IoU 不足 %.3f' % v))

    n = len(args.images)
    bar = '=' * 62
    print('\n' + bar)
    print('%s  n=%d  conf>=%.2f  nms=%.2f' % (args.tag or '评测', n, args.conf, args.nms))
    print(bar)
    print('命中(类别对 且 IoU>=0.5) : %4d / %d   = %6.2f%%' % (hit, n, 100.0 * hit / n))
    print('错类 / IoU 不足           : %4d' % wrong)
    print('漏检(一个框都没有)        : %4d' % miss)
    print('多余框总数(潜在误检)      : %4d  (平均每张 %.3f)' % (extra_total, extra_total / n))
    if ious:
        ious.sort()
        print('命中样本 IoU : 均值 %.4f  中位 %.4f  最小 %.4f' %
              (sum(ious) / len(ious), ious[len(ious) // 2], ious[0]))
        confs.sort()
        print('命中样本 conf: 均值 %.4f  中位 %.4f  最小 %.4f' %
              (sum(confs) / len(confs), confs[len(confs) // 2], confs[0]))
    if times:
        times.sort()
        print('CPU 单张推理 : 中位 %.0f ms  (%d 线程 x %d 进程, 墙钟总计 %.1f s)' %
              (times[len(times) // 2], 1, args.jobs, dt))
    print('\n分类别:')
    print('  %-9s %4s %5s %5s %5s  %s' % ('类别', 'n', '命中', '错类', '漏检', '命中率'))
    for c in CLASSES:
        d = per_class[c]
        if not d['n']: continue
        print('  %-9s %4d %5d %5d %5d  %6.2f%%' %
              (c, d['n'], d['hit'], d['wrong'], d['miss'], 100.0 * d['hit'] / d['n']))
    print('\n混淆矩阵 (行=真值, 列=预测最高分框):')
    print('  %-10s' % '' + ''.join('%-10s' % c for c in CLASSES))
    for c in CLASSES:
        print('  %-10s' % c + ''.join('%-10d' % confusion[c][d] for d in CLASSES))
    if failures:
        print('\n未命中明细 (%d 条):' % len(failures))
        for f, why in failures[:25]:
            print('  %-34s %s' % (f, why))
    if args.save:
        json.dump({'n': n, 'hit': hit, 'wrong': wrong, 'miss': miss,
                   'per_class': per_class, 'confusion': confusion,
                   'failures': failures},
                  open(args.save, 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
