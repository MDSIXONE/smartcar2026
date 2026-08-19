#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""负样本测试：喂纯室内背景（画面里没有任何红绿灯），统计误检率。
数据集里一张负样本都没有，这是训练时完全没覆盖的场景。"""
import argparse, os, queue, subprocess, sys, threading, time, collections
# detect.py 在上一级目录 —— tools/ 现在是 yolo_tiny_cuda 的子目录
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detect import Detector

def _load_names():
    """类名从 cfg/obj.names 读, 别再散着写死 —— 4 类改 7 类时漏改一处
    就会把 'yellow left' 当成越界报错或者显示成别的类。"""
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.join(os.path.dirname(here), 'cfg', 'obj.names')
    with open(p, encoding='utf-8') as f:
        return [l.strip() for l in f if l.strip() and not l.startswith('#')]


CLASSES = _load_names()

def worker(exe, w, conf, q, out, lock):
    with Detector(exe, w, 'cpu', 0, stderr=subprocess.DEVNULL) as det:
        det.configure(conf=conf, nms=0.45, timings=False)
        while True:
            try: img = q.get_nowait()
            except queue.Empty: return
            try: r = det.detect_file(img)
            except Exception as e: r = {'det': [], 'err': str(e)}
            with lock: out.append((img, r))

ap = argparse.ArgumentParser()
ap.add_argument('--exe', default='yolo_tiny_cuda/yolo_tiny_cuda')
ap.add_argument('--weights', default='real.weights')
ap.add_argument('--list', required=True)
ap.add_argument('--n', type=int, default=120)
ap.add_argument('--conf', type=float, default=0.25)
ap.add_argument('--jobs', type=int, default=2)
a = ap.parse_args()

imgs = [l.strip() for l in open(a.list) if l.strip()][:a.n]
q = queue.Queue()
for i in imgs: q.put(i)
out, lock = [], threading.Lock()
t0 = time.time()
ts = [threading.Thread(target=worker, args=(a.exe,a.weights,a.conf,q,out,lock)) for _ in range(a.jobs)]
for t in ts: t.start()
for t in ts: t.join()

fp_imgs = [(i,r) for i,r in out if r['det']]
nfp = sum(len(r['det']) for _,r in fp_imgs)
print('='*62)
print('负样本测试  n=%d  conf>=%.2f   （真值：应该 0 个框）' % (len(imgs), a.conf))
print('='*62)
print('干净(0 个框)   : %4d / %d = %6.2f%%' % (len(imgs)-len(fp_imgs), len(imgs), 100.0*(len(imgs)-len(fp_imgs))/len(imgs)))
print('出现误检的图   : %4d / %d = %6.2f%%' % (len(fp_imgs), len(imgs), 100.0*len(fp_imgs)/len(imgs)))
print('误检框总数     : %4d   (平均每张 %.3f)' % (nfp, nfp/len(imgs)))
if fp_imgs:
    cc = collections.Counter(d['name'] for _,r in fp_imgs for d in r['det'])
    print('误检类别分布   : %s' % dict(cc))
    cs = sorted((d['conf'] for _,r in fp_imgs for d in r['det']), reverse=True)
    print('误检置信度     : 最高 %.3f  中位 %.3f  最低 %.3f' % (cs[0], cs[len(cs)//2], cs[-1]))
    print('\n置信度阈值扫描（把阈值提高能挡掉多少误检）:')
    for th in (0.25,0.5,0.7,0.9,0.95,0.99):
        k = sum(1 for c in cs if c >= th)
        bad = len({i for i,r in fp_imgs if any(d['conf']>=th for d in r['det'])})
        print('   conf>=%.2f : 剩余误检框 %3d, 受影响图 %3d (%.1f%%)' % (th,k,bad,100.0*bad/len(imgs)))
    print('\n误检最严重的 10 张:')
    for i,r in sorted(fp_imgs, key=lambda x:-max(d['conf'] for d in x[1]['det']))[:10]:
        top = max(r['det'], key=lambda d:d['conf'])
        print('   %-22s %-9s conf=%.3f  框数=%d' % (os.path.basename(i), top['name'], top['conf'], len(r['det'])))
print('\n墙钟 %.1f s' % (time.time()-t0))
