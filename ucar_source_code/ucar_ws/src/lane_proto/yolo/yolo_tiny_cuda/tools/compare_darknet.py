#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
和 darknet 逐层对拍。用来证明本实现的数值和 darknet 一致，而不是"看起来能跑"。

前置：darknet 需要打一个小补丁，让它把每层输出落盘。在 src/network.c 的
forward_network() 里加：

    // 函数开头
    const char *dk_dump = getenv("DK_DUMP");
    // l.forward(l, state); 之后
    if (dk_dump) {
        char _p[512]; sprintf(_p, "%s_%02d.bin", dk_dump, i);
        FILE *_f = fopen(_p, "wb");
        if (_f) { fwrite(l.output, sizeof(float), l.outputs, _f); fclose(_f); }
    }

然后 make 重编。本脚本会自动跑两边并比对。

用法:
    python3 tools/compare_darknet.py \
        --darknet ../darknet/darknet --data /tmp/t.data --cfg xxx.cfg \
        --weights xxx.weights --image a.jpg
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

try:
    import numpy as np
except ImportError:
    sys.exit('需要 numpy: pip3 install numpy')

# darknet 在 yolo 层内部就做了 sigmoid + scale_x_y，本实现放在后处理，
# 所以这两层不能直接逐元素比对（语义位置不同，结果等价）。
YOLO_LAYERS = {30, 37}


def run(cmd, env=None, quiet=True):
    e = dict(os.environ)
    if env:
        e.update(env)
    return subprocess.run(cmd, env=e,
                          stdout=subprocess.PIPE,
                          stderr=(subprocess.DEVNULL if quiet else None))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--darknet', required=True, help='打过 dump 补丁的 darknet 可执行文件')
    ap.add_argument('--exe', default=os.path.join(os.path.dirname(__file__), '..', 'yolo_tiny_cuda'))
    ap.add_argument('--data', required=True)
    ap.add_argument('--cfg', required=True)
    ap.add_argument('--weights', required=True)
    ap.add_argument('--image', required=True)
    ap.add_argument('--backend', default='cpu', choices=['cpu', 'cuda'])
    ap.add_argument('--thresh', type=float, default=0.25)
    ap.add_argument('--layers', type=int, default=38)
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix='yolocmp_')
    dk_pre, yt_pre = os.path.join(tmp, 'dk'), os.path.join(tmp, 'yt')

    print('[1/3] 跑 darknet ...')
    r = run([args.darknet, 'detector', 'test', args.data, args.cfg, args.weights,
             args.image, '-thresh', str(args.thresh), '-dont_show', '-ext_output'],
            env={'DK_DUMP': dk_pre}, quiet=False)
    dk_dets = [m.groups() for m in re.finditer(
        r'(\w+): (\d+)%\s+\(left_x:\s*(-?\d+)\s+top_y:\s*(-?\d+)\s+width:\s*(-?\d+)\s+height:\s*(-?\d+)\)',
        r.stdout.decode('utf-8', 'replace'))]

    print('[2/3] 跑本实现 ...')
    r2 = run([args.exe, '--weights', args.weights, '--backend', args.backend,
              '--oneshot', args.image], env={'YT_DUMP': yt_pre})
    import json
    yt = json.loads(r2.stdout.decode('utf-8').strip().splitlines()[-1])

    print('[3/3] 比对\n')
    print('%-5s %-11s %-14s %-12s' % ('层', '元素数', '最大绝对误差', '相对幅值'))
    print('-' * 48)
    worst, worst_l = 0.0, None
    for i in range(args.layers):
        fa, fb = '%s_%02d.bin' % (dk_pre, i), '%s_%02d.bin' % (yt_pre, i)
        if not (os.path.exists(fa) and os.path.exists(fb)):
            print('%-5d 缺文件（darknet 没打 dump 补丁？）' % i)
            continue
        a = np.fromfile(fa, dtype='<f4')
        b = np.fromfile(fb, dtype='<f4')
        if a.size != b.size:
            print('%-5d 元素数不同: %d vs %d  <== 结构对不上' % (i, a.size, b.size))
            continue
        if i in YOLO_LAYERS:
            print('%-5d %-11d yolo 层，语义位置不同，跳过' % (i, a.size))
            continue
        d = float(np.abs(a - b).max())
        rel = d / max(1e-6, float(np.abs(a).max()))
        print('%-5d %-11d %-14.3e %-12.3e' % (i, a.size, d, rel))
        if rel > worst:
            worst, worst_l = rel, i

    print('\n逐层最大相对误差: %.3e (层 %s)' % (worst, worst_l))
    print('darknet 检出 %d 个框，本实现 %d 个框' % (len(dk_dets), yt['n']))
    print('\n临时文件在 %s（比对完可删）' % tmp)
    print('判据：相对误差 < 1e-4 即认为一致（float32 累加顺序噪声）。' if worst < 1e-4
          else '⚠️ 误差偏大，检查结构或权重解析。')


if __name__ == '__main__':
    main()
