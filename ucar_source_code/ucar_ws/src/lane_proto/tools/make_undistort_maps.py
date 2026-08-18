#!/usr/bin/env python3
# 在 PC 上重新生成去畸变映射表(换标定后跑一次): fisheye_640.npz -> maps_640.npz
import sys
import numpy as np, cv2
src = sys.argv[1] if len(sys.argv) > 1 else "fisheye_640.npz"
z = np.load(src, allow_pickle=True)
K, D = z['K'].astype(np.float64), z['D'].astype(np.float64)
size = (int(z['W']), int(z['H']))
nK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K, D, size, np.eye(3), balance=0.0)
m1, m2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), nK, size, cv2.CV_16SC2)
np.savez_compressed("maps_640.npz", m1=m1, m2=m2, nK=nK,
                    W=size[0], H=size[1])
print("-> maps_640.npz (放进 lane_proto/config/)")
