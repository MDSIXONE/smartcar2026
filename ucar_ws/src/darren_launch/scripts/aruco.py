#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import time
import cv2
import cv2.aruco as aruco



# mtx = np.array([
#         [2946.48,       0, 1980.53],
#         [      0, 2945.41, 1129.25],
#         [      0,       0,       1],
#         ])
# #我的手机拍棋盘的时候图片大小是 4000 x 2250
# #ip摄像头拍视频的时候设置的是 1920 x 1080，长宽比是一样的，
# #ip摄像头设置分辨率的时候注意一下
#
#
# dist = np.array( [0.226317, -1.21478, 0.00170689, -0.000334551, 1.9892] )


#相机纠正参数

# dist=np.array(([[-0.51328742,  0.33232725 , 0.01683581 ,-0.00078608, -0.1159959]]))
#
# mtx=np.array([[464.73554153, 0.00000000e+00 ,323.989155],
#  [  0.,         476.72971528 ,210.92028],
#  [  0.,           0.,           1.        ]])
dist=np.array(([[-0.322818, 0.100218, -0.002753, -0.00122, 0]]))

mtx=np.array([[408.357552, 0, 324.777511],
 [  0, 411.483817, 259.122099],
 [  0.,           0.,           1.        ]])



cap = cv2.VideoCapture(0)


font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

#num = 0
while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame,1,dst=None)
    h1, w1 = frame.shape[:2]
    # 读取摄像头画面
    # 纠正畸变
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    frame=dst1


    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters_create()
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点 坐标
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

#    如果找不打id
    if ids is not None:

        rvec, tvec= aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # 估计每个标记的姿态并返回值rvet和tvec ---不同
        # from camera coeficcients
        (rvec-tvec).any() # get rid of that nasty numpy value array error

#        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #绘制轴
#        aruco.drawDetectedMarkers(frame, corners) #在标记周围画一个正方形
	print(ids)

        for i in range(rvec.shape[0]):
            aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            aruco.drawDetectedMarkers(frame, corners)
        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    # 显示结果框架
    cv2.imshow("frame",frame)

    key = cv2.waitKey(1)

    if key == 27:         # 按esc键退出
        print('esc break...')
        cap.release()
        cv2.destroyAllWindows()
        break

    if key == ord(' '):   # 按空格键保存
#        num = num + 1
#        filename = "frames_%s.jpg" % num  # 保存一张图像
        filename = str(time.time())[:10] + ".jpg"
        cv2.imwrite(filename, frame)
