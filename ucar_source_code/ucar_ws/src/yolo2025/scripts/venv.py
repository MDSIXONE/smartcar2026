#!/home/ucar/myenv/bin/python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int8
from cv_bridge import CvBridge
import cv2
import fcntl
import time
import os
from std_msgs.msg import String as ROSString

class QRCodeDetector:
    def __init__(self):
        rospy.init_node('qrcode')
        
        # 初始化变量
        self.qrcode_start_flag = 0  # 使用成员变量替代全局变量
        
        # 设置发布者和订阅者
        self.out = rospy.Publisher("/qr_result", ROSString, queue_size=10)
        rospy.Subscriber("/qrcode_start_flag", Int8, self.qrcode_start_callback)
        
        # 加载二维码检测模型
        model_dir = "/home/ucar/myenv/qr_file"
        detect_prototxt_path = os.path.join(model_dir, "detect.prototxt")
        detect_caffemodel_path = os.path.join(model_dir, "detect.caffemodel")
        sr_prototxt_path = os.path.join(model_dir, "sr.prototxt")
        sr_caffemodel_path = os.path.join(model_dir, "sr.caffemodel")
        self.detector = cv2.wechat_qrcode_WeChatQRCode(
            detect_prototxt_path, detect_caffemodel_path, 
            sr_prototxt_path, sr_caffemodel_path
        )
        
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
    def qrcode_start_callback(self,msg):
        self.qrcode_start_flag = msg.data
        rospy.loginfo(f"QR code detection status changed to: {self.qrcode_start_flag}")
    
    def image_callback(self, msg):
        if self.qrcode_start_flag == 1:  # 仅在标志为1时处理图像
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                res, _ = self.detector.detectAndDecode(cv_image)
                
                decoded_text = 'None' if len(res) == 0 else res[0]
                rospy.loginfo(f"Detected QR code: {decoded_text}")
                
                msg = ROSString()
                msg.data = decoded_text
                self.out.publish(msg)
                
            except Exception as e:
                rospy.logerr(f"Error in QR code detection: {str(e)}")
        # 如果标志为0，则不做任何处理

if __name__ == '__main__':
    try:
        qr_detector = QRCodeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass