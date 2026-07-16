#!/home/ucar/myenv/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
detector = cv2.QRCodeDetector()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 提高检测率的预处理
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # 检测二维码
        data, vertices, _ = detector.detectAndDecode(binary)  # 使用二值化图像
        
        if vertices is not None:
            rospy.loginfo(f"检测到二维码: {data}")
            # 打印二维码顶点坐标（调试用）
            print("二维码位置:", vertices.astype(int))
        else:
            rospy.logdebug("未检测到二维码")
            
    except CvBridgeError as e:
        rospy.logerr(f"图像转换错误: {e}")
    except Exception as e:
        rospy.logerr(f"处理错误: {e}")

if __name__ == "__main__":
    rospy.init_node('qr_detect', log_level=rospy.INFO)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    rospy.spin()