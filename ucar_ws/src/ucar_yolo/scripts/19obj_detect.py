#!/usr/bin/python3

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
import argparse
import os
import glob
import random
import darknet
import time
import cv2
import numpy as np
import darknet

def white_balance(img):
    '''
    第一种简单的求均值白平衡法
    :param img: cv2.imread读取的图片数据
    :return: 返回的白平衡结果图片数据
    '''
    # 读取图像
    r, g, b = cv2.split(img)
    r_avg = cv2.mean(r)[0]
    g_avg = cv2.mean(g)[0]
    b_avg = cv2.mean(b)[0]
    # 求各个通道所占增益
    k = (r_avg + g_avg + b_avg) / 3
    kr = k / r_avg
    kg = k / g_avg
    kb = k / b_avg
    r = cv2.addWeighted(src1=r, alpha=kr, src2=0, beta=0, gamma=0)
    g = cv2.addWeighted(src1=g, alpha=kg, src2=0, beta=0, gamma=0)
    b = cv2.addWeighted(src1=b, alpha=kb, src2=0, beta=0, gamma=0)
    balance_img = cv2.merge([b, g, r])
    return balance_img

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default="/home/ucar/ucar_ws/src/image/",
                        help="image source. It can be a single image, a"
                        "txt with paths to them, or a folder. Image valid"
                        " formats are jpg, jpeg or png."
                        "If no input is given, ")
    parser.add_argument("--batch_size", default=1, type=int,
                        help="number of images to be processed at the same time")
    parser.add_argument("--weights", default="src/ucar_yolo/scripts/5.weights ",
                        help="yolo weights path")
    parser.add_argument("--dont_show", action='store_true',
                        help="windown inference display. For headless systems")
    parser.add_argument("--ext_output", action='store_true',
                        help="display bbox coordinates of detected objects")
    parser.add_argument("--save_labels", action='store_true',
                        help="save detections bbox for each image in yolo format")
    parser.add_argument("--config_file", default="./src/ucar_yolo/scripts/cfg/v4-tiny-19race.cfg ",
                        help="path to config file")
    parser.add_argument("--data_file", default="./src/ucar_yolo/scripts/cfg/19xunfei.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.45,
                        help="remove detections with lower confidence")
    return parser.parse_args()



def load_images(images_path):
    """
    If image path is given, return it directly
    For txt file, read it and return each line as image path
    In other case, it's a folder, return a list with names of each
    jpg, jpeg and png file
    """
    input_path_extension = images_path.split('.')[-1]
    if input_path_extension in ['jpg', 'jpeg', 'png']:
        return [images_path]
    elif input_path_extension == "txt":
        with open(images_path, "r") as f:
            return f.read().splitlines()
    else:
        return glob.glob(
            os.path.join(images_path, "*.jpg")) + \
            glob.glob(os.path.join(images_path, "*.png")) + \
            glob.glob(os.path.join(images_path, "*.jpeg"))


def image_detection(image_path, network, class_names, class_colors, thresh):
    # Darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    darknet_image = darknet.make_image(width, height, 3)

    image = cv2.imread(image_path)
    # image = white_balance(image)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height),
                               interpolation=cv2.INTER_LINEAR)

    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    darknet.free_image(darknet_image)
    image = darknet.draw_boxes(detections, image_resized, class_colors)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB), detections




yolo_obj_start_flag = 0

def main():
    # args = parser()
    rospy.loginfo("yolo object detect GO GO GO")

    random.seed(3)  # deterministic bbox colors
    net.load_network(
        "/home/ucar/ucar_ws/src/ucar_yolo/scripts/cfg/v4-tiny-19race.cfg",
        "/home/ucar/ucar_ws/src/ucar_yolo/scripts/cfg/19xunfei.data",
        "/home/ucar/ucar_ws/src/ucar_yolo/scripts/5.weights",
        batch_size=1
    )

    # images = load_images(args.input)
    images = load_images("/home/ucar/ucar_ws/src/image_obj/")
    index = 0
    while True:
        if yolo_obj_start_flag == 1:
            break
    while True:
        # loop asking for new image paths if no list is given
        # if args.input:
        if "/home/ucar/ucar_ws/src/image_obj/":
            if index >= len(images):
                break
            image_name = images[index]
        else:
            rospy.loginfo("------------------------- not find image path -------------------------")
        image, detections = image_detection(
            image_name, network, class_names, class_colors, 0.9
            )
        # image_name, network, class_names, class_colors, args.thresh
        save_annotations(image_name, image, detections, class_names)
        # darknet.print_detections(detections, args.ext_output)
        darknet.print_detections(detections)
        # cv2.imwrite("/home/ucar/ucar_ws/src/image/" + image_name + ".jpg", image)
        index += 1

def callback_yolo_obj_start(msg):
    global yolo_obj_start_flag
    yolo_obj_start_flag = msg.data

if __name__ == "__main__":
    try:
        # 初始化ros节点
        rospy.init_node("yolo_obj")
        rospy.loginfo("Starting yolo node obj_detect")
        yolo_obj_start_sub = rospy.Subscriber("/yolo_obj_start_flag", Int8 , callback_yolo_obj_start)
        yolo_obj_label_pub = rospy.Publisher("/detected_objects", String, queue_size=10)
        yolo_obj_over_pub = rospy.Publisher("/yolo_obj_over_flag", Int8, queue_size=1)

        main()
        yolo_obj_over_pub.publish(1)
        rospy.loginfo("yolo detect over")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down yolo node.")
