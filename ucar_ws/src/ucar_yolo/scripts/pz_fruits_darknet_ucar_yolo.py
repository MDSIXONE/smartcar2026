#!/usr/bin/python3

import rospy
from std_msgs.msg import Int8

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
    parser.add_argument("--weights", default="src/ucar_yolo/scripts/4.weights ",
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


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if args.input and not os.path.exists(args.input):
        raise(ValueError("Invalid image path {}".format(os.path.abspath(args.input))))


def check_batch_shape(images, batch_size):
    """
        Image sizes should be the same width and height
    """
    shapes = [image.shape for image in images]
    if len(set(shapes)) > 1:
        raise ValueError("Images don't have same shape")
    if len(shapes) > batch_size:
        raise ValueError("Batch size higher than number of images")
    return shapes[0]


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


def prepare_batch(images, network, channels=3):
    width = darknet.network_width(network)
    height = darknet.network_height(network)

    darknet_images = []
    for image in images:

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)
        custom_image = image_resized.transpose(2, 0, 1)
        darknet_images.append(custom_image)

    batch_array = np.concatenate(darknet_images, axis=0)
    batch_array = np.ascontiguousarray(batch_array.flat, dtype=np.float32)/255.0
    darknet_images = batch_array.ctypes.data_as(darknet.POINTER(darknet.c_float))
    return darknet.IMAGE(width, height, channels, darknet_images)


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


def batch_detection(network, images, class_names, class_colors,
                    thresh=0.25, hier_thresh=.5, nms=.45, batch_size=4):
    image_height, image_width, _ = check_batch_shape(images, batch_size)
    darknet_images = prepare_batch(images, network)
    batch_detections = darknet.network_predict_batch(network, darknet_images, batch_size, image_width,
                                                     image_height, thresh, hier_thresh, None, 0, 0)
    batch_predictions = []
    for idx in range(batch_size):
        num = batch_detections[idx].num
        detections = batch_detections[idx].dets
        if nms:
            darknet.do_nms_obj(detections, num, len(class_names), nms)
        predictions = darknet.remove_negatives(detections, class_names, num)
        images[idx] = darknet.draw_boxes(predictions, images[idx], class_colors)
        batch_predictions.append(predictions)
    darknet.free_batch_detections(batch_detections, batch_size)
    return images, batch_predictions


def image_classification(image, network, class_names):
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height),
                                interpolation=cv2.INTER_LINEAR)
    darknet_image = darknet.make_image(width, height, 3)
    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.predict_image(network, darknet_image)
    predictions = [(name, detections[idx]) for idx, name in enumerate(class_names)]
    darknet.free_image(darknet_image)
    return sorted(predictions, key=lambda x: -x[1])


def convert2relative(image, bbox):
    """
    YOLO format use relative coordinates for annotation
    """
    x, y, w, h = bbox
    height, width, _ = image.shape
    return x/width, y/height, w/width, h/height


def save_annotations(name, image, detections, class_names):
    """
    Files saved with image_name.txt and relative coordinates
    """
    file_name = os.path.splitext(name)[0] + ".txt"
    # rospy.loginfo(file_name)
    with open(file_name, "w") as f:
        for label, confidence, bbox in detections:
            x, y, w, h = convert2relative(image, bbox)
            # label = class_names.index(label)
            f.write("{} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}\n".format(label, x, y, w, h, float(confidence)))

def clear_result():
    file_name = "/home/ucar/ucar_ws/src/image/result.txt"
    # rospy.loginfo(file_name)
    with open(file_name, "w") as f:
            f.write("")

def save_result():
    
    """
    Files saved with image_name.txt and relative coordinates
    """
    file_name = "/home/ucar/ucar_ws/src/image/result.txt"
    # with open(file_name, "w") as f:
    #     f.write("")
    # result_list1 = []
    # result_list2 = []
    # result_list3 = []
    # result_list4 = []
    result_list5 = []

    # with open('/home/ucar/ucar_ws/src/image_vegetation/1_0.txt','r') as f:
    #     for line in f.readlines():
    #         result_list1.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/1_1.txt','r') as f:
    #     for line in f.readlines():
    #         result_list1.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/1_2.txt','r') as f:
    #     for line in f.readlines():
    #         result_list1.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image/1_3.txt','r') as f:
    #     for line in f.readlines():
    #         result_list1.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image/1_4.txt','r') as f:
    #     for line in f.readlines():
    #         result_list1.append(line.split()[0])
    # while len(result_list1) < 3:
    #     result_list1.append("nan")
    
    # result_list1.append('\n')

    # with open('/home/ucar/ucar_ws/src/image_vegetation/2_0.txt','r') as f:
    #     for line in f.readlines():
    #         result_list2.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/2_1.txt','r') as f:
    #     for line in f.readlines():
    #         result_list2.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/2_2.txt','r') as f:
    #     for line in f.readlines():
    #         result_list2.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/2_3.txt','r') as f:
    #     for line in f.readlines():
    #         result_list2.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image/2_4.txt','r') as f:
    #     for line in f.readlines():
    #         result_list2.append(line.split()[0])
    # while len(result_list2) < 3:
    #     result_list2.append("nan")

    # result_list2.append('\n')

    # with open('/home/ucar/ucar_ws/src/image_vegetation/3_0.txt','r') as f:
    #     for line in f.readlines():
    #         result_list3.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/3_1.txt','r') as f:
    #     for line in f.readlines():
    #         result_list3.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/3_2.txt','r') as f:
    #     for line in f.readlines():
    #         result_list3.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image/3_3.txt','r') as f:
    #     for line in f.readlines():
    #         result_list3.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image/3_4.txt','r') as f:
    #     for line in f.readlines():
    #         result_list3.append(line.split()[0])
    # while len(result_list3) < 3:
    #     result_list3.append("nan")

    # result_list3.append('\n')

    # with open('/home/ucar/ucar_ws/src/image_vegetation/4_0.txt','r') as f:
    #     for line in f.readlines():
    #         result_list4.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/4_1.txt','r') as f:
    #     for line in f.readlines():
    #         result_list4.append(line.split()[0])
    # with open('/home/ucar/ucar_ws/src/image_vegetation/4_2.txt','r') as f:
    #     for line in f.readlines():
    #         result_list4.append(line.split()[0])
    # while len(result_list1) < 1:
    #     result_list4.append("nan")
    
    # result_list4.append('\n')
    boardsNum = 0
    with open('/home/ucar/ucar_ws/src/image_fruits/5_0.txt','r') as f:
        for line in f.readlines():
            result_list5.append(line.split()[0])
            boardsNum += 1
    if(boardsNum < 3):
        with open('/home/ucar/ucar_ws/src/image_fruits/5_3.txt','r') as f:
            for line in f.readlines():
                result_list5.append(line.split()[0])
        with open('/home/ucar/ucar_ws/src/image_fruits/5_4.txt','r') as f:
            for line in f.readlines():
                result_list5.append(line.split()[0])        
    with open('/home/ucar/ucar_ws/src/image_fruits/5_1.txt','r') as f:
        for line in f.readlines():
            result_list5.append(line.split()[0])
    with open('/home/ucar/ucar_ws/src/image_fruits/5_2.txt','r') as f:
        for line in f.readlines():
            result_list5.append(line.split()[0])

    while len(result_list5) < 3:
        result_list5.append("nan")
    
    result_list5.append('\n')

    with open(file_name, "a") as file:
        # for result1 in result_list1:
        #     if result1 == '\n':
        #         file.write("{}".format(result1))
        #     else:
        #         file.write("{} ".format(result1))
        # for result2 in result_list2:
        #     if result2 == '\n':
        #         file.write("{}".format(result2))
        #     else:
        #         file.write("{} ".format(result2))
        # for result3 in result_list3:
        #     if result3 == '\n':
        #         file.write("{}".format(result3))
        #     else:
        #         file.write("{} ".format(result3))
        # for result4 in result_list4:
        #     if result4 == '\n':
        #         file.write("{}".format(result4))
        #     else:
        #         file.write("{} ".format(result4))
        for result5 in result_list5:
            if result5 == '\n':
                file.write("{}".format(result5))
            else:
                file.write("{} ".format(result5))

yolo_fruits_start_flag = 0

def main():
    clear_result()
    # args = parser()
    rospy.loginfo("yoloyolofruitsfruitsfruitsfruits")
    # rospy.loginfo(args.batch_size)
    # rospy.loginfo(args.config_file)
    # check_arguments_errors(args)

    random.seed(3)  # deterministic bbox colors
    # network, class_names, class_colors = darknet.load_network(
    #     args.config_file,
    #     args.data_file,
    #     args.weights,
    #     batch_size=args.batch_size
    # )
    network, class_names, class_colors = darknet.load_network(
        "/home/ucar/ucar_ws/src/ucar_yolo/scripts/cfg/v4-tiny-19race.cfg",
        "/home/ucar/ucar_ws/src/ucar_yolo/scripts/cfg/19xunfei.data",
        "/home/ucar/ucar_ws/src/ucar_yolo/scripts/4.weights",
        batch_size=1
    )

    # images = load_images(args.input)
    images = load_images("/home/ucar/ucar_ws/src/image_fruits/")
    index = 0
    while True:
        if yolo_fruits_start_flag == 1:
            break
    while True:
        # loop asking for new image paths if no list is given
        # if args.input:
        if "/home/ucar/ucar_ws/src/image_fruits/":
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

def callback_yolo_fruits_start(msg):
    global yolo_fruits_start_flag
    yolo_fruits_start_flag = msg.data

if __name__ == "__main__":
    try:
        # 初始化ros节点
        rospy.init_node("yolo_vegation")
        rospy.loginfo("Starting yolo node")
        yolo_fruits_start_sub = rospy.Subscriber("/yolo_vegetation_over_flag", Int8 , callback_yolo_fruits_start)
        yolo_fruits_over_pub = rospy.Publisher("/yolo_over_flag", Int8, queue_size=1)

        main()
        save_result()
        yolo_fruits_over_pub.publish(1)
        rospy.loginfo("yolo detect over")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down yolo node.")