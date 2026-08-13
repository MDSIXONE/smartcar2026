#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,Int8
from cv_bridge import CvBridge
import cv2
import fcntl
import time
from std_msgs.msg import String as ROSString
import time
import os
import subprocess
import socket
done=-1
start=-1


def sim_start_callback(data):
    global start
    start=data.data
    #print("start sim")

def recv_exactly(sock, n_bytes):
    """确保读取 n_bytes 字节，否则抛出异常"""
    data = b""
    while len(data) < n_bytes:
        chunk = sock.recv(n_bytes - len(data))
        if not chunk:  # 连接关闭
            return
        data += chunk
    return data




rospy.init_node('sim_server')
sim_start = rospy.Subscriber("/sim_start", Int8,sim_start_callback)
sim_done = rospy.Publisher("/sim_done", Int32,queue_size=10)



def int_to_fixed_str(num):
    s = str(num)
    if num >= 0:
        return s.zfill(3)  # 正数补0到2位
    else:
        return "-"+str(-num).zfill(2)  # 负数不补0



server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 防止端口占用

# 绑定地址和端口
HOST = '192.168.1.100'  # 本地回环地址
PORT = 60007        # 监听端口
server_socket.bind((HOST, PORT))
server_socket.listen()
server_socket.settimeout(0.5)
print(f"服务器正在监听 {HOST}:{PORT}...")
i=0
while not rospy.is_shutdown():
    # if i<5:
    #     i=i+1
    # else:
    #     start=0
    time.sleep(0.5)
    sim_done.publish(done)
    # print(done)

    client_socket=None
    try:
    
        client_socket, addr = server_socket.accept()
        client_socket.settimeout(0.3)
        while True:
            data = recv_exactly(client_socket,3)#读两个字节 数字统一用两个字节表示 可能是 -1 00 01 02 
            if not data:
                break
            
            print(f"收到数据: {data.decode('utf-8')}")
            result=int(data.decode('utf-8'))
            if result>=0:
                done=result
                sim_done.publish(done)
                break
            client_socket.sendall(int_to_fixed_str(start).encode('utf-8'))
    except BaseException as  e:
        # print(e)
        pass
    finally:
        if client_socket:
            client_socket.close()  # 关闭客户端连接





