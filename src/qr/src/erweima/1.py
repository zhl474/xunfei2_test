import cv2
import numpy as np
from pyzbar.pyzbar import decode
import os



# 读取二维码图像，提取文件夹名
qr_image_path = "F:/zhn/PythonProject/xf_1/水果.jpg"
with open(qr_image_path, 'rb') as f:
    data = f.read()
qr_img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

decoded = decode(qr_img)
if not decoded:
    print("二维码读取失败")
    exit()

def aruco_demo():
    #创立节点
    rospy.init_node('aruco_demo', anonymous=True) 
    
    #订阅usb_cam发出的图像消息，接收到消息后进入回调函数callback()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)  
    
    #等待
    rospy.spin()  

if __name__ == '__main__':
    aruco_demo()