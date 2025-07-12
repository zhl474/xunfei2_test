#!/usr/bin/python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import argparse
import time
from pathlib import Path
import os, shutil, time
import cv2
import torch
import torch.backends.cudnn as cudnn
import os
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized

person = 0
longhair = 0
glasses = 0

parser = argparse.ArgumentParser()
parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')#置信度阈值
parser.add_argument('--iou-thres', type=float, default=0.05, help='IOU threshold for NMS')#防止出现重复框
parser.add_argument('--max-det', type=int, default=1000, help='maximum number of detections per image')
parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
opt = parser.parse_args()

img_size = 320 #必须是32的倍数,yolov5是先压缩再识别，尽量多的特征应该不压缩
img_size1 = 160 #必须是32的倍数,yolov5是先压缩再识别，尽量多的特征应该不压缩
weights = '/home/ucar/ucar_ws/src/yolo/src/yolov5-develop/weights/last.pt'
weights1 = '/home/ucar/ucar_ws/src/yolo/src/yolov5-develop/weights/yolov5s.pt'
source = '/home/ucar/ucar_ws/src/opencv_test/src/yolo/'
source1 = '/home/ucar/ucar_ws/src/opencv_test/src/yoloP/'
savepath = '/home/ucar/ucar_ws/src/opencv_test/src/yolo_debug/detect/'
savepathp = '/home/ucar/ucar_ws/src/opencv_test/src/yolo_debug/person/'
debug = 1 #是否保存调试图片，1保存，0不保存
angle = 1 #是否去重复操作，1去重复，0不去重

# Initialize
set_logging()
device = select_device(opt.device)#'cuda device, i.e. 0 or 0,1,2,3 or cpu
half = device.type != 'cpu'  # half precision only supported on CUDA

# Load model
model = attempt_load(weights, map_location=device)  # load FP32 model
stride = int(model.stride.max())  # model stride
names = model.module.names if hasattr(model, 'module') else model.names  # get class names

# Load model1
model1 = attempt_load(weights1, map_location=device)  # load FP32 model
stride1 = int(model1.stride.max())  # model stride
names1 = model1.module.names if hasattr(model1, 'module') else model1.names  # get class names

if half:
  model.half()  # to FP16
  model1.half()  # to FP16
  
#/////////////////////////////////////清空yolo的几个文件夹里的图片////////////////////////////////////////////////////
filelist = os.listdir(source)   #获取文件路径
total_num = len(filelist)  #获取文件长度（文件夹下图片个数）
for item in filelist:
  item1 = os.path.join(source, item)
  os.remove(item1)
  
filelist = os.listdir(source1)   #获取文件路径
total_num = len(filelist)  #获取文件长度（文件夹下图片个数）
for item in filelist:
  item1 = os.path.join(source1, item)
  os.remove(item1)
  
filelist = os.listdir(savepath)   #获取文件路径
total_num = len(filelist)  #获取文件长度（文件夹下图片个数）
for item in filelist:
  item1 = os.path.join(savepath, item)
  os.remove(item1)
  
filelist = os.listdir(savepathp)   #获取文件路径
total_num = len(filelist)  #获取文件长度（文件夹下图片个数）
for item in filelist:
  item1 = os.path.join(savepathp, item)
  os.remove(item1)
shutil.copyfile('/home/ucar/ucar_ws/src/opencv_test/src/keep.jpg','/home/ucar/ucar_ws/src/opencv_test/src/yolo/keep.jpg')#复制文件

        
@torch.no_grad()
def detect1():
    # Set Dataloader
    dataset = LoadImages(source1, img_size=img_size1, stride=stride)

    # Run inference
    if device.type != 'cpu':
        model1(torch.zeros(1, 3, img_size1, img_size1).to(device).type_as(next(model1.parameters())))  # run once

    global person
    
    
    num = 0
    for path, img, im0s, vid_cap in dataset:# 有几张图片这个就会循环几次
        num += 1
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        pred = model1(img)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, opt.classes, opt.agnostic_nms,
                                   max_det=opt.max_det)
        
        # Process detections
        for i, det in enumerate(pred):  # detections per image 此处只运行一次，取出对象操作
            im0 = im0s.copy()
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for *xyxy, conf, cls in reversed(det):# 检测出某一对象几个就会循环几次
                    if(int(cls) == 0):
                      if ((xyxy[3]-xyxy[1])>400 ): #and (xyxy[2]<376) and ((xyxy[2]-xyxy[0])>120)):and (xyxy[2]-xyxy[0])>150
                        person += 1
                        savepath1 = source + str(person) + '.jpg'
                        #//////////////////////////ljj 对目标剪裁，裁剪坐标为[y0:y1, x0:x1]，并存入列表
                        #//////////////////////////ljj 其中xyxy[0]-xyxy[3]对应x0\y0\x1\y1
                        cv2.imwrite(savepath1, im0[int(xyxy[1]):(int(xyxy[1])+320), int(xyxy[0]):int(xyxy[2])])
                      
                    
                    
            if debug:
                for *xyxy, conf, cls in reversed(det):# 检测出某一对象几个就会循环几次
                    if ((xyxy[3]-xyxy[1])>400): #and (xyxy[2]<376) and ((xyxy[2]-xyxy[0])>120)):
                      # label = str(names2[int(cls)]) + str(float(conf))
                      label = f'{names1[int(cls)]} {conf:.2f}'
                      plot_one_box(xyxy, im0, label=label, color=colors(int(cls), True), line_thickness=3)
                savepath1 = savepathp + str(num) + 'person.jpg'
                cv2.imwrite(savepath1, im0)
                
    #******************/ 打印类似 3 persons /*****************************/
    # print("\r\n")
    # print("longhair", longhair)#///////////ljj 对应某个类，在data.yaml里面去数，比如coco数据集里person是0、zebra是22
    # print("glasses", glasses)#///////////ljj 对应某个类，在data.yaml里面去数，比如coco数据集里person是0、zebra是22   


@torch.no_grad()
def detect():
    # Set Dataloader
    dataset = LoadImages(source, img_size=img_size, stride=stride)

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, img_size, img_size).to(device).type_as(next(model.parameters())))  # run once

    global longhair
    global glasses
    global lst
    longhair2 = 0
    glasses2  = 0
    num = 0
    for path, img, im0s, vid_cap in dataset:# 有几张图片这个就会循环几次
        num += 1
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        pred = model(img)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, opt.classes, opt.agnostic_nms,
                                   max_det=opt.max_det)
        
        longhair1 = 0
        glasses1  = 0

        # Process detections
        for i, det in enumerate(pred):  # detections per image 此处只运行一次，取出对象操作
            im0 = im0s.copy()
            if len(det) :
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                if angle == 1:
		                for c in det[:, -1].unique():#///////////////////////////////////////////////// ljj 检测出几个不同的对象就会循环几次
		                    n = (det[:, -1] == c).sum()  # detections per class
		                    if int(c) == 0:# 长头发
		                        longhair1 = int(n)
		                    if int(c) == 1:# 戴眼镜
		                         glasses1  = int(n)
		                print('\r\n')
		                print(longhair2)
		                print(glasses2)
		                print('\r\n')
		                if(longhair1 == longhair2 and glasses1 == glasses2) :
		                    print('\r\nrepeat detect pic\r\n')
		                    print('\r\nrepeat detect pic\r\n')
		                    print('\r\nrepeat detect pic\r\n')
		                else :
		                    longhair += longhair1
		                    glasses  += glasses1
		                    longhair2 = longhair1
		                    glasses2  = glasses1
                else :
                    for c in det[:, -1].unique():#///////////////////////////////////////////////// ljj 检测出几个不同的对象就会循环几次
		                    n = (det[:, -1] == c).sum()  # detections per class
		                    if int(c) == 0:# 长头发
		                        longhair += int(n)
		                    if int(c) == 1:# 戴眼镜
		                        glasses  += int(n)
            if debug:
                for *xyxy, conf, cls in reversed(det):# 检测出某一对象几个就会循环几次
                    # label = str(names2[int(cls)]) + str(float(conf))
                    label = f'{names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=colors(int(cls), True), line_thickness=3)
                savepath1 = savepath + str(num) + '.jpg'
                cv2.imwrite(savepath1, im0)
    #******************/ 打印类似 3 persons /*****************************/
    # print("\r\n")
    # print("longhair", longhair)#///////////ljj 对应某个类，在data.yaml里面去数，比如coco数据集里person是0、zebra是22
    # print("glasses", glasses)#///////////ljj 对应某个类，在data.yaml里面去数，比如coco数据集里person是0、zebra是22   





# 创建话题发布者
#yolo_pub = rospy.Publisher('/yolo', Int8, queue_size=10)
#longhair_pub = rospy.Publisher('/longhair', Int8, queue_size=10)
#glasses_pub = rospy.Publisher( '/glasses', Int8, queue_size=10)
flag = 0
def control_mode_callback(msg):
  global flag
  control_mode = msg.data
  if control_mode == 6:
    a1 = time.time()
    detect1()
    b1 = time.time()
    c1 = b1-a1

    a = time.time()
    detect()
    b = time.time()
    c = b-a
    
    #longhair_pub.publish(longhair)
    #glasses_pub.publish(glasses)
    #time.sleep(1)
    #yolo_pub.publish(1)
  #if control_mode == 7:
    '''if(longhair == 0):
      os.system("play ~/ucar_ws/src/mp3/0longhair.mp3");
    elif(longhair == 1):
      os.system("play ~/ucar_ws/src/mp3/1longhair.mp3");
    elif(longhair == 2):
      os.system("play ~/ucar_ws/src/mp3/2longhair.mp3");
    elif(longhair > 2):
      os.system("play ~/ucar_ws/src/mp3/2longhair.mp3");
    if(glasses == 0):
      os.system("play ~/ucar_ws/src/mp3/0glasses.mp3");
    elif(glasses == 1):
      os.system("play ~/ucar_ws/src/mp3/1glasses.mp3");
    elif(glasses == 2):
      os.system("play ~/ucar_ws/src/mp3/2glasses.mp3");
    elif(glasses > 2):
      os.system("play ~/ucar_ws/src/mp3/2glasses.mp3");
    '''
    print('\r\n')
    print("person:   " ,person)
    print("longhair: " ,longhair)
    print("glasses:  " ,glasses)
    print('\r\n')
    print("识别特征用时：",c)
    print('\r\n')
    print("识别人物用时：",c1)
    flag = 1


def finnal_callback(msg):
  global flag
  if msg.data == 1 and flag == 1:
    if(longhair == 0):
      os.system("play ~/ucar_ws/src/mp3/0longhair.mp3");
    elif(longhair == 1):
      os.system("play ~/ucar_ws/src/mp3/1longhair.mp3");
    elif(longhair == 2):
      os.system("play ~/ucar_ws/src/mp3/2longhair.mp3");
    elif(longhair > 2):
      os.system("play ~/ucar_ws/src/mp3/2longhair.mp3");
    if(glasses == 0):
      os.system("play ~/ucar_ws/src/mp3/0glasses.mp3");
    elif(glasses == 1):
      os.system("play ~/ucar_ws/src/mp3/1glasses.mp3");
    elif(glasses == 2):
      os.system("play ~/ucar_ws/src/mp3/2glasses.mp3");
    elif(glasses > 2):
      os.system("play ~/ucar_ws/src/mp3/2glasses.mp3");
    os.system("play ~/ucar_ws/src/mp3/over1.mp3");
    flag = 0




def rosyolo():
  #初始化ROS节点
  rospy.init_node('yolo', anonymous=True)
  # 创建话题订阅者
  rospy.Subscriber('/control_mode', Int8, control_mode_callback)
  rospy.Subscriber('/finnal', Int8, finnal_callback)
  while not rospy.is_shutdown():
    rospy.spin()
	



if __name__ == '__main__':
  try:
      rosyolo()
  except rospy.ROSInterruptException:
      pass
  
    


