# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
import argparse
import os
import sys
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

from utils.general import *
from models.experimental import attempt_load

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams, MyLoadImages
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

def init():
    cap = cv2.VideoCapture(0)
    return cap

def Histogram(frame):
    (b, g, r) = cv2.split(frame)  # 通道分离
    bH = cv2.equalizeHist(b)
    gH = cv2.equalizeHist(g)
    rH = cv2.equalizeHist(r)
    result = cv2.merge((bH, gH, rH))  # 实现图像通道的合并
    return result
# 增加运行参数，原来的参数是通过命令行解析对象提供的，这里改为由调用者在代码中提供。需要一个
# 大体上完成一样功能的参数对象。
# 我想要的功能是传一组由cv2读取的图片，交给api，然后得到一组打上标签的图片，以及每张图片对应的标签类别引索，位置信息，置信度的信息，还有类别名称字典
# 要实现这个功能，需要权重文件，输入文件两个参数，其他参数与原代码命令行默认参数保持一致就行。
class simulation_opt:# 参数对象。

    def __init__(self,weights,img_size=640,conf_thres=0.25,iou_thres=0.45,device='',view_img=False,
                 classes=None,agnostic_nms=False,augment=False,update=False,exist_ok=False):
        self.weights=weights
        self.source=None
        self.img_size=img_size
        self.conf_thres=conf_thres
        self.iou_thres=iou_thres
        self.device=device
        self.view_img=view_img
        self.classes=classes
        self.agnostic_nms=agnostic_nms
        self.augment=augment
        self.update=update
        self.exist_ok=exist_ok

#增加一个新类，这个新类是在原来detect函数上进行删减。可以先复制原来的detect函数代码，再着手修改
class detectapi:
    def __init__(self,weights,img_size=640):
        # 构造函数中先做好必要的准备，如初始化参数，加载模型
        #改为
        self.opt=simulation_opt(weights=weights,img_size=img_size)
        weights, imgsz= self.opt.weights, self.opt.img_size

    # Initialize
        set_logging()
        self.device = select_device(self.opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
        self.model = attempt_load(weights,self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check img_size
        if self.half:
            self.model.half()  # to FP16

    # Second-stage classifier
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet101', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('weights/resnet101.pt',self.device)['model']).to(self.device).eval()
        '''
        self.names,和self.colors是由后面的代码拉到这里来的。names是类别名称字典，colors是画框时用到的颜色。
        '''
    # read names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]


    def detect(self,source): # 使用时，调用这个函数
        if type(source)!=list:
                raise TypeError('source must be a list which contain  pictures read by cv2')

        dataset = MyLoadImages(source, img_size=self.imgsz, stride=self.stride)
       

    # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        result=[]
        ''' 删掉
        for path, img, im0s, vid_cap in dataset: 因为不用保存，所以path可以不要，因为不处理视频，所以vid_cap不要。
        ''' #改为
        for img, im0s in dataset:
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            # t1 = time_synchronized() #计算预测用时的，可以不要
            pred = self.model(img, augment=self.opt.augment)[0]

            # Apply NMS
            pred = non_max_suppression(pred, self.opt.conf_thres, self.opt.iou_thres, classes=self.opt.classes, agnostic=self.opt.agnostic_nms)
            # t2 = time_synchronized() #计算预测用时的，可以不要

            # Apply Classifier
            if self.classify:
                pred = apply_classifier(pred, self.modelc, img, im0s)
          # [tensor[[x1,y1,x2,y2,置信度,类别],[...],...]]
 # 改为
            # Process detections
            det=pred[0] #原来的情况是要保持图片，因此多了很多关于保持路径上的处理。另外，pred
            # 其实是个列表。元素个数为batch_size。由于对于我这个api，每次只处理一个图片，
            # 所以pred中只有一个元素，直接取出来就行，不用for循环。
            im0 = im0s.copy() # 这是原图片，与被传进来的图片是同地址的，需要copy一个副本，否则，原来的图片会受到影响
            # s += '%gx%g ' % img.shape[2:]  # print string
            # gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            result_txt = []
            # 对于一张图片，可能有多个可被检测的目标。所以结果标签也可能有多个。
            # 每被检测出一个物体，result_txt的长度就加一。result_txt中的每个元素是个列表，记录着
            # 被检测物的类别引索，在图片上的位置，以及置信度
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
               
                # Write results

                for *xyxy, conf, cls in reversed(det):  # reversed为反向索引
                    myclass = int(cls.item())
                    myconf = conf.item()

                    #xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #line = (int(cls.item()), [int(_.item()) for _ in xyxy], conf.item())  # label format
# item()返回的是一个浮点型数据
                    #result_txt.append(line)
                    #label = f'{self.names[int(cls)]} {conf:.2f}'
                    #annotator = Annotator(im0, line_width=3, example=None)
                    #annotator.box_label(xyxy, label, color = colors(cls))
            #result.append((im0,result_txt)) # 对于每张图片，返回画完框的图片，以及该图片的标签列表。
       # return result, self.names
        return myclass,myconf



