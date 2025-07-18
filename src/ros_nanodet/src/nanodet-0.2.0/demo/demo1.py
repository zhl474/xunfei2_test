import cv2
import os
import time
import torch
import argparse
import time
from nanodet.util import cfg, load_config, Logger
from nanodet.model.arch import build_model
from nanodet.util import load_model_weight
from nanodet.data.transform import Pipeline
from calibrate_helper import Calibrator
m = 0
image_ext = ['.jpg', '.jpeg', '.webp', '.bmp', '.png']
video_ext = ['mp4', 'mov', 'avi', 'mkv']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('demo', default='image', help='demo type, eg. image, video and webcam')#
    parser.add_argument('--config', help='model config file path')#model config
    parser.add_argument('--model', help='model file path')
    parser.add_argument('--path', default='./demo', help='path to images or video')#/dev/video0相机位置
    parser.add_argument('--camid', type=int, default=0, help='webcam demo camera id')
    parser.add_argument('--save_result', action='store_true', help='whether to save the inference result of image/video')
    args = parser.parse_args()
    return args


class Predictor(object):
    def __init__(self, cfg, model_path, logger, device='cuda:0'):
        self.cfg = cfg
        self.device = device
        model = build_model(cfg.model)
        ckpt = torch.load(model_path, map_location=lambda storage, loc: storage)
        load_model_weight(model, ckpt, logger)
        if cfg.model.arch.backbone.name == 'RepVGG':
            deploy_config = cfg.model
            deploy_config.arch.backbone.update({'deploy': True})
            deploy_model = build_model(deploy_config)
            from nanodet.model.backbone.repvgg import repvgg_det_model_convert
            model = repvgg_det_model_convert(model, deploy_model)
        self.model = model.to(device).eval()
        self.pipeline = Pipeline(cfg.data.val.pipeline, cfg.data.val.keep_ratio)

    def inference(self, img):
        img_info = {}
        if isinstance(img, str):
            img_info['file_name'] = os.path.basename(img)
            img = cv2.imread(img)
        else:
            img_info['file_name'] = None

        height, width = img.shape[:2]
        img_info['height'] = height
        img_info['width'] = width
        meta = dict(img_info=img_info,
                    raw_img=img,
                    img=img)
        meta = self.pipeline(meta, self.cfg.data.val.input_size)
        meta['img'] = torch.from_numpy(meta['img'].transpose(2, 0, 1)).unsqueeze(0).to(self.device)
        with torch.no_grad():
            results = self.model.inference(meta)
        return meta, results

    def visualize(self, dets, meta, class_names, score_thres, wait=0):
        time1 = time.time()
        result_img = self.model.head.show_result(meta['raw_img'], dets, class_names, score_thres=score_thres, show=True)
        print('viz time: {:.3f}s'.format(time.time()-time1))
        return result_img


def get_image_list(path):
    image_names = []
    for maindir, subdir, file_name_list in os.walk(path):
        for filename in file_name_list:
            apath = os.path.join(maindir, filename)
            ext = os.path.splitext(apath)[1]
            if ext in image_ext:
                image_names.append(apath)
    return image_names

def Histogram(frame):
    (b, g, r) = cv2.split(frame)
    bH = cv2.equalizeHist(b)
    gH = cv2.equalizeHist(g)
    rH = cv2.equalizeHist(r)
    result = cv2.merge((bH, gH, rH))
    return result
def detect(cap,predictor):  
#    global m
    ret_val, frame = cap.read()
#    frame = Histogram(frame)
#    m = m+1
    if ret_val:
        meta, res = predictor.inference(frame)
#        cv2.imwrite('picture'+str(m)+'.jpg',frame)
#        print("success to save" + str(m) + ".jpg")
    return meta,res
def init():
    # args = parse_args()
    path = '/dev/video0'
    model = '/home/ucar/hdu_detect/src/ros_nanodet/src/nanodet-0.2.0/workspace/nanodet_m/model_best/nanodet_model_714_2.pth'  # 已修改
    #config = '/home/ucar/hdu_detect/src/ros_nanodet/src/nanodet-0.2.0/config/nanodet_custom_xml_dataset_overfitting.yml'
    config = '/home/ucar/hdu_detect/src/ros_nanodet/src/nanodet-0.2.0/config/nanodet_714_2.yml'
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True
    load_config(cfg, config)
    logger = Logger(-1, use_tensorboard=False)
    predictor = Predictor(cfg, model, logger, device='cuda:0')
    # if demo == 'video' or demo == 'webcam':
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    return cap,predictor

def main():
    args = parse_args()
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True
    print(no1)
    load_config(cfg, args.config)
    logger = Logger(-1, use_tensorboard=False)
    predictor = Predictor(cfg, args.model, logger, device='cuda:0')
    logger.log('Press "Esc", "q" or "Q" to exit.')
    current_time = time.localtime()
    if args.demo == 'image':
        if os.path.isdir(args.path):
            files = get_image_list(args.path)
        else:
            files = [args.path]
        files.sort()
        for image_name in files:
            meta, res = predictor.inference(image_name)
            result_image = predictor.visualize(res, meta, cfg.class_names, 0.5)
            if args.save_result:
                save_folder = os.path.join(cfg.save_dir, time.strftime("%Y_%m_%d_%H_%M_%S", current_time))
                if not os.path.exists(save_folder):
                    #os.mkdir(save_folder)11111111111
                     pass
                save_file_name = os.path.join(save_folder, os.path.basename(image_name))
                #cv2.imwrite(save_file_name, result_image)111111111111111111
            ch = cv2.waitKey(0)
            if ch == 27 or ch == ord('q') or ch == ord('Q'):
                break
    elif args.demo == 'video' or args.demo == 'webcam':
        cap = cv2.VideoCapture(args.path , cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,800)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,600)
        #cap.set(cv2.CAP_PROP_FRAME_WIDTH,960)
        #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,540)
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float
        print(cv2.CAP_PROP_FRAME_WIDTH,cv2.CAP_PROP_FRAME_HEIGHT)
        print(width,height)
        fps = cap.get(cv2.CAP_PROP_FPS)
        print("fps.get is : {}".format(fps))
        cap.set(cv2.CAP_PROP_FPS, 30)
        save_folder = os.path.join(cfg.save_dir, time.strftime("%Y_%m_%d_%H_%M_%S", current_time))
        if not os.path.exists(save_folder):
            os.mkdir(save_folder)
        save_path = os.path.join(save_folder, args.path.split('/')[-1]) if args.demo == 'video' else os.path.join(save_folder, 'camera.mp4')
        print(f'save_path is {save_path}')
        #----------------------以下修改用以测试标定效果
        #img_dir = "./demo/img"#这个地方要注意程序在哪里运行 此处目录也要相应更改 一般是在nanodet-0.2.0目录下运行
        #shape_inner_corner = (8, 6) #内角点个数
        #size_grid = 0.034 #格子长度 单位米
        # create calibrator
        #calibrator = Calibrator(img_dir, shape_inner_corner, size_grid)
        # calibrate the camera
        #mat_intri, coff_dis = calibrator.calibrate_camera()
        #---------------------------------------------------
        #vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (int(width), int(height)))
        while True:
            t=time.time()
            ret_val, frame = cap.read()
            #frame = Histogram(frame)
            k = cv2.waitKey(1)
            if k == ord('s'):#按键按下s之后图片就会保存
                cv2.imwrite('picture'+str(m)+'.jpg',frame)
                print("success to save" + str(m) + ".jpg")
                m = m+1
            if ret_val:
                #frame = calibrator.my_dedistortion(frame)
                meta, res = predictor.inference(frame)
                #print(type(res))
                print(res)
                result_frame = predictor.visualize(res, meta, cfg.class_names, 0.4)
                #if args.save_result:
                    #vid_writer.write(result_frame)
                ch = cv2.waitKey(1)
                if ch == 27 or ch == ord('q') or ch == ord('Q'):
                    break
            else:
                break
            print('fps is ',1/(time.time()-t))


if __name__ == '__main__':
    main()
