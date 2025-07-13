#! /usr/bin/env python
import cv2
import numpy as np

video = cv2.VideoCapture('/dev/video0')     # 调用摄像头，PC电脑中0为内置摄像头，1为外接摄像头
judge = video.isOpened()      # 判断video是否打开
i = 411
print(video.get(cv2.CAP_PROP_FRAME_WIDTH))

def resize_with_padding(image, target_size):
    """
    按比例缩放图像并在多余部分填充黑色
    :param image: 输入图像
    :param target_size: 目标尺寸 (width, height)
    :return: 缩放并填充后的图像
    """
    # 获取原始图像尺寸
    h, w = image.shape[:2]
    target_w, target_h = target_size
    
    # 计算缩放比例
    scale = min(target_w / w, target_h / h)
    
    # 计算缩放后的新尺寸
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    # 缩放图像
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    # 创建新的画布（黑色背景）
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    
    # 计算粘贴位置（居中）
    x_offset = (target_w - new_w) // 2
    y_offset = (target_h - new_h) // 2
    
    # 将缩放后的图像粘贴到画布中心
    canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
    
    return canvas

output_filename = "/home/ucar/ucar_car/src/ztestnav2025/nanodet_debug/image_data.avi"#录制视频防止可视化卡顿
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # MP4格式，其他选项：'XVID'->AVI, 'MJPG'->MJPEG
fps = 10.0
frame_size = (416, 416)  # 必须和实际帧尺寸一致
out = cv2.VideoWriter(output_filename, fourcc, fps, frame_size)
# 检查是否成功创建
if not out.isOpened():
    print("无法创建视频文件！")
while judge:
    ret, frame = video.read()
    
    resize_img = resize_with_padding(frame, (416, 416))
    cv2.imshow("frame", resize_img)
    keyword = cv2.waitKey(1)
    if keyword == ord('s'):      # 按s保存当前图片
        i += 1
        imgname = "/home/ucar/ucar_car/picture_"+str(i)+".jpg"
        cv2.imwrite(imgname, resize_img)
        print(str(i))
    elif keyword == ord('q'):     # 按q退出
        break

    out.write(resize_img)
    #elif i == 10:     # 按q退出
    #   break

# 释放窗口
video.release()
cv2.destroyAllWindows()
out.release()
