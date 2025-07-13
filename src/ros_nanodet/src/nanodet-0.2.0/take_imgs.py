#! /usr/bin/env python
import cv2

video = cv2.VideoCapture('/dev/video0')     # 调用摄像头，PC电脑中0为内置摄像头，1为外接摄像头
judge = video.isOpened()      # 判断video是否打开
i = 411
print(video.get(cv2.CAP_PROP_FRAME_WIDTH))

output_filename = "/home/ucar/ucar_car/src/ztestnav2025/nanodet_debug/nanodet.avi"#录制视频防止可视化卡顿
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # MP4格式，其他选项：'XVID'->AVI, 'MJPG'->MJPEG
fps = 10.0
frame_size = (640, 480)  # 必须和实际帧尺寸一致
out = cv2.VideoWriter(output_filename, fourcc, fps, frame_size)
# 检查是否成功创建
if not out.isOpened():
    print("无法创建视频文件！")
while judge:
    ret, frame = video.read()
    
    cv2.imshow("frame", frame)
    resize_img = cv2.resize(frame, dsize=(960, 540))
    keyword = cv2.waitKey(1)
    if keyword == ord('s'):      # 按s保存当前图片
        i += 1
        imgname = "/home/ucar/ucar_car/picture_"+str(i)+".jpg"
        cv2.imwrite(imgname, resize_img)
        print(str(i))
    elif keyword == ord('q'):     # 按q退出
        break
    out.write(frame)
    #elif i == 10:     # 按q退出
    #   break

# 释放窗口
video.release()
cv2.destroyAllWindows()
out.release()
