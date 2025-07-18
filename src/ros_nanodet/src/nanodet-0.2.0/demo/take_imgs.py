#! /usr/bin/env python
import cv2

video = cv2.VideoCapture(0)     # 调用摄像头，PC电脑中0为内置摄像头，1为外接摄像头

judge = video.isOpened()      # 判断video是否打开
i=0
while judge:
    ret, frame = video.read()
    
    cv2.imshow("frame", frame)
    resize_img = cv2.resize(frame,dsize=(960,540))
  
    keyword = cv2.waitKey(1)
    print(keyword)
    print(ord('s'))
    print(ord('q'))
    if keyword == ord('s'):      # 按s保存当前图片
        i+=1
        imgname = "/home/ucar/hdu_detect/src/ros_nanodet/src/nanodet-0.2.0/demo/img/"+str(i)+".jpg"
        cv2.imwrite(imgname, resize_img)
    elif keyword == ord('q'):     # 按q退出
        break

# 释放窗口
video.release()
cv2.destroyAllWindows()

