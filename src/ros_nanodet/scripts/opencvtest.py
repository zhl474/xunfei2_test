#!/usr/bin/env python3
#coding=utf-8

import cv2

# 打开默认摄像头（通常是 /dev/video0）
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("Error: Could not open video device.")
else:
    print("Camera opened successfully.")

# 设置摄像头分辨率（可选）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"实际分辨率: {actual_width}x{actual_height}")
while True:
    # 从摄像头读取一帧图像
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture image.")
        break

    # 显示捕获到的图像
    cv2.imshow("Camera Feed", frame)

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
