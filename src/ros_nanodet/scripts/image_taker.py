import cv2

# 打开摄像头设备
cap = cv2.VideoCapture(0)

# 设置摄像头分辨率（部分设备可能不支持）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 验证实际分辨率
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
need_resize = (actual_width, actual_height) != (640, 480)

if need_resize:
    print(f"摄像头当前分辨率：{actual_width}x{actual_height}，将自动缩放至640x480")

save_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("错误：无法获取视频帧")
        break

    # 分辨率自动修正
    if need_resize:
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)

    cv2.imshow('Camera Feed', frame)

    key = cv2.waitKey(20) & 0xFF

    if key in (ord('0'), 0x1000000 + ord('0')) or key in (96, 0x1000000 + 96):
        # 确保保存前已调整分辨率
        if frame.shape[1] != 640 or frame.shape[0] != 480:
            frame = cv2.resize(frame, (640, 480))
            
        filename = f"capture_{save_count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"已保存640x480截图：{filename}")
        save_count += 1
    elif key == ord('q'):
        print("正在退出...")
        break

cap.release()
cv2.destroyAllWindows()