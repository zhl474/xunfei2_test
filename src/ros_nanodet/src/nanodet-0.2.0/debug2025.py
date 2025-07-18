
from demo.demo import detect
from demo.demo import init
import cv2

predictor = init()

frame = cv2.imread('/home/ucar/ucar_car/ypicture/picture_14.jpg')
res = detect(frame,predictor)# 识别
res = detect(frame,predictor)# 识别2次
print("warm up done")

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

COLOR_MAP = {
    0: (0, 255, 0),   # 绿色
    1: (255, 0, 0),   # 红色
    2: (0, 165, 255)  # 橙色
}

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.6
THICKNESS = 2

save_count = 76

while True:
    rec, frame = cap.read()
    # print(rec)
    if not rec:
        print("Failed to read frame from video capture device.")
        continue
    copy = frame.copy()
    res = detect(frame, predictor)
    for label in res:
        for bbox in res[label]:
            score = bbox[-1]
            if score>0.5:
                # print("find object")
                x0, y0, x1, y1 = [int(i) for i in bbox[:4]]
                color = COLOR_MAP.get(label, (0, 255, 255))  # 默认黄色
                cv2.rectangle(frame, 
                            (x0, y0), 
                            (x1, y1),
                            color, 
                            thickness=2)
                
                # 绘制标签文本[7](@ref)
                text = f"{label}: {score:.2f}"
                cv2.putText(frame, text, 
                          (x0, y0-10), 
                          FONT, 
                          FONT_SCALE,
                          color,
                          THICKNESS)
    cv2.imshow('Detection Results', frame)
    key = cv2.waitKey(20) & 0xFF

    if key in (ord('0'), 0x1000000 + ord('0')) or key in (96, 0x1000000 + 96):
        # 确保保存前已调整分辨率
        if copy.shape[1] != 640 or copy.shape[0] != 480:
            copy = cv2.resize(copy, (640, 480))
            
        filename = f"/home/ucar/ucar_car/missing_{save_count}.jpg"
        cv2.imwrite(filename, copy)
        print(f"已保存640x480截图：{filename}")
        save_count += 1
    elif key == ord('q'):
        print("正在退出...")
        break
