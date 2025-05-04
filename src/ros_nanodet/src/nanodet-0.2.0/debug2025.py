
from demo.demo import detect
from demo.demo import init
import cv2

predictor = init()

frame = cv2.imread('/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/2/1.png')
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

while True:
    rec, frame = cap.read()
    print(rec)
    if not rec:
        print("Failed to read frame from video capture device.")
        continue
    res = detect(frame, predictor)
    for label in res:
        for bbox in res[label]:
            score = bbox[-1]
            if score>0.7:
                print("find object")
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
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
