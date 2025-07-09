import cv2
import numpy as np
#计算透视变换矩阵用的
# 全局变量
points = []  # 存储选择的四个点
dragging = False  # 是否正在拖动点
current_point = -1  # 当前拖动的点索引
dst_size = (400, 300)  # 目标矩形尺寸 (宽度, 高度)

def mouse_callback(event, x, y, flags, param):
    global points, dragging, current_point
    
    # 左键点击：添加或选择点
    if event == cv2.EVENT_LBUTTONDOWN:
        # 如果已选4个点，检查是否点击了现有点
        if len(points) == 4:
            for i, pt in enumerate(points):
                if np.linalg.norm(np.array(pt) - np.array((x, y))) < 15:  # 15像素内
                    current_point = i
                    dragging = True
                    break
        # 如果少于4个点，添加新点
        elif len(points) < 4:
            points.append((x, y))
    
    # 左键释放：停止拖动
    elif event == cv2.EVENT_LBUTTONUP:
        dragging = False
        current_point = -1
    
    # 鼠标移动：更新被拖动的点
    if dragging and current_point != -1:
        points[current_point] = (x, y)

def calculate_perspective():
    if len(points) != 4:
        return None, None
    
    # 源点顺序：左上、右上、右下、左下
    src_pts = np.array(points, dtype="float32")
    
    # 目标点顺序：左上、右上、右下、左下
    dst_pts = np.array([
        [0, 0],
        [dst_size[0]-1, 0],
        [dst_size[0]-1, dst_size[1]-1],
        [0, dst_size[1]-1]
    ], dtype="float32")
    
    # 计算变换矩阵
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return M, dst_pts

def main():
    global dst_size
    
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    # 创建窗口
    cv2.namedWindow("原始图像")
    cv2.namedWindow("透视变换")
    cv2.setMouseCallback("原始图像", mouse_callback)
    
    # 创建尺寸调整滑块
    def update_size(val):
        global dst_size
        dst_size = (cv2.getTrackbarPos("宽度", "透视变换"), 
                    cv2.getTrackbarPos("高度", "透视变换"))
    
    cv2.createTrackbar("宽度", "透视变换", dst_size[0], 800, update_size)
    cv2.createTrackbar("高度", "透视变换", dst_size[1], 600, update_size)
    
    print("使用说明:")
    print("1. 在原始图像窗口点击4个点（顺序：左上、右上、右下、左下）")
    print("2. 拖动点调整位置")
    print("3. 调整宽度/高度滑块改变输出尺寸")
    print("4. 按 's' 保存变换矩阵")
    print("5. 按 'r' 重置所有点")
    print("6. 按 'q' 退出")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取帧")
            break
        
        # 在原始图像上绘制点和连线
        display_frame = frame.copy()
        for i, pt in enumerate(points):
            cv2.circle(display_frame, (int(pt[0]), int(pt[1])), 8, (0, 0, 255), -1)
            cv2.putText(display_frame, str(i+1), (int(pt[0])+10, int(pt[1])+5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if len(points) >= 2:
            for i in range(len(points)):
                cv2.line(display_frame, 
                         (int(points[i][0]), int(points[i][1])),
                         (int(points[(i+1)%len(points)][0]), int(points[(i+1)%len(points)][1])),
                         (0, 255, 255), 2)
        
        # 计算并应用透视变换
        if len(points) == 4:
            M, dst_pts = calculate_perspective()
            if M is not None:
                warped = cv2.warpPerspective(frame, M, dst_size)
                
                # 在变换图像上绘制网格
                for i in range(0, warped.shape[1], 50):
                    cv2.line(warped, (i, 0), (i, warped.shape[0]), (0, 255, 0), 1)
                for i in range(0, warped.shape[0], 50):
                    cv2.line(warped, (0, i), (warped.shape[1], i), (0, 255, 0), 1)
                
                # 显示变换矩阵
                matrix_text = f"变换矩阵:"
                cv2.putText(warped, matrix_text, (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                for i, row in enumerate(M):
                    row_text = f"[{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}]"
                    cv2.putText(warped, row_text, (10, 40 + i*20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow("透视变换", warped)
        
        # 显示图像
        cv2.imshow("原始图像", display_frame)
        
        # 键盘控制
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 退出
            break
        elif key == ord('r'):  # 重置点
            points = []
        elif key == ord('s') and len(points) == 4:  # 保存矩阵
            M, _ = calculate_perspective()
            np.savetxt('perspective_matrix.txt', M)
            print("变换矩阵已保存为 perspective_matrix.txt")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()