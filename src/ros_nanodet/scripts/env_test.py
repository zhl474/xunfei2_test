import cv2
import numpy as np
import os
import time
# 1. 加载标定参数
def load_calibration(yaml_path):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    
    # 读取相机内参矩阵
    camera_matrix = fs.getNode("camera_matrix").mat()
    
    # 读取畸变系数 (注意：OpenCV需要5个参数，即使最后一个是0)
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    
    fs.release()
    return camera_matrix, dist_coeffs

# 2. 初始化视频流
def process_video_stream(yaml_path, video_source=0):
    # 加载标定参数
    camera_matrix, dist_coeffs = load_calibration(yaml_path)
    
    # 创建视频捕获对象
    cap = cv2.VideoCapture(video_source)
    h = 480
    w = 640
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)   # 匹配标定宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)  # 匹配标定高度
    if not cap.isOpened():
        print("无法打开视频源")
        return

    # 计算最优新相机矩阵
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w,h), 1, (w,h)
    )

    # 实时校正循环
    while True:
        start_time = time.time()  
        
        ret, frame = cap.read()
        if not ret:
            break

        # 执行畸变校正
        undistorted = cv2.undistort(
            frame, 
            camera_matrix, 
            dist_coeffs, 
            None, 
            new_camera_matrix
        )

        end_time = time.time()
        print(f'Time elapsed: {end_time - start_time:.2f} seconds')

        # 并排显示对比
        combined = np.hstack((frame, undistorted))
        cv2.putText(combined, "Original", (10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(combined, "Undistorted", (w//2+10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        
        cv2.imshow('Camera Calibration Demo', combined)

        # 按Q退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def debug_calibration(yaml_path):
    # 基础检查
    if not os.path.exists(yaml_path):
        print(f"错误：文件 {yaml_path} 不存在")
        return False
        
    if not os.access(yaml_path, os.R_OK):
        print(f"错误：文件无读取权限")
        return False

    # 尝试读取
    try:
        fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            print("错误：FileStorage打开失败")
            return False
            
        # 验证关键节点
        for node in ["camera_matrix", "distortion_coefficients"]:
            if fs.getNode(node).empty():
                print(f"错误：缺失 {node} 节点")
                return False
                
        print("文件结构验证通过")
        return True
        
    except Exception as e:
        print(f"异常: {str(e)}")
        return False
    finally:
        if 'fs' in locals():
            fs.release()

# 使用示例
if debug_calibration("/home/ucar/ucar_car/src/ros_nanodet/srv/head_camera800_600.yaml"):
    print("文件可正常读取")
else:
    print("请根据上述提示修正文件")

# 使用示例
if __name__ == "__main__":
    calibration_file = "/home/ucar/ucar_car/src/ros_nanodet/srv/head_camera800_600.yaml"  # YAML文件路径
    video_source = 0  # 0=默认摄像头，或视频文件路径
    
    process_video_stream(calibration_file, video_source)
    