#! /usr/bin/env python

import cv2
import numpy as np
import sys
import os

# 硬编码添加 build 目录到 Python 路径
build_dir = "/home/ucar/ucar_car/src/wb_cpp/build"
sys.path.insert(0, build_dir)
print(f"添加构建路径: {build_dir}")

try:
    import whitebalance
    print("模块导入成功!")
except ImportError as e:
    print(f"模块导入失败: {e}")
    sys.exit(1)

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
# frame_size = (416, 416)  # 必须和实际帧尺寸一致
frame_size = (832, 416)
out = cv2.VideoWriter(output_filename, fourcc, fps, frame_size)
# 检查是否成功创建
if not out.isOpened():
    print("无法创建视频文件！")
while judge:
    ret, frame = video.read()
    resize_img = resize_with_padding(frame, (416, 416))
    resize_whitebalanc = whitebalance.process(resize_img)
    hconcat_img = cv2.hconcat([resize_img, resize_whitebalanc])
    cv2.imshow("frame", resize_img)
    keyword = cv2.waitKey(1)
    if keyword == ord('s'):      # 按s保存当前图片
        i += 1
        imgname = "/home/ucar/ucar_car/picture_"+str(i)+".jpg"
        cv2.imwrite(imgname, resize_img)
        print(str(i))
    elif keyword == ord('q'):     # 按q退出
        break

    out.write(hconcat_img)
    #elif i == 10:     # 按q退出
    #   break

# 释放窗口
video.release()
cv2.destroyAllWindows()
out.release()


# import cv2
# import numpy as np
# import sys
# import os
# import time

# # 硬编码添加 build 目录到 Python 路径
# build_dir = "/home/ucar/ucar_car/src/wb_cpp/build"
# sys.path.insert(0, build_dir)
# print(f"添加构建路径: {build_dir}")

# try:
#     import whitebalance
#     print("白平衡模块导入成功!")
# except ImportError as e:
#     print(f"白平衡模块导入失败: {e}")
#     sys.exit(1)

# def create_side_by_side(orig, processed):
#     """
#     创建左右对比图像
    
#     Args:
#         orig (np.ndarray): 原始图像
#         processed (np.ndarray): 处理后的图像
    
#     Returns:
#         np.ndarray: 拼接后的对比图像
#     """
#     # 确保两个图像尺寸相同
#     if orig.shape != processed.shape:
#         h, w = orig.shape[:2]
#         processed = cv2.resize(processed, (w, h))
    
#     # 水平拼接
#     combined = np.hstack((orig, processed))
    
#     # 添加文字标注
#     cv2.putText(combined, "原始图像", (50, 40), 
#                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#     cv2.putText(combined, "白平衡后", (50 + w, 40), 
#                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
#     return combined

# def resize_with_padding(image, target_size):
#     """
#     按比例缩放图像并在多余部分填充黑色
    
#     Args:
#         image (np.ndarray): 输入图像
#         target_size (tuple): 目标尺寸 (width, height)
    
#     Returns:
#         np.ndarray: 缩放并填充后的图像
#     """
#     # 获取原始图像尺寸
#     h, w = image.shape[:2]
#     target_w, target_h = target_size
    
#     # 计算缩放比例
#     scale = min(target_w / w, target_h / h)
    
#     # 计算缩放后的新尺寸
#     new_w = int(w * scale)
#     new_h = int(h * scale)
    
#     # 缩放图像
#     resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
#     # 创建新的画布（黑色背景）
#     canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    
#     # 计算粘贴位置（居中）
#     x_offset = (target_w - new_w) // 2
#     y_offset = (target_h - new_h) // 2
    
#     # 将缩放后的图像粘贴到画布中心
#     canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
    
#     return canvas

# def main():
#     # 初始化摄像头
#     video = cv2.VideoCapture('/dev/video0')
#     if not video.isOpened():
#         print("无法打开摄像头!")
#         return
    
#     print(f"摄像头分辨率: {video.get(cv2.CAP_PROP_FRAME_WIDTH)}x{video.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
#     print(f"摄像头FPS: {video.get(cv2.CAP_PROP_FPS)}")
    
#     # 创建输出视频文件
#     output_filename = "/home/ucar/ucar_car/src/ztestnav2025/nanodet_debug/whitebalance_comparison.avi"
#     fourcc = cv2.VideoWriter_fourcc(*'XVID')
#     fps = 10.0
#     frame_size = (832, 416)  # 左右对比图像尺寸
    
#     # 创建视频写入器
#     out = cv2.VideoWriter(output_filename, fourcc, fps, frame_size)
#     if not out.isOpened():
#         print("无法创建视频文件!")
#         return
    
#     print(f"正在录制视频: {output_filename} [按q退出]")
    
#     # 创建性能统计变量
#     frame_count = 0
#     start_time = time.time()
    
#     # 图像处理循环
#     while True:
#         ret, frame = video.read()
#         if not ret:
#             print("无法读取视频帧")
#             break
        
#         # 原始帧备份（用于对比）
#         orig_frame = frame.copy()
        
#         # 应用白平衡处理
#         try:
#             # 处理前的时间戳
#             start_process = time.time()
            
#             # 应用白平衡
#             processed = whitebalance.process(frame)
            
#             # 处理后时间戳
#             proc_time = time.time() - start_process
            
#             # 调整两个图像的大小为416x416
#             orig_resized = resize_with_padding(orig_frame, (416, 416))
#             proc_resized = resize_with_padding(processed, (416, 416))
            
#             # 创建左右对比图像
#             combined = create_side_by_side(orig_resized, proc_resized)
            
#             # 性能统计
#             frame_count += 1
#             current_fps = frame_count / (time.time() - start_time)
            
#             # 在图像上添加性能信息
#             fps_text = f"白平衡耗时: {proc_time:.3f}ms | FPS: {current_fps:.1f}"
#             cv2.putText(combined, fps_text, (10, combined.shape[0] - 20), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
#         except Exception as e:
#             print(f"白平衡处理错误: {e}")
#             combined = resize_with_padding(orig_frame, (832, 416))
        
#         # 显示对比图像
#         cv2.imshow("白平衡效果对比", combined)
        
#         # 写入视频文件
#         out.write(combined)
        
#         # 按键处理
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('s'):
#             img_name = f"/home/ucar/ucar_car/picture_{int(time.time())}.jpg"
#             cv2.imwrite(img_name, combined)
#             print(f"已保存截图: {img_name}")
#         elif key == ord('q'):
#             break
    
#     # 清理资源
#     video.release()
#     out.release()
#     cv2.destroyAllWindows()
    
#     # 输出统计信息
#     total_time = time.time() - start_time
#     print(f"\n录制结束，共处理 {frame_count} 帧")
#     print(f"总时间: {total_time:.2f}秒 | 平均FPS: {frame_count/total_time:.2f}")
#     print(f"输出视频已保存到: {output_filename}")

# if __name__ == "__main__":
#     main()