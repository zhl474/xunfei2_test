import cv2
import time

def optimized_capture():
    # 使用更稳健的设置方法
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    
    # 尝试设置首选参数组合
    if not cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')):
        print("MJPG格式不支持，尝试YUYV")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    
    # 安全分辨率设置
    for width, height in [(640, 480), (320, 240), (1280, 720)]:
        if cap.set(cv2.CAP_PROP_FRAME_WIDTH, width) and \
           cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height):
            print(f"成功设置分辨率: {width}x{height}")
            break
    else:
        print("无法设置分辨率，使用默认值")
    
    # 关键延迟优化
    cap.set(cv2.CAP_PROP_FPS,60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    
    # 打印实际参数
    print(f"实际分辨率: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"实际帧率: {cap.get(cv2.CAP_PROP_FPS)}")
    print(f"实际缓冲区: {cap.get(cv2.CAP_PROP_BUFFERSIZE)}")
    
    # 预热
    for _ in range(5): cap.read()
    
    # 测试
    delays = []
    for i in range(10):
        start = time.perf_counter()
        ret, frame = cap.read()
        if not ret: break
        latency = (time.perf_counter() - start) * 1000
        delays.append(latency)
        print(f"捕获延迟: {latency:.2f}ms")
    
    cap.release()
    return delays

if __name__ == "__main__":
    print("=== 紧急优化测试 ===")
    results = optimized_capture()
    if results:
        avg = sum(results) / len(results)
        print(f"\n最终平均延迟: {avg:.2f}ms")