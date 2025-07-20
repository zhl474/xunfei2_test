import cv2
import sys
import os

# 添加 build 目录到 Python 路径
build_dir = os.path.join(os.path.dirname(__file__), 'build')
sys.path.insert(0, build_dir)
import whitebalance

# 读取图像
img = cv2.imread("/home/ucar/ucar_car/missing_76.jpg")  # 替换为你的测试图片路径
if img is None:
    print("无法读取图像，请检查路径是否正确")
else:
    # 调用白平衡处理函数
    result = whitebalance.process(img)
    
    # 显示原图和处理后的结果
    cv2.imshow("Original", img)
    cv2.imshow("White Balanced", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 保存处理结果
    cv2.imwrite("result.png", result)
    print("处理结果已保存为 result.png")


    