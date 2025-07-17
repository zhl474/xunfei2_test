import numpy as np
import cv2
import os

def dynamic_awb(im):
    # 获取图像尺寸 (高度, 宽度, 通道数)
    m, n, k = im.shape  
    
    # 分离RGB通道（注意OpenCV是BGR顺序）
    R = im[:, :, 2].copy()  # 红色通道
    G = im[:, :, 1].copy()  # 绿色通道 
    B = im[:, :, 0].copy()  # 蓝色通道

    # 计算YCbCr颜色空间分量（ITU-R BT.601标准）
    Y = 0.257 * R + 0.504 * G + 0.098 * B + 16 / 255  # 亮度分量
    Cb = -0.148 * R - 0.291 * G + 0.439 * B + 128 / 255  # 蓝色色度
    Cr = -0.439 * R - 0.368 * G - 0.071 * B + 128 / 255  # 红色色度

    # 将图像划分为3x3块
    row = m // 3  # 每块行数
    col = n // 3   # 每块列数

    # 初始化统计量列表
    Mb_list = []  # 存储各块Cb均值
    Mr_list = []  # 存储各块Cr均值  
    Db_list = []  # 存储各块Cb绝对偏差
    Dr_list = []  # 存储各块Cr绝对偏差

    # 遍历每个分块
    for i in range(0, m, row):
        for j in range(0, n, col):
            # 处理边界情况（防止越界）
            end_i = min(i + row, m)
            end_j = min(j + col, n)
            
            # 提取当前块的Cb和Cr分量
            Ib = Cb[i:end_i, j:end_j]  
            Ir = Cr[i:end_i, j:end_j]
            
            # 计算当前块的统计量
            Mbt = np.mean(Ib)  # Cb均值
            Mrt = np.mean(Ir)  # Cr均值
            Dbt = np.mean(np.abs(Ib - Mbt))  # Cb绝对偏差
            Drt = np.mean(np.abs(Ir - Mrt))  # Cr绝对偏差
            
            # 存储统计量
            Mb_list.append(Mbt)
            Mr_list.append(Mrt)
            Db_list.append(Dbt) 
            Dr_list.append(Drt)

    # 计算全局统计量
    Mb = np.mean(Mb_list)  # 全局Cb均值
    Mr = np.mean(Mr_list)  # 全局Cr均值
    Db = np.mean(Db_list)  # 全局Cb平均偏差
    Dr = np.mean(Dr_list)  # 全局Cr平均偏差

    # 初始化候选白点矩阵（符合条件的位置设为1）
    J = np.zeros((m, n))  

    # 遍历每个像素，筛选候选白点
    for i in range(m):
        for j in range(n):
            # 计算当前像素与全局统计量的偏差
            bv = abs(Cb[i, j] - (Mb + Db * np.sign(Mb)))
            rv = abs(Cr[i, j] - (1.5 * Mr + Dr * np.sign(Mr)))
            
            # 如果满足候选条件（在统计量允许范围内）
            if (bv < 1.5 * Db) and (rv < 1.5 * Dr):
                J[i, j] = 1  # 标记为候选白点

    # 选择亮度前10%的候选点
    candidate = (Y * J).flatten()  # 获取所有候选点的亮度值
    candidate = np.sort(candidate)[::-1]  # 降序排序
    kk = int(np.sum(J) * 0.1)  # 前10%的数量
    min_v = candidate[kk] if kk < len(candidate) else candidate[-1]  # 安全处理

    # 生成参考白点掩模（亮度大于阈值的候选点）
    Y1 = (Y > min_v).astype(float)  

    # 计算参考白点的RGB均值
    R1 = R * Y1
    G1 = G * Y1
    B1 = B * Y1
    
    Ravg = np.sum(R1) / np.sum(Y1)  # 红色通道均值
    Gavg = np.sum(G1) / np.sum(Y1)  # 绿色通道均值
    Bavg = np.sum(B1) / np.sum(Y1)  # 蓝色通道均值

    # 获取图像最大亮度值
    Ymax = np.max(Y)  

    # 计算各通道增益（使白点达到最大亮度）
    Rgain = Ymax / Ravg  # 红色增益
    Ggain = Ymax / Gavg  # 绿色增益
    Bgain = Ymax / Bavg  # 蓝色增益

    # 应用增益并限制在[0,1]范围
    im[:, :, 2] = np.clip(im[:, :, 2] * Rgain, 0, 1)  # 调整R通道
    im[:, :, 1] = np.clip(im[:, :, 1] * Ggain, 0, 1)  # 调整G通道
    im[:, :, 0] = np.clip(im[:, :, 0] * Bgain, 0, 1)  # 调整B通道

    return im

if __name__ == "__main__":
    # 替换为你的图片路径（注意转义或使用原始字符串）
    input_path = r"C:\Users\10850\Desktop\picture_77.jpg"  # 使用原始字符串避免转义
    # 或
    # input_path = "C:\\Users\\10850\\Desktop\\picture_77.jpg"  # 双反斜杠转义

    # 读取图像（确保路径存在）
    if not os.path.exists(input_path):
        print(f"错误：图片路径不存在 - {input_path}")
    else:
        I = cv2.imread(input_path)
        if I is None:
            print(f"错误:图片加载失败,请检查格式(支持JPEG/PNG/BMP等)")
        else:
            # 归一化并处理
            I_normalized = I.astype(np.float32) / 255.0
            result = dynamic_awb(I_normalized.copy())
            
            # 显示结果（Windows需确保OpenCV GUI支持）
            cv2.imshow("Original Image", I)
            cv2.imshow("AWB Result", (result * 255).astype(np.uint8))
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            # 保存结果到桌面
            output_path = os.path.join(os.path.expanduser("~"), "Desktop", "awb_result.jpg")
            cv2.imwrite(output_path, (result * 255).astype(np.uint8))
            print(f"白平衡结果已保存至：{output_path}")
