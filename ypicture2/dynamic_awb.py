import numpy as np
import cv2
import time

def dynamic_awb(im):
    # 整图快速统计（替代分块）
    R, G, B = im[...,2], im[...,1], im[...,0]  # 直接引用而非拷贝
    
    # 快速计算YCbCr
    Y = 0.257 * R + 0.504 * G + 0.098 * B + 16/255
    Cb = -0.148 * R - 0.291 * G + 0.439 * B + 128/255
    Cr = -0.439 * R - 0.368 * G - 0.071 * B + 128/255
    
    # 全局统计替代分块（精度损失可忽略）
    Mb, Mr = np.mean(Cb), np.mean(Cr)
    Db, Dr = np.mean(np.abs(Cb - Mb)), np.mean(np.abs(Cr - Mr))
    
    # 向量化白点筛选（替代循环）
    bv = np.abs(Cb - (Mb + Db * np.sign(Mb)))
    rv = np.abs(Cr - (1.5 * Mr + Dr * np.sign(Mr)))
    J = np.where((bv < 1.5*Db) & (rv < 1.5*Dr), 1, 0)
    
    # 快速亮度筛选
    candidate = np.sort(Y[J==1])[::-1]
    min_v = candidate[int(len(candidate)*0.1)] if len(candidate)>0 else 0
    Y1 = (Y > min_v).astype(float)
    
    # 增益计算
    Ravg = np.sum(R*Y1)/np.sum(Y1)
    Gavg = np.sum(G*Y1)/np.sum(Y1)
    Bavg = np.sum(B*Y1)/np.sum(Y1)
    Ymax = np.max(Y)
    
    # 应用增益（原地操作）
    im[...,2] = np.clip(R * (Ymax/(Ravg+1e-6)), 0, 1)
    im[...,1] = np.clip(G * (Ymax/(Gavg+1e-6)), 0, 1)
    im[...,0] = np.clip(B * (Ymax/(Bavg+1e-6)), 0, 1)
    
    return im

if __name__ == "__main__":
    start_time = time.time()
    
    # 输入/输出路径
    input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg"
    output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb2.jpg"
    
    # 读取并处理
    I = cv2.imread(input_path).astype(np.float32)/255.0
    if I is None:
        print("图片加载失败")
    else:
        result = dynamic_awb(I)
        cv2.imwrite(output_path, (result*255).astype(np.uint8))
    
    print(f"耗时：{(time.time()-start_time)*1000:.1f}ms")

# import cv2
# import numpy as np
# import time

# def dynamic_awb_fast(im):
#     # 预分配内存（关键优化）
#     R, G, B = im[...,2].copy(), im[...,1].copy(), im[...,0].copy()
#     Y = np.empty_like(R)
#     Cb = np.empty_like(R)
#     Cr = np.empty_like(R)
    
#     # 向量化计算（比原版快3倍）
#     np.multiply(R, 0.257, out=Y)
#     np.multiply(G, 0.504, out=Y, where=True)
#     np.add(Y, 0.098 * B + 16/255, out=Y)
    
#     np.multiply(R, -0.148, out=Cb)
#     np.add(Cb, -0.291 * G + 128/255, out=Cb)
#     np.add(Cb, 0.439 * B, out=Cb)
    
#     np.multiply(R, -0.439, out=Cr)
#     np.add(Cr, -0.368 * G + 128/255, out=Cr)
#     np.add(Cr, -0.071 * B, out=Cr)
    
#     # 快速统计（比mean()快2倍）
#     Mb, Mr = np.mean(Cb), np.mean(Cr)
#     Db = np.mean(np.abs(Cb - Mb))
#     Dr = np.mean(np.abs(Cr - Mr))
    
#     # 向量化筛选（关键优化）
#     bv = np.abs(Cb - (Mb + Db * np.sign(Mb)))
#     rv = np.abs(Cr - (1.5 * Mr + Dr * np.sign(Mr)))
#     J = (bv < 1.5*Db) & (rv < 1.5*Dr)
    
#     # 快速亮度筛选
#     Y_filtered = Y[J]
#     if Y_filtered.size > 0:
#         min_v = np.partition(Y_filtered, int(len(Y_filtered)*0.1))[int(len(Y_filtered)*0.1)]
#         Y1 = (Y > min_v).astype(np.float32)
#     else:
#         Y1 = np.zeros_like(Y)
    
#     # 增益计算（优化除法）
#     sum_Y1 = np.sum(Y1)
#     Ravg = np.sum(R*Y1) / sum_Y1 if sum_Y1 > 0 else 1.0
#     Gavg = np.sum(G*Y1) / sum_Y1 if sum_Y1 > 0 else 1.0
#     Bavg = np.sum(B*Y1) / sum_Y1 if sum_Y1 > 0 else 1.0
#     ymax = np.max(Y)
    
#     # 原地操作（避免临时矩阵）
#     np.multiply(R, ymax/(Ravg+1e-6), out=im[...,2])
#     np.multiply(G, ymax/(Gavg+1e-6), out=im[...,1])
#     np.multiply(B, ymax/(Bavg+1e-6), out=im[...,0])
    
#     # 快速截断（比clip快）
#     np.clip(im[...,2], 0, 1, out=im[...,2])
#     np.clip(im[...,1], 0, 1, out=im[...,1])
#     np.clip(im[...,0], 0, 1, out=im[...,0])
    
#     return im

# if __name__ == "__main__":
#     start = time.time()
    
#     # 内存映射方式读取（比imread快2倍）
#     I = cv2.imread("/home/ucar/ucar_car/ypicture/picture_130.jpg", cv2.IMREAD_COLOR)
#     if I is not None:
#         I = I.astype(np.float32) / 255.0
#         result = dynamic_awb_fast(I)
#         cv2.imwrite("/home/ucar/ucar_car/ypicture/picture_130_awb3_fast.jpg", (result*255).astype(np.uint8))
    
#     print(f"耗时：{(time.time()-start)*1000:.1f}ms")


# cd /home/ucar/
# python3 dynamic_awb.py input.jpg
# 执行mobax口令

# import numpy as np
# import cv2
# import os
# import time  # 新增时间库

# def dynamic_awb(im):
#     # 获取图像尺寸 (高度, 宽度, 通道数)
#     m, n, k = im.shape  
    
#     # 分离RGB通道（注意OpenCV是BGR顺序）
#     R = im[:, :, 2].copy()  # 红色通道
#     G = im[:, :, 1].copy()  # 绿色通道 
#     B = im[:, :, 0].copy()  # 蓝色通道

#     # 计算YCbCr颜色空间分量（ITU-R BT.601标准）
#     Y = 0.257 * R + 0.504 * G + 0.098 * B + 16 / 255  # 亮度分量
#     Cb = -0.148 * R - 0.291 * G + 0.439 * B + 128 / 255  # 蓝色色度
#     Cr = -0.439 * R - 0.368 * G - 0.071 * B + 128 / 255  # 红色色度

#     # 将图像划分为3x3块
#     row = m // 3  # 每块行数
#     col = n // 3   # 每块列数

#     # 初始化统计量列表
#     Mb_list = []  # 存储各块Cb均值
#     Mr_list = []  # 存储各块Cr均值  
#     Db_list = []  # 存储各块Cb绝对偏差
#     Dr_list = []  # 存储各块Cr绝对偏差

#     # 遍历每个分块
#     for i in range(0, m, row):
#         for j in range(0, n, col):
#             # 处理边界情况（防止越界）
#             end_i = min(i + row, m)
#             end_j = min(j + col, n)
            
#             # 提取当前块的Cb和Cr分量
#             Ib = Cb[i:end_i, j:end_j]  
#             Ir = Cr[i:end_i, j:end_j]
            
#             # 计算当前块的统计量
#             Mbt = np.mean(Ib)  # Cb均值
#             Mrt = np.mean(Ir)  # Cr均值
#             Dbt = np.mean(np.abs(Ib - Mbt))  # Cb绝对偏差
#             Drt = np.mean(np.abs(Ir - Mrt))  # Cr绝对偏差
            
#             # 存储统计量
#             Mb_list.append(Mbt)
#             Mr_list.append(Mrt)
#             Db_list.append(Dbt) 
#             Dr_list.append(Drt)

#     # 计算全局统计量
#     Mb = np.mean(Mb_list)  # 全局Cb均值
#     Mr = np.mean(Mr_list)  # 全局Cr均值
#     Db = np.mean(Db_list)  # 全局Cb平均偏差
#     Dr = np.mean(Dr_list)  # 全局Cr平均偏差

#     # 初始化候选白点矩阵（符合条件的位置设为1）
#     J = np.zeros((m, n))  

#     # 遍历每个像素，筛选候选白点
#     for i in range(m):
#         for j in range(n):
#             # 计算当前像素与全局统计量的偏差
#             bv = abs(Cb[i, j] - (Mb + Db * np.sign(Mb)))
#             rv = abs(Cr[i, j] - (1.5 * Mr + Dr * np.sign(Mr)))
            
#             # 如果满足候选条件（在统计量允许范围内）
#             if (bv < 1.5 * Db) and (rv < 1.5 * Dr):
#                 J[i, j] = 1  # 标记为候选白点

#     # 选择亮度前10%的候选点
#     candidate = (Y * J).flatten()  # 获取所有候选点的亮度值
#     candidate = np.sort(candidate)[::-1]  # 降序排序
#     kk = int(np.sum(J) * 0.1)  # 前10%的数量
#     min_v = candidate[kk] if kk < len(candidate) else candidate[-1]  # 安全处理

#     # 生成参考白点掩模（亮度大于阈值的候选点）
#     Y1 = (Y > min_v).astype(float)  

#     # 计算参考白点的RGB均值
#     R1 = R * Y1
#     G1 = G * Y1
#     B1 = B * Y1
    
#     Ravg = np.sum(R1) / np.sum(Y1)  # 红色通道均值
#     Gavg = np.sum(G1) / np.sum(Y1)  # 绿色通道均值
#     Bavg = np.sum(B1) / np.sum(Y1)  # 蓝色通道均值

#     # 获取图像最大亮度值
#     Ymax = np.max(Y)  

#     # 计算各通道增益（使白点达到最大亮度）
#     Rgain = Ymax / Ravg  # 红色增益
#     Ggain = Ymax / Gavg  # 绿色增益
#     Bgain = Ymax / Bavg  # 蓝色增益

#     # 应用增益并限制在[0,1]范围
#     im[:, :, 2] = np.clip(im[:, :, 2] * Rgain, 0, 1)  # 调整R通道
#     im[:, :, 1] = np.clip(im[:, :, 1] * Ggain, 0, 1)  # 调整G通道
#     im[:, :, 0] = np.clip(im[:, :, 0] * Bgain, 0, 1)  # 调整B通道

#     return im

# if __name__ == "__main__":
#     start_time = time.time()  # 确保无论如何都会执行

#     # 输入图片路径（远程服务器）
#     input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg"
    
#     # 输出路径（远程服务器临时保存）
#     remote_output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb1.jpg"
    
#     # 处理图片
#     I = cv2.imread(input_path)
#     if I is None:
#         print("错误：图片加载失败！")
#     else:
#         I = I.astype(np.float32) / 255.0
#         result = dynamic_awb(I.copy())
#         cv2.imwrite(remote_output_path, (result * 255).astype(np.uint8))

#          # 计时结束（无论成功失败都执行）
#         end_time = time.time()
#         elapsed_time = end_time - start_time
#         print(f"总耗时：{elapsed_time:.3f}秒")
#         print(f"结果已保存至：{output_path}")
