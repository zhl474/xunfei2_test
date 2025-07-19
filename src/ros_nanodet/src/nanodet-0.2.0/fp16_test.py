# import torch
# import torch.nn as nn
# import time

# class FP16ModelWrapper(nn.Module):
#     """将模型包装为 FP16 推理模式"""
#     def __init__(self, model):
#         super(FP16ModelWrapper, self).__init__()
#         self.model = model.half()  # 将模型转换为 FP16
    
#     def forward(self, x):
#         # 确保输入数据也是 FP16
#         x = x.half()
#         with torch.no_grad():
#             return self.model(x)

# def convert_to_fp16(model_path, output_path):
#     """将模型转换为 FP16 并保存"""
#     # 加载原始模型
#     model = torch.load(model_path, map_location='cpu')
    
#     # 转换为 FP16
#     model_fp16 = model.half()
    
#     # 保存 FP16 模型
#     torch.save(model_fp16, output_path)
#     print(f"FP16 model saved to: {output_path}")

# def benchmark_fp16(model, input_size=(416, 416), iterations=100):
#     """基准测试 FP16 与 FP32 的性能差异"""
#     device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
#     # 创建输入张量
#     input_tensor = torch.randn(1, 3, *input_size).to(device)
    
#     # FP32 测试
#     model_fp32 = model.to(device).float()
#     model_fp32.eval()
    
#     # 预热
#     for _ in range(10):
#         _ = model_fp32(input_tensor)
    
#     # 计时
#     torch.cuda.synchronize()
#     start = time.time()
#     for _ in range(iterations):
#         _ = model_fp32(input_tensor)
#     torch.cuda.synchronize()
#     fp32_time = time.time() - start
    
#     # FP16 测试
#     model_fp16 = model.to(device).half()
#     model_fp16.eval()
    
#     # 预热
#     for _ in range(10):
#         _ = model_fp16(input_tensor.half())
    
#     # 计时
#     torch.cuda.synchronize()
#     start = time.time()
#     for _ in range(iterations):
#         _ = model_fp16(input_tensor.half())
#     torch.cuda.synchronize()
#     fp16_time = time.time() - start
    
#     # 打印结果
#     print(f"FP32 time: {fp32_time:.4f}s ({fp32_time/iterations:.6f}s per inference)")
#     print(f"FP16 time: {fp16_time:.4f}s ({fp16_time/iterations:.6f}s per inference)")
#     print(f"Speedup: {fp32_time/fp16_time:.2f}x")
    
#     return fp32_time, fp16_time

# # 加载配置和模型
# from nanodet.util import Logger, cfg, load_config, load_model_weight
# from nanodet.model.arch import build_model

# def main():
#     # 加载配置
#     config_path = "/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/config/nanodet-m-416-copy.yml"
#     model_path = "/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/workspace/nanodet_m_416/model_best/model_best.pth"
#     load_config(cfg, config_path)
    
#     # 构建模型
#     logger = Logger(-1, cfg.save_dir, False)
#     model = build_model(cfg.model)
    
#     # 加载权重
#     checkpoint = torch.load(model_path, map_location='cpu')
#     load_model_weight(model, checkpoint, logger)
    
#     # 转换为 FP16
#     model_fp16 = model.half()
    
#     # 保存 FP16 模型
#     fp16_model_path = "/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/workspace/nanodet_m_416/model_best/model_best_fp16.pth"
#     torch.save(model_fp16, fp16_model_path)
#     print(f"FP16 model saved to: {fp16_model_path}")
    
#     # 性能测试
#     print("\nRunning benchmark on Jetson Nano...")
#     benchmark_fp16(model, input_size=cfg.data.train.input_size)

# if __name__ == "__main__":
#     main()



















import torch
import time
import argparse

# 设置环境（确保CUDA可用且设备支持FP16）
assert torch.cuda.is_available(), "需要CUDA环境!"
device = torch.device("cuda")
torch.backends.cudnn.benchmark = True  # 启用cuDNN自动调优

# 参数配置
parser = argparse.ArgumentParser()
parser.add_argument("--size", type=int, default=416)    # 矩阵大小（默认8192x8192）
parser.add_argument("--iter", type=int, default=100)     # 测试迭代次数
args = parser.parse_args()

def benchmark(dtype, iters=args.iter):
    """基准测试函数"""
    # 创建大型随机矩阵
    # x = torch.randn(args.size, args.size, device=device, dtype=dtype)
    # y = torch.randn(args.size, args.size, device=device, dtype=dtype)
    # x = torch.randn(args.size, args.size, device=device, torch.float32)
    # y = torch.randn(args.size, args.size, device=device, torch.float16)
    # # 预热（排除首次启动延迟）
    # for _ in range(3):
    #     torch.mm(x, y)
    if (dtype==torch.float32):
        x = torch.randn(args.size, args.size, device=device, dtype=torch.float32)
        y = torch.randn(args.size, args.size, device=device, dtype=torch.float32)
        for _ in range(4):
            torch.mm(x, y)
        # 同步并计时
        torch.cuda.synchronize()
        start_time = time.time()
        
        for _ in range(iters):
            torch.mm(x, y)  # 执行矩阵乘法
        
        torch.cuda.synchronize()
        return time.time() - start_time
    else:
        x = torch.randn(args.size, args.size, device=device, dtype=torch.float32)
        y = torch.randn(args.size, args.size, device=device, dtype=torch.float32)
        for _ in range(4):
            torch.mm(x.half(), y.half())
        torch.cuda.synchronize()
        start_time = time.time()
        
        for _ in range(iters):
            torch.mm(x.half(), y.half())  # 执行矩阵乘法
        
        torch.cuda.synchronize()
        return time.time() - start_time

if __name__ == "__main__":
    # FP32测试
    fp32_time = benchmark(torch.float32)
    # FP16测试
    fp16_time = benchmark(torch.float16)
    
    # 打印结果
    speedup = fp32_time / fp16_time
    print(f"矩阵大小: {args.size}x{args.size} | 迭代次数: {args.iter}")
    print(f"FP32 耗时: {fp32_time:.4f} 秒")
    print(f"FP16 耗时: {fp16_time:.4f} 秒")
    print(f"FP16 速度提升: {speedup:.2f}x")