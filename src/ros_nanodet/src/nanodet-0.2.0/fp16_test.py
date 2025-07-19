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
    x = torch.randn(args.size, args.size, device=device, dtype=dtype)
    y = torch.randn(args.size, args.size, device=device, dtype=dtype)
    
    # 预热（排除首次启动延迟）
    for _ in range(3):
        torch.mm(x, y)
    
    # 同步并计时
    torch.cuda.synchronize()
    start_time = time.time()
    
    for _ in range(iters):
        torch.mm(x, y)  # 执行矩阵乘法
    
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