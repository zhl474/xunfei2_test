import torch

if torch.cuda.get_device_capability(0)[0] >= 5:  # 计算能力 ≥5.0
    print("支持 FP16 计算")
else:
    print("不支持 FP16")