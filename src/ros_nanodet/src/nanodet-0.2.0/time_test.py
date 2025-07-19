import time
import torch
from nanodet.util import cfg, load_config
from nanodet.data.dataset import build_dataset
from nanodet.model.arch import build_model

def main():
    # 加载配置
    config_path = "/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/config/nanodet-m-416-copy.yml"
    model_path = "/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/workspace/nanodet_m_416/model_best/model_best.pth"
    load_config(cfg, config_path)
    
    # 构建数据集
    dataset = build_dataset(cfg.data.val, "val")
    data_loader = torch.utils.data.DataLoader(
        dataset, batch_size=1, shuffle=False, num_workers=0
    )
    
    # 构建模型
    model = build_model(cfg.model)
    checkpoint = torch.load(model_path, map_location="cpu")
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    
    # 预热
    dummy_input = torch.randn(1, 3, 416, 416)
    for _ in range(3):
        _ = model(dummy_input)
    
    # 时间分析
    preprocess_times = []
    inference_times = []
    postprocess_times = []
    total_times = []
    
    for i, meta in enumerate(data_loader):
        # 总时间
        total_start = time.perf_counter()
        
        # 预处理
        pre_start = time.perf_counter()
        img_tensor = meta["img"]
        pre_end = time.perf_counter()
        preprocess_times.append(pre_end - pre_start)
        
        # 推理
        inf_start = time.perf_counter()
        with torch.no_grad():
            preds = model(img_tensor)
        inf_end = time.perf_counter()
        inference_times.append(inf_end - inf_start)
        
        # 后处理
        post_start = time.perf_counter()
        results = model.head.post_process(preds, meta)
        post_end = time.perf_counter()
        postprocess_times.append(post_end - post_start)
        
        # 总时间
        total_end = time.perf_counter()
        total_times.append(total_end - total_start)
        
        # 每10张打印一次
        if i % 10 == 0 and i > 0:
            print(f"已处理 {i} 张图像")
            print(f"预处理: {sum(preprocess_times[-10:])/10 * 1000:.2f}ms")
            print(f"推理: {sum(inference_times[-10:])/10 * 1000:.2f}ms")
            print(f"后处理: {sum(postprocess_times[-10:])/10 * 1000:.2f}ms")
            print(f"总时间: {sum(total_times[-10:])/10 * 1000:.2f}ms")
    
    # 最终报告
    print("\n=== 最终时间分析 ===")
    print(f"平均预处理时间: {sum(preprocess_times)/len(preprocess_times)*1000:.2f}ms")
    print(f"平均推理时间: {sum(inference_times)/len(inference_times)*1000:.2f}ms")
    print(f"平均后处理时间: {sum(postprocess_times)/len(postprocess_times)*1000:.2f}ms")
    print(f"平均总时间: {sum(total_times)/len(total_times)*1000:.2f}ms")
    print(f"预估 FPS: {1/(sum(total_times)/len(total_times)):.2f}")

if __name__ == "__main__":
    main()