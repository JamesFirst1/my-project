from ultralytics import YOLO
import os
import torch
import gc

print("开始YOLOv8模型训练(极低内存版)...")

# 检查并清理GPU内存
if torch.cuda.is_available():
    torch.cuda.empty_cache()
    gc.collect()
    print(f"当前可用GPU: {torch.cuda.get_device_name(0)}")
    print(f"GPU内存总量: {torch.cuda.get_device_properties(0).total_memory/1024**2:.1f}MB")
    print(f"GPU内存状态: {torch.cuda.memory_reserved(0)/1024**2:.1f}MB 保留 / {torch.cuda.memory_allocated(0)/1024**2:.1f}MB 已分配")

# 检查配置文件
yaml_path = "VOCyolo/voc.yaml"
if not os.path.exists(yaml_path):
    print(f"错误: 找不到配置文件 {yaml_path}")
    exit(1)

# 加载模型
model = YOLO("yolov8n.yaml")  # 使用最小的yolov8n模型

# 设置极低内存训练配置
print(f"使用数据集配置: {yaml_path}")
print("开始训练过程(极低内存设置)...")

# 训练参数调整为极低内存需求
results = model.train(
    data=yaml_path,
    epochs=10,           # 训练10轮
    imgsz=320,           # 减小图像尺寸到320x320
    batch=1,             # 批次大小设为1
    workers=1,           # 数据加载线程为1
    verbose=True,        # 显示详细输出
    amp=False,           # 关闭混合精度训练
    half=False,          # 不使用半精度
    device=0,            # 使用GPU
    cache=True,          # 缓存图像以减少IO
    optimizer="SGD",     # 使用SGD优化器(内存占用小)
    lr0=0.001,           # 降低学习率
    lrf=0.01,            # 学习率衰减因子
    momentum=0.937,      # 动量
    weight_decay=0.0005, # 权重衰减
    warmup_epochs=1.0,   # 减少预热轮数
    close_mosaic=0,      # 关闭马赛克增强
    nbs=1,               # 标称批次大小
    overlap_mask=False,  # 减少mask重叠计算
    val=False,           # 禁用验证环节
    plots=False,         # 不保存图表
    save_period=5        # 每5轮保存一次模型
)

print("训练完成！模型保存在runs/detect/train/weights/目录")
