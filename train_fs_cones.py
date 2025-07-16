#!/usr/bin/env python3
from ultralytics import YOLO
import os
import torch
import gc

print("开始训练Formula Student锥桶检测模型...")

# 清理内存
if torch.cuda.is_available():
    torch.cuda.empty_cache()
    gc.collect()
    print(f"GPU: {torch.cuda.get_device_name(0)}")

# 检查配置文件
yaml_path = "FSCones/fs_cones.yaml"
if not os.path.exists(yaml_path):
    print(f"错误: 找不到配置文件 {yaml_path}")
    print("请先运行 generate_dataset_existing_track.py 创建数据集")
    exit(1)

# 加载模型
model = YOLO("yolov8n.yaml")  # 创建新模型

print(f"使用数据集配置: {yaml_path}")
print("开始训练Formula Student锥桶检测模型...")

# 训练模型
results = model.train(
    data=yaml_path,
    epochs=100,              # 训练轮数
    imgsz=640,               # 图像大小
    batch=16,                # 批量大小
    workers=8,               # 数据加载线程数
    device=0,                # 使用GPU
    patience=20,             # 提前停止耐心值
    project="FS_Detection",  # 项目名称
    name="cones_detect",     # 实验名称
    exist_ok=True,           # 覆盖现有实验
    pretrained=True,         # 使用预训练权重
    cos_lr=True,             # 余弦学习率调度
    close_mosaic=10,         # 最后10轮关闭马赛克增强
    augment=True,            # 使用数据增强
    dropout=0.1              # 使用Dropout
)

print("训练完成！模型保存在FS_Detection/cones_detect/weights/目录")
