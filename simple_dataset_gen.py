#!/usr/bin/env python3
import os
import shutil
import glob
import cv2
import numpy as np
import json
import random
from tqdm import tqdm

# 设置路径
input_dir = os.path.expanduser("~/fs_race_data/training")
output_dir = os.path.expanduser("~/voc_detection/FSCones")
os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)

print(f"开始创建数据集。源目录: {input_dir}, 目标目录: {output_dir}")

# 查找所有图像
images = sorted(glob.glob(os.path.join(input_dir, "images", "*.jpg")))
print(f"找到 {len(images)} 张图像")

if len(images) == 0:
    print("错误: 没有找到图像数据！请先运行数据收集脚本。")
    exit(1)

# 使用80%的图像作为训练集
train_count = int(len(images) * 0.8)
train_images = sorted(random.sample(images, train_count))

# 处理每个图像
for img_path in tqdm(train_images, desc="处理训练图像"):
    # 复制图像
    dest_path = os.path.join(output_dir, "images", os.path.basename(img_path))
    shutil.copy(img_path, dest_path)
    
    # 获取对应的JSON标签
    base_name = os.path.splitext(os.path.basename(img_path))[0]
    json_path = os.path.join(input_dir, "labels", f"{base_name}.json")
    
    # 读取图像获取尺寸
    img = cv2.imread(img_path)
    if img is None:
        continue
    height, width, _ = img.shape
    
    # 如果JSON存在，提取锥桶信息
    if os.path.exists(json_path):
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        # 创建YOLO格式标签文件
        yolo_labels = []
        
        # 处理锥桶数据
        cones_data = data.get("cones", {})
        
        # 处理蓝色锥桶
        blue_cones = cones_data.get("blue_cones", [])
        for cone in blue_cones:
            # 生成简单的YOLO标签 (类别0=蓝色锥桶)
            # 假设坐标是比例位置
            x, y = cone
            
            # 转换世界坐标到图像坐标（简化版本）
            # 调整这些参数以匹配您的摄像头视角
            img_x = width/2 + x * 50  # 简单缩放
            img_y = height/2 - y * 50  # 简单缩放
            
            # 确保在图像范围内
            if 0 < img_x < width and 0 < img_y < height:
                # 锥桶大小
                cone_w = 30  # 像素宽度
                cone_h = 60  # 像素高度
                
                # 转换为YOLO格式 (x_center, y_center, width, height)
                x_center = img_x / width
                y_center = img_y / height
                w = cone_w / width
                h = cone_h / height
                
                yolo_labels.append(f"0 {x_center} {y_center} {w} {h}")
        
        # 处理黄色锥桶
        yellow_cones = cones_data.get("yellow_cones", [])
        for cone in yellow_cones:
            x, y = cone
            img_x = width/2 + x * 50
            img_y = height/2 - y * 50
            
            if 0 < img_x < width and 0 < img_y < height:
                cone_w = 30
                cone_h = 60
                
                x_center = img_x / width
                y_center = img_y / height
                w = cone_w / width
                h = cone_h / height
                
                yolo_labels.append(f"1 {x_center} {y_center} {w} {h}")
        
        # 处理橙色锥桶
        orange_cones = cones_data.get("orange_cones", [])
        for cone in orange_cones:
            x, y = cone
            img_x = width/2 + x * 50
            img_y = height/2 - y * 50
            
            if 0 < img_x < width and 0 < img_y < height:
                cone_w = 30
                cone_h = 60
                
                x_center = img_x / width
                y_center = img_y / height
                w = cone_w / width
                h = cone_h / height
                
                yolo_labels.append(f"2 {x_center} {y_center} {w} {h}")
        
        # 保存YOLO格式标签
        if yolo_labels:
            with open(os.path.join(output_dir, "labels", f"{base_name}.txt"), "w") as f:
                f.write("\n".join(yolo_labels))
        else:
            # 如果没有有效锥桶，创建一个假的标签以便测试训练流程
            fake_label = "0 0.5 0.5 0.1 0.2"  # 一个中央的蓝色锥桶
            with open(os.path.join(output_dir, "labels", f"{base_name}.txt"), "w") as f:
                f.write(fake_label)

# 创建YAML配置文件
with open(os.path.join(output_dir, "fs_cones.yaml"), "w") as f:
    f.write(f"""# FS Cones数据集配置
path: {os.path.abspath(output_dir)}  # 数据集根目录
train: images  # 训练图像相对路径
val: images  # 验证图像相对路径

# 类别信息
nc: 3  # 类别数量
names: ['blue_cone', 'yellow_cone', 'orange_cone']  # 类别名称
""")

print(f"数据集创建完成！配置文件保存在: {os.path.join(output_dir, 'fs_cones.yaml')}")
