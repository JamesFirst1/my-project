#!/usr/bin/env python3
import os
import json
import cv2
import numpy as np
from tqdm import tqdm
import shutil

class FSConesDatasetGenerator:
    def __init__(self, data_dir="~/fs_race_data/training", output_dir="~/voc_detection/FSCones"):
        self.data_dir = os.path.expanduser(data_dir)
        self.images_dir = os.path.join(self.data_dir, "images")
        self.labels_dir = os.path.join(self.data_dir, "labels")
        
        self.output_dir = os.path.expanduser(output_dir)
        self.output_images = os.path.join(self.output_dir, "images")
        self.output_labels = os.path.join(self.output_dir, "labels")
        
        # 创建输出目录
        os.makedirs(self.output_images, exist_ok=True)
        os.makedirs(self.output_labels, exist_ok=True)
        
        # 类别定义
        self.classes = {
            "blue_cone": 0,
            "yellow_cone": 1, 
            "orange_cone": 2
        }
        
        # 图像投影变换矩阵
        # 这些参数需要根据您的摄像头视角调整
        self.camera_matrix = np.array([
            [600, 0, 320],
            [0, 600, 240],
            [0, 0, 1]
        ])
        
        print(f"数据集生成器已初始化。输入目录: {self.data_dir}, 输出目录: {self.output_dir}")
    
    def convert_to_yolo_format(self):
        """将CarMaker数据转换为YOLO格式"""
        # 获取所有图像文件
        image_files = sorted([f for f in os.listdir(self.images_dir) if f.endswith(".jpg")])
        
        print(f"发现 {len(image_files)} 个图像文件")
        
        # 处理每个图像和标签文件
        for i, img_file in enumerate(tqdm(image_files, desc="转换数据集")):
            # 图像路径
            img_path = os.path.join(self.images_dir, img_file)
            
            # 标签路径
            base_name = os.path.splitext(img_file)[0]
            label_path = os.path.join(self.labels_dir, f"{base_name}.json")
            
            if not os.path.exists(label_path):
                continue
            
            # 读取图像获取尺寸
            img = cv2.imread(img_path)
            if img is None:
                continue
                
            height, width, _ = img.shape
            
            # 读取标签数据
            with open(label_path, 'r') as f:
                label_data = json.load(f)
            
            # 提取锥桶数据
            cones_data = label_data.get("cones", {})
            
            # 获取车辆位置和朝向（用于坐标转换）
            car_x = label_data.get("road_deviation", 0)
            car_y = 0  # 假设车辆在赛道中心
            car_yaw = label_data.get("yaw_angle", 0)
            
            # 创建YOLO格式标签
            yolo_labels = []
            
            # 处理蓝色锥桶
            blue_cones = cones_data.get("blue_cones", [])
            for cone in blue_cones:
                bbox = self._world_to_image(cone, car_x, car_y, car_yaw, width, height)
                if bbox:
                    yolo_labels.append(f"0 {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}")
            
            # 处理黄色锥桶
            yellow_cones = cones_data.get("yellow_cones", [])
            for cone in yellow_cones:
                bbox = self._world_to_image(cone, car_x, car_y, car_yaw, width, height)
                if bbox:
                    yolo_labels.append(f"1 {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}")
            
            # 处理橙色锥桶
            orange_cones = cones_data.get("orange_cones", [])
            for cone in orange_cones:
                bbox = self._world_to_image(cone, car_x, car_y, car_yaw, width, height)
                if bbox:
                    yolo_labels.append(f"2 {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}")
            
            # 只保存有锥桶的图像
            if yolo_labels:
                # 保存图像
                output_img_path = os.path.join(self.output_images, img_file)
                shutil.copy(img_path, output_img_path)
                
                # 保存YOLO格式标签
                output_label_path = os.path.join(self.output_labels, f"{base_name}.txt")
                with open(output_label_path, 'w') as f:
                    f.write("\n".join(yolo_labels))
        
        # 创建数据集配置文件
        self._create_dataset_config()
        
        print(f"数据集转换完成! 输出目录: {self.output_dir}")
    
    def _world_to_image(self, cone_pos, car_x, car_y, car_yaw, img_width, img_height):
        """将世界坐标系中的锥桶位置转换为图像中的边界框"""
        # 提取锥桶世界坐标
        cone_x, cone_y = cone_pos
        
        # 计算相对于车辆的坐标
        dx = cone_x - car_x
        dy = cone_y - car_y
        
        # 根据车辆朝向旋转坐标
        yaw_rad = np.radians(car_yaw)
        rel_x = dx * np.cos(yaw_rad) + dy * np.sin(yaw_rad)
        rel_y = -dx * np.sin(yaw_rad) + dy * np.cos(yaw_rad)
        rel_z = 0  # 假设锥桶在地面上
        
        # 忽略在车辆后方的锥桶
        if rel_x < 0:
            return None
            
        # 计算距离（用于确定锥桶大小）
        distance = np.sqrt(rel_x**2 + rel_y**2)
        if distance > 50:  # 忽略太远的锥桶
            return None
            
        # 投影到图像平面
        if rel_x > 0:  # 避免除以零
            # 简化的投影计算
            img_x = (rel_y / rel_x * self.camera_matrix[0, 0] + self.camera_matrix[0, 2])
            img_y = self.camera_matrix[1, 2]  # 假设锥桶总是在地平面上
            
            # 根据距离确定锥桶大小
            cone_width = 100 / distance
            cone_height = 200 / distance
            
            # 创建边界框（相对大小）
            x_min = img_x - cone_width/2
            y_min = img_y - cone_height
            x_max = img_x + cone_width/2
            y_max = img_y
            
            # 检查边界框是否在图像内
            if (x_min < 0 and x_max < 0) or (x_min > img_width and x_max > img_width) or \
               (y_min < 0 and y_max < 0) or (y_min > img_height and y_max > img_height):
                return None
            
            # 裁剪超出图像的边界框部分
            x_min = max(0, min(img_width, x_min))
            y_min = max(0, min(img_height, y_min))
            x_max = max(0, min(img_width, x_max))
            y_max = max(0, min(img_height, y_max))
            
            # 转换为YOLO格式
            x_center = ((x_min + x_max) / 2) / img_width
            y_center = ((y_min + y_max) / 2) / img_height
            w = (x_max - x_min) / img_width
            h = (y_max - y_min) / img_height
            
            return x_center, y_center, w, h
        
        return None
    
    def _create_dataset_config(self):
        """创建YOLO数据集配置文件"""
        config_path = os.path.join(self.output_dir, "fs_cones.yaml")
        
        config_content = f"""# FS Cones数据集配置
path: {os.path.abspath(self.output_dir)}  # 数据集根目录
train: images  # 训练图像相对路径
val: images  # 验证图像相对路径（实际应用中应该拆分）

# 类别信息
nc: 3  # 类别数量
names: ['blue_cone', 'yellow_cone', 'orange_cone']  # 类别名称
"""
        
        with open(config_path, 'w') as f:
            f.write(config_content)
        
        print(f"数据集配置文件已保存: {config_path}")

if __name__ == "__main__":
    generator = FSConesDatasetGenerator()
    generator.convert_to_yolo_format()
