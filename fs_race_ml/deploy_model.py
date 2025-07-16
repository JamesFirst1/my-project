#!/usr/bin/env python3
import os
import sys
import numpy as np
import tensorflow as tf
import time
import socket
import struct
import json
import argparse
import cv2
from tensorflow.keras.models import load_model
from stable_baselines3 import PPO

class FSModelDriver:
    def __init__(self, model_path, model_type="sl", host="127.0.0.1", port=2214):
        # 设置连接参数
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
        # 图像处理参数
        self.img_width = 224
        self.img_height = 224
        
        # 加载模型
        self.model_type = model_type
        self.model = self.load_model(model_path)
        
        print(f"已加载{model_type}模型: {model_path}")
        
        # 连接到CarMaker
        self.connect()
    
    def load_model(self, model_path):
        """根据类型加载模型"""
        if self.model_type == "sl":
            # 监督学习模型 (TF/Keras)
            return load_model(model_path)
        elif self.model_type == "rl":
            # 强化学习模型 (Stable Baselines)
            return PPO.load(model_path)
        else:
            raise ValueError(f"不支持的模型类型: {self.model_type}")
    
    def connect(self):
        """连接到CarMaker仿真器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"已连接到CarMaker: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接CarMaker失败: {e}")
            self.connected = False
            return False
    
    def receive_image(self):
        """接收图像"""
        if not self.connected:
            return None
        
        try:
            # 接收图像头信息
            header = self.socket.recv(12)
            if len(header) < 12:
                return None
            
            # 解析图像尺寸
            width, height, channels = struct.unpack("!III", header)
            
            # 接收图像数据
            img_size = width * height * channels
            img_data = bytearray()
            
            while len(img_data) < img_size:
                chunk = self.socket.recv(min(4096, img_size - len(img_data)))
                if not chunk:
                    break
                img_data.extend(chunk)
            
            if len(img_data) < img_size:
                return None
            
            # 转换为numpy数组
            image = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, channels)
            
            # 调整大小并归一化
            image = cv2.resize(image, (self.img_width, self.img_height))
            image = image / 255.0
            
            return image
            
        except Exception as e:
            print(f"接收图像失败: {e}")
            self.connected = False
            return None
    
    def send_control(self, steering, throttle, brake):
        """发送控制命令"""
        if not self.connected:
            return False
            
        try:
            # 创建控制命令
            control = {
                "steering": float(steering),
                "throttle": float(throttle),
                "brake": float(brake)
            }
            
            # 编码为JSON
            control_json = json.dumps(control)
            
            # 发送命令大小和内容
            size = len(control_json)
            header = struct.pack("!I", size)
            self.socket.sendall(header + control_json.encode('utf-8'))
            
            return True
            
        except Exception as e:
            print(f"发送控制命令失败: {e}")
            self.connected = False
            return False
    
    def predict_sl_control(self, image):
        """使用监督学习模型预测控制"""
        # 添加批次维度
        img_batch = np.expand_dims(image, axis=0)
        
        # 预测控制命令 [转向, 油门, 刹车]
        controls = self.model.predict(img_batch)[0]
        
        steering = controls[0]  # 转向(-1到1)
        throttle = max(0.0, min(1.0, controls[1]))  # 油门(0到1)
        brake = max(0.0, min(1.0, controls[2]))  # 刹车(0到1)
        
        return steering, throttle, brake
    
    def predict_rl_control(self, image):
        """使用强化学习模型预测控制"""
        # 预测动作
        action, _ = self.model.predict(image, deterministic=True)
        
        # 解包动作
        if len(action) == 3:
            # [转向, 油门, 刹车]
            steering, throttle, brake = action
        else:
            # [转向, 油/刹车合并]
            steering = action[0]
            if action[1] >= 0:
                throttle = action[1]
                brake = 0.0
            else:
                throttle = 0.0
                brake = -action[1]
        
        # 确保值在正确范围
        steering = max(-1.0, min(1.0, steering))
        throttle = max(0.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))
        
        return steering, throttle, brake
    
    def run(self):
        """运行自动驾驶模型"""
        print("开始自动驾驶...")
        
        try:
            while True:
                # 接收图像
                image = self.receive_image()
                if image is None:
                    print("接收图像失败，尝试重新连接...")
                    time.sleep(1)
                    self.connect()
                    continue
                
                # 预测控制命令
                start_time = time.time()
                
                if self.model_type == "sl":
                    steering, throttle, brake = self.predict_sl_control(image)
                else:
                    steering, throttle, brake = self.predict_rl_control(image)
                
                # 测量推理时间
                inference_time = (time.time() - start_time) * 1000  # 毫秒
                
                # 发送控制命令
                self.send_control(steering, throttle, brake)
                
                # 显示控制信息
                print(f"\r转向: {steering:.4f}, 油门: {throttle:.4f}, 刹车: {brake:.4f}, 推理时间: {inference_time:.1f}ms", end="")
                
        except KeyboardInterrupt:
            print("\n自动驾驶已停止")
        finally:
            if self.socket:
                self.socket.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="部署自动驾驶模型")
    parser.add_argument("--model", type=str, required=True, help="模型路径")
    parser.add_argument("--type", type=str, default="sl", choices=["sl", "rl"], help="模型类型(sl=监督学习, rl=强化学习)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="CarMaker主机")
    parser.add_argument("--port", type=int, default=2214, help="CarMaker端口")
    
    args = parser.parse_args()
    
    driver = FSModelDriver(args.model, args.type, args.host, args.port)
    driver.run()
