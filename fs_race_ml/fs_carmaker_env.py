#!/usr/bin/env python3
import gym
from gym import spaces
import numpy as np
import socket
import struct
import json
import time
import cv2

class FSCarMakerEnv(gym.Env):
    """Formula Student CarMaker 强化学习环境"""
    
    def __init__(self, host="127.0.0.1", port=2212):
        super(FSCarMakerEnv, self).__init__()
        
        # 连接参数
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
        # 环境参数
        self.img_width = 224
        self.img_height = 224
        self.img_channels = 3
        
        # 定义动作空间 [转向, 油门, 刹车]
        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0, 0.0]),  # 左转最大, 无油门, 无刹车
            high=np.array([1.0, 1.0, 1.0]),  # 右转最大, 最大油门, 最大刹车
            dtype=np.float32
        )
        
        # 定义观察空间 (RGB图像)
        self.observation_space = spaces.Box(
            low=0,
            high=255,
            shape=(self.img_height, self.img_width, self.img_channels),
            dtype=np.uint8
        )
        
        # 连接到CarMaker
        self.connect()
        
        # 重置环境
        self.reset()
    
    def connect(self):
        """连接到CarMaker RL服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"已连接到CarMaker RL服务器: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接CarMaker RL服务器失败: {e}")
            self.connected = False
            return False
    
    def step(self, action):
        """执行一步，返回新状态、奖励、是否结束和额外信息"""
        if not self.connected and not self.connect():
            raise ConnectionError("未连接到CarMaker")
            
        try:
            # 发送动作命令
            cmd = {
                "cmd": "step",
                "steering": float(action[0]),
                "throttle": float(action[1]),
                "brake": float(action[2])
            }
            self.send_command(cmd)
            
            # 接收步骤结果
            obs, reward, done, info = self.receive_step_result()
            return obs, reward, done, info
            
        except Exception as e:
            print(f"执行步骤失败: {e}")
            self.connected = False
            raise
    
    def reset(self):
        """重置环境"""
        if not self.connected and not self.connect():
            raise ConnectionError("未连接到CarMaker")
            
        try:
            # 发送重置命令
            cmd = {"cmd": "reset"}
            self.send_command(cmd)
            
            # 接收初始观察
            obs = self.receive_observation()
            return obs
            
        except Exception as e:
            print(f"重置环境失败: {e}")
            self.connected = False
            raise
    
    def render(self, mode='human'):
        """渲染当前状态"""
        # CarMaker已经有自己的GUI，这里不需要额外实现
        pass
    
    def close(self):
        """关闭环境"""
        if self.connected:
            try:
                # 发送关闭命令
                cmd = {"cmd": "close"}
                self.send_command(cmd)
                
                # 关闭连接
                self.socket.close()
                self.connected = False
                print("已关闭CarMaker RL环境")
                
            except Exception as e:
                print(f"关闭环境失败: {e}")
    
    def send_command(self, cmd):
        """发送命令到CarMaker"""
        if not self.connected:
            return False
            
        try:
            # 编码命令为JSON
            cmd_json = json.dumps(cmd)
            
            # 发送命令大小和内容
            cmd_bytes = cmd_json.encode('utf-8')
            header = struct.pack("!I", len(cmd_bytes))
            self.socket.sendall(header + cmd_bytes)
            return True
            
        except Exception as e:
            print(f"发送命令失败: {e}")
            self.connected = False
            return False
    
    def receive_observation(self):
        """接收观察(图像)"""
        if not self.connected:
            return np.zeros((self.img_height, self.img_width, self.img_channels), dtype=np.uint8)
            
        try:
            # 接收图像头信息
            header = self.socket.recv(12)
            if len(header) < 12:
                raise ConnectionError("接收图像头信息失败")
                
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
                raise ConnectionError(f"图像数据不完整: {len(img_data)}/{img_size}")
            
            # 转换为numpy数组
            image = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, channels)
            
            # 调整大小
            if width != self.img_width or height != self.img_height:
                image = cv2.resize(image, (self.img_width, self.img_height))
            
            return image
            
        except Exception as e:
            print(f"接收观察失败: {e}")
            self.connected = False
            raise
    
    def receive_step_result(self):
        """接收步骤结果"""
        # 首先接收观察
        obs = self.receive_observation()
        
        try:
            # 接收结果大小
            size_data = self.socket.recv(4)
            if len(size_data) < 4:
                raise ConnectionError("接收结果大小失败")
                
            # 解析结果大小
            data_size = struct.unpack("!I", size_data)[0]
            
            # 接收结果数据
            result_data = bytearray()
            while len(result_data) < data_size:
                chunk = self.socket.recv(min(4096, data_size - len(result_data)))
                if not chunk:
                    break
                result_data.extend(chunk)
            
            if len(result_data) < data_size:
                raise ConnectionError(f"结果数据不完整: {len(result_data)}/{data_size}")
            
            # 解析结果
            result = json.loads(result_data.decode('utf-8'))
            
            # 提取结果
            reward = float(result.get("reward", 0.0))
            done = bool(result.get("done", False))
            info = result.get("info", {})
            
            return obs, reward, done, info
            
        except Exception as e:
            print(f"接收步骤结果失败: {e}")
            self.connected = False
            raise
