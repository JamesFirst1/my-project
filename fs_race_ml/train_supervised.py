#!/usr/bin/env python3
import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, Input, GlobalAveragePooling2D
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, TensorBoard
from tensorflow.keras.optimizers import Adam
import matplotlib.pyplot as plt
import cv2
import json
from sklearn.model_selection import train_test_split
from datetime import datetime
import random

class FSAutonomousDriver:
    def __init__(self, data_dir="~/fs_race_data/training"):
        # 设置路径
        self.data_dir = os.path.expanduser(data_dir)
        self.images_dir = os.path.join(self.data_dir, "images")
        self.labels_dir = os.path.join(self.data_dir, "labels")
        self.models_dir = os.path.join(os.path.dirname(self.data_dir), "models")
        
        # 创建模型目录
        os.makedirs(self.models_dir, exist_ok=True)
        
        # 图像参数
        self.img_width = 224
        self.img_height = 224
        self.img_channels = 3
        
        print(f"初始化自动驾驶模型训练器, 数据路径: {self.data_dir}")
    
    def load_data(self, max_samples=20000):
        """加载训练数据"""
        print("加载训练数据...")
        
        # 获取图像文件列表
        image_files = [f for f in os.listdir(self.images_dir) if f.endswith(".jpg")]
        
        # 随机抽样(如果图像太多)
        if len(image_files) > max_samples:
            print(f"随机抽样 {max_samples}/{len(image_files)} 图像")
            image_files = random.sample(image_files, max_samples)
        else:
            print(f"加载所有 {len(image_files)} 图像")
        
        # 初始化数据容器
        X = []  # 图像数据
        Y = []  # 标签数据 [转向角, 油门, 刹车]
        
        # 加载数据
        for i, img_file in enumerate(image_files):
            if i % 500 == 0:
                print(f"处理: {i}/{len(image_files)}")
            
            # 图像路径
            img_path = os.path.join(self.images_dir, img_file)
            
            # 标签路径
            base_name = os.path.splitext(img_file)[0]
            label_path = os.path.join(self.labels_dir, f"{base_name}.json")
            
            if not os.path.exists(label_path):
                continue
            
            # 加载图像
            img = cv2.imread(img_path)
            if img is None:
                continue
                
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (self.img_width, self.img_height))
            img = img / 255.0  # 归一化
            
            # 加载标签
            with open(label_path, 'r') as f:
                label_data = json.load(f)
            
            # 提取控制数据
            steering = label_data.get("steering_angle", 0) / 630.0  # 归一化到[-1, 1]
            throttle = label_data.get("throttle", 0)
            brake = label_data.get("brake", 0)
            
            # 添加到数据集
            X.append(img)
            Y.append([steering, throttle, brake])
        
        # 转换为numpy数组
        X = np.array(X, dtype=np.float32)
        Y = np.array(Y, dtype=np.float32)
        
        print(f"数据加载完成: {X.shape[0]} 样本")
        
        # 分割训练集和验证集
        X_train, X_val, Y_train, Y_val = train_test_split(
            X, Y, test_size=0.2, random_state=42)
        
        return X_train, X_val, Y_train, Y_val
    
    def build_model(self):
        """构建端到端自动驾驶模型"""
        print("构建自动驾驶模型...")
        
        # 使用MobileNetV2作为特征提取器
        base_model = MobileNetV2(
            weights='imagenet',
            include_top=False,
            input_shape=(self.img_height, self.img_width, self.img_channels)
        )
        
        # 冻结基础模型
        base_model.trainable = False
        
        # 构建完整模型
        inputs = Input(shape=(self.img_height, self.img_width, self.img_channels))
        x = base_model(inputs)
        x = GlobalAveragePooling2D()(x)
        x = Dense(128, activation='relu')(x)
        x = Dropout(0.5)(x)
        x = Dense(64, activation='relu')(x)
        x = Dropout(0.3)(x)
        outputs = Dense(3)(x)  # [转向, 油门, 刹车]
        
        model = Model(inputs=inputs, outputs=outputs)
        
        # 编译模型
        model.compile(
            optimizer=Adam(learning_rate=0.001),
            loss='mse',
            metrics=['mae']
        )
        
        model.summary()
        return model
    
    def train(self, epochs=30, batch_size=32):
        """训练模型"""
        print("开始训练模型...")
        
        # 加载数据
        X_train, X_val, Y_train, Y_val = self.load_data()
        
        # 构建模型
        model = self.build_model()
        
        # 设置回调函数
        log_dir = os.path.join(self.models_dir, 'logs', datetime.now().strftime("%Y%m%d-%H%M%S"))
        callbacks = [
            ModelCheckpoint(
                filepath=os.path.join(self.models_dir, 'best_model.h5'),
                monitor='val_loss',
                save_best_only=True,
                verbose=1
            ),
            EarlyStopping(
                monitor='val_loss',
                patience=5,
                restore_best_weights=True,
                verbose=1
            ),
            TensorBoard(
                log_dir=log_dir,
                histogram_freq=1
            )
        ]
        
        # 训练模型
        history = model.fit(
            X_train, Y_train,
            validation_data=(X_val, Y_val),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=callbacks,
            verbose=1
        )
        
        # 保存最终模型
        model.save(os.path.join(self.models_dir, 'final_model.h5'))
        
        print(f"模型训练完成，已保存到: {os.path.join(self.models_dir, 'final_model.h5')}")
        
        # 绘制训练历史
        self.plot_training_history(history)
        
        return model, history
    
    def plot_training_history(self, history):
        """绘制训练历史"""
        plt.figure(figsize=(12, 4))
        
        # 绘制损失
        plt.subplot(1, 2, 1)
        plt.plot(history.history['loss'], label='训练损失')
        plt.plot(history.history['val_loss'], label='验证损失')
        plt.title('模型损失')
        plt.xlabel('Epoch')
        plt.ylabel('损失')
        plt.legend()
        
        # 绘制MAE
        plt.subplot(1, 2, 2)
        plt.plot(history.history['mae'], label='训练MAE')
        plt.plot(history.history['val_mae'], label='验证MAE')
        plt.title('模型MAE')
        plt.xlabel('Epoch')
        plt.ylabel('MAE')
        plt.legend()
        
        # 保存图表
        plt.tight_layout()
        plt.savefig(os.path.join(self.models_dir, 'training_history.png'))
        plt.close()
        
        print(f"训练历史图表已保存到: {os.path.join(self.models_dir, 'training_history.png')}")

if __name__ == "__main__":
    model_trainer = FSAutonomousDriver()
    model_trainer.train(epochs=30, batch_size=32)
