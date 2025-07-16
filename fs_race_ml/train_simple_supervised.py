#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.optimizers import Adam
import cv2
from sklearn.model_selection import train_test_split
import random

# 设置路径
models_dir = os.path.expanduser("~/fs_race_data/models")
os.makedirs(models_dir, exist_ok=True)

# 生成简单的模拟数据
def generate_synthetic_data(num_samples=100):
    X = []  # 图像
    Y = []  # 控制信号 [转向, 油门, 刹车]
    
    for i in range(num_samples):
        # 创建简单的道路场景图像
        img = np.ones((224, 224, 3), dtype=np.float32) * 0.5  # 灰色背景
        
        # 随机位置的蓝色锥桶
        blue_x = random.randint(50, 100)
        cv2.circle(img, (blue_x, 150), 10, (1.0, 0, 0), -1)
        
        # 随机位置的黄色锥桶
        yellow_x = random.randint(124, 174)
        cv2.circle(img, (yellow_x, 150), 10, (1.0, 1.0, 0), -1)
        
        # 计算中心点
        center_x = (blue_x + yellow_x) / 2
        
        # 转向控制：基于锥桶位置计算（-1到1）
        steering = ((center_x - 112) / 112) * -1  # 如果中心偏右，则左转
        
        # 简单的油门控制
        throttle = 0.5  # 恒定油门
        brake = 0.0     # 不刹车
        
        X.append(img)
        Y.append([steering, throttle, brake])
    
    return np.array(X), np.array(Y)

# 生成数据
print("生成模拟训练数据...")
X, Y = generate_synthetic_data(500)

# 分割训练集和验证集
X_train, X_val, Y_train, Y_val = train_test_split(X, Y, test_size=0.2, random_state=42)

print(f"训练数据: {X_train.shape}, {Y_train.shape}")
print(f"验证数据: {X_val.shape}, {Y_val.shape}")

# 构建简单模型
print("构建自动驾驶模型...")
model = Sequential([
    Conv2D(16, (3, 3), activation='relu', input_shape=(224, 224, 3)),
    MaxPooling2D(pool_size=(2, 2)),
    Conv2D(32, (3, 3), activation='relu'),
    MaxPooling2D(pool_size=(2, 2)),
    Conv2D(64, (3, 3), activation='relu'),
    MaxPooling2D(pool_size=(2, 2)),
    Flatten(),
    Dense(100, activation='relu'),
    Dropout(0.3),
    Dense(50, activation='relu'),
    Dense(3)  # [转向, 油门, 刹车]
])

model.compile(optimizer=Adam(learning_rate=0.001), loss='mse', metrics=['mae'])
model.summary()

# 创建回调函数
checkpoint = ModelCheckpoint(
    os.path.join(models_dir, 'best_model.h5'),
    monitor='val_loss',
    save_best_only=True,
    verbose=1
)

# 训练模型
print("开始训练模型...")
history = model.fit(
    X_train, Y_train,
    validation_data=(X_val, Y_val),
    epochs=20,
    batch_size=32,
    callbacks=[checkpoint],
    verbose=1
)

# 保存最终模型
model.save(os.path.join(models_dir, 'final_model.h5'))
print(f"模型训练完成，已保存到: {os.path.join(models_dir, 'final_model.h5')}")

# 绘制训练历史
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
plt.savefig(os.path.join(models_dir, 'training_history.png'))
print(f"训练历史图表已保存到: {os.path.join(models_dir, 'training_history.png')}")
