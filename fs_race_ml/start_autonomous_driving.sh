#!/bin/bash

# 设置工作目录
FS_ML_DIR="$HOME/fs_race_ml"
IPG_SIM_DIR="$HOME/Sim-Dev/uwe-ipg-sim"
YOLO_DIR="$HOME/voc_detection"

# 检查工作目录是否存在
if [ ! -d "$IPG_SIM_DIR" ]; then
    echo "错误: IPG CarMaker目录不存在: $IPG_SIM_DIR"
    exit 1
fi

if [ ! -d "$YOLO_DIR" ]; then
    echo "错误: YOLOv8目录不存在: $YOLO_DIR"
    exit 1
fi

# 模型类型和路径
MODEL_TYPE="$1"
if [ -z "$MODEL_TYPE" ]; then
    MODEL_TYPE="sl"  # 默认使用监督学习模型
fi

if [ "$MODEL_TYPE" == "sl" ]; then
    MODEL_PATH="$HOME/fs_race_data/models/final_model.h5"
    echo "使用监督学习模型: $MODEL_PATH"
elif [ "$MODEL_TYPE" == "rl" ]; then
    MODEL_PATH="$HOME/fs_race_data/rl_models/final_ppo_model"
    echo "使用强化学习模型: $MODEL_PATH"
else
    echo "错误: 未知模型类型 '$MODEL_TYPE'. 使用 'sl' 或 'rl'"
    exit 1
fi

# 启动CarMaker
echo "启动CarMaker..."
cd "$IPG_SIM_DIR"
./CMStart.sh &
CM_PID=$!

# 等待CarMaker启动
echo "等待CarMaker启动..."
sleep 5

# 启动自动驾驶模型
echo "启动自动驾驶模型..."
cd "$FS_ML_DIR"
python3 deploy_model.py --model "$MODEL_PATH" --type "$MODEL_TYPE" &
MODEL_PID=$!

# 设置清理函数
function cleanup {
    echo "停止自动驾驶系统..."
    kill -SIGINT $MODEL_PID 2>/dev/null
    kill -SIGINT $CM_PID 2>/dev/null
    wait
    echo "已停止"
}

# 捕获中断信号
trap cleanup SIGINT SIGTERM

echo "自动驾驶系统已启动。按Ctrl+C停止..."
wait
