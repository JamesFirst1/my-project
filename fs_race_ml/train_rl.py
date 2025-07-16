#!/usr/bin/env python3
import os
import argparse
import numpy as np
import gym
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack
from stable_baselines3.common.env_util import make_vec_env
import time
from datetime import datetime

# 导入自定义环境
from fs_carmaker_env import FSCarMakerEnv

class RLTrainer:
    def __init__(self, model_dir="~/fs_race_data/rl_models", algorithm="ppo"):
        # 设置目录
        self.model_dir = os.path.expanduser(model_dir)
        os.makedirs(self.model_dir, exist_ok=True)
        
        # 设置模型类型
        self.algorithm = algorithm.lower()
        
        # 日志目录
        self.log_dir = os.path.join(self.model_dir, "logs", datetime.now().strftime("%Y%m%d-%H%M%S"))
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 创建环境
        self.env = self._create_env(port=2212)
        
        # 创建评估环境(不同端口)
        self.eval_env = self._create_env(port=2213)
        
        print(f"RL训练器已初始化，算法: {self.algorithm}, 模型将保存在: {self.model_dir}")
    
    def _create_env(self, port=2212):
        """创建环境"""
        def _make_env():
            env = FSCarMakerEnv(port=port)
            env = Monitor(env)
            return env
        
        # 创建向量化环境
        vec_env = DummyVecEnv([_make_env])
        vec_env = VecFrameStack(vec_env, n_stack=4)  # 堆叠4帧
        
        return vec_env
    
    def train(self, total_timesteps=500000, save_freq=10000, eval_freq=50000):
        """训练RL模型"""
        print(f"开始训练 {self.algorithm.upper()} 模型...")
        
        # 设置回调函数
        checkpoint_callback = CheckpointCallback(
            save_freq=save_freq,
            save_path=os.path.join(self.model_dir, "checkpoints"),
            name_prefix=f"{self.algorithm}_model"
        )
        
        eval_callback = EvalCallback(
            self.eval_env,
            best_model_save_path=os.path.join(self.model_dir, "best_model"),
            log_path=self.log_dir,
            eval_freq=eval_freq,
            deterministic=True,
            render=False
        )
        
        # 创建并训练模型
        if self.algorithm == "ppo":
            model = PPO(
                "CnnPolicy",
                self.env,
                verbose=1,
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                n_epochs=10,
                gamma=0.99,
                gae_lambda=0.95,
                clip_range=0.2,
                ent_coef=0.01,
                tensorboard_log=self.log_dir
            )
        elif self.algorithm == "sac":
            model = SAC(
                "CnnPolicy",
                self.env,
                verbose=1,
                learning_rate=3e-4,
                buffer_size=50000,
                learning_starts=1000,
                batch_size=64,
                gamma=0.99,
                tau=0.005,
                ent_coef="auto",
                tensorboard_log=self.log_dir
            )
        else:
            raise ValueError(f"不支持的算法: {self.algorithm}")
        
        # 训练模型
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_callback, eval_callback],
            tb_log_name=f"{self.algorithm}"
        )
        
        # 保存最终模型
        final_model_path = os.path.join(self.model_dir, f"final_{self.algorithm}_model")
        model.save(final_model_path)
        
        print(f"训练完成，最终模型已保存到: {final_model_path}")
        
        # 关闭环境
        self.env.close()
        self.eval_env.close()
        
        return model
    
    def evaluate(self, model_path=None):
        """评估模型"""
        if model_path is None:
            # 使用最佳模型
            model_path = os.path.join(self.model_dir, "best_model", f"{self.algorithm}_model")
        
        print(f"评估模型: {model_path}")
        
        # 加载模型
        if self.algorithm == "ppo":
            model = PPO.load(model_path, env=self.eval_env)
        elif self.algorithm == "sac":
            model = SAC.load(model_path, env=self.eval_env)
        else:
            raise ValueError(f"不支持的算法: {self.algorithm}")
        
        # 评估10个回合
        episodes = 10
        total_rewards = []
        
        for i in range(episodes):
            obs = self.eval_env.reset()
            done = False
            total_reward = 0
            step = 0
            
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, info = self.eval_env.step(action)
                total_reward += reward
                step += 1
            
            total_rewards.append(total_reward)
            print(f"回合 {i+1}/{episodes}: 奖励={total_reward}, 步数={step}")
        
        # 输出统计信息
        mean_reward = np.mean(total_rewards)
        std_reward = np.std(total_rewards)
        print(f"平均奖励: {mean_reward:.2f} ± {std_reward:.2f}")
        
        return mean_reward, std_reward

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="训练RL自动驾驶模型")
    parser.add_argument("--algorithm", type=str, default="ppo", choices=["ppo", "sac"], help="RL算法")
    parser.add_argument("--timesteps", type=int, default=500000, help="总训练步数")
    
    args = parser.parse_args()
    
    trainer = RLTrainer(algorithm=args.algorithm)
    model = trainer.train(total_timesteps=args.timesteps)
    trainer.evaluate()
