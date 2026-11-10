#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from collections import deque
import dynamic_reconfigure.client
import gym
from gym import spaces
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import threading
import time
import math


class AdvancedUKFEnvironment(gym.Env):
    """高级UKF环境，包含所有可调参数"""

    def __init__(self, window_size=50, update_interval=1.0):
        super(AdvancedUKFEnvironment, self).__init__()

        # ROS初始化
        self.pose_buffer = deque(maxlen=window_size)
        self.velocity_buffer = deque(maxlen=10)
        self.window_size = window_size
        self.update_interval = update_interval

        # 扩展动作空间：包含所有UKF参数
        # P矩阵(6个对角元素) + Q矩阵(6个) + R矩阵(3个) + UKF参数(3个) + 自适应增益(1个)
        # 总计：6 + 6 + 3 + 3 + 1 = 19个参数
        self.action_space = spaces.Box(
            low=-2.0, high=2.0, shape=(19,), dtype=np.float32
        )

        # 扩展状态空间：包含更多统计信息和滤波器内部状态
        # [x_mean, y_mean, theta_mean, x_std, y_std, theta_std,
        #  x_range, y_range, theta_range, is_static, time_static,
        #  velocity_mean, velocity_std, angular_velocity_mean, angular_velocity_std,
        #  prediction_error, innovation_norm, convergence_rate]
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32
        )

        # 基础参数值（更精细的初始化）
        self.base_P = np.array([0.15, 0.15, 1.0, math.pi, 1.0, 1.0])  # P对角元素
        self.base_Q = np.array([0.01, 0.01, 0.04, 0.01, 0.01, 0.1])  # Q对角元素
        self.base_R = np.array([0.0004, 0.0004, 0.0003])  # R对角元素
        self.base_ukf = np.array([2.0, 0.001, 2.0])  # [lambda, alpha, beta]
        self.base_adapt = 0.1  # 自适应增益

        # 参数范围限制
        self.param_ranges = {
            'P': (0.001, 10.0),  # 协方差范围
            'Q': (1e-6, 1.0),  # 过程噪声范围
            'R': (1e-6, 0.1),  # 测量噪声范围
            'lambda': (0.1, 10.0),  # UKF lambda范围
            'alpha': (1e-4, 1.0),  # UKF alpha范围
            'beta': (0.0, 4.0),  # UKF beta范围
            'adapt': (0.01, 0.5)  # 自适应增益范围
        }

        # 动态重配置客户端
        self.dyn_client = None
        self.connect_dynamic_reconfigure()

        # 性能跟踪
        self.static_threshold = 0.03
        self.static_duration = 0.0
        self.last_update_time = time.time()
        self.convergence_history = deque(maxlen=20)
        self.innovation_history = deque(maxlen=10)
        self.prediction_errors = deque(maxlen=10)

        # 订阅位姿话题
        self.pose_sub = rospy.Subscriber(
            "/pose_stamped", PoseStamped, self.pose_callback, queue_size=10
        )

    def connect_dynamic_reconfigure(self):
        """连接到动态重配置服务器"""
        try:
            self.dyn_client = dynamic_reconfigure.client.Client(
                "mcu0_serial_node", timeout=5.0
            )
            rospy.loginfo("Connected to dynamic reconfigure server")
        except Exception as e:
            rospy.logerr(f"Failed to connect to dynamic reconfigure: {e}")

    def pose_callback(self, msg):
        """位姿数据回调"""
        pose_data = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
            'qw': msg.pose.orientation.w,
            'timestamp': msg.header.stamp.to_sec()
        }

        # 计算偏航角
        siny_cosp = 2 * (pose_data['qw'] * pose_data['qz'] +
                         pose_data['qx'] * pose_data['qy'])
        cosy_cosp = 1 - 2 * (pose_data['qy'] ** 2 + pose_data['qz'] ** 2)
        pose_data['yaw'] = np.arctan2(siny_cosp, cosy_cosp)

        # 计算速度信息
        if len(self.pose_buffer) > 0:
            prev_pose = self.pose_buffer[-1]
            dt = pose_data['timestamp'] - prev_pose['timestamp']
            if dt > 0:
                vx = (pose_data['x'] - prev_pose['x']) / dt
                vy = (pose_data['y'] - prev_pose['y']) / dt
                vyaw = self.angle_diff(pose_data['yaw'], prev_pose['yaw']) / dt

                velocity_data = {
                    'vx': vx, 'vy': vy, 'vyaw': vyaw,
                    'speed': np.sqrt(vx ** 2 + vy ** 2),
                    'timestamp': pose_data['timestamp']
                }
                self.velocity_buffer.append(velocity_data)

        self.pose_buffer.append(pose_data)

    def get_state(self):
        """计算增强的状态特征"""
        if len(self.pose_buffer) < 10:
            return np.zeros(18)

        # 基础位姿统计
        recent_poses = list(self.pose_buffer)
        xs = [p['x'] for p in recent_poses]
        ys = [p['y'] for p in recent_poses]
        yaws = [p['yaw'] for p in recent_poses]
        timestamps = [p['timestamp'] for p in recent_poses]

        # 位置统计
        x_mean, y_mean = np.mean(xs), np.mean(ys)
        x_std, y_std = np.std(xs), np.std(ys)
        x_range = np.max(xs) - np.min(xs)
        y_range = np.max(ys) - np.min(ys)

        # 角度统计（处理环绕）
        yaw_sin = np.mean(np.sin(yaws))
        yaw_cos = np.mean(np.cos(yaws))
        yaw_mean = np.arctan2(yaw_sin, yaw_cos)
        yaw_diffs = np.array([self.angle_diff(y, yaw_mean) for y in yaws])
        yaw_std = np.std(yaw_diffs)
        yaw_range = np.max(yaw_diffs) - np.min(yaw_diffs)

        # 速度统计
        if len(self.velocity_buffer) >= 3:
            recent_vels = list(self.velocity_buffer)
            speeds = [v['speed'] for v in recent_vels]
            angular_vels = [abs(v['vyaw']) for v in recent_vels]

            velocity_mean = np.mean(speeds)
            velocity_std = np.std(speeds)
            angular_velocity_mean = np.mean(angular_vels)
            angular_velocity_std = np.std(angular_vels)

            # 静止检测
            current_speed = speeds[-1] if speeds else 0
            is_static = 1.0 if current_speed < self.static_threshold else 0.0

            if is_static:
                self.static_duration += (timestamps[-1] - timestamps[-2] if len(timestamps) > 1 else 0)
            else:
                self.static_duration = 0.0
        else:
            velocity_mean = velocity_std = 0.0
            angular_velocity_mean = angular_velocity_std = 0.0
            is_static = 0.0

        time_static = min(self.static_duration / 5.0, 1.0)

        # 滤波器性能指标
        prediction_error = np.mean(self.prediction_errors) if self.prediction_errors else 0.0
        innovation_norm = np.mean(self.innovation_history) if self.innovation_history else 0.0
        convergence_rate = self.calculate_convergence_rate()

        state = np.array([
            x_mean, y_mean, yaw_mean,
            x_std, y_std, yaw_std,
            x_range, y_range, yaw_range,
            is_static, time_static,
            velocity_mean, velocity_std,
            angular_velocity_mean, angular_velocity_std,
            prediction_error, innovation_norm, convergence_rate
        ], dtype=np.float32)

        return state

    def calculate_convergence_rate(self):
        """计算收敛速度"""
        if len(self.convergence_history) < 5:
            return 0.0

        recent_stds = list(self.convergence_history)[-10:]
        if len(recent_stds) < 2:
            return 0.0

        # 计算标准差的变化率
        std_changes = np.diff(recent_stds)
        convergence_rate = -np.mean(std_changes)  # 负值表示标准差在减小（收敛）
        return np.clip(convergence_rate, -1.0, 1.0)

    def angle_diff(self, a1, a2):
        """计算两个角度之间的最小差值"""
        diff = a1 - a2
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def step(self, action):
        """执行一步环境交互"""
        # 将动作转换为参数值
        params = self.action_to_params(action)

        # 应用参数到UKF
        if self.dyn_client:
            try:
                self.dyn_client.update_configuration(params)
                rospy.logdebug("Updated UKF parameters")
            except Exception as e:
                rospy.logwarn(f"Failed to update parameters: {e}")

        # 等待系统稳定并收集数据
        rospy.sleep(self.update_interval)

        # 获取新状态
        state = self.get_state()

        # 更新性能历史
        self.update_performance_history(state)

        # 计算奖励
        reward = self.calculate_advanced_reward(state, action)

        done = False

        info = {
            'x_std': state[3], 'y_std': state[4], 'yaw_std': state[5],
            'is_static': state[9], 'convergence_rate': state[17],
            'prediction_error': state[15], 'innovation_norm': state[16],
            'applied_params': params
        }

        return state, reward, done, info

    def action_to_params(self, action):
        """将动作转换为UKF参数"""
        # 使用sigmoid和指数变换确保参数在合理范围内
        params = {}
        idx = 0

        # P矩阵参数 (6个)
        for i in range(6):
            scale = np.exp(action[idx] * 1.5)  # 指数缩放
            value = self.base_P[i] * scale
            value = np.clip(value, self.param_ranges['P'][0], self.param_ranges['P'][1])
            params[f'P{i}'] = float(value)
            idx += 1

        # Q矩阵参数 (6个)
        for i in range(6):
            scale = np.exp(action[idx] * 2.0)
            value = self.base_Q[i] * scale
            value = np.clip(value, self.param_ranges['Q'][0], self.param_ranges['Q'][1])
            params[f'Q{i}'] = float(value)
            idx += 1

        # R矩阵参数 (3个)
        for i in range(3):
            scale = np.exp(action[idx] * 1.5)
            value = self.base_R[i] * scale
            value = np.clip(value, self.param_ranges['R'][0], self.param_ranges['R'][1])
            params[f'R{i}'] = float(value)
            idx += 1

        # UKF参数 (3个)
        # Lambda
        lambda_val = self.base_ukf[0] + action[idx] * 3.0
        params['lambda'] = float(np.clip(lambda_val,
                                         self.param_ranges['lambda'][0],
                                         self.param_ranges['lambda'][1]))
        idx += 1

        # Alpha
        alpha_val = self.base_ukf[1] * np.exp(action[idx] * 2.0)
        params['alpha'] = float(np.clip(alpha_val,
                                        self.param_ranges['alpha'][0],
                                        self.param_ranges['alpha'][1]))
        idx += 1

        # Beta
        beta_val = self.base_ukf[2] + action[idx] * 1.0
        params['beta'] = float(np.clip(beta_val,
                                       self.param_ranges['beta'][0],
                                       self.param_ranges['beta'][1]))
        idx += 1

        # 自适应增益
        adapt_val = self.base_adapt + action[idx] * 0.2
        params['adapt_gain'] = float(np.clip(adapt_val,
                                             self.param_ranges['adapt'][0],
                                             self.param_ranges['adapt'][1]))

        return params

    def update_performance_history(self, state):
        """更新性能历史记录"""
        # 记录位置标准差的总和作为收敛指标
        position_std = state[3] + state[4] + state[5]  # x_std + y_std + yaw_std
        self.convergence_history.append(position_std)

        # 模拟新息计算（在实际应用中应该从滤波器获取）
        if len(self.pose_buffer) >= 2:
            innovation = abs(state[0]) + abs(state[1])  # 简化的新息计算
            self.innovation_history.append(innovation)

        # 模拟预测误差
        if len(self.velocity_buffer) >= 2:
            pred_error = state[11] * 0.1  # velocity_std as proxy for prediction error
            self.prediction_errors.append(pred_error)

    def calculate_advanced_reward(self, state, action):
        """计算高级奖励函数"""
        x_std, y_std, yaw_std = state[3], state[4], state[5]
        x_range, y_range, yaw_range = state[6], state[7], state[8]
        is_static, time_static = state[9], state[10]
        velocity_std, angular_velocity_std = state[12], state[14]
        prediction_error, innovation_norm, convergence_rate = state[15], state[16], state[17]

        reward = 0.0

        if is_static > 0.5:  # 静止状态
            # 1. 位置稳定性奖励（主要目标）
            position_stability = -(x_std + y_std) * 20.0
            angle_stability = -yaw_std * 10.0

            # 2. 范围收敛奖励
            range_reward = -(x_range + y_range + yaw_range) * 5.0

            # 3. 收敛速度奖励
            convergence_reward = convergence_rate * 5.0

            # 4. 创新协方差奖励（较小的创新表示好的预测）
            innovation_reward = -innovation_norm * 2.0

            # 5. 静止时间奖励（鼓励长期稳定）
            static_time_reward = time_static * 3.0

            # 6. 速度一致性奖励（静止时速度应该很小且稳定）
            velocity_consistency = -(velocity_std + angular_velocity_std) * 5.0

            reward = (position_stability + angle_stability + range_reward +
                      convergence_reward + innovation_reward + static_time_reward +
                      velocity_consistency)

        else:  # 运动状态
            # 保持响应性，不要过度惩罚
            responsiveness_reward = -prediction_error * 1.0

            # 适度的稳定性要求
            moderate_stability = -(x_std + y_std + yaw_std) * 2.0

            reward = responsiveness_reward + moderate_stability

        # 7. 参数变化惩罚（避免参数剧烈变化）
        action_penalty = -np.sum(np.abs(action)) * 0.05

        # 8. 参数合理性奖励（鼓励参数在合理范围内）
        param_reasonableness = 0.0
        for i, act in enumerate(action):
            if abs(act) < 1.0:  # 鼓励适度的参数调整
                param_reasonableness += 0.1

        total_reward = reward + action_penalty + param_reasonableness

        return total_reward

    def reset(self):
        """重置环境"""
        self.pose_buffer.clear()
        self.velocity_buffer.clear()
        self.convergence_history.clear()
        self.innovation_history.clear()
        self.prediction_errors.clear()
        self.static_duration = 0.0

        # 重置参数到基础值
        if self.dyn_client:
            reset_params = {}
            for i in range(6):
                reset_params[f'P{i}'] = float(self.base_P[i])
                reset_params[f'Q{i}'] = float(self.base_Q[i])
            for i in range(3):
                reset_params[f'R{i}'] = float(self.base_R[i])
            reset_params.update({
                'lambda': float(self.base_ukf[0]),
                'alpha': float(self.base_ukf[1]),
                'beta': float(self.base_ukf[2]),
                'adapt_gain': float(self.base_adapt)
            })

            try:
                self.dyn_client.update_configuration(reset_params)
            except:
                pass

        # 等待收集数据
        rospy.sleep(3.0)
        return self.get_state()


class AdvancedPPOAgent:
    """增强的PPO智能体，专门处理高维参数空间"""

    def __init__(self, state_dim, action_dim, lr=3e-4):
        self.state_dim = state_dim
        self.action_dim = action_dim

        # 更深的网络以处理复杂的参数关系
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim * 2)  # 均值和对数标准差
        )

        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)

        # 参数初始化
        self.actor.apply(self.init_weights)
        self.critic.apply(self.init_weights)

    def init_weights(self, m):
        """初始化网络权重"""
        if isinstance(m, nn.Linear):
            torch.nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
            torch.nn.init.constant_(m.bias, 0)

    def get_action(self, state, deterministic=False):
        """获取动作"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)

        with torch.no_grad():
            output = self.actor(state_tensor)
            mean = output[:, :self.action_dim]
            log_std = output[:, self.action_dim:]

            # 限制标准差范围
            log_std = torch.clamp(log_std, -2, 1)
            std = torch.exp(log_std)

            if deterministic:
                action = mean
            else:
                dist = Normal(mean, std)
                action = dist.sample()

            # 使用tanh激活确保动作在[-2, 2]范围内
            action = torch.tanh(action) * 2.0

        return action.squeeze(0).numpy()

    def evaluate_action(self, state, action):
        """评估动作的概率和价值"""
        output = self.actor(state)
        mean = output[:, :self.action_dim]
        log_std = output[:, self.action_dim:]
        log_std = torch.clamp(log_std, -2, 1)
        std = torch.exp(log_std)

        # 反tanh变换
        action_raw = torch.atanh(torch.clamp(action / 2.0, -0.99, 0.99))

        dist = Normal(mean, std)
        log_prob = dist.log_prob(action_raw).sum(dim=1, keepdim=True)

        # Jacobian修正（tanh变换）
        log_prob -= torch.log(1 - (action / 2.0).pow(2) + 1e-6).sum(dim=1, keepdim=True)

        entropy = dist.entropy().sum(dim=1, keepdim=True)
        value = self.critic(state)

        return log_prob, entropy, value

    def update(self, states, actions, rewards, next_states, dones, old_log_probs):
        """PPO更新"""
        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones).unsqueeze(1)
        old_log_probs = torch.FloatTensor(old_log_probs).unsqueeze(1)

        # 计算优势
        with torch.no_grad():
            values = self.critic(states)
            next_values = self.critic(next_states)
            targets = rewards + 0.99 * next_values * (1 - dones)
            advantages = targets - values
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # PPO更新
        for _ in range(4):  # PPO epochs
            log_probs, entropy, current_values = self.evaluate_action(states, actions)

            # 策略损失
            ratio = torch.exp(log_probs - old_log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 0.8, 1.2) * advantages
            actor_loss = -torch.min(surr1, surr2).mean() - 0.01 * entropy.mean()

            # 价值损失
            critic_loss = nn.MSELoss()(current_values, targets.detach())

            # 更新网络
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 0.5)
            self.actor_optimizer.step()

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 0.5)
            self.critic_optimizer.step()

        return critic_loss.item(), actor_loss.item()


class AdvancedUKFTuner:
    """高级UKF参数调优器"""

    def __init__(self):
        rospy.init_node('advanced_ukf_rl_tuner', anonymous=True)

        # 创建高级环境和智能体
        self.env = AdvancedUKFEnvironment(window_size=50, update_interval=2.0)
        self.agent = AdvancedPPOAgent(state_dim=18, action_dim=19, lr=1e-4)

        # 经验回放
        self.buffer_size = 2000
        self.buffer = {
            'states': deque(maxlen=self.buffer_size),
            'actions': deque(maxlen=self.buffer_size),
            'rewards': deque(maxlen=self.buffer_size),
            'next_states': deque(maxlen=self.buffer_size),
            'dones': deque(maxlen=self.buffer_size),
            'log_probs': deque(maxlen=self.buffer_size)
        }

        # 训练参数
        self.batch_size = 64
        self.update_frequency = 20
        self.step_count = 0

        # 性能追踪
        self.episode_rewards = []
        self.episode_count = 0
        self.best_reward = float('-inf')
        self.stability_scores = []

    def train(self):
        """高级训练循环"""
        rospy.loginfo("Starting advanced UKF parameter tuning...")

        while not rospy.is_shutdown():
            state = self.env.reset()
            episode_reward = 0
            episode_steps = 0
            episode_stability = []

            while not rospy.is_shutdown() and episode_steps < 150:
                # 获取动作和对数概率
                action = self.agent.get_action(state)
                with torch.no_grad():
                    state_tensor = torch.FloatTensor(state).unsqueeze(0)
                    output = self.agent.actor(state_tensor)
                    mean = output[:, :19]
                    log_std = torch.clamp(output[:, 19:], -2, 1)
                    std = torch.exp(log_std)
                    action_raw = torch.atanh(torch.clamp(torch.FloatTensor(action) / 2.0, -0.99, 0.99))
                    dist = Normal(mean, std)
                    log_prob = dist.log_prob(action_raw).sum().item()
                    log_prob -= torch.log(1 - (torch.FloatTensor(action) / 2.0).pow(2) + 1e-6).sum().item()

                # 执行动作
                next_state, reward, done, info = self.env.step(action)

                # 存储经验
                self.buffer['states'].append(state)
                self.buffer['actions'].append(action)
                self.buffer['rewards'].append(reward)
                self.buffer['next_states'].append(next_state)
                self.buffer['dones'].append(done)
                self.buffer['log_probs'].append(log_prob)

                episode_reward += reward
                episode_steps += 1
                self.step_count += 1

                # 记录稳定性
                if info['is_static'] > 0.5:
                    stability_score = -(info['x_std'] + info['y_std'] + info['yaw_std'])
                    episode_stability.append(stability_score)

                # 更新网络
                if self.step_count % self.update_frequency == 0 and len(self.buffer['states']) >= self.batch_size:
                    self.update_agent()

                state = next_state

                # 详细日志
                if episode_steps % 15 == 0:
                    rospy.loginfo(
                        f"Episode {self.episode_count}, Step {episode_steps}: "
                        f"x_std={info['x_std']:.4f}, y_std={info['y_std']:.4f}, "
                        f"yaw_std={info['yaw_std']:.4f}, static={info['is_static']:.1f}, "
                        f"convergence={info['convergence_rate']:.3f}, reward={reward:.2f}"
                    )

            # 回合结束统计
            self.episode_count += 1
            self.episode_rewards.append(episode_reward)

            if episode_stability:
                avg_stability = np.mean(episode_stability)
                self.stability_scores.append(avg_stability)

                if episode_reward > self.best_reward:
                    self.best_reward = episode_reward
                    self.save_model(f"best_ukf_rl_model_ep{self.episode_count}.pth")

            rospy.loginfo(
                f"Episode {self.episode_count} completed: "
                f"Total reward={episode_reward:.2f}, "
                f"Avg stability={np.mean(episode_stability) if episode_stability else 0:.4f}, "
                f"Steps={episode_steps}"
            )

            # 定期保存
            if self.episode_count % 25 == 0:
                self.save_model(f"ukf_rl_checkpoint_ep{self.episode_count}.pth")
                self.print_training_summary()

    def update_agent(self):
        """更新智能体"""
        # 采样批次
        indices = np.random.choice(len(self.buffer['states']), self.batch_size, replace=False)

        states = np.array([self.buffer['states'][i] for i in indices])
        actions = np.array([self.buffer['actions'][i] for i in indices])
        rewards = np.array([self.buffer['rewards'][i] for i in indices])
        next_states = np.array([self.buffer['next_states'][i] for i in indices])
        dones = np.array([self.buffer['dones'][i] for i in indices])
        old_log_probs = np.array([self.buffer['log_probs'][i] for i in indices])

        # 更新网络
        critic_loss, actor_loss = self.agent.update(
            states, actions, rewards, next_states, dones, old_log_probs
        )

        if self.step_count % 100 == 0:
            rospy.loginfo(
                f"Network update {self.step_count}: "
                f"critic_loss={critic_loss:.4f}, actor_loss={actor_loss:.4f}"
            )

    def print_training_summary(self):
        """打印训练摘要"""
        if self.episode_rewards:
            avg_reward = np.mean(self.episode_rewards[-25:])
            avg_stability = np.mean(self.stability_scores[-10:]) if self.stability_scores else 0

            rospy.loginfo("=" * 60)
            rospy.loginfo(f"Training Summary (Episode {self.episode_count}):")
            rospy.loginfo(f"Average reward (last 25): {avg_reward:.3f}")
            rospy.loginfo(f"Best reward: {self.best_reward:.3f}")
            rospy.loginfo(f"Average stability (last 10): {avg_stability:.4f}")
            rospy.loginfo(f"Total steps: {self.step_count}")
            rospy.loginfo("=" * 60)

    def save_model(self, filepath):
        """保存模型"""
        torch.save({
            'actor_state_dict': self.agent.actor.state_dict(),
            'critic_state_dict': self.agent.critic.state_dict(),
            'episode_rewards': self.episode_rewards,
            'stability_scores': self.stability_scores,
            'episode_count': self.episode_count,
            'best_reward': self.best_reward
        }, filepath)
        rospy.loginfo(f"Model saved to {filepath}")

    def load_model(self, filepath):
        """加载模型"""
        checkpoint = torch.load(filepath)
        self.agent.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.agent.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.episode_rewards = checkpoint['episode_rewards']
        self.stability_scores = checkpoint['stability_scores']
        self.episode_count = checkpoint['episode_count']
        self.best_reward = checkpoint['best_reward']
        rospy.loginfo(f"Model loaded from {filepath}")


if __name__ == '__main__':
    try:
        tuner = AdvancedUKFTuner()
        tuner.train()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Saving model before exit...")
        tuner.save_model("final_advanced_ukf_rl_model.pth")