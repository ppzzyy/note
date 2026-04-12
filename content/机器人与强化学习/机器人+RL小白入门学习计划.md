# 🤖 机器人 + 强化学习（RL）小白入门学习计划

---

## 📌 总览

```
阶段1：数学与编程基础（4-6周）
  ↓
阶段2：强化学习理论入门（6-8周）
  ↓
阶段3：机器人学基础（4-6周）
  ↓
阶段4：机器人+RL结合实践（6-8周）
  ↓
阶段5：前沿论文与项目实战（持续）
```

---

## 阶段1：数学与编程基础（4-6周）

### 1.1 数学基础

| 主题 | 推荐资料 | 重点内容 |
|------|---------|---------|
| 线性代数 | 3Blue1Brown《线性代数的本质》(B站有) | 矩阵运算、特征值、SVD |
| 概率与统计 | 陈希孺《概率论与数理统计》 | 贝叶斯、马尔可夫链、期望 |
| 微积分/优化 | 《Convex Optimization》Boyd | 梯度下降、凸优化基础 |

### 1.2 编程基础

- **Python**：熟练掌握 NumPy、Matplotlib
- **PyTorch**：官方教程 → https://pytorch.org/tutorials/
- **Linux基础**：命令行操作、SSH、conda环境管理

### 1.3 推荐快速入门

- 李宏毅机器学习课程（B站免费）—— 先看前10讲建立直觉

---

## 阶段2：强化学习理论入门（6-8周）

### 2.1 核心教材

| 优先级 | 教材 | 说明 |
|--------|------|------|
| ⭐⭐⭐ | **Sutton & Barto《Reinforcement Learning: An Introduction》** 2nd Edition | RL圣经，必读！免费在线版：http://incompleteideas.net/book/the-book.html |
| ⭐⭐ | OpenAI Spinning Up | 从零到深度RL的最佳实践指南：https://spinningup.openai.com/ |
| ⭐⭐ | 周博磊《强化学习纲要》(B站) | 中文讲解，适合入门 |

### 2.2 课程推荐

| 课程 | 来源 | 说明 |
|------|------|------|
| **David Silver RL Course** | UCL / YouTube | DeepMind大佬，经典入门课 |
| **CS285: Deep RL** | UC Berkeley / Sergey Levine | 深度RL最佳课程，偏机器人应用 |
| **CS234: RL** | Stanford / Emma Brunskill | 理论扎实 |

### 2.3 需要掌握的核心概念（按顺序）

```
Week 1-2: MDP、Bellman方程、动态规划（Value/Policy Iteration）
Week 3-4: MC方法、TD学习、Q-Learning、SARSA
Week 5-6: DQN、Policy Gradient（REINFORCE）
Week 7-8: Actor-Critic、PPO、SAC、TD3
```

### 2.4 经典论文（按时间线阅读）

| 年份 | 论文 | 意义 |
|------|------|------|
| 2013 | **Playing Atari with Deep RL** (Mnih et al.) | DQN开山之作 |
| 2015 | **Human-level control through deep RL** (Nature DQN) | DQN登上Nature |
| 2015 | **Continuous control with deep RL** (DDPG) | 连续动作空间RL |
| 2016 | **Asynchronous Methods for Deep RL** (A3C) | 异步训练框架 |
| 2017 | **Proximal Policy Optimization (PPO)** (Schulman et al.) | 目前最常用的算法之一 |
| 2018 | **Soft Actor-Critic (SAC)** (Haarnoja et al.) | 机器人领域最流行的算法 |
| 2018 | **TD3** (Fujimoto et al.) | DDPG改进，实用性强 |

---

## 阶段3：机器人学基础（4-6周）

### 3.1 核心教材

| 教材 | 说明 |
|------|------|
| **《Modern Robotics》** Lynch & Park | 现代机器人学入门，配套Coursera课程和视频 |
| **《Robotics: Modelling, Planning and Control》** Siciliano | 经典机器人控制教材 |
| 《机器人学导论》Craig | 中文经典教材 |

### 3.2 需要掌握的核心知识

```
- 刚体运动与齐次变换（SE3）
- 正/逆运动学（FK/IK）
- 动力学基础（牛顿-欧拉 / 拉格朗日）
- PID控制基础
- URDF机器人描述文件
```

### 3.3 仿真平台（⭐重要，必须上手）

| 平台 | 特点 | 链接 |
|------|------|------|
| **MuJoCo** | DeepMind开源，物理仿真最精确，RL标配 | https://github.com/google-deepmind/mujoco |
| **Isaac Gym / Isaac Lab** | NVIDIA GPU并行仿真，速度极快 | https://github.com/isaac-sim/IsaacLab |
| **PyBullet** | 轻量开源，适合入门 | https://github.com/bulletphysics/bullet3 |
| **Gazebo + ROS2** | 工业界标准，偏传统机器人 | https://gazebosim.org/ |

> 💡 **建议**：先用 MuJoCo + Gymnasium 上手，后面转 Isaac Lab 做大规模训练

---

## 阶段4：机器人 + RL 结合实践（6-8周）

### 4.1 入门实践路线

```
Step 1: Gymnasium（原OpenAI Gym）经典环境
        → CartPole, MountainCar, Pendulum
        → 用DQN/PPO跑通

Step 2: MuJoCo 连续控制任务
        → HalfCheetah, Ant, Humanoid
        → 用SAC/PPO训练

Step 3: 机械臂操作任务
        → Gymnasium-Robotics (Fetch系列任务)
        → 学习HER（Hindsight Experience Replay）

Step 4: 灵巧手/四足机器人
        → 尝试Isaac Lab中的任务
```

### 4.2 关键开源项目

| 项目 | 说明 | 链接 |
|------|------|------|
| **Gymnasium** | RL标准环境接口 | https://github.com/Farama-Foundation/Gymnasium |
| **Stable-Baselines3** | 最易用的RL算法库（PPO/SAC/TD3等） | https://github.com/DLR-RM/stable-baselines3 |
| **CleanRL** | 单文件RL实现，适合学习算法细节 | https://github.com/vwxyzjn/cleanrl |
| **rl_games** | 高性能RL库，Isaac Lab默认使用 | https://github.com/Denys88/rl_games |
| **Gymnasium-Robotics** | 机器人操作任务环境 | https://github.com/Farama-Foundation/Gymnasium-Robotics |
| **Legged Gym** | 四足机器人RL训练 | https://github.com/leggedrobotics/legged_gym |
| **OmniIsaacGymEnvs** | NVIDIA机器人RL环境集 | https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs |

### 4.3 经典论文（机器人+RL方向）

| 年份 | 论文 | 方向 |
|------|------|------|
| 2017 | **Hindsight Experience Replay (HER)** | 稀疏奖励/机械臂操作 |
| 2018 | **Sim-to-Real: Learning Agile Locomotion For Quadruped Robots** | Sim2Real四足 |
| 2019 | **Learning Dexterous In-Hand Manipulation** (OpenAI Rubik's Cube) | 灵巧手操作 |
| 2020 | **Learning Agile Robotic Locomotion Skills by Imitating Animals** | 动物运动模仿 |
| 2021 | **Learning to Walk in Minutes Using Massively Parallel Deep RL** | Isaac Gym大规模训练 |
| 2022 | **Walk These Ways** (Margolis et al.) | 四足多步态控制 |
| 2023 | **Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ACT)** | 双臂模仿学习 |
| 2023 | **DexPoint / DexArt** | 灵巧操作 |
| 2024 | **Humanoid Locomotion (Berkeley Humanoid)** | 人形机器人RL |

---

## 阶段5：前沿方向与持续学习

### 5.1 当前热门研究方向

```
🔥 Sim-to-Real Transfer    — 仿真到真机的迁移（Domain Randomization）
🔥 Imitation Learning      — 模仿学习（ACT, Diffusion Policy）
🔥 Foundation Models + RL  — 大模型+RL（RT-2, Octo）
🔥 Humanoid Locomotion     — 人形机器人运动控制
🔥 Dexterous Manipulation  — 灵巧手操作
🔥 Multi-Agent RL          — 多智能体协作
```

### 5.2 必关注的前沿论文/项目

| 名称 | 说明 |
|------|------|
| **RT-2 / RT-X** (Google DeepMind) | 机器人基础模型 |
| **Diffusion Policy** (Chi et al.) | 扩散模型做机器人策略 |
| **Octo** (Berkeley) | 开源机器人通用策略 |
| **LeRobot** (Hugging Face) | 开源机器人学习框架 https://github.com/huggingface/lerobot |
| **ALOHA / Mobile ALOHA** | 低成本双臂系统 |
| **Genesis** | 新一代机器人仿真平台 https://github.com/Genesis-Embodied-AI/Genesis |

### 5.3 论文跟踪渠道

- **arXiv Robotics**: https://arxiv.org/list/cs.RO/recent
- **Papers With Code**: https://paperswithcode.com/
- **知乎/B站**: 搜索"具身智能"、"机器人强化学习"
- **Twitter/X**: 关注 @GoogleDeepMind, @berkeley_ai, @huggingface
- **会议**: CoRL, RSS, ICRA, IROS, NeurIPS, ICML

---

## 📅 推荐学习时间表

```
Month 1     → 数学补课 + Python/PyTorch + 李宏毅ML前几讲
Month 2-3   → Sutton书前半 + David Silver课 + DQN/PPO实现
Month 3-4   → CS285课 + SAC/TD3 + MuJoCo环境跑通
Month 4-5   → 机器人学基础 + URDF + 仿真平台上手
Month 5-6   → 机械臂/四足RL训练 + 读经典论文
Month 6+    → 选一个方向深入 + 复现论文 + 关注前沿
```

---

## 💡 给小白的建议

1. **先跑通再理解** — 先用 Stable-Baselines3 跑通一个 MuJoCo 任务，建立信心
2. **CleanRL 学原理** — 看单文件实现，理解每一行代码
3. **仿真先行** — 真机太贵太慢，先在仿真里练
4. **一个方向深挖** — 四足/机械臂/灵巧手/人形，选一个方向钻进去
5. **复现论文** — 读10篇不如复现1篇，动手最重要
