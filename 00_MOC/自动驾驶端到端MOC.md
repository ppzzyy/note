# 自动驾驶端到端技术 MOC

> 从感知到规划的端到端学习框架，目标是应用到自动对焦领域

## 🎯 学习目标

1. 理解端到端自动驾驶的核心思想与架构
2. 掌握主流方法：UniAD、TransFuser、ST-P3
3. 熟悉感知-预测-规划一体化训练
4. 将端到端思想迁移到自动对焦场景

## 📖 内容索引

### 基础理论
- [[01_基础理论/端到端自动驾驶概述]] - 核心概念与发展历程
- [[01_基础理论/imitation_learning|模仿学习基础]]
- [[01_基础理论/reinforcement_learning|强化学习基础]]

### 核心架构
- [[02_端到端架构/UniAD]] - CVPR 2023 Best Paper
- [[02_端到端架构/TransFuser]] - Transformer传感器融合
- [[02_端到端架构/ST-P3]] - 时空特征学习
- [[02_端到端架构/VAD]] - 向量场景表示

### 感知与预测
- [[03_感知与预测/BEV感知]] - 鸟瞰图表示
- [[03_感知与预测/Occupancy]] - 占用网格预测
- [[03_感知与预测/Motion_Forecasting|运动预测]]

### 规划与控制
- [[04_规划与控制/轨迹规划]]
- [[04_规划与控制/行为决策]]

### 实战
- [[05_开源项目/CARLA实战]]
- [[06_论文精读/论文清单]]
- [[07_实战项目/自动对焦应用]]

## 🔗 自动对焦应用方向

端到端思想可迁移到自动对焦：

| 自动驾驶 | 自动对焦 |
|---------|---------|
| 感知：检测目标、预测运动 | 感知：检测主体、场景分析 |
| 规划：轨迹生成 | 规划：对焦策略 |
| 控制：方向盘/油门 | 控制：镜头移动 |

**具体应用：**
1. **场景感知辅助对焦**：理解场景 → 智能选择对焦区域
2. **多帧融合决策**：类似时序融合，多帧对焦信息融合
3. **端到端对焦**：图像 → 直接预测焦距/对焦位置

## 📚 推荐资源

### 课程
- [Udacity Self-Driving Car Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)
- [NVIDIA DLI: Deep Learning for Autonomous Vehicles](https://developer.nvidia.com/blog/dli-training-deep-learning-for-autonomous-vehicles/)
- [MuSHR Deep Learning Tutorial](https://mushr.io/tutorials/deep_learning/)

### 开源项目
- [UniAD](https://github.com/OpenDriveLab/UniAD) - CVPR 2023 Best Paper
- [TransFuser](https://github.com/autonomousvision/transfuser) - PAMI 2023
- [CARLA Simulator](https://github.com/carla-simulator/carla)

### 数据集
- nuScenes
- Waymo Open Dataset
- CARLA (仿真)

---

#自动驾驶 #端到端 #MOC
