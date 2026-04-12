# CARLA 仿真器实战

## 🎮 CARLA 简介

CARLA 是开源的自动驾驶仿真器，基于 Unreal Engine，支持：
- 多传感器（相机、LiDAR、GPS、IMU）
- 真实物理引擎
- 灵活的场景配置
- Python/C++ API

## 📦 安装

### 方式1：预编译包（推荐）
```bash
# 下载 CARLA 0.9.15
wget https://github.com/carla-simulator/carla/releases/download/0.9.15/CARLA_0.9.15.tar.gz
tar -xzf CARLA_0.9.15.tar.gz
cd CARLA_0.9.15

# 安装 Python 客户端
pip install carla
```

### 方式2：Docker
```bash
docker pull carlasim/carla:0.9.15
```

## 🚀 快速开始

### 启动服务器
```bash
cd CARLA_0.9.15
./CarlaUE4.sh
```

### Python 客户端示例
```python
import carla
import random

# 连接服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 加载世界
world = client.load_world('Town01')

# 获取蓝图库
blueprint_library = world.get_blueprint_library()

# 生成车辆
vehicle_bp = blueprint_library.filter('model3')[0]
spawn_point = random.choice(world.get_map().get_spawn_points())
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# 添加摄像头
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera = world.spawn_actor(camera_bp, carla.Transform(), attach_to=vehicle)

# 注册回调
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))
```

## 🔧 训练端到端模型

### 1. 数据收集
使用 CARLA 内置的 autopilot 收集数据：
```bash
cd PythonAPI/examples
python generate_traffic.py -n 50
python manual_control.py --autopilot
```

### 2. 训练模型
参考 TransFuser 或 ST-P3 的训练流程。

## 📚 相关项目

| 项目 | 说明 |
|------|------|
| [TransFuser](https://github.com/autonomousvision/transfuser) | CARLA 端到端 |
| [ST-P3](https://github.com/OpenDriveLab/ST-P3) | CARLA + nuScenes |
| [InterFuser](https://github.com/opendilab/InterFuser) | 多传感器融合 |
| [CARLA Leaderboard](https://leaderboard.carla.org/) | 排行榜 |

## 🎯 练习任务

1. **任务1**：在 Town01 完成自动驾驶一圈
2. **任务2**：收集 10k 帧训练数据
3. **任务3**：训练简单的行为克隆模型
4. **任务4**：实现传感器融合

---

#CARLA #仿真 #实战
