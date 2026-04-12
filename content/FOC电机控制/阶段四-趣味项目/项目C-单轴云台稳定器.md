# 项目C：单轴云台稳定器

> **难度**：⭐⭐⭐⭐☆ 挑战推荐
> **预计用时**：1-2周
> **效果**：FOC电机 + IMU 实现相机/手机防抖，可实际用于拍摄，成品感极强

---

## 一、原理说明

### 系统结构

```
[MPU6050 IMU]
      ↓ 手持端姿态角θ_handle
[姿态解算] → Kalman/互补滤波
      ↓
[云台控制器] → 目标电机角度 = -θ_handle（补偿抖动）
      ↓
[FOC位置控制] → 电机转到目标角度
      ↓
[相机保持水平]
```

### 核心控制思路

- IMU安装在手持端，实时检测手的抖动角度 `θ_handle`
- 云台电机做**反向补偿**：手向左转5°，电机向右转5°
- 相机固定在电机输出轴上，始终保持绝对水平
- 使用FOC**位置控制模式**，精确跟踪目标角度

### 单轴 vs 三轴

本项目实现**Roll轴（横滚轴）**稳定，即防止左右倾斜抖动。
这是最容易实现且效果最明显的轴，适合入门。
三轴云台需要3个电机和更复杂的欧拉角/四元数解算，可作为后续进阶。

---

## 二、硬件清单

| 器件 | 型号 | 数量 | 说明 |
|------|------|------|------|
| 无刷电机 | GM2804 云台电机 | 1 | 低速大扭矩，适合云台 |
| FOC驱动板 | SimpleFOC Shield v2 | 1 | |
| MCU | STM32F4 / ESP32 | 1 | |
| IMU | MPU6050 或 ICM-42688 | 1 | 安装在手持端 |
| 磁编码器 | AS5600 | 1 | 安装在电机轴 |
| 电池 | 2S LiPo 1000mAh | 1 | 约7.4V，轻量化 |
| 手持架 | 3D打印 | 1 | 见下方设计说明 |
| 相机夹 | 3D打印 | 1 | 固定手机/运动相机 |

### 3D打印结构设计要点

```
[手持握把]
    |
[MCU + 驱动板 + 电池]  ← IMU固定在此处
    |
[电机] ← AS5600编码器对准电机轴
    |
[相机夹] ← 固定手机/GoPro
```

- 电机轴与相机重心对齐（减少静态力矩）
- IMU尽量靠近手持端，远离电机（减少振动干扰）
- 整体重量控制在300g以内（手持舒适）
- 参考模型：Thingiverse 搜索 "single axis gimbal FOC"

---

## 三、完整代码

```cpp
/**
 * 单轴云台稳定器 - Single Axis Gimbal
 * 基于SimpleFOC + MPU6050
 *
 * 控制模式：FOC位置控制
 * 稳定轴：Roll（横滚轴）
 */

#include <SimpleFOC.h>
#include <Wire.h>
#include "MPU6050.h"
#include "KalmanFilter.h"  // 复用项目B中的Kalman滤波器

// ===== 电机配置 =====
BLDCMotor motor = BLDCMotor(7);  // 极对数根据实际电机修改
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ===== IMU =====
MPU6050 mpu;
KalmanFilter kalman;

// ===== 云台控制参数 =====
// 位置环PID（SimpleFOC内置）
float Kp_pos = 8.0;    // 位置环比例增益
float Ki_pos = 0.0;    // 通常不需要积分
float Kd_pos = 0.3;    // 微分增益，抑制超调

// 速度环PID（SimpleFOC内置）
float Kp_vel = 0.5;
float Ki_vel = 10.0;
float Kd_vel = 0.0;

// 滤波参数
float alpha = 0.1;     // 目标角度低通滤波系数（0~1，越小越平滑）
float filtered_target = 0.0;

// 工作模式
enum GimbalMode {
  MODE_STABILIZE,   // 稳定模式：补偿手持抖动
  MODE_FOLLOW,      // 跟随模式：缓慢跟随手持方向
  MODE_LOCK         // 锁定模式：锁定在固定角度
};
GimbalMode currentMode = MODE_STABILIZE;

// 状态变量
float handle_angle = 0;    // 手持端角度（度）
float target_angle = 0;    // 电机目标角度（弧度）
float lock_angle = 0;      // 锁定角度
unsigned long lastTime = 0;
float dt = 0.002;          // 控制周期 2ms = 500Hz

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("单轴云台初始化...");

  // 初始化IMU
  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);  // 低通滤波42Hz，减少高频噪声
  if (!mpu.testConnection()) {
    Serial.println("MPU6050连接失败！");
    while(1);
  }
  Serial.println("MPU6050 OK");

  // 初始化编码器
  sensor.init();
  motor.linkSensor(&sensor);

  // 驱动器配置
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // FOC位置控制模式
  motor.controller = MotionControlType::angle;

  // 位置环PID
  motor.PID_velocity.P = Kp_vel;
  motor.PID_velocity.I = Ki_vel;
  motor.PID_velocity.D = Kd_vel;

  // 角度环PID
  motor.P_angle.P = Kp_pos;
  motor.P_angle.I = Ki_pos;
  motor.P_angle.D = Kd_pos;

  // 速度和电流限制
  motor.velocity_limit = 20;   // rad/s
  motor.voltage_limit = 8;     // V（云台电机电压不宜过高）

  // 初始化电机
  motor.init();
  motor.initFOC();

  // 等待静止，记录初始角度
  delay(2000);
  lock_angle = getHandleAngle();
  filtered_target = lock_angle;
  Serial.print("初始角度: "); Serial.println(lock_angle);

  lastTime = micros();
  Serial.println("云台就绪！");
}

void loop() {
  // FOC内环（尽可能快）
  motor.loopFOC();

  // 控制外环（500Hz）
  unsigned long now = micros();
  if (now - lastTime >= 2000) {
    dt = (now - lastTime) / 1e6;
    lastTime = now;
    gimbalControl();
  }

  // 串口调试
  handleSerial();
}

void gimbalControl() {
  // 1. 读取手持端角度
  handle_angle = getHandleAngle();

  // 2. 根据模式计算目标角度
  float raw_target = 0;
  switch (currentMode) {
    case MODE_STABILIZE:
      // 稳定模式：电机补偿手持抖动，相机保持绝对水平
      raw_target = -handle_angle;  // 反向补偿
      break;

    case MODE_FOLLOW:
      // 跟随模式：缓慢跟随手持方向（死区内不动，超出死区缓慢跟随）
      {
        float deadzone = 5.0;  // 5度死区
        float diff = handle_angle - (-filtered_target);
        if (abs(diff) > deadzone) {
          raw_target = filtered_target - (diff - deadzone * sign(diff)) * 0.1;
        } else {
          raw_target = filtered_target;
        }
      }
      break;

    case MODE_LOCK:
      // 锁定模式：保持在lock_angle，忽略手持运动
      raw_target = lock_angle;
      break;
  }

  // 3. 低通滤波，平滑目标角度（减少抖动传递）
  filtered_target = alpha * raw_target + (1 - alpha) * filtered_target;

  // 4. 转换为弧度并发送给电机
  target_angle = filtered_target * PI / 180.0;
  motor.target = target_angle;

  // 5. 调试输出（每50ms输出一次）
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    Serial.print("handle:"); Serial.print(handle_angle, 1);
    Serial.print("\ttarget:"); Serial.print(filtered_target, 1);
    Serial.print("\tmotor:"); Serial.println(motor.shaft_angle * 180 / PI, 1);
  }
}

float getHandleAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Roll角（横滚）：绕X轴旋转
  float accel_angle = atan2(ay, az) * 180.0 / PI;
  float gyro_rate = gx / 131.0;  // ±250°/s量程

  return kalman.getAngle(accel_angle, gyro_rate, dt);
}

float sign(float x) {
  return (x > 0) ? 1.0 : ((x < 0) ? -1.0 : 0.0);
}

void handleSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "s") {
      currentMode = MODE_STABILIZE;
      Serial.println("切换到稳定模式");
    } else if (cmd == "f") {
      currentMode = MODE_FOLLOW;
      Serial.println("切换到跟随模式");
    } else if (cmd == "l") {
      currentMode = MODE_LOCK;
      lock_angle = filtered_target;
      Serial.print("锁定角度: "); Serial.println(lock_angle);
    } else if (cmd.startsWith("P")) {
      Kp_pos = cmd.substring(1).toFloat();
      motor.P_angle.P = Kp_pos;
      Serial.print("Kp_pos="); Serial.println(Kp_pos);
    } else if (cmd.startsWith("D")) {
      Kd_pos = cmd.substring(1).toFloat();
      motor.P_angle.D = Kd_pos;
      Serial.print("Kd_pos="); Serial.println(Kd_pos);
    } else if (cmd.startsWith("a")) {
      alpha = cmd.substring(1).toFloat();
      Serial.print("alpha="); Serial.println(alpha);
    }
  }
}
```

---

## 四、调试步骤

### Step 1：验证IMU读数
先单独测试IMU，确认Roll角读数方向正确：
- 向左倾斜 → 角度为负
- 向右倾斜 → 角度为正
- 如方向相反，在代码中取反：`accel_angle = -atan2(ay, az) * 180.0 / PI`

### Step 2：验证电机位置控制
不接IMU，手动发送目标角度，验证电机能精确跟踪：
```
// 串口发送数字，测试电机转到对应角度（弧度）
motor.target = 1.57;  // 90度
```

### Step 3：调整位置环PID
| 参数 | 参考值 | 效果 |
|------|--------|------|
| Kp_pos | 5~15 | 越大响应越快，过大振荡 |
| Kd_pos | 0.1~0.5 | 抑制超调 |
| Kp_vel | 0.3~1.0 | 速度环增益 |

### Step 4：调整滤波系数 alpha
- `alpha = 0.1`：非常平滑，但响应慢（适合慢速运动）
- `alpha = 0.5`：响应较快，轻微抖动
- 根据实际拍摄需求调整

### Step 5：实拍测试
- 手持云台走动，观察相机是否保持水平
- 快速抖动手腕，观察稳定效果
- 录制对比视频（有/无稳定器）

---

## 五、进阶功能

### 1. 加入跟随延迟（Follow Delay）
```cpp
// 用一阶低通滤波实现跟随延迟
float follow_alpha = 0.02;  // 越小跟随越慢
filtered_follow = follow_alpha * handle_angle + (1 - follow_alpha) * filtered_follow;
```

### 2. 加入自动水平校准
上电时自动检测水平面，消除安装误差：
```cpp
float install_offset = 0;
void calibrateHorizon() {
  install_offset = getHandleAngle();  // 记录当前角度为水平基准
}
// 使用时：handle_angle_corrected = handle_angle - install_offset
```

### 3. 升级为两轴云台
在Roll轴基础上增加Pitch轴（俯仰轴），需要：
- 再增加一个电机 + 编码器 + 驱动板
- IMU读取Pitch角（绕Y轴）
- 两轴独立控制，互不干扰

---

## 六、里程碑验收

- [ ] 云台能稳定补偿±30°范围内的手持抖动
- [ ] 三种模式（稳定/跟随/锁定）均正常工作
- [ ] 录制对比视频：手持手机 vs 手持云台拍摄同一场景
- [ ] 代码上传GitHub，附上演示视频链接
- [ ] 写一篇技术总结博客（可发布到知乎/B站）
