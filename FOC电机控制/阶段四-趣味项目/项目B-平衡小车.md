# 项目B：平衡小车（倒立摆）

> **难度**：⭐⭐⭐☆☆ 进阶推荐
> **预计用时**：1-2周
> **效果**：FOC电机 + MPU6050 IMU 实现自平衡，经典控制理论的完美展示

---

## 一、原理说明

### 系统结构

```
[MPU6050 IMU]
      ↓ 倾斜角θ（Roll轴）
[姿态解算] → Kalman/互补滤波
      ↓ 滤波后角度
[位置环PID] → 目标速度
      ↓
[速度环PID] → 目标力矩
      ↓
[FOC力矩控制] → 电机驱动
      ↓
[车轮转动] → 车体平衡
```

### 控制逻辑

倒立摆是经典的不稳定系统，核心控制思路：
- 车体向前倾斜 → 电机向前加速 → 车体恢复竖直
- 车体向后倾斜 → 电机向后加速 → 车体恢复竖直
- 通过**级联PID**（角度环 + 速度环）实现稳定控制

### 坐标定义

- **平衡角度**：车体竖直时IMU读数（需标定，通常不为0°）
- **正方向**：车头朝前为正
- **控制目标**：保持 `θ_actual = θ_balance`

---

## 二、硬件清单

| 器件 | 型号 | 数量 | 说明 |
|------|------|------|------|
| 无刷电机 | 5010 360KV | 2 | 左右轮各一个 |
| FOC驱动板 | SimpleFOC Shield v2 | 2 | 每个电机一块 |
| MCU | STM32F405 / ESP32 | 1 | 主控 |
| IMU | MPU6050 | 1 | 姿态传感器 |
| 磁编码器 | AS5600 | 2 | 左右轮各一个 |
| 电池 | 3S LiPo 2200mAh | 1 | 约11.1V |
| 车架 | 3D打印 | 1 | 见下方设计说明 |
| 车轮 | 直径100mm橡胶轮 | 2 | 增大摩擦力 |

### 3D打印车架设计要点

- 车轮轴距：约200mm（越宽越稳定）
- 重心高度：约150mm（越高越难控制，但效果越震撼）
- 电池放置：尽量靠近车轮轴（降低重心）
- MCU和驱动板：固定在车架中部
- 参考模型：搜索 Thingiverse "self balancing robot FOC"

---

## 三、完整代码

### 主程序（balance_robot.ino）

```cpp
/**
 * FOC平衡小车 - Self Balancing Robot
 * 基于SimpleFOC + MPU6050
 *
 * 控制架构：
 *   角度环PID → 速度环PID → FOC力矩控制
 */

#include <SimpleFOC.h>
#include <Wire.h>
#include "MPU6050.h"
#include "KalmanFilter.h"

// ===== 电机配置 =====
// 左轮
BLDCMotor motorL = BLDCMotor(7);
BLDCDriver3PWM driverL = BLDCDriver3PWM(9, 5, 6, 8);
MagneticSensorI2C sensorL = MagneticSensorI2C(AS5600_I2C);  // I2C地址0x36

// 右轮（注意：右轮安装方向相反，需要反转方向）
BLDCMotor motorR = BLDCMotor(7);
BLDCDriver3PWM driverR = BLDCDriver3PWM(3, 11, 10, 7);
// 右轮编码器使用不同I2C总线或地址（AS5600只有一个地址，需要用TCA9548A多路复用器）

// ===== IMU =====
MPU6050 mpu;
KalmanFilter kalman;

// ===== PID控制器 =====
// 角度环
float Kp_angle = 25.0;
float Ki_angle = 0.0;
float Kd_angle = 0.8;

// 速度环
float Kp_speed = 0.5;
float Ki_speed = 0.02;
float Kd_speed = 0.0;

// PID状态变量
float angle_error_prev = 0;
float angle_integral = 0;
float speed_error_prev = 0;
float speed_integral = 0;

// ===== 平衡参数 =====
float balance_angle = 0.5;   // 平衡点角度（度），需要标定
float max_torque = 1.5;       // 最大力矩限制（A）
float max_speed = 30.0;       // 最大速度限制（rad/s）

// ===== 时间管理 =====
unsigned long lastTime = 0;
float dt = 0.005;  // 控制周期 5ms = 200Hz

// ===== 状态变量 =====
float current_angle = 0;
float current_speed = 0;  // 车轮平均速度
float target_speed = 0;   // 目标速度（遥控输入）
bool isBalancing = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // I2C 400kHz

  Serial.println("平衡小车初始化...");

  // 初始化IMU
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050连接失败！检查接线");
    while(1);
  }
  kalman.setAngle(0);
  Serial.println("MPU6050 OK");

  // 初始化左轮
  sensorL.init();
  motorL.linkSensor(&sensorL);
  driverL.voltage_power_supply = 12;
  driverL.init();
  motorL.linkDriver(&driverL);
  motorL.torque_controller = TorqueControlType::foc_current;
  motorL.controller = MotionControlType::torque;
  motorL.PID_current_q.P = 5; motorL.PID_current_q.I = 300;
  motorL.PID_current_d.P = 5; motorL.PID_current_d.I = 300;
  motorL.current_limit = 3.0;
  motorL.init();
  motorL.initFOC();

  // 初始化右轮（方向取反）
  // motorR 配置同左轮，省略...
  // motorR.sensor_direction = Direction::CW;  // 反转方向

  // 等待静止后标定平衡角
  Serial.println("请将小车竖直放置，3秒后开始标定...");
  delay(3000);
  calibrateBalanceAngle();

  isBalancing = true;
  lastTime = micros();
  Serial.println("开始平衡控制！");
}

void loop() {
  // FOC内环（尽可能快）
  motorL.loopFOC();
  // motorR.loopFOC();

  // 控制外环（固定频率200Hz）
  unsigned long now = micros();
  if (now - lastTime >= 5000) {  // 5000μs = 5ms
    dt = (now - lastTime) / 1e6;
    lastTime = now;

    if (isBalancing) {
      balanceControl();
    }
  }

  // 串口调试
  handleSerial();
}

void balanceControl() {
  // 1. 读取IMU，获取倾斜角度
  current_angle = getFilteredAngle();

  // 2. 读取车轮速度
  current_speed = (motorL.shaft_velocity) / 2.0;  // 单轮速度，双轮取平均

  // 3. 角度环PID → 输出目标速度
  float angle_error = balance_angle - current_angle;
  angle_integral += angle_error * dt;
  angle_integral = constrain(angle_integral, -10, 10);  // 积分限幅
  float angle_derivative = (angle_error - angle_error_prev) / dt;
  angle_error_prev = angle_error;

  float target_wheel_speed = Kp_angle * angle_error
                           + Ki_angle * angle_integral
                           + Kd_angle * angle_derivative;
  target_wheel_speed += target_speed;  // 叠加遥控速度指令
  target_wheel_speed = constrain(target_wheel_speed, -max_speed, max_speed);

  // 4. 速度环PID → 输出力矩
  float speed_error = target_wheel_speed - current_speed;
  speed_integral += speed_error * dt;
  speed_integral = constrain(speed_integral, -2, 2);
  float speed_derivative = (speed_error - speed_error_prev) / dt;
  speed_error_prev = speed_error;

  float torque_cmd = Kp_speed * speed_error
                   + Ki_speed * speed_integral
                   + Kd_speed * speed_derivative;
  torque_cmd = constrain(torque_cmd, -max_torque, max_torque);

  // 5. 安全检查：倾角过大时停止（防止摔倒后电机狂转）
  if (abs(current_angle - balance_angle) > 30.0) {
    motorL.target = 0;
    // motorR.target = 0;
    Serial.println("倾角过大，停止控制！");
    return;
  }

  // 6. 输出力矩
  motorL.target = torque_cmd;
  // motorR.target = -torque_cmd;  // 右轮方向相反
}

float getFilteredAngle() {
  // 读取MPU6050原始数据
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 加速度计计算角度（度）
  float accel_angle = atan2(ay, az) * 180.0 / PI;

  // 陀螺仪角速度（度/秒）
  float gyro_rate = gx / 131.0;  // MPU6050量程±250°/s时，LSB=131

  // Kalman滤波融合
  return kalman.getAngle(accel_angle, gyro_rate, dt);
}

void calibrateBalanceAngle() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += getFilteredAngle();
    delay(10);
  }
  balance_angle = sum / 100.0;
  Serial.print("标定平衡角度: ");
  Serial.print(balance_angle);
  Serial.println("°");
}

void handleSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("P")) { Kp_angle = cmd.substring(1).toFloat(); Serial.print("Kp_angle="); Serial.println(Kp_angle); }
    if (cmd.startsWith("D")) { Kd_angle = cmd.substring(1).toFloat(); Serial.print("Kd_angle="); Serial.println(Kd_angle); }
    if (cmd.startsWith("s")) { target_speed = cmd.substring(1).toFloat(); }
    if (cmd == "stop") { isBalancing = false; motorL.target = 0; }
    if (cmd == "start") { isBalancing = true; }
    if (cmd == "cal") { calibrateBalanceAngle(); }
  }
}
```

### Kalman滤波器（KalmanFilter.h）

```cpp
#pragma once

class KalmanFilter {
public:
  KalmanFilter() {
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;
    angle = 0; bias = 0;
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    // 预测
    rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // 更新
    float S = P[0][0] + R_measure;
    float K[2] = { P[0][0] / S, P[1][0] / S };
    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
    return angle;
  }

  void setAngle(float a) { angle = a; }

private:
  float Q_angle, Q_bias, R_measure;
  float angle, bias, rate;
  float P[2][2];
};
```

---

## 四、PID调参步骤

> 调参是平衡小车最关键也最耗时的环节，按以下顺序进行：

### Step 1：标定平衡角
上电后将小车竖直放置，等待自动标定。若标定不准，手动通过串口发送 `cal` 重新标定。

### Step 2：调角度环 Kp
- 先将 `Ki_angle=0, Kd_angle=0`
- 逐渐增大 `Kp_angle`（发送 `P30`、`P40`...）
- 目标：小车能大致保持平衡但有轻微振荡

### Step 3：加入角度环 Kd 抑制振荡
- 逐渐增大 `Kd_angle`（发送 `D0.5`、`D1.0`...）
- 目标：振荡消失，响应平滑

### Step 4：调速度环 Kp 防止漂移
- 增大 `Kp_speed` 可以减少小车前后漂移
- 过大会导致低频振荡

### 参考初始参数

| 参数 | 参考值 | 说明 |
|------|--------|------|
| Kp_angle | 20~40 | 越大响应越快，过大振荡 |
| Kd_angle | 0.5~2.0 | 阻尼，抑制振荡 |
| Kp_speed | 0.3~1.0 | 防漂移 |
| Ki_speed | 0.01~0.05 | 消除稳态漂移 |

---

## 五、里程碑验收

- [ ] 小车能在平地上稳定平衡30秒以上
- [ ] 轻推后能自动恢复平衡
- [ ] 串口能实时调整PID参数
- [ ] 录制演示视频（含推一下恢复的效果）
- [ ] 代码上传GitHub
