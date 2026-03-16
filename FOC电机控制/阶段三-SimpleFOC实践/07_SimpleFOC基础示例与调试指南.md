# SimpleFOC 基础示例与调试指南

## 概述

本文档覆盖从环境搭建到闭环控制的完整流程，对应计划阶段三（第9-12周）。

---

## 一、开发环境搭建

### 1.1 安装 PlatformIO（推荐）

```bash
# VS Code 扩展市场搜索安装 PlatformIO IDE
# 新建项目时选择：
# Board: STM32F401CCU6 (BlackPill) 或 ESP32 Dev Module
# Framework: Arduino
```

### 1.2 安装 SimpleFOC 库

在 `platformio.ini` 中添加依赖：

```ini
[env:blackpill_f401cc]
platform = ststm32
board = blackpill_f401cc
framework = arduino
lib_deps =
    askuric/Simple FOC @ ^2.3.3
monitor_speed = 115200
```

### 1.3 安装 SimpleFOC Studio（串口调试工具）

```bash
pip install simplefoc-studio
simplefoc-studio
```

---

## 二、硬件连接确认

| 信号         | STM32 引脚 | 说明                  |
|------------|----------|--------------------|
| PWM_A      | PA8      | 驱动板 IN1（A相高侧）      |
| PWM_B      | PA9      | 驱动板 IN2（B相高侧）      |
| PWM_C      | PA10     | 驱动板 IN3（C相高侧）      |
| AS5600_SDA | PB7      | 编码器 I2C 数据          |
| AS5600_SCL | PB6      | 编码器 I2C 时钟          |
| ENABLE     | PB4      | 驱动板使能（高电平有效）       |

> ⚠️ 上电前务必用万用表确认电源正负极，避免反接损坏驱动板。

---

## 三、示例代码

### 3.1 Step 0：编码器读数验证（最先运行）

```cpp
#include <Arduino.h>
#include <Wire.h>

// AS5600 I2C 地址
#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_H 0x0C

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(500);
    Serial.println("AS5600 编码器测试启动...");
}

void loop() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() >= 2) {
        uint16_t raw = (Wire.read() << 8) | Wire.read();
        float angle_deg = raw * 360.0f / 4096.0f;
        Serial.print("原始值: ");
        Serial.print(raw);
        Serial.print("  角度: ");
        Serial.print(angle_deg, 2);
        Serial.println(" °");
    }
    delay(100);
}
```

**验证要点**：手动转动电机轴，角度值应连续变化（0~360°），无跳变。

---

### 3.2 Step 1：开环速度控制

```cpp
#include <Arduino.h>
#include <SimpleFOC.h>

// 三相无刷电机：极对数根据实际电机填写
// 5010 航模电机极对数通常为 7
BLDCMotor motor = BLDCMotor(7);

// 驱动板引脚（3PWM 模式）
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PB4);

void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    // 驱动板初始化
    driver.voltage_power_supply = 24.0f; // 电源电压（V）
    driver.voltage_limit = 12.0f;        // 限制输出电压
    driver.init();

    // 电机绑定驱动板
    motor.linkDriver(&driver);

    // 开环速度控制模式
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = 3.0f; // 开环限压，防止过热

    motor.init();

    Serial.println("开环速度控制就绪，输入目标转速（rad/s）：");
}

float target_velocity = 0;

void loop() {
    // 运行开环控制
    motor.move(target_velocity);

    // 串口接收目标速度
    if (Serial.available()) {
        target_velocity = Serial.parseFloat();
        Serial.print("目标速度: ");
        Serial.print(target_velocity);
        Serial.println(" rad/s");
    }
}
```

**测试步骤**：
1. 串口发送 `2` → 电机以 2 rad/s 旋转
2. 串口发送 `-2` → 反转
3. 串口发送 `0` → 停止
4. 观察电机是否平稳，有无异响

---

### 3.3 Step 2：闭环速度控制（含编码器）

```cpp
#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PB4);

// AS5600 磁编码器（I2C）
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// 串口指令接口
Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    // 编码器初始化
    Wire.begin();
    sensor.init();
    motor.linkSensor(&sensor);

    // 驱动板初始化
    driver.voltage_power_supply = 24.0f;
    driver.init();
    motor.linkDriver(&driver);

    // 速度 PID 参数（初始值，需根据实际调整）
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 20.0f;
    motor.PID_velocity.D = 0.001f;
    motor.LPF_velocity.Tf = 0.01f; // 速度低通滤波时间常数

    motor.voltage_limit = 12.0f;
    motor.velocity_limit = 20.0f; // 最大速度限制（rad/s）

    // 闭环速度控制
    motor.controller = MotionControlType::velocity;

    // 注册串口指令
    command.add('M', onMotor, "电机控制");

    motor.useMonitoring(Serial);
    motor.init();
    motor.initFOC(); // 自动校准电机零位（会转动几圈，属正常现象）

    Serial.println("闭环速度控制就绪！");
    Serial.println("指令格式：M+目标速度（如 M5 表示 5 rad/s）");
}

void loop() {
    motor.loopFOC();   // FOC 电流环（尽量快，不要加延时）
    motor.move();      // 速度环控制

    command.run();     // 处理串口指令
    motor.monitor();   // 输出监控数据到 SimpleFOC Studio
}
```

---

### 3.4 Step 3：闭环位置控制

```cpp
// 在 Step 2 基础上修改以下部分：

// 位置 PID 参数
motor.P_angle.P = 20.0f;
motor.P_angle.I = 0.0f;
motor.P_angle.D = 0.0f;
motor.velocity_limit = 10.0f; // 位置控制时限制最大速度

// 切换为位置控制模式
motor.controller = MotionControlType::angle;

// loop() 中 move() 传入目标角度（弧度）
// 例：motor.move(3.14f); // 转到 π rad（180°）位置
```

**测试步骤**：
1. 串口发送 `M3.14` → 电机转到 180° 并保持
2. 串口发送 `M6.28` → 电机转到 360°
3. 用手轻推电机轴，松手后应自动回到目标位置（位置保持）

---

## 四、SimpleFOC Studio 调试

### 4.1 连接方法

1. 打开 SimpleFOC Studio
2. 选择对应串口，波特率 115200
3. 点击 Connect

### 4.2 监控变量说明

在代码中添加监控标志：

```cpp
motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
motor.monitor_downsample = 100; // 每100次FOC循环输出一次
```

| 变量标志         | 含义         |
|--------------|------------|
| `_MON_TARGET` | 目标值        |
| `_MON_VEL`   | 实际速度       |
| `_MON_ANGLE` | 实际角度       |
| `_MON_CURR_Q` | q轴电流（力矩电流） |
| `_MON_CURR_D` | d轴电流（励磁电流） |

### 4.3 PID 在线调参

通过串口指令实时修改 PID 参数（无需重新烧录）：

```
# 修改速度环 P 参数
MV P0.5

# 修改速度环 I 参数
MV I30

# 查看当前所有参数
M?
```

---

## 五、常见问题排查

| 现象              | 可能原因              | 解决方法                        |
|-----------------|-------------------|-----------------------------|
| 电机抖动不转          | 极对数设置错误           | 查电机规格书，修改 `BLDCMotor(N)` 中的 N |
| initFOC() 失败    | 编码器未检测到或接线错误      | 先运行 Step 0 验证编码器            |
| 电机转动方向相反        | 电机三相线序问题          | 交换任意两根相线，或代码中设置 `motor.sensor_direction` |
| 速度振荡            | PID_velocity.P 过大 | 将 P 减半，逐步增加                 |
| 电机发热严重          | voltage_limit 过高  | 降低 voltage_limit，空载时 3-5V 足够 |
| initFOC() 转圈后停止 | 正常校准流程            | 等待完成，看串口输出 "FOC init success" |

---

## 六、里程碑验证清单

- [ ] 编码器读数正确，手转电机角度连续变化
- [ ] 开环速度控制：电机能正反转，速度可调
- [ ] 闭环速度控制：给定速度后电机稳定运行，扰动后能恢复
- [ ] 位置控制：电机能精确转到指定角度并保持
- [ ] 能通过 SimpleFOC Studio 观察 FOC 波形
- [ ] 能通过串口指令在线修改 PID 参数

完成以上所有项目，即达到阶段三里程碑 ✅
