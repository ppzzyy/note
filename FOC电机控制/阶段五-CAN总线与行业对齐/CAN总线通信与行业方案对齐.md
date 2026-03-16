# CAN总线通信与行业方案对齐

> 阶段五 · 第21-24周 | 里程碑：能够描述机器人关节电机控制的完整技术链路

---

## 一、CAN总线协议基础

### 1.1 为什么机器人用CAN总线？

| 特性 | CAN总线 | UART | SPI | I2C |
|------|---------|------|-----|-----|
| 拓扑 | 多主总线（最多110节点） | 点对点 | 点对点 | 多从单主 |
| 速率 | 1 Mbps（标准）/ 5 Mbps（FD） | 最高~5 Mbps | 最高50 Mbps | 最高3.4 Mbps |
| 抗干扰 | 差分信号，极强 | 弱 | 弱 | 弱 |
| 错误检测 | 硬件CRC + 应答 + 错误帧 | 无/软件 | 无 | 应答位 |
| 实时性 | 优先级仲裁，确定性延迟 | 无优先级 | 无优先级 | 无优先级 |
| 典型应用 | 机器人关节、汽车ECU | 调试串口 | 传感器 | 低速传感器 |

**结论**：CAN总线天生为多节点、强干扰、实时性要求高的场景设计，完美匹配机器人多关节控制需求。

---

### 1.2 CAN帧格式详解

```
标准帧（11位ID）结构：
┌─────┬────────┬───┬───┬──────────┬─────┬───┬──────┐
│ SOF │ ID[11] │IDE│RTR│ DLC[4]   │DATA │CRC│ ACK  │
│  1  │   11   │ 1 │ 1 │    4     │0~64 │16 │  2   │
└─────┴────────┴───┴───┴──────────┴─────┴───┴──────┘

扩展帧（29位ID）结构：
┌─────┬────────┬───┬────────┬───┬──────────┬─────┬───┐
│ SOF │ ID[11] │SRR│ ID[18] │RTR│ DLC[4]   │DATA │CRC│
└─────┴────────┴───┴────────┴───┴──────────┴─────┴───┘
```

**关键字段说明**：
- **ID（仲裁ID）**：决定消息优先级，ID越小优先级越高
- **DLC**：数据长度码，0~8字节（标准CAN），CAN FD最高64字节
- **RTR**：远程帧标志，用于请求数据
- **CRC**：15位循环冗余校验，硬件自动计算

### 1.3 总线仲裁机制（CSMA/CD+AMP）

```
节点A发送ID: 0x001  → 1 0 0 0 0 0 0 0 0 0 1
节点B发送ID: 0x003  → 1 0 0 0 0 0 0 0 0 1 1
                                          ↑
                              节点B在此位检测到冲突
                              节点B退出，节点A继续发送
```

**规则**：显性位（0）覆盖隐性位（1），ID小的节点赢得总线。

### 1.4 波特率与终端电阻

```
常用波特率配置（STM32 CAN，42MHz APB1时钟）：
- 1 Mbps:  Prescaler=3,  BS1=11, BS2=2, SJW=1
- 500 Kbps: Prescaler=6,  BS1=11, BS2=2, SJW=1
- 250 Kbps: Prescaler=12, BS1=11, BS2=2, SJW=1

终端电阻：总线两端各接120Ω，防止信号反射
总线长度 vs 波特率：
  1 Mbps  → 最长 40m
  500Kbps → 最长 100m
  250Kbps → 最长 250m
```

---

## 二、MIT Mini Cheetah电机控制协议

### 2.1 协议概述

MIT Mini Cheetah是开源四足机器人，其电机控制协议被广泛用于机器人行业，是学习关节电机控制的最佳参考。

**GitHub**: https://github.com/bgkatz/motorcontrol

**核心特点**：
- 基于CAN总线，单总线最多支持多个电机节点
- 支持**位置/速度/力矩混合控制**（MIT模式）
- 控制周期 1kHz，延迟极低
- 每帧8字节，信息密度极高

### 2.2 MIT模式控制帧格式

**发送帧（主控 → 电机，8字节）**：

```
Byte[0:1] = p_des  (期望位置, 16bit, 范围 -12.5~12.5 rad)
Byte[2:3] = v_des  (期望速度, 12bit, 范围 -65~65 rad/s)
Byte[4]   = kp     (位置增益, 12bit高4位, 范围 0~500 N·m/rad)
Byte[5]   = kd     (速度增益, 12bit低8位, 范围 0~100 N·m·s/rad)
Byte[6:7] = t_ff   (前馈力矩, 12bit, 范围 -18~18 N·m)
```

**实际控制律**：
```
τ = kp * (p_des - p_meas) + kd * (v_des - v_meas) + t_ff
```

这是一个**阻抗控制**公式，通过调整kp/kd可以实现：
- `kp=0, kd=0, t_ff≠0` → 纯力矩控制
- `kp≠0, kd≠0, t_ff=0` → 位置+速度控制（类弹簧阻尼）
- `kp≠0, kd≠0, t_ff≠0` → 完整阻抗控制

**接收帧（电机 → 主控，6字节）**：

```
Byte[0:1] = p_meas (实测位置, 16bit)
Byte[2:3] = v_meas (实测速度, 12bit高位)
Byte[4:5] = i_meas (实测电流, 12bit低位)
```

### 2.3 Python仿真MIT协议打包/解包

```python
import struct
import numpy as np

# 参数范围定义
P_MIN, P_MAX = -12.5, 12.5    # rad
V_MIN, V_MAX = -65.0, 65.0    # rad/s
KP_MIN, KP_MAX = 0.0, 500.0   # N·m/rad
KD_MIN, KD_MAX = 0.0, 100.0   # N·m·s/rad
T_MIN, T_MAX = -18.0, 18.0    # N·m

def float_to_uint(x, x_min, x_max, bits):
    """将浮点数映射到无符号整数"""
    span = x_max - x_min
    x = max(x_min, min(x_max, x))  # 限幅
    return int((x - x_min) * ((1 << bits) - 1) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    """将无符号整数还原为浮点数"""
    span = x_max - x_min
    return x_int * span / ((1 << bits) - 1) + x_min

def pack_cmd(p_des, v_des, kp, kd, t_ff):
    """
    打包MIT模式控制指令
    返回8字节bytearray
    """
    p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12)
    t_int  = float_to_uint(t_ff,  T_MIN, T_MAX, 12)

    buf = bytearray(8)
    buf[0] = (p_int >> 8) & 0xFF
    buf[1] = p_int & 0xFF
    buf[2] = (v_int >> 4) & 0xFF
    buf[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    buf[4] = kp_int & 0xFF
    buf[5] = (kd_int >> 4) & 0xFF
    buf[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    buf[7] = t_int & 0xFF
    return buf

def unpack_reply(buf):
    """
    解包电机回复帧（6字节）
    返回 (position_rad, velocity_rad_s, current_A)
    """
    p_int = (buf[0] << 8) | buf[1]
    v_int = (buf[2] << 4) | (buf[3] >> 4)
    i_int = ((buf[3] & 0xF) << 8) | buf[4]

    p = uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = uint_to_float(v_int, V_MIN, V_MAX, 12)
    i = uint_to_float(i_int, -40.0, 40.0, 12)
    return p, v, i

# ---- 测试 ----
if __name__ == "__main__":
    # 发送：期望位置1.0rad，速度0，kp=10，kd=1，前馈力矩0
    cmd = pack_cmd(p_des=1.0, v_des=0.0, kp=10.0, kd=1.0, t_ff=0.0)
    print(f"控制帧 (hex): {cmd.hex(' ').upper()}")

    # 模拟电机回复
    reply = bytearray(6)
    # 假设电机回复位置0.98rad，速度0.1rad/s，电流2A
    p_int = float_to_uint(0.98, P_MIN, P_MAX, 16)
    v_int = float_to_uint(0.1,  V_MIN, V_MAX, 12)
    i_int = float_to_uint(2.0,  -40.0, 40.0, 12)
    reply[0] = (p_int >> 8) & 0xFF
    reply[1] = p_int & 0xFF
    reply[2] = (v_int >> 4) & 0xFF
    reply[3] = ((v_int & 0xF) << 4) | ((i_int >> 8) & 0xF)
    reply[4] = i_int & 0xFF
    reply[5] = 0

    p, v, i = unpack_reply(reply)
    print(f"电机回复: 位置={p:.3f}rad, 速度={v:.3f}rad/s, 电流={i:.3f}A")
```

---

## 三、大疆RoboMaster GM6020 CAN控制

### 3.1 GM6020协议概述

GM6020是大疆RoboMaster系列云台电机，内置FOC控制器，通过CAN总线接收力矩指令。

**CAN ID分配**：
```
发送（主控 → 电机）：
  0x1FF  → 控制电机ID 1~4（电流值）
  0x2FF  → 控制电机ID 5~7（电流值）

接收（电机 → 主控）：
  0x205  → 电机ID 1 反馈
  0x206  → 电机ID 2 反馈
  ...
  0x20B  → 电机ID 7 反馈
```

### 3.2 控制帧格式

**发送帧（8字节，控制4个电机）**：
```
Byte[0:1] = 电机1电流值 (int16, -30000~30000, 对应-3A~3A)
Byte[2:3] = 电机2电流值
Byte[4:5] = 电机3电流值
Byte[6:7] = 电机4电流值
```

**反馈帧（8字节）**：
```
Byte[0:1] = 转子机械角度 (0~8191, 对应0~360°)
Byte[2:3] = 转子转速 (rpm, int16)
Byte[4:5] = 实际转矩电流 (int16)
Byte[6]   = 电机温度 (°C)
Byte[7]   = 保留
```

### 3.3 STM32 CAN控制GM6020代码示例

```c
#include "stm32f4xx_hal.h"

/* CAN句柄（需在CubeMX中配置） */
extern CAN_HandleTypeDef hcan1;

/* 电机反馈数据结构 */
typedef struct {
    uint16_t angle;      // 机械角度 0~8191
    int16_t  speed;      // 转速 rpm
    int16_t  current;    // 实际电流
    uint8_t  temperature;
} GM6020_Feedback_t;

GM6020_Feedback_t motor_fb[4] = {0};  // 4个电机反馈

/**
 * @brief 发送GM6020电流控制指令
 * @param cur1~cur4: 电机1~4电流值，范围-30000~30000
 */
void GM6020_SendCurrent(int16_t cur1, int16_t cur2,
                         int16_t cur3, int16_t cur4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.StdId = 0x1FF;          // 控制ID 1~4
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    tx_data[0] = (cur1 >> 8) & 0xFF;
    tx_data[1] = cur1 & 0xFF;
    tx_data[2] = (cur2 >> 8) & 0xFF;
    tx_data[3] = cur2 & 0xFF;
    tx_data[4] = (cur3 >> 8) & 0xFF;
    tx_data[5] = cur3 & 0xFF;
    tx_data[6] = (cur4 >> 8) & 0xFF;
    tx_data[7] = cur4 & 0xFF;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

/**
 * @brief CAN接收中断回调，解析GM6020反馈帧
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /* 判断是否为GM6020反馈帧 (0x205~0x208) */
    if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208) {
        uint8_t idx = rx_header.StdId - 0x205;  // 电机索引 0~3
        motor_fb[idx].angle       = (rx_data[0] << 8) | rx_data[1];
        motor_fb[idx].speed       = (rx_data[2] << 8) | rx_data[3];
        motor_fb[idx].current     = (rx_data[4] << 8) | rx_data[5];
        motor_fb[idx].temperature = rx_data[6];
    }
}

/**
 * @brief 简单速度控制示例（电机1）
 * 在1kHz定时器中断中调用
 */
void Motor1_SpeedControl(float target_rpm)
{
    static float integral = 0.0f;
    float kp = 10.0f, ki = 0.5f;

    float error = target_rpm - (float)motor_fb[0].speed;
    integral += error * 0.001f;  // dt = 1ms

    /* 限制积分防止饱和 */
    integral = fmaxf(-5000.0f, fminf(5000.0f, integral));

    float output = kp * error + ki * integral;
    int16_t current_cmd = (int16_t)fmaxf(-30000.0f,
                                          fminf(30000.0f, output));

    GM6020_SendCurrent(current_cmd, 0, 0, 0);
}
```

---

## 四、宇树科技电机方案研究

### 4.1 宇树Go1/H1技术栈概览

```
宇树机器人电机控制技术链路：

上层控制（ROS2 / 强化学习策略）
        ↓ UDP / 以太网
中层控制（运动控制器，1kHz）
        ↓ CAN FD / RS485
底层驱动（关节电机驱动器）
        ↓ FOC算法（10~20kHz）
执行器（PMSM + 磁编码器）
```

**关键参数**（Go1关节电机）：
| 参数 | 数值 |
|------|------|
| 峰值力矩 | 23.7 N·m |
| 额定转速 | 2900 rpm |
| 减速比 | 1:6.33 |
| 编码器分辨率 | 14bit（16384线） |
| 控制频率 | 1kHz |
| 通信接口 | CAN FD（5Mbps） |

### 4.2 宇树SDK电机控制接口（unitree_legged_sdk）

```python
# 宇树SDK Python接口示例（基于unitree_legged_sdk）
# 参考：https://github.com/unitreerobotics/unitree_legged_sdk

import sys
sys.path.append('../lib/python/amd64')
import robot_interface as sdk

# 初始化UDP通信
udp = sdk.UDP(0xee, 8080, "192.168.123.10", 8007)
safe = sdk.Safety(sdk.LeggedType.Go1)

# 电机控制命令结构
cmd = sdk.LowCmd()
state = sdk.LowState()

udp.InitCmdData(cmd)

def control_loop():
    """1kHz控制循环"""
    udp.Recv()
    udp.GetRecv(state)

    # 读取关节状态（以右前腿髋关节为例，电机索引0）
    q   = state.motorState[0].q      # 关节角度 (rad)
    dq  = state.motorState[0].dq     # 关节速度 (rad/s)
    tau = state.motorState[0].tauEst # 估计力矩 (N·m)

    # 发送阻抗控制指令（MIT模式）
    cmd.motorCmd[0].q   = 0.0    # 期望位置
    cmd.motorCmd[0].dq  = 0.0    # 期望速度
    cmd.motorCmd[0].Kp  = 20.0   # 位置增益
    cmd.motorCmd[0].Kd  = 0.5    # 速度增益
    cmd.motorCmd[0].tau = 0.0    # 前馈力矩

    safe.PowerProtect(cmd, state, 1)  # 安全保护
    udp.SetSend(cmd)
    udp.Send()
```

---

## 五、完整技术链路总结

```
机器人关节电机控制完整技术链路：

┌─────────────────────────────────────────────────────┐
│                   上层任务规划                        │
│         (ROS2 Nav2 / 强化学习 / 轨迹规划)             │
└──────────────────────┬──────────────────────────────┘
                       │ 关节角度/速度/力矩目标 (100Hz)
┌──────────────────────▼──────────────────────────────┐
│                   运动控制器                          │
│    (逆运动学 / 步态规划 / 全身控制 WBC, 1kHz)         │
└──────────────────────┬──────────────────────────────┘
                       │ CAN FD / EtherCAT (1kHz)
┌──────────────────────▼──────────────────────────────┐
│                  关节驱动器                           │
│   阻抗控制律: τ = kp*(q_d-q) + kd*(dq_d-dq) + τ_ff  │
└──────────────────────┬──────────────────────────────┘
                       │ 电流指令 (10~20kHz)
┌──────────────────────▼──────────────────────────────┐
│                   FOC算法                            │
│  Clark变换 → Park变换 → PI电流环 → SVPWM → 逆变器    │
└──────────────────────┬──────────────────────────────┘
                       │ 三相PWM
┌──────────────────────▼──────────────────────────────┐
│              PMSM电机 + 磁编码器                      │
│         (输出力矩，反馈位置/速度)                     │
└─────────────────────────────────────────────────────┘
```

---

## 六、作品集整理建议

### 6.1 GitHub仓库结构

```
foc-motor-learning/
├── 01_theory/
│   ├── clark_park_simulation.py      # 任务2：坐标变换仿真
│   └── pid_simulation.py             # 任务3：PID仿真
├── 02_hardware/
│   ├── wiring_diagram.pdf            # 任务6：接线图
│   └── 3d_models/                    # 任务5：3D打印文件
├── 03_simplefoc/
│   ├── basic_velocity_control/       # 任务7：速度控制
│   └── torque_control/               # 任务8：力矩控制
├── 04_projects/
│   ├── music_motor/                  # 任务9A：音乐电机
│   ├── balance_car/                  # 任务9B：平衡小车
│   └── gimbal_stabilizer/            # 任务9C：云台稳定器
├── 05_can_protocol/
│   ├── mit_protocol.py               # 本任务：MIT协议
│   └── gm6020_stm32/                 # 本任务：GM6020控制
└── README.md
```

### 6.2 技术博客发布建议

推荐发布平台：**知乎 + B站**

文章系列建议：
1. 《从零开始学FOC：三相无刷电机工作原理》
2. 《手推Clark/Park变换，彻底搞懂FOC数学》
3. 《SimpleFOC实战：用Arduino跑起来你的第一个FOC电机》
4. 《音乐电机：用FOC让电机唱歌》
5. 《CAN总线入门：机器人关节电机通信协议详解》
6. 《MIT Mini Cheetah协议解析：机器人行业的电机控制标准》

---

## 七、里程碑自测清单

完成本阶段后，你应该能够回答以下问题：

- [ ] CAN总线为什么比UART/I2C更适合机器人多关节控制？
- [ ] CAN总线仲裁机制是如何工作的？ID越小优先级越高的原因？
- [ ] MIT Mini Cheetah的控制律公式是什么？kp/kd/t_ff各控制什么？
- [ ] 阻抗控制和纯位置控制的区别是什么？
- [ ] 宇树Go1的电机控制频率是多少？通信接口是什么？
- [ ] 从强化学习策略到电机PWM，完整的技术链路有哪些层次？
- [ ] 如何用Python打包/解包MIT协议的CAN帧？
