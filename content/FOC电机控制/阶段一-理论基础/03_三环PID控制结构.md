# 三环PID控制结构：FOC电机控制的核心调节器

## 1. 为什么需要三环控制？

FOC算法解决了"如何高效产生旋转磁场"的问题，而**三环PID**解决的是"如何精确控制电机的电流、速度和位置"的问题。两者缺一不可。

```
位置指令 → [位置环PID] → 速度指令 → [速度环PID] → 电流指令 → [电流环PID] → PWM占空比 → 电机
                ↑                        ↑                       ↑
           位置反馈(编码器)          速度反馈(编码器微分)       电流反馈(ADC采样)
```

三环从外到内：
| 环路 | 控制量 | 反馈来源 | 典型带宽 | 采样频率 |
|------|--------|----------|----------|----------|
| 位置环（外环） | 转子角度 θ | 编码器 | 10~50 Hz | 1 kHz |
| 速度环（中环） | 转速 ω | 编码器微分 | 50~200 Hz | 1~5 kHz |
| 电流环（内环） | d/q轴电流 | ADC采样 | 1~5 kHz | 10~50 kHz |

> **关键原则**：内环带宽必须比外环高 5~10 倍，否则系统不稳定。

---

## 2. PID控制器基础

### 2.1 连续域 PID 公式

$$u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

- $e(t) = r(t) - y(t)$：误差 = 目标值 - 实际值
- $K_p$：比例增益，决定响应速度
- $K_i$：积分增益，消除稳态误差
- $K_d$：微分增益，抑制超调、预测趋势

### 2.2 离散域 PID 公式（嵌入式实现）

$$u[k] = K_p \cdot e[k] + K_i \cdot T_s \sum_{j=0}^{k} e[j] + K_d \cdot \frac{e[k] - e[k-1]}{T_s}$$

其中 $T_s$ 为采样周期。

### 2.3 各参数的物理直觉

```
Kp 太小 → 响应慢，稳态误差大
Kp 太大 → 振荡，甚至不稳定

Ki 太小 → 稳态误差消除慢
Ki 太大 → 积分饱和（windup），超调严重

Kd 太小 → 超调大
Kd 太大 → 对噪声敏感，高频振荡
```

---

## 3. 三环详解

### 3.1 电流环（最内环）⚡

**目标**：控制 d 轴电流 $I_d = 0$，q 轴电流 $I_q = I_{q,ref}$（力矩指令）

**为什么最重要**：
- 电流直接决定力矩（$T = \frac{3}{2} p \psi_f I_q$）
- 带宽最高，决定整个系统的动态性能上限
- 电流环不稳定 → 电机烧毁

**典型参数范围**（SimpleFOC中）：
```cpp
motor.PID_current_q.P = 5;    // Kp，通常 1~20
motor.PID_current_q.I = 100;  // Ki，通常 10~500
motor.PID_current_q.D = 0;    // 电流环通常不用D（噪声太大）
motor.PID_current_q.limit = 12; // 输出限幅（V）
```

### 3.2 速度环（中环）🔄

**目标**：控制转速 ω 跟踪速度指令 $\omega_{ref}$

**速度计算**：
$$\omega = \frac{\Delta\theta}{\Delta t} = \frac{\theta[k] - \theta[k-1]}{T_s}$$

注意：直接微分噪声大，实际中常用**低通滤波器**平滑速度：
$$\omega_{filtered} = \alpha \cdot \omega_{raw} + (1-\alpha) \cdot \omega_{filtered,prev}$$

**典型参数范围**：
```cpp
motor.PID_velocity.P = 0.5;   // Kp
motor.PID_velocity.I = 10;    // Ki
motor.PID_velocity.D = 0;     // 速度环D项通常为0
motor.LPF_velocity.Tf = 0.01; // 速度低通滤波时间常数（s）
```

### 3.3 位置环（外环）📍

**目标**：控制转子角度 θ 精确到达目标位置 $\theta_{ref}$

**注意事项**：
- 位置环输出是速度指令，需要限幅（防止突然大速度）
- 多圈位置需要处理角度累积（编码器计数）
- 接近目标时速度自动降低（P控制器天然具备此特性）

**典型参数范围**：
```cpp
motor.P_angle.P = 20;         // 位置环通常只用P控制器
motor.P_angle.I = 0;          // I项容易导致位置超调
motor.P_angle.D = 0;
motor.velocity_limit = 10;    // 位置环输出速度限幅（rad/s）
```

---

## 4. PID 参数整定方法

### 4.1 Ziegler-Nichols 临界增益法

1. 将 Ki = 0，Kd = 0，只保留 Kp
2. 逐渐增大 Kp，直到系统出现**等幅振荡**
3. 记录此时的临界增益 $K_u$ 和振荡周期 $T_u$
4. 按下表计算参数：

| 控制器类型 | Kp | Ki | Kd |
|-----------|----|----|-----|
| P | $0.5 K_u$ | — | — |
| PI | $0.45 K_u$ | $0.54 K_u / T_u$ | — |
| PID | $0.6 K_u$ | $1.2 K_u / T_u$ | $0.075 K_u T_u$ |

### 4.2 经验调参顺序（推荐）

```
Step 1: 先调电流环
  → 增大 Kp 直到电流响应快速无振荡
  → 增大 Ki 消除稳态误差
  → 验证：阶跃电流指令，响应时间 < 1ms

Step 2: 再调速度环
  → 电流环稳定后，增大速度环 Kp
  → 加 Ki 消除速度稳态误差
  → 调 LPF_velocity.Tf 平衡噪声和响应速度
  → 验证：阶跃速度指令，超调 < 10%

Step 3: 最后调位置环
  → 通常只需调 P_angle.P
  → 增大直到位置响应快速，出现轻微超调时回退 20%
  → 验证：阶跃位置指令，无持续振荡
```

### 4.3 积分饱和（Anti-Windup）

当执行器饱和（输出已达上限）时，积分项仍在累积，导致超调严重。解决方案：

```cpp
// 方法1：限制积分项范围
integral = constrain(integral + Ki * error * dt, -limit, limit);

// 方法2：条件积分（只在未饱和时积分）
if (abs(output) < output_limit) {
    integral += Ki * error * dt;
}
```

---

## 5. 频域分析视角（进阶）

### 5.1 开环传递函数

对于电流环，电机绕组可建模为 RL 电路：

$$G_{plant}(s) = \frac{1}{Ls + R}$$

PID控制器传递函数：

$$C(s) = K_p + \frac{K_i}{s} + K_d s = \frac{K_d s^2 + K_p s + K_i}{s}$$

开环传递函数：$L(s) = C(s) \cdot G_{plant}(s)$

### 5.2 稳定性判据

- **相位裕度** > 45°（推荐 60°）：系统稳定且有足够阻尼
- **增益裕度** > 6 dB：系统对参数变化有鲁棒性
- 可用 Python `scipy.signal` 绘制 Bode 图验证

---

## 6. 里程碑验证清单

- [ ] 能够解释为什么内环带宽必须高于外环
- [ ] 能够手写离散 PID 的 C 代码实现
- [ ] 理解积分饱和问题及其解决方案
- [ ] 完成 Python 速度环仿真，观察不同 Kp/Ki 下的阶跃响应
- [ ] 理解 Ziegler-Nichols 法的使用场景和局限性

---

## 7. 参考资料

- 《现代控制工程》（Ogata）第 8 章 PID 控制
- SimpleFOC 官方文档：[Motion Control](https://docs.simplefoc.com/motion_control)
- Texas Instruments：《Digital Motor Control》应用笔记 SPRABQ8
- Brian Douglas YouTube：《PID Control - A brief introduction》
