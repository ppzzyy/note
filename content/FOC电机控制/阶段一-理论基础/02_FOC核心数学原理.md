# FOC核心数学原理

> 对应计划任务2 | 预计学习时间：1.5周

---

## 一、为什么需要坐标变换？

三相无刷电机的三相电流 $i_a, i_b, i_c$ 是随转子位置不断变化的**交流量**，直接控制非常困难。
FOC（磁场定向控制）的核心思想是：**通过两次坐标变换，将交流量转换为直流量，从而像控制直流电机一样控制无刷电机。**

变换链路：
```
三相静止坐标系 (abc)
        ↓  Clark变换
两相静止坐标系 (αβ)
        ↓  Park变换
两相旋转坐标系 (dq)  ← 在这里做PID控制（直流量！）
        ↓  逆Park变换
两相静止坐标系 (αβ)
        ↓  SVPWM / 逆Clark变换
三相PWM输出 (abc)
```

---

## 二、Clark 变换（abc → αβ）

### 2.1 物理意义

将三相对称坐标系（互差120°）投影到两相正交坐标系（α轴、β轴），**坐标系静止不动**。

### 2.2 数学推导

三相电流满足基尔霍夫电流定律（星形接法）：
$$i_a + i_b + i_c = 0$$

Clark变换矩阵（等幅值变换）：

$$\begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix} = \frac{2}{3} \begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2} \\ 0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \end{bmatrix} \begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix}$$

利用 $i_c = -i_a - i_b$ 化简后（只需采样两相）：

$$i_\alpha = i_a$$
$$i_\beta = \frac{1}{\sqrt{3}}(i_a + 2i_b)$$

> **工程实践**：实际只需采样 $i_a$ 和 $i_b$，$i_c$ 由基尔霍夫定律推算，节省一个ADC通道。

### 2.3 逆Clark变换（αβ → abc）

$$\begin{bmatrix} v_a \\ v_b \\ v_c \end{bmatrix} = \begin{bmatrix} 1 & 0 \\ -\frac{1}{2} & \frac{\sqrt{3}}{2} \\ -\frac{1}{2} & -\frac{\sqrt{3}}{2} \end{bmatrix} \begin{bmatrix} v_\alpha \\ v_\beta \end{bmatrix}$$

---

## 三、Park 变换（αβ → dq）

### 3.1 物理意义

将静止的 αβ 坐标系旋转到**与转子磁场同步旋转**的 dq 坐标系：
- **d轴（直轴）**：与转子磁通方向对齐，控制磁通（对PMSM通常令 $i_d = 0$）
- **q轴（交轴）**：垂直于磁通方向，控制电磁力矩（$T_e \propto i_q$）

### 3.2 数学推导

设转子电角度为 $\theta$（由编码器实时测量）：

$$\begin{bmatrix} i_d \\ i_q \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix}$$

### 3.3 逆Park变换（dq → αβ）

$$\begin{bmatrix} v_\alpha \\ v_\beta \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} v_d \\ v_q \end{bmatrix}$$

### 3.4 关键结论

| 量 | 物理含义 | 控制目标 |
|---|---|---|
| $i_d$ | 励磁电流分量 | PMSM中令 $i_d = 0$（最大力矩/安培） |
| $i_q$ | 力矩电流分量 | 正比于输出力矩，是力矩控制的核心 |

---

## 四、SVPWM 空间矢量脉宽调制

### 4.1 为什么用SVPWM而不是正弦PWM？

| 对比项 | 正弦PWM (SPWM) | 空间矢量PWM (SVPWM) |
|---|---|---|
| 直流母线利用率 | ~86.6% | ~100%（提升15.5%） |
| 谐波含量 | 较高 | 较低 |
| 实现复杂度 | 简单 | 稍复杂 |
| 适用场景 | 入门学习 | 工业/机器人应用 |

### 4.2 基本原理

六个基本电压矢量（$V_1$~$V_6$）将空间分为6个扇区，加上两个零矢量（$V_0, V_7$）。
任意目标电压矢量 $V_{ref}$ 可由相邻两个基本矢量**时间加权合成**：

$$V_{ref} \cdot T_s = V_x \cdot T_x + V_{x+1} \cdot T_{x+1} + V_0 \cdot T_0$$

其中 $T_s$ 为PWM周期，$T_x + T_{x+1} + T_0 = T_s$。

### 4.3 扇区判断

根据 $v_\alpha, v_\beta$ 计算参考矢量所在扇区（1~6），再查表得到对应的开关时序。

---

## 五、完整FOC控制环路

```
目标力矩/速度/位置
        ↓
    外环PID（位置/速度）
        ↓ i_q_ref, i_d_ref=0
    电流环PID（dq轴）
        ↓ v_d, v_q
    逆Park变换
        ↓ v_α, v_β
      SVPWM
        ↓ PWM占空比
    三相逆变器
        ↓ 三相电压
      PMSM电机
        ↓ 编码器反馈θ，电流传感器反馈i_a,i_b
    Clark变换 → Park变换 → 反馈到电流环
```

---

## 六、里程碑自测

完成本节学习后，你应该能够：
- [ ] 徒手写出Clark变换和Park变换的矩阵公式
- [ ] 解释为什么 $i_d = 0$ 能实现最大力矩控制
- [ ] 说出SVPWM相比SPWM的核心优势
- [ ] 运行下方Python仿真脚本并解释输出图形

---

## 七、参考资料

- TI应用笔记：[SPRABQ3 - Sensored Field Oriented Control](https://www.ti.com/lit/an/sprabq3/sprabq3.pdf)
- SimpleFOC理论文档：https://docs.simplefoc.com/foc_theory
- 知乎专栏：搜索"FOC电机控制 坐标变换"
- B站：稚晖君《自制FOC驱动器》系列
