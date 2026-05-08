# Z变换基础

> 离散信号与系统的复频域分析工具——拉普拉斯变换的离散版本

## 🎯 为什么需要Z变换？

类似于拉普拉斯变换，Z变换引入额外变量 r，使更多离散信号可变换。

DTFT 要求信号**绝对可和**：
$$
\sum_{n=-\infty}^{+\infty} |x[n]| < \infty
$$

很多信号不满足，如：aⁿu[n]（|a|≥1），u[n] 等。

## 📐 定义

### 双边Z变换

$$
X(z) = \sum_{n=-\infty}^{+\infty} x[n] z^{-n}
$$

### Z变换与DTFT的关系

设 $z = re^{j\omega}$：

$$
X(z) = \sum_{n=-\infty}^{+\infty} x[n] (re^{j\omega})^{-n} = \sum_{n=-\infty}^{+\infty} (x[n]r^{-n}) e^{-j\omega n}
$$

即 $X(z) = \text{DTFT}[x[n]r^{-n}]$

**当 r=1（即 z=e^{jω}）时，Z变换退化为DTFT。**

### Z反变换

$$
x[n] = \frac{1}{2\pi j} \oint X(z) z^{n-1} dz
$$

通常用**部分分式展开**或**留数法**计算。

## 🔑 核心概念：收敛域 (ROC)

### ROC 性质

1. **收敛域必不包含极点**
2. **有限长序列**：ROC 为全平面（除可能的 z=0 或 z=∞）
3. **右边序列**：ROC 为某圆外 |z|>R
4. **左边序列**：ROC 为某圆内 |z|<R
5. **双边序列**：ROC 为圆环 R₁<|z|<R₂
6. **因果序列**：是右边序列，ROC 为 |z|>R
7. **稳定序列**：ROC 包含单位圆 |z|=1

## 📋 常用变换对

| 时域 x[n] | 频域 X(z) | ROC |
|-----------|-----------|-----|
| δ[n] | 1 | 全平面 |
| δ[n-n₀] | z^{-n₀} | 全平面 |
| u[n] | 1/(1-z^{-1}) | \|z\|>1 |
| -u[-n-1] | 1/(1-z^{-1}) | \|z\|<1 |
| aⁿu[n] | 1/(1-az^{-1}) | \|z\|>\|a\| |
| -aⁿu[-n-1] | 1/(1-az^{-1}) | \|z\|<\|a\| |
| naⁿu[n] | az^{-1}/(1-az^{-1})² | \|z\|>\|a\| |
| cos(ω₀n)u[n] | (1-z^{-1}cosω₀)/(1-2z^{-1}cosω₀+z^{-2}) | \|z\|>1 |

## 📏 基本性质

### 1. 线性性质
$$
ax_1[n] + bx_2[n] \leftrightarrow aX_1(z) + bX_2(z)
$$

### 2. 时移性质
$$
x[n-n_0] \leftrightarrow X(z)z^{-n_0}
$$

### 3. z域尺度变换
$$
a^n x[n] \leftrightarrow X(z/a)
$$

### 4. 时域反转
$$
x[-n] \leftrightarrow X(1/z)
$$

### 5. 时域扩展
$$
x_{(k)}[n] \leftrightarrow X(z^k)
$$

### 6. 共轭性质
$$
x^*[n] \leftrightarrow X^*(z^*)
$$

### 7. 卷积定理 ⭐
$$
x[n]*h[n] \leftrightarrow X(z) \cdot H(z)
$$

### 8. z域微分
$$
nx[n] \leftrightarrow -z\frac{dX(z)}{dz}
$$

## 🔄 Z平面与s平面的映射

$$
z = e^{sT} = e^{(\sigma+j\omega)T} = e^{\sigma T} \cdot e^{j\omega T}
$$

- s平面 jω 轴 → z平面单位圆
- s平面左半平面 → z平面单位圆内
- s平面右半平面 → z平面单位圆外

## 🎯 初值与终值定理

**初值定理**（因果序列）：
$$
x[0] = \lim_{z\to\infty} X(z)
$$

**终值定理**（因果序列，X(z) 收敛）：
$$
\lim_{n\to\infty}x[n] = \lim_{z\to 1} (z-1)X(z)
$$

## 🔗 相关链接

- [[Z变换性质]]
- [[03_傅里叶变换/离散时间傅里叶变换]]
- [[04_拉普拉斯变换/拉氏变换基础]]

---

#信号与系统 #Z变换
