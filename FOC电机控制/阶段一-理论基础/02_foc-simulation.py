"""
FOC 坐标变换仿真脚本
任务2：掌握FOC核心数学原理

依赖安装：pip install numpy matplotlib
运行方式：python 02-foc-simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ============================================================
# 1. Clark 变换（abc → αβ）
# ============================================================

def clark_transform(ia, ib, ic=None):
    """
    Clark变换：三相静止坐标系 → 两相静止坐标系
    参数：
        ia, ib: 三相电流中的两相（numpy数组）
        ic: 第三相，若为None则由基尔霍夫定律推算
    返回：
        i_alpha, i_beta
    """
    if ic is None:
        ic = -ia - ib  # 基尔霍夫电流定律
    i_alpha = ia  # 等幅值Clark变换简化形式
    i_beta = (ia + 2 * ib) / np.sqrt(3)
    return i_alpha, i_beta


def clark_transform_full(ia, ib, ic):
    """完整Clark变换矩阵形式（用于验证）"""
    i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic)
    i_beta  = (2/3) * (np.sqrt(3)/2 * ib - np.sqrt(3)/2 * ic)
    return i_alpha, i_beta


# ============================================================
# 2. Park 变换（αβ → dq）
# ============================================================

def park_transform(i_alpha, i_beta, theta):
    """
    Park变换：两相静止坐标系 → 两相旋转坐标系
    参数：
        i_alpha, i_beta: αβ坐标系分量
        theta: 转子电角度（弧度）
    返回：
        id, iq
    """
    id_ = i_alpha * np.cos(theta) + i_beta * np.sin(theta)
    iq  = -i_alpha * np.sin(theta) + i_beta * np.cos(theta)
    return id_, iq


def inverse_park_transform(vd, vq, theta):
    """逆Park变换：dq → αβ"""
    v_alpha = vd * np.cos(theta) - vq * np.sin(theta)
    v_beta  = vd * np.sin(theta) + vq * np.cos(theta)
    return v_alpha, v_beta


def inverse_clark_transform(v_alpha, v_beta):
    """逆Clark变换：αβ → abc"""
    va =  v_alpha
    vb = -0.5 * v_alpha + (np.sqrt(3)/2) * v_beta
    vc = -0.5 * v_alpha - (np.sqrt(3)/2) * v_beta
    return va, vb, vc


# ============================================================
# 3. 仿真主程序
# ============================================================

def simulate_foc_transforms():
    # 时间轴：模拟电机旋转一圈（电角度 0~2π）
    t = np.linspace(0, 2*np.pi, 1000)
    omega = 1.0          # 电角速度（归一化）
    theta = omega * t    # 转子电角度
    I_peak = 1.0         # 电流峰值（归一化）

    # 生成三相对称正弦电流（模拟PMSM定子电流）
    ia = I_peak * np.cos(theta)
    ib = I_peak * np.cos(theta - 2*np.pi/3)
    ic = I_peak * np.cos(theta + 2*np.pi/3)

    # Clark变换
    i_alpha, i_beta = clark_transform(ia, ib)

    # Park变换（使用真实转子角度）
    id_, iq = park_transform(i_alpha, i_beta, theta)

    # 验证逆变换还原精度
    v_alpha_rec, v_beta_rec = inverse_park_transform(id_, iq, theta)
    va_rec, vb_rec, vc_rec = inverse_clark_transform(v_alpha_rec, v_beta_rec)
    reconstruction_error = np.max(np.abs(ia - va_rec))

    print("=" * 50)
    print("FOC 坐标变换仿真结果")
    print("=" * 50)
    print(f"三相电流峰值：{I_peak:.3f} A")
    print(f"Clark变换后 i_alpha 峰值：{np.max(i_alpha):.3f}")
    print(f"Clark变换后 i_beta  峰值：{np.max(i_beta):.3f}")
    print(f"Park变换后  id 均值：{np.mean(id_):.4f}（理论值：0.000，即 id=0）")
    print(f"Park变换后  iq 均值：{np.mean(iq):.4f}（理论值：{I_peak:.3f}，即 iq=I_peak）")
    print(f"逆变换还原误差（最大值）：{reconstruction_error:.2e}（应接近0）")
    print("=" * 50)

    # ============================================================
    # 4. 可视化
    # ============================================================
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("FOC 坐标变换仿真 - Clark & Park Transform", fontsize=14, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # 图1：三相电流（abc坐标系）
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, ia, label='$i_a$', color='red')
    ax1.plot(t, ib, label='$i_b$', color='green')
    ax1.plot(t, ic, label='$i_c$', color='blue')
    ax1.set_title("① 三相电流（abc坐标系）—— 交流量，随角度变化")
    ax1.set_xlabel("电角度 θ (rad)")
    ax1.set_ylabel("电流 (A)")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(0, color='black', linewidth=0.5)

    # 图2：Clark变换后（αβ坐标系）
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, i_alpha, label='$i_α$', color='purple')
    ax2.plot(t, i_beta,  label='$i_β$', color='orange')
    ax2.set_title("② Clark变换后（αβ坐标系）\n仍是交流量，但从3相变为2相正交")
    ax2.set_xlabel("电角度 θ (rad)")
    ax2.set_ylabel("电流 (A)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 图3：αβ平面轨迹（应为圆形）
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(i_alpha, i_beta, color='purple', linewidth=1.5)
    ax3.set_title("③ αβ平面电流轨迹\n（圆形 = 三相对称，椭圆 = 不对称）")
    ax3.set_xlabel("$i_α$")
    ax3.set_ylabel("$i_β$")
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(0, color='black', linewidth=0.5)
    ax3.axvline(0, color='black', linewidth=0.5)

    # 图4：Park变换后（dq坐标系）—— 核心！
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(t, id_, label='$i_d$（励磁分量）', color='blue')
    ax4.plot(t, iq,  label='$i_q$（力矩分量）', color='red')
    ax4.set_title("④ Park变换后（dq坐标系）\n直流量！id≈0，iq≈常数 → 可直接PID控制")
    ax4.set_xlabel("电角度 θ (rad)")
    ax4.set_ylabel("电流 (A)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axhline(0, color='black', linewidth=0.5)

    # 图5：逆变换还原验证
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.plot(t, ia,     label='原始 $i_a$', color='red', linewidth=2)
    ax5.plot(t, va_rec, label='还原 $v_a$', color='black', linewidth=1, linestyle='--')
    ax5.set_title(f"⑤ 逆变换还原验证\n最大误差：{reconstruction_error:.2e}（越小越好）")
    ax5.set_xlabel("电角度 θ (rad)")
    ax5.set_ylabel("电流 (A)")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    plt.savefig("02-foc-transform-result.png", dpi=150, bbox_inches='tight')
    print("\n图像已保存为：02-foc-transform-result.png")
    plt.show()


# ============================================================
# 5. 扩展：不同 id 设定值对力矩的影响演示
# ============================================================

def demo_id_effect():
    """演示 id=0 vs id≠0 对力矩（iq）的影响"""
    print("\n--- id 设定值对力矩的影响 ---")
    print("PMSM中，电磁力矩 Te = 1.5 * p * ψf * iq")
    print("当 id=0 时，所有电流都用于产生力矩（最大力矩/安培控制）")
    print("当 id≠0 时，部分电流用于励磁，力矩效率下降")
    I_total = 1.0
    for id_val in [0.0, 0.3, 0.5, 0.8]:
        iq_val = np.sqrt(I_total**2 - id_val**2)
        print(f"  id={id_val:.1f} A → iq={iq_val:.3f} A（力矩利用率：{iq_val/I_total*100:.1f}%）")


if __name__ == "__main__":
    simulate_foc_transforms()
    demo_id_effect()
