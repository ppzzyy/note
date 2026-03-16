"""
FOC 三环 PID 控制仿真
=====================
演示内容：
  1. 离散 PID 控制器实现（含 Anti-Windup）
  2. 简化电机模型（一阶惯性系统）
  3. 速度环阶跃响应仿真
  4. 不同 Kp/Ki 参数对比
  5. 三环级联仿真（位置→速度→电流）

运行依赖：
  pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


# ─────────────────────────────────────────────
# 1. 离散 PID 控制器
# ─────────────────────────────────────────────
class PIDController:
    """
    离散域 PID 控制器（含积分饱和保护）

    参数
    ----
    kp, ki, kd : float  比例/积分/微分增益
    dt         : float  采样周期 (s)
    output_limit: float 输出限幅（对称）
    """

    def __init__(self, kp, ki, kd, dt, output_limit=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = output_limit

        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # 比例项
        p_term = self.kp * error

        # 积分项（Anti-Windup：只在未饱和时积分）
        self._integral += error * self.dt
        i_term = self.ki * self._integral

        # 微分项（后向差分）
        d_term = self.kd * (error - self._prev_error) / self.dt
        self._prev_error = error

        output = p_term + i_term + d_term

        # 输出限幅 + 积分饱和保护
        if abs(output) > self.output_limit:
            output = np.clip(output, -self.output_limit, self.output_limit)
            # 回退积分（防止 windup）
            self._integral -= error * self.dt

        return output


# ─────────────────────────────────────────────
# 2. 简化电机模型
# ─────────────────────────────────────────────
class MotorModel:
    """
    简化 PMSM 电机模型（一阶惯性 + 摩擦）

    机械方程：J * dω/dt = T_e - B*ω - T_load
    电气方程（简化）：电流环视为理想，T_e = Kt * Iq

    参数
    ----
    J    : 转动惯量 (kg·m²)
    B    : 粘性摩擦系数
    Kt   : 力矩常数 (N·m/A)
    dt   : 仿真步长 (s)
    """

    def __init__(self, J=0.001, B=0.005, Kt=0.1, dt=0.001):
        self.J = J
        self.B = B
        self.Kt = Kt
        self.dt = dt

        self.omega = 0.0    # 角速度 (rad/s)
        self.theta = 0.0    # 角度 (rad)

    def reset(self):
        self.omega = 0.0
        self.theta = 0.0

    def step(self, iq_cmd, t_load=0.0):
        """输入 q 轴电流指令，返回 (omega, theta)"""
        torque = self.Kt * iq_cmd - t_load
        # 欧拉积分
        alpha = (torque - self.B * self.omega) / self.J
        self.omega += alpha * self.dt
        self.theta += self.omega * self.dt
        return self.omega, self.theta


# ─────────────────────────────────────────────
# 3. 速度环阶跃响应仿真
# ─────────────────────────────────────────────
def sim_velocity_loop(kp, ki, kd=0.0, target_speed=10.0, sim_time=2.0, dt=0.001):
    """
    仿真速度环阶跃响应

    返回：时间数组、速度数组、控制量数组
    """
    motor = MotorModel(dt=dt)
    pid = PIDController(kp, ki, kd, dt, output_limit=5.0)  # 电流限幅 5A

    t_arr = np.arange(0, sim_time, dt)
    omega_arr = np.zeros_like(t_arr)
    u_arr = np.zeros_like(t_arr)

    for i, t in enumerate(t_arr):
        ref = target_speed if t >= 0.1 else 0.0  # 0.1s 时阶跃
        iq = pid.update(ref, motor.omega)
        motor.step(iq)
        omega_arr[i] = motor.omega
        u_arr[i] = iq

    return t_arr, omega_arr, u_arr


# ─────────────────────────────────────────────
# 4. 三环级联仿真
# ─────────────────────────────────────────────
def sim_cascade_control(target_angle=6.28, sim_time=3.0, dt=0.001):
    """
    三环级联仿真：位置环 → 速度环 → 电流环（简化）

    返回：时间、角度、速度、电流数组
    """
    motor = MotorModel(dt=dt)

    # 位置环（纯P）
    pid_pos = PIDController(kp=15.0, ki=0.0, kd=0.0, dt=dt, output_limit=20.0)
    # 速度环
    pid_vel = PIDController(kp=0.5, ki=5.0, kd=0.0, dt=dt, output_limit=5.0)
    # 电流环（简化为理想，直接输出力矩）

    t_arr = np.arange(0, sim_time, dt)
    theta_arr = np.zeros_like(t_arr)
    omega_arr = np.zeros_like(t_arr)
    iq_arr = np.zeros_like(t_arr)

    for i, t in enumerate(t_arr):
        ref_theta = target_angle if t >= 0.2 else 0.0

        # 位置环输出速度指令
        vel_cmd = pid_pos.update(ref_theta, motor.theta)
        # 速度环输出电流指令
        iq_cmd = pid_vel.update(vel_cmd, motor.omega)
        # 电机执行
        motor.step(iq_cmd)

        theta_arr[i] = motor.theta
        omega_arr[i] = motor.omega
        iq_arr[i] = iq_cmd

    return t_arr, theta_arr, omega_arr, iq_arr


# ─────────────────────────────────────────────
# 5. 绘图
# ─────────────────────────────────────────────
def plot_velocity_comparison():
    """对比不同 Kp/Ki 参数下的速度环响应"""
    configs = [
        {'kp': 0.1, 'ki': 1.0,  'label': 'Kp=0.1, Ki=1  (响应慢)'},
        {'kp': 0.5, 'ki': 5.0,  'label': 'Kp=0.5, Ki=5  (推荐)'},
        {'kp': 1.5, 'ki': 5.0,  'label': 'Kp=1.5, Ki=5  (超调大)'},
        {'kp': 0.5, 'ki': 50.0, 'label': 'Kp=0.5, Ki=50 (积分过强)'},
    ]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('速度环 PID 参数对比（阶跃响应）', fontsize=14)

    target = 10.0
    for cfg in configs:
        t, omega, u = sim_velocity_loop(cfg['kp'], cfg['ki'], target_speed=target)
        axes[0].plot(t, omega, label=cfg['label'])
        axes[1].plot(t, u, label=cfg['label'])

    axes[0].axhline(target, color='k', linestyle='--', linewidth=1, label='目标速度')
    axes[0].axhline(target * 1.1, color='r', linestyle=':', linewidth=0.8, label='+10% 超调线')
    axes[0].set_ylabel('角速度 (rad/s)')
    axes[0].set_title('速度响应')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim(0, 2)

    axes[1].axhline(5.0, color='r', linestyle='--', linewidth=1, label='电流限幅')
    axes[1].axhline(-5.0, color='r', linestyle='--', linewidth=1)
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('q轴电流指令 (A)')
    axes[1].set_title('控制量（电流指令）')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim(0, 2)

    plt.tight_layout()
    plt.savefig('pid_velocity_comparison.png', dpi=150, bbox_inches='tight')
    print("已保存：pid_velocity_comparison.png")
    plt.show()


def plot_cascade_control():
    """绘制三环级联控制仿真结果"""
    t, theta, omega, iq = sim_cascade_control(target_angle=2 * np.pi)

    fig = plt.figure(figsize=(11, 9))
    fig.suptitle('三环级联控制仿真（位置→速度→电流）', fontsize=14)
    gs = gridspec.GridSpec(3, 1, hspace=0.4)

    # 位置
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, np.degrees(theta), label='实际角度', color='steelblue')
    ax1.axhline(360, color='k', linestyle='--', linewidth=1, label='目标 360°')
    ax1.set_ylabel('角度 (°)')
    ax1.set_title('位置响应')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 速度
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, omega, label='实际速度', color='darkorange')
    ax2.set_ylabel('角速度 (rad/s)')
    ax2.set_title('速度响应')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 电流
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, iq, label='q轴电流', color='green')
    ax3.axhline(5.0, color='r', linestyle='--', linewidth=0.8, label='电流限幅')
    ax3.axhline(-5.0, color='r', linestyle='--', linewidth=0.8)
    ax3.set_xlabel('时间 (s)')
    ax3.set_ylabel('电流 (A)')
    ax3.set_title('电流（力矩）指令')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    plt.savefig('pid_cascade_control.png', dpi=150, bbox_inches='tight')
    print("已保存：pid_cascade_control.png")
    plt.show()


def print_performance_metrics():
    """打印速度环性能指标"""
    print("\n" + "=" * 50)
    print("速度环性能指标分析（推荐参数 Kp=0.5, Ki=5）")
    print("=" * 50)

    t, omega, _ = sim_velocity_loop(kp=0.5, ki=5.0, target_speed=10.0)
    target = 10.0
    dt = t[1] - t[0]
    step_idx = int(0.1 / dt)  # 阶跃时刻索引

    omega_after = omega[step_idx:]
    t_after = t[step_idx:] - 0.1

    # 上升时间（10%→90%）
    try:
        t10 = t_after[np.where(omega_after >= target * 0.1)[0][0]]
        t90 = t_after[np.where(omega_after >= target * 0.9)[0][0]]
        print(f"  上升时间 (10%→90%): {(t90 - t10) * 1000:.1f} ms")
    except IndexError:
        print("  上升时间：未达到目标")

    # 超调量
    overshoot = (np.max(omega_after) - target) / target * 100
    print(f"  超调量: {overshoot:.1f}%")

    # 调节时间（±2% 误差带）
    try:
        settled = np.where(np.abs(omega_after - target) / target > 0.02)[0]
        if len(settled) > 0:
            t_settle = t_after[settled[-1]]
            print(f"  调节时间 (±2%): {t_settle * 1000:.1f} ms")
        else:
            print("  调节时间 (±2%): < 1 ms（已稳定）")
    except Exception:
        pass

    # 稳态误差
    steady_state = np.mean(omega[-200:])
    print(f"  稳态值: {steady_state:.4f} rad/s（目标 {target} rad/s）")
    print(f"  稳态误差: {abs(steady_state - target):.4f} rad/s")
    print("=" * 50)


# ─────────────────────────────────────────────
# 主程序
# ─────────────────────────────────────────────
if __name__ == '__main__':
    print("FOC 三环 PID 控制仿真")
    print("正在运行速度环参数对比仿真...")
    plot_velocity_comparison()

    print("\n正在运行三环级联控制仿真...")
    plot_cascade_control()

    print_performance_metrics()
