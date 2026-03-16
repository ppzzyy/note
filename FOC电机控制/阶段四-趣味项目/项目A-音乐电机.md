# 项目A：音乐电机（Musical Motor）

> **难度**：⭐⭐☆☆☆ 入门推荐
> **预计用时**：1-2天
> **效果**：通过FOC精确控制电机振动频率，让电机"演奏"音乐，效果震撼、代码量少

---

## 一、原理说明

### 核心思路

音乐电机的本质是：**用FOC力矩控制模式，以音符频率驱动电机做往复振动**。

- 每个音符对应一个频率（Hz），例如中央C = 261.63 Hz
- 以该频率向电机施加正弦力矩：`τ = A × sin(2π × f × t)`
- 电机轴会以该频率振动，产生对应音调的声音
- 通过控制振幅 `A` 可以调节音量

### 为什么FOC适合做这个？

普通PWM驱动电机响应慢、谐波多，无法精确控制瞬时力矩。
FOC电流环带宽可达数kHz，能精确跟踪高频力矩指令，音调纯净。

### 音符频率对照表（C4大调）

| 音符 | 频率(Hz) | 音符 | 频率(Hz) |
|------|---------|------|---------|
| C4 (do) | 261.63 | G4 (sol) | 392.00 |
| D4 (re) | 293.66 | A4 (la)  | 440.00 |
| E4 (mi) | 329.63 | B4 (si)  | 493.88 |
| F4 (fa) | 349.23 | C5 (高do)| 523.25 |

---

## 二、硬件准备

- FOC驱动套件（SimpleFOC Shield + STM32/ESP32）
- 无刷电机（已完成校准）
- AS5600磁编码器（已安装）
- 无需额外硬件，电机本身就是"扬声器"

---

## 三、完整Arduino代码

```cpp
/**
 * 音乐电机 - Musical Motor
 * 基于SimpleFOC库，使用力矩控制模式让电机演奏音乐
 *
 * 硬件：SimpleFOC Shield v2 + STM32F4 + AS5600编码器
 */

#include <SimpleFOC.h>

// ===== 硬件引脚配置（根据实际接线修改）=====
// 电机PWM引脚（SimpleFOC Shield v2默认引脚）
BLDCMotor motor = BLDCMotor(7);  // 7 = 极对数，根据实际电机修改
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);  // pwmA, pwmB, pwmC, enable

// AS5600磁编码器（I2C）
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ===== 音符频率定义（Hz）=====
#define NOTE_C4  261.63
#define NOTE_D4  293.66
#define NOTE_E4  329.63
#define NOTE_F4  349.23
#define NOTE_G4  392.00
#define NOTE_A4  440.00
#define NOTE_B4  493.88
#define NOTE_C5  523.25
#define NOTE_REST  0.0   // 休止符

// ===== 曲谱定义：两只老虎 =====
// 格式：{频率, 时长(ms)}
struct Note {
  float freq;
  int duration;
};

Note song[] = {
  {NOTE_C4, 400}, {NOTE_D4, 400}, {NOTE_E4, 400}, {NOTE_C4, 400},  // 两只老虎
  {NOTE_C4, 400}, {NOTE_D4, 400}, {NOTE_E4, 400}, {NOTE_C4, 400},  // 两只老虎
  {NOTE_E4, 400}, {NOTE_F4, 400}, {NOTE_G4, 800},                  // 跑得快
  {NOTE_E4, 400}, {NOTE_F4, 400}, {NOTE_G4, 800},                  // 跑得快
  {NOTE_G4, 200}, {NOTE_A4, 200}, {NOTE_G4, 200}, {NOTE_F4, 200}, {NOTE_E4, 400}, {NOTE_C4, 400},  // 一只没有眼睛
  {NOTE_G4, 200}, {NOTE_A4, 200}, {NOTE_G4, 200}, {NOTE_F4, 200}, {NOTE_E4, 400}, {NOTE_C4, 400},  // 一只没有耳朵
  {NOTE_D4, 400}, {NOTE_G3, 400}, {NOTE_C4, 800},                  // 真奇怪
  {NOTE_D4, 400}, {NOTE_G3, 400}, {NOTE_C4, 800},                  // 真奇怪
  {NOTE_REST, 500}
};

// G3需要补充定义
#define NOTE_G3  196.00

int songLength = sizeof(song) / sizeof(song[0]);

// ===== 力矩控制参数 =====
float torqueAmplitude = 0.8;  // 力矩振幅（0.0~1.0），控制音量
float currentFreq = 0.0;      // 当前播放频率
unsigned long noteStartTime = 0;
int currentNoteIndex = 0;
bool isPlaying = false;

// 相位累积器（用于生成正弦波）
float phase = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("音乐电机初始化中...");

  // 初始化编码器
  sensor.init();
  motor.linkSensor(&sensor);

  // 驱动器配置
  driver.voltage_power_supply = 24;  // 电源电压（V）
  driver.init();
  motor.linkDriver(&driver);

  // FOC控制模式：力矩控制
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // 电流环PID参数（根据实际电机调整）
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 300;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 300;

  // 电流限制
  motor.current_limit = 2.0;  // 安培，根据电机额定电流设置

  // 初始化电机
  motor.init();
  motor.initFOC();

  Serial.println("初始化完成！开始演奏...");
  isPlaying = true;
  noteStartTime = millis();
}

void loop() {
  // FOC主循环（必须尽可能快速调用）
  motor.loopFOC();

  // 音乐播放逻辑
  if (isPlaying) {
    playMusic();
  }

  // 串口命令处理
  handleSerial();
}

void playMusic() {
  unsigned long now = millis();

  // 检查是否需要切换到下一个音符
  if (currentNoteIndex < songLength) {
    Note& note = song[currentNoteIndex];

    if (now - noteStartTime >= note.duration) {
      // 切换到下一个音符
      currentNoteIndex++;
      noteStartTime = now;
      phase = 0.0;  // 重置相位，避免爆音

      if (currentNoteIndex < songLength) {
        currentFreq = song[currentNoteIndex].freq;
        Serial.print("播放音符: ");
        Serial.print(currentFreq);
        Serial.println(" Hz");
      }
    }

    // 生成正弦力矩指令
    if (currentNoteIndex < songLength && song[currentNoteIndex].freq > 0) {
      // 相位步进（基于loop执行频率约10kHz估算）
      float dt = 0.0001;  // 100μs，根据实际loop频率调整
      phase += 2.0 * PI * currentFreq * dt;
      if (phase > 2.0 * PI) phase -= 2.0 * PI;

      // 施加正弦力矩
      motor.target = torqueAmplitude * sin(phase);
    } else {
      // 休止符：零力矩
      motor.target = 0;
    }
  } else {
    // 曲子播放完毕，循环
    currentNoteIndex = 0;
    currentFreq = song[0].freq;
    noteStartTime = millis();
    Serial.println("--- 重新开始 ---");
  }
}

void handleSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '+':
        torqueAmplitude = min(torqueAmplitude + 0.1f, 1.5f);
        Serial.print("音量+: "); Serial.println(torqueAmplitude);
        break;
      case '-':
        torqueAmplitude = max(torqueAmplitude - 0.1f, 0.1f);
        Serial.print("音量-: "); Serial.println(torqueAmplitude);
        break;
      case 'p':
        isPlaying = !isPlaying;
        if (!isPlaying) motor.target = 0;
        Serial.println(isPlaying ? "播放" : "暂停");
        break;
      case 'r':
        currentNoteIndex = 0;
        noteStartTime = millis();
        Serial.println("重新开始");
        break;
    }
  }
}
```

---

## 四、调试步骤

### Step 1：验证FOC基础运行
先确认电机能正常运行力矩控制模式（参考任务8文档），再加入音乐逻辑。

### Step 2：调整力矩振幅
- 从 `torqueAmplitude = 0.3` 开始，逐渐增大
- 太小：声音微弱；太大：电机过热或失步
- 推荐范围：`0.5 ~ 1.2`（视电机额定电流而定）

### Step 3：校准loop频率
代码中 `dt = 0.0001` 假设loop频率为10kHz。
实际测量方法：
```cpp
// 在loop()开头添加
static unsigned long lastTime = 0;
unsigned long now = micros();
float dt_measured = (now - lastTime) / 1e6;
lastTime = now;
// 打印dt_measured，取平均值后填入dt变量
```

### Step 4：添加更多曲目
按照 `{频率, 时长}` 格式扩展 `song[]` 数组即可。

---

## 五、进阶玩法

1. **和弦效果**：快速交替两个频率（每5ms切换一次），模拟和弦
2. **颤音效果**：在基础频率上叠加低频调制：`freq = baseFreq * (1 + 0.05 * sin(2π * 5 * t))`
3. **MIDI输入**：通过串口接收MIDI消息，实时演奏
4. **多电机合奏**：两台电机分别演奏高低声部

---

## 六、常见问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 声音沙哑/失真 | 力矩振幅过大 | 降低 `torqueAmplitude` |
| 音调不准 | dt参数不准确 | 实测loop频率后校准dt |
| 电机发热严重 | 电流过大 | 降低振幅，检查电流限制设置 |
| 没有声音 | FOC未正常运行 | 先验证基础力矩控制是否正常 |

---

## 七、里程碑验收

- [ ] 电机能演奏《两只老虎》完整曲目
- [ ] 串口命令能控制音量和播放/暂停
- [ ] 录制演示视频（30秒以上）
- [ ] 代码上传GitHub，添加README说明
