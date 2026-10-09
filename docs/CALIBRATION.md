# MDP21 车辆动力学标定实验设计

**版本**: v1.0
**日期**: 2026-08-26
**车型**: 前舵机转向 + 后双轮独立 PWM 驱动（阿克曼几何）
**目标**: 通过 6 个标定实验，把代码里手调/缺失的参数全部用实测值替换，并验证阿克曼自行车模型，为后续轨迹/速度闭环打下基础。

---

## 0. 车辆动力学模型（所有标定的理论根据）

### 0.1 自行车模型（bicycle model）

记：
- `v` 后轴中心纵向速度 (m/s 或 cm/s)
- `δ` 前轮等效转向角 (rad)，规定右打为正
- `θ` 航向角 (rad)，规定左转为正
- `L` 轴距 = 前轴中心到后轴中心的水平距离
- `γ = θ̇` 偏航角速度 (rad/s)
- `R` 后轴中心转弯半径

阿克曼/自行车模型核心关系：

```
γ = (v / L) · tan(δ)          ← ① 核心公式
R = L / tan(δ)                ← ② 转弯半径
ẋ = v·cos(θ)                 ← ③ 位姿积分
ẏ = v·sin(θ)
```

> 这个模型假设：轮胎无侧偏、低速、刚体。对小车低速场景足够。

### 0.2 后轮编码器里程

记 `v_L, v_R` 左右后轮线速度，`T` 后轮距（左右接地点距离）：

```
v   = (v_L + v_R) / 2         ← ④ 后轴中心速度
γ_w = (v_R - v_L) / T         ← ⑤ 由编码器推偏航（差速公式，Ackermann 上仅小滑移近似成立）
```

对纯阿克曼车，理想情况下后两轮速度也应满足 Ackermann 关系（内慢外快），但本车两后轮独立 PWM，直行时应相等；差异仅作 yaw 修正 hack（见实验 5）。

### 0.3 编码器 → 距离/速度

```
d = N / counts_per_cm         ← ⑥ N 为累计计数
v = d / dt
```

滚动周长 `C_roll = counts_per_rev / counts_per_cm`，几何周长 `C_geo = π·D`。`C_roll` 与 `C_geo` 因胎压/形变/接地滑移**不一定相等**。本车 `counts_per_rev` 与 `counts_per_cm` 已实测且自洽（得 `C_roll≈21.0`），但代码里 `WHEEL_CIRCUMFERENCE_CM=19.8` 是遗留错值，应直接改成 21.0（非标定，见实验 1）。

### 0.4 阿克曼内/外轮关系（理想连杆）

```
cot(δ_o) − cot(δ_i) = T / L   ← ⑦
δ_bicycle ≈ (δ_i + δ_o)/2     （小角近似）
```

标定转向角时，若只量一个轮的角，需注明是内轮还是外轮；实验 2 会处理。

### 0.5 电机稳态 + 舵机线性

```
v(PWM) = k_m · (PWM − PWM_dead)   线性区，饱和趋近 v_max     ← ⑧
δ(p)   = k_s · (p − p_center)     线性区，机械限位钳位       ← ⑨
γ ≈ k_diff · ΔPWM                 差速 hack，经验线性          ← ⑩
```

---

## 1. 现有代码标定状态盘点（缺什么）

| 参数 | 现值 / 状态 | 文件位置 | 是否需要标定 |
|---|---|---|---|
| `LEFT_COUNTS_PER_REV` | 1560.13 | test_distance.c:57 | **已标好，保留**（3×10 圈实测，硬件常数） |
| `RIGHT_COUNTS_PER_REV` | 1559.37 | test_distance.c:58 | **已标好，保留** |
| `LEFT_COUNTS_PER_CM` | 74.29 | test_distance.c:60 | **已标好，保留**（与 counts/rev 自洽：1560.13/74.29=21.0） |
| `RIGHT_COUNTS_PER_CM` | 74.26 | test_distance.c:61 | **已标好，保留** |
| `WHEEL_CIRCUMFERENCE_CM` | 19.8 | test_distance.c:63 | **错值，直接改 21.0**（=1560.13/74.29，非标定） |
| `REV_ERROR_COMPENSATION` | 0.4 rev | test_distance.c:64 | **经验值，需实测** |
| `SERVO_CENTER_US` | 1712 | test_distance.c:76 | "候选"，**未标定** |
| 轴距 `L` | **无定义** | — | **缺失，必需** |
| 轮距 `T` | **无定义** | — | **缺失，必需** |
| 舵机角度系数 `k_s` | 无 | — | **缺失** |
| 舵机左/右极限脉宽 | 无 | — | **缺失** |
| 电机 deadband | 无 | — | **缺失**（影响低速/启动） |
| 电机 PWM↔速度 `k_m` | 无 | — | **缺失**（速度前馈） |
| 左右电机不对称 | 无 | — | **缺失**（直行偏置） |
| `KP_YAW=40 / KI_YAW=8` | 手调 | test_distance.c:171-172 | 凭经验，应用实验 5 标定 |
| 陀螺零偏 | 运行时标定 | test_distance.c:491 | 已实现 |
| 陀螺标度 `GYRO_SENSITIVITY` | 131 | test_distance.c:118 | 芯片标称，可实验验证 |

---

## 2. 标定实验总览

| # | 实验 | 测什么 | 得到什么参数 | 喂给哪个公式 |
|---|---|---|---|---|
| 1 | 超调补偿标定 | 停车超调圈数 | `REV_ERROR_COMPENSATION`（counts/cm、counts/rev 已标好，保留） | ⑥ |
| 2 | 舵机转向标定 | 居中、左右极限、脉宽↔角度 | `SERVO_CENTER_US`、`SERVO_LEFT/RIGHT_US`、`SERVO_DEG_PER_US` | ⑨ |
| 3 | 几何参数测量 | 轴距、轮距（钢尺） | `WHEELBASE_MM`、`TRACK_WIDTH_MM` | ①②④⑤⑦ |
| 4 | 电机 PWM-速度特性 | deadband、线性斜率、左右不对称 | `PWM_DEAD_L/R`、`K_MOTOR_L/R`、`PWM_BALANCE_OFFSET` | ⑧ |
| 5 | 差速偏航响应 | ΔPWM↔偏航角速度 | `K_DIFF`、重算 `KP_YAW` | ⑩ |
| 6 | 阿克曼转向偏航响应 | v, δ, γ 联测，反算有效轴距 | `L_eff`、验证模型 ① | ①② |

---

## 2.5 标定读数方案：OLED 为主，UART 为辅

> **核心调整**：标定结果直接打在 OLED 上，不用反复敲 `ENC,GET` / `GYRO,Z` 命令。UART 只作可选全量日志。

### 读数策略

- **OLED（128×64，6×8 字体，8 行×21 字符）**：实时值 + 每步算出的结果常量。复用 [test_distance.c:258 OLED_ShowStatus](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L258) 同款 `OLED_Init/Clear/ShowString/Refresh_Gram`。
- **UART**：可选。开了 PuTTY 就有完整日志（每步原始数据），不开也照样标定。
- **推进机制**（无需敲命令）：
  - 自动扫描 + 每点停留 2~3s，OLED 滚动显示；
  - 单步实验（如超调）跑完**结果常驻 OLED**，复位即可重跑；
  - 需要人手介入的步骤（放车、量角）用 3s 倒计时（同 [test_distance.c:700](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L700) 的 START_DELAY 模式）。

### 通用 OLED 显示助手（代码设计，仅签名）

```c
// 把最多 6 行文本一次性刷到 OLED（第 0 行做标题，1~6 行内容，第 7 行提示）
void Calib_OLED_Show(const char* title,
                     const char* l1, const char* l2, const char* l3,
                     const char* l4, const char* l5, const char* l6,
                     const char* hint);

// 标定结束后常驻显示一个"结果行"（如 "OVER=+2.3cm REV=0.110"），直到复位
void Calib_OLED_Hold(const char* title, const char* result_line);
```

> 刷新 ~30ms 阻塞，标定场景（非实时控制）完全可接受。

### test_calib.c 整体结构

```c
// Core/Inc/tests/test_calib.h
void Calib_Rev_Error(void);        // 实验1
void Calib_Servo(void);            // 实验2
void Calib_Motor(void);            // 实验4
void Calib_Diff_Yaw(void);        // 实验5
void Calib_Ackermann(void);       // 实验6

// test_config.h: #define TEST_CALIB 11
// test_runner.c: #elif ACTIVE_TEST == TEST_CALIB -> 选调其中之一
//   （或加一个 TEST_CALIB_MENU 用复位切子项；最简：每次只编译一个进去）
```

> 实验 3（几何）纯钢尺，无 OLED。

### OLED 屏幕布局约定（每实验结果屏）

```
实验1:  REV_ERR CALIB / PWM=3000 / TGT=50.0 / ACT=52.3 / OVER=+2.3cm / REV=0.110 / [reset rerun]
实验2:  SERVO CALIB / CENTER=1712 / L_MIN=1500 / R_MAX=1900 / (扫描时 PULSE=xxxx)
实验4:  MOTOR CALIB / PWM=2000 / vL=12.3 vR=12.1 / [next...] / 末页: kM_L=.. kM_R=.. DEAD=..
实验5:  DIFF YAW / dPWM=+200 / gamma=+3.2dps / k_diff=0.016 / [reset]
实验6:  ACKERMANN / d=10deg v=20 / gamma=2.1dps / L_eff=185mm / [reset]
```

---

## 3. 实验 1：超调补偿标定（counts/cm、counts/rev 已标好保留）

> 本节已精简：编码器 `counts/cm`、`counts/rev` 已实测且自洽，**保留不动**，不重测。

### 3.1 已标好保留的常量（无需碰）

| 常量 | 值 | 位置 |
|---|---|---|
| `LEFT_COUNTS_PER_REV` | 1560.13 | [test_distance.c:57](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L57) |
| `RIGHT_COUNTS_PER_REV` | 1559.37 | [test_distance.c:58](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L58) |
| `LEFT_COUNTS_PER_CM` | 74.29 | [test_distance.c:60](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L60) |
| `RIGHT_COUNTS_PER_CM` | 74.26 | [test_distance.c:61](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L61) |

### 3.2 直接改的错值（不用你测，我改）

`WHEEL_CIRCUMFERENCE_CM` 现值 19.8 与上面两组自洽值矛盾（应为 `1560.13/74.29 ≈ 21.0`）。我会直接改成 21.0（或删掉改用派生）。位置 [test_distance.c:63](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L63)。

### 3.3 需实测：超调补偿 `REV_ERROR_COMPENSATION`

**原理**：车到达目标后惯性会多走一段才停。`test_distance.c` 现用 `target_revolutions = 目标距离/周长 − 0.4` 提前停（[test_distance.c:781-783](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L781-L783)），这个 `0.4` 圈是经验值，应按实际 PWM/速度实测。

**代码设计**（`test_calib.c`，OLED 显示结果，仅签名/伪码）：

```c
// 自动：给 PWM 走 target_cm，停车测超调，结果打 OLED 常驻
void Calib_Rev_Error(uint32_t pwm, float target_cm);
```

```
OLED: "PLACE ROBOT / START 3s"   // 倒计时放车
ENC counter -> 0
MOTOR forward pwm
loop 每 20ms:
    read encoder delta, acc -> d_now
    OLED live: "PWM=.. TGT=50.0 ACT=..cm"   // 实时显示已走距离
    if d_now >= target_counts: MOTOR,STOP; break
wait 1500ms  // 惯性停下，期间继续累加 encoder
d_after = acc / counts_per_cm
over_cm = d_after - target_cm
REV_ERROR = over_cm / 21.0       // C_roll
OLED Hold: "REV_ERR CALIB / PWM=3000 / TGT=50.0 / ACT=52.3 / OVER=+2.3cm / REV=0.110 / [reset rerun]"
// 同时 UART 打一行日志（可选）
```

> 不同 PWM 超调不同，建议在 2500/3000/3500 三个 PWM 各测一次取代表值。改 PWM 需重新编译或加菜单。

### 3.4 你要做什么

1. 找一段 **2m+ 直道**，车起点对齐。
2. 开机 → OLED 显示 "PLACE ROBOT / START 3s"，3s 后自动起步。
3. 车到 50cm 自动停，惯性滑完，**OLED 常驻显示** `ACT`、`OVER`、`REV`。
4. **直接读 OLED** 记下 `OVER` 和 `REV`；复位重跑换 PWM（或改源码 PWM 值）。

### 3.5 你要测什么（从 OLED 读，填表）

| PWM | OLED 显示 ACT (实停 cm) | OVER (多走 cm) | REV (折合圈数) |
|---|---|---|---|
| 2500 | | | |
| 3000 | | | |
| 3500 | | | |

### 3.6 反馈给我什么

- 上表 3 个 REV 值。
- 我会取代表值替换 `REV_ERROR_COMPENSATION = 0.4`（[test_distance.c:64](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L64)），并改 `WHEEL_CIRCUMFERENCE_CM` 为 21.0。

---

## 4. 实验 2：舵机转向标定

### 4.1 原理

舵机脉宽 `p` (µs) 与前轮转向角 `δ` 近似线性：

```
δ(p) = k_s · (p − p_center)        ← ⑨
```

需标定 4 个量：
- `p_center`：δ=0 时的脉宽
- `p_left_max` / `p_right_max`：机械左/右极限（到此钳位，别超）
- `k_s`：µs→度的斜率

### 4.2 现有代码能否直接做

舵机角度无法靠车自身传感器测（需人眼/量角器判断"直"与"角"），所以这一项**必须人手介入**。但脉宽扫描可让 `test_calib.c` 自动跑、OLED 显示当前脉宽，省去手敲 `STEER,US`。**人只负责看/量，读 OLED 上的脉宽**。

### 4.3 代码设计（OLED 显示当前脉宽）

`test_calib.c` 加：

```c
// 居中扫描：脉宽在 [lo,hi] 自动步进，每步停 2s，OLED 显示 PULSE=xxxx
//   人在车前看前轮，直了就记下 OLED 上的 PULSE
void Calib_Servo_Center_Sweep(uint16_t lo, uint16_t hi, uint16_t step);

// 极限扫描：单向递增/递减，每步停 1.5s，OLED 显示 PULSE
//   人听到堵转声即记下当前 PULSE（代码不自动判堵转，避免烧舵机）
void Calib_Servo_Limit_Sweep(uint16_t start, int16_t dir, uint16_t step);
```

实现要点：
- 用 `__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, us)` 设脉宽（与 [test_distance.c:314](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L314) 一致）。
- 每步 OLED：`SERVO SWEEP / PULSE=1712 / look straight?`。
- 标定后角度域函数：

```c
// 标定后：角度(度) -> 脉宽(µs)，钳位到 [SERVO_LEFT_US, SERVO_RIGHT_US]
uint16_t Steer_AngleToUs(float deg);   // us = p_center + deg / k_s
```

> **免量角器选项（若舵机能带载转向）**：跳过 4.4.3 量角器，改用实验 6 的"画圆法"反算 δ = atan(γ·L/v)，OLED 直接显示 δ。需先做实验 3 拿到 L。代价：耦合实验 2/3/6，且依赖舵机扭矩够。默认仍走量角器，稳。

### 4.4 你要做什么

#### 4.4.1 找居中 `p_center`
1. 关电机，后轮抬起或贴地均可。前轮正前方放**长直边/激光线**对准车身纵轴。
2. 开机 → `Calib_Servo_Center_Sweep` 自动扫脉宽，**OLED 显示当前 PULSE**，每值停 2s。
3. 看前轮，"最直"那刻记下 OLED 上的 PULSE = `p_center`。
4. 验证：放地上手推前进 1m 走直即可（量角器无需）。

#### 4.4.2 找左右极限
1. 从 p_center 跑 `Calib_Servo_Limit_Sweep` 向右，每步停 1.5s。
2. 听到舵机到挡块的轻微堵转声，**读 OLED 上的 PULSE** = `p_right_max`，立即复位（别久堵）。
3. 反向找 `p_left_max`。

#### 4.4.3 角度拟合 `k_s`（量角器，本项 OLED 帮不上）
1. 在 `[p_left_max, p_right_max]` 取 7 个脉宽点（等分）。
2. 每个点，用量角器量**右前轮纵平面与车身纵轴夹角**（规定右转为正）。
3. 左右前轮角不同（Ackermann），**统一量右轮**，记录"右轮是内轮还是外轮"（右转时右轮=内轮）。
4. 脉宽值可直接读 OLED（手动设个 `STEER,US` 或用扫描停在 7 点上）。

### 4.5 你要测什么（记录表）

| 脉宽 p(µs) | 右轮角 δ(°) | 备注 |
|---|---|---|
| p_left_max | | |
| ...（7 点） | | |
| p_center | 0 | |
| ...（7 点） | | |
| p_right_max | | |

### 4.6 反馈给我什么

- `p_center`、`p_left_max`、`p_right_max`（µs）。
- 上表 7+ 点 (p, δ)。
- 我会线性回归出 `k_s`、`SERVO_CENTER_US`、`SERVO_LEFT/RIGHT_US`，并在 `test_calib.c` 实现 `Steer_AngleToUs()`，替换 [test_distance.c:76](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L76) 的 `SERVO_CENTER_US`。

---

## 5. 实验 3：几何参数测量（轴距/轮距）

### 5.1 原理

纯几何，钢尺量。关键在"接地点投影"要准：
- **轴距 L**：前轴中心 ↔ 后轴中心，沿车身纵轴投影到地面的距离。
- **轮距 T**：左右后轮**接地点中心**的间距（前轮距同理，但阿克曼里程主要用后轮距）。

### 5.2 现有代码

无（参数缺失，本实验填坑）。

### 5.3 代码设计

在 `test_calib.h` 或新建 `Core/Inc/vehicle_params.h` 添加常量集中管理（让所有 test 共用）：

```c
// vehicle_params.h —— 所有标定结果集中于此
// ✅ = 已有实测值，直接搬；🆕 = 待标定后填入
#define VEH_WHEELBASE_MM      ???   // 🆕 L, 实验3+6
#define VEH_TRACK_WIDTH_MM    ???   // 🆕 T, 实验3
#define VEH_WHEEL_CIRC_CM     21.0f // ✅ =1560.13/74.29，改自原错值 19.8
#define VEH_COUNTS_PER_CM_L   74.29f  // ✅ 已标，保留
#define VEH_COUNTS_PER_CM_R   74.26f  // ✅ 已标，保留
#define VEH_COUNTS_PER_REV_L  1560.13f // ✅ 已标，保留
#define VEH_COUNTS_PER_REV_R  1559.37f // ✅ 已标，保留
#define VEH_REV_ERROR         ???   // 🆕 实验1 实测超调圈数
#define VEH_PWM_DEAD_L        ???   // 🆕 实验4
#define VEH_PWM_DEAD_R        ???   // 🆕 实验4
#define VEH_K_MOTOR_L         ???   // 🆕 实验4
#define VEH_K_MOTOR_R         ???   // 🆕 实验4
#define VEH_PWM_BALANCE       ???   // 🆕 实验4 左右偏置
#define VEH_SERVO_CENTER_US   ???   // 🆕 实验2
#define VEH_SERVO_LEFT_US     ???   // 🆕 实验2
#define VEH_SERVO_RIGHT_US    ???   // 🆕 实验2
#define VEH_SERVO_DEG_PER_US  ???   // 🆕 实验2 k_s
#define VEH_K_DIFF            ???   // 🆕 实验5
```

然后把 test_distance.c / test_straight_control.c 里散落的同名 `#define` 删掉，统一 `#include "vehicle_params.h"`。

### 5.4 你要做什么

1. 车放平地，前轮打直（用实验 2 的 `p_center`）。
2. 用钢卷尺量：
   - 左前轮接地点中心 ↔ 左后轮接地点中心 = `L_left`
   - 右前轮接地点中心 ↔ 右后轮接地点中心 = `L_right`
   - 取均 `L = (L_left + L_right)/2`
3. 量后两轮接地点中心间距 = `T_rear`；前两轮接地点中心间距 = `T_front`。
4. 各量 3 次取均。

### 5.5 你要测什么

| 量 | 测1 | 测2 | 测3 | 均 |
|---|---|---|---|---|
| L_left (mm) | | | | |
| L_right (mm) | | | | |
| T_rear (mm) | | | | |
| T_front (mm) | | | | |

### 5.6 反馈给我什么

- 上表。我会填 `VEH_WHEELBASE_MM`、`VEH_TRACK_WIDTH_MM`。

---

## 6. 实验 4：电机 PWM-速度特性

### 6.1 原理

直流电机 + H 桥稳态：

```
v(PWM) = k_m · (PWM − PWM_dead)      PWM_dead ≤ PWM ≤ PWM_sat     ← ⑧
```

- `PWM_dead`：克服静摩擦的最低启动 PWM（左右不同）。
- `k_m`：线性区斜率（cm/s 每 PWM 计数）。
- 左右 `k_m` 差异 → 直行偏置 `PWM_BALANCE`（让 v_L = v_R）。
- 饱和：PWM 再大速度不增（约 ARR=7199 附近）。

### 6.2 现有代码能否直接做

- `MOTOR,B` + `ENC,GET,CM` 手动逐点也能做，但**逐点手记太累**。
- 用 `test_calib.c` 自动扫描 + OLED 显示，**每点 OLED 实时出 (PWM, vL, vR)**，你只需边看边抄。

### 6.3 代码设计（自动扫描 + OLED）

`test_calib.c` 加：

```c
// deadband 扫描：PWM 递增，OLED 实时显示，启动即停并 Hold 显示临界值
void Calib_Motor_Deadband(uint16_t T_hold_ms);

// 速度扫描：按 pwm_list 各跑 measure_ms，OLED 逐点显示 (PWM,vL,vR)，末页 Hold deadband
typedef struct { uint32_t pwm; float v_L; float v_R; } MotorPt;
void Calib_Motor_Speed_Sweep(const uint32_t* pwm_list, uint16_t n,
                             uint16_t measure_ms, MotorPt* out);
```

伪码（Speed_Sweep）：

```
OLED: "MOTOR CALIB / PLACE / START 3s"
for each pwm in pwm_list:
    ENC counter -> 0
    MOTOR forward pwm,pwm
    t0=HAL_GetTick(); HAL_Delay(measure_ms); t1=HAL_GetTick()
    dL = enc left cm;  dR = enc right cm
    dt_s = (t1-t0)/1000
    vL=dL/dt_s; vR=dR/dt_s
    out[i]={pwm,vL,vR}
    OLED: "MOTOR CALIB / PWM=2000 / vL=12.3 vR=12.1 / (i+1)/n"   // 停 2.5s 让你抄
    MOTOR,STOP
    HAL_Delay(1500)  // 散热
OLED Hold: "MOTOR DONE / DEAD_L=.. DEAD_R=.. / kM_L=.. kM_R=.. / [reset]"
```

注意：
- PWM 设值用 `Drive_Forward`/`Motor_Stop` 同款（[test_distance.c:319-336](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L319-L336)）。
- `dt` 用 `HAL_GetTick()` 差值（比 `HAL_Delay` 标称值准）。
- k_m 回归可在 MCU 上做（sum x, sum y, sum xy, sum x²，4 个 float），OLED 末页直接出斜率；你也可只抄原始点，回归我来。
- 跑 1~2m 停车读，避免撞墙。

### 6.4 你要做什么

1. 找一段 **3m+ 直道**，车起点对齐。
2. 开机 → OLED "PLACE / START 3s"，3s 后自动扫。
3. 每个 PWM 点 OLED 显示 `(PWM, vL, vR)` 停 2.5s，**边看边抄**进下表。
4. 扫完 OLED 末页常驻 `DEAD_L/DEAD_R`（和 kM，若代码算了）。
5. 车快撞墙立即断电。

PWM 列表建议：`[800, 1200, 1600, 2000, 2400, 2800, 3200, 3600, 4500, 5500, 6500]`，`measure_ms=1000`。

### 6.5 你要测什么（从 OLED 逐行抄）

| PWM | v_L (cm/s) | v_R (cm/s) |
|---|---|---|
| 800 | | |
| ... | | |
| 6500 | | |

外加：OLED 末页的 deadband 临界 PWM（左/右分别）。

### 6.6 反馈给我什么

- 上表 + deadband 值。
- 我会回归 `k_m_L/R`、`PWM_dead_L/R`、`PWM_BALANCE`，填进 `vehicle_params.h`，并据此设计**速度 PID + 前馈**（`v_target → PWM = PWM_dead + v/k_m`）。

---

## 7. 实验 5：差速偏航响应标定（喂给当前 yaw PI）

### 7.1 原理

本车现状（见 [test_distance.c:155-174](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L155-L174)）：舵机静态居中，靠左右后轮 PWM 差速 `ΔPWM = L−R` 产生偏航修正。经验线性：

```
γ (deg/s) ≈ k_diff · ΔPWM       ← ⑩
```

标定 `k_diff` 后，`KP_YAW` 就有物理依据：要让 1° 偏航产生 γ_corr deg/s 修正，反推 ΔPWM。

### 7.2 现有代码

无 yaw 响应标定。需新代码（要联读陀螺）。

### 7.3 代码设计

`test_calib.c` 加：

```c
// 固定 BASE_PWM，施加 ±ΔPWM，测稳态 γ(deg/s)
void Calib_Diff_Yaw(uint32_t base_pwm, const int32_t* delta_list, uint16_t n,
                    uint16_t settle_ms);
```

依赖：复用 [test_distance.c](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c) 的 `Gyro_Init / Gyro_Calibrate / Gyro_ReadZ_Raw / ICM_*` 低层（建议把这些提取到独立 `gyro_drv.c` 让 calib 与 test 共用，避免重复代码——但这是重构，标定阶段先 copy 也可）。

伪码：

```
Gyro_Init(); Gyro_Calibrate();     // 静止
Servo_SetPulse(VEH_SERVO_CENTER_US)
for each ΔPWM in delta_list:       // e.g. [-400,-200,0,200,400]
    L = base + ΔPWM;  R = base - ΔPWM
    ENC,RESET; yaw_acc=0; t0=tick
    MOTOR,B,L,R
    HAL_Delay(settle_ms)           // e.g. 1500ms 让瞬态过去
    for i in [0, 200ms]: yaw_acc += Gyro_ReadZ - offset   // 求 γ 均值
    γ = mean(gyro_z - offset)/131   // deg/s
    MOTOR,STOP
    print ΔPWM, γ
    HAL_Delay(1500)
```

### 7.4 你要做什么

1. 找 **2m+ 空地**（差速会让车画弧，需空间）。
2. 静止开机让陀螺标定完成。
3. 跑扫描，`base_pwm=2500`，`delta_list=[-300,-150,0,150,300]`。
4. 每点会自动停车后打印 (ΔPWM, γ_deg_s)。

### 7.5 你要测什么

| ΔPWM (L−R) | γ (deg/s) |
|---|---|
| -300 | |
| -150 | |
| 0 | |
| 150 | |
| 300 | |

### 7.6 反馈给我什么

- 上表。
- 我会线性回归 `k_diff = γ/ΔPWM`，并据此**重算 `KP_YAW`**：目标"1° 偏航产生 γ_target deg/s 修正"→ `KP_YAW = γ_target / k_diff / 1°`，替换手调的 40。

---

## 8. 实验 6：阿克曼转向偏航响应标定（验证模型 ①）

### 8.1 原理

核心验证 ① `γ = (v/L)·tan(δ)`。已知 v（编码器）、δ（实验 2）、γ（陀螺），反算有效轴距：

```
L_eff = v · tan(δ) / γ           （γ 转 rad/s，v 用 m/s）
```

若 `L_eff ≈ L_ruler`（实验 3），模型成立，可做轨迹闭环；若显著偏大，说明有侧偏/滑移，需修正。

> 前提：舵机扭矩要能带负载转向。文档 [test_distance.c:18-24](file:///c:/Users/HU0021AN/Downloads/mdp21_8/Core/Src/tests/test_distance.c#L18-L24) 提到实测"舵机扭矩 < 地面摩擦"。若地面摩擦太大，实验 6 需在**较滑地面**或**降速**做。先验证舵机能动态转向再跑。

### 8.2 现有代码

无。需新代码（联控舵机+电机+陀螺+编码器）。

### 8.3 代码设计

`test_calib.c` 加：

```c
// 给定 δ(deg) 与 v_target(cm/s)，跑 T_ms，测稳态 γ
typedef struct { float delta_deg; float v_cm_s; float gamma_dps; float L_eff_mm; } AckermannPt;
void Calib_Ackermann_Yaw(float delta_deg, float v_target_cm_s,
                         uint16_t settle_ms, uint16_t measure_ms, AckermannPt* out);
```

伪码：

```
Gyro_Init(); Gyro_Calibrate()
p = Steer_AngleToUs(delta_deg)           // 实验2结果
Servo_SetPulse(p)
// 用实验4的 k_m 把 v_target 转 PWM（开环前馈即可，精度够）
pwm = VEH_PWM_DEAD_L + v_target / VEH_K_MOTOR_L
ENC,RESET
MOTOR,B, pwm, pwm
HAL_Delay(settle_ms)                     // 进入稳态圆弧
for i in measure_ms/20:                  // 20ms 采
    yaw += (Gyro_ReadZ-offset)/131 * 0.02
    dL += encL delta; dR += encR delta
γ = mean rate;  v = (dL+dR)/2 / measure_s
L_eff = v * tan(δ_rad) / γ_rad
MOTOR,STOP; Servo center
print delta, v, γ, L_eff
```

### 8.4 你要做什么

1. 前提：实验 2 已完成（有 `Steer_AngleToUs`）。
2. 在**空旷/较滑地面**，跑 3 组：
   - `δ = 10°, v = 20 cm/s`
   - `δ = 15°, v = 20 cm/s`
   - `δ = 10°, v = 30 cm/s`
3. 每组车会画弧，确保半径够大不撞物（`R = L/tan(δ)`，10° 时约 `L/0.176`）。

### 8.5 你要测什么（代码自算打印）

| δ (°) | v (cm/s) | γ (deg/s) | L_eff (mm) |
|---|---|---|---|
| 10 | 20 | | |
| 15 | 20 | | |
| 10 | 30 | | |

### 8.6 反馈给我什么

- 上表 3 组 + `L_eff` 均值。
- 与实验 3 的 `L_ruler` 对比结论。
- 我会确认模型可用性，并据此设计**基于前轮转向的偏航控制器**（替代当前差速 hack），或在 `L_eff≠L` 时引入侧偏修正。

---

## 9. 全部跑完后，给我这 1 张汇总表

```
项目                  数值              来源
────────────────────────────────────────────
COUNTS_PER_CM_L       74.29 counts/cm   ✅ 已有
COUNTS_PER_CM_R       74.26 counts/cm   ✅ 已有
COUNTS_PER_REV_L      1560.13 counts/rev ✅ 已有
COUNTS_PER_REV_R      1559.37 counts/rev ✅ 已有
C_roll                21.0 cm            ✅ 派生（改自原错值19.8）
REV_ERROR_COMP        ???? rev           🆕 实验1 实测
PWM_DEAD_L            ????               🆕 实验4
PWM_DEAD_R            ????               🆕 实验4
K_MOTOR_L             ???? cm/s/PWM     🆕 实验4
K_MOTOR_R             ???? cm/s/PWM     🆕 实验4
PWM_BALANCE           ????               🆕 实验4
SERVO_CENTER_US       ???? µs           🆕 实验2
SERVO_LEFT_US         ???? µs           🆕 实验2
SERVO_RIGHT_US        ???? µs           🆕 实验2
SERVO_DEG_PER_US      ???? deg/µs       🆕 实验2
WHEELBASE_MM          ???? mm           🆕 实验3
TRACK_WIDTH_MM        ???? mm           🆕 实验3
K_DIFF                ???? deg/s/PWM    🆕 实验5
L_eff                 ???? mm           🆕 实验6
────────────────────────────────────────────
```

收到后我会：
1. 生成 `Core/Inc/vehicle_params.h`（集中常量，✅ 项直接填）。
2. 让 test_distance.c / test_straight_control.c 改 `#include` 它们。
3. 写 `test_calib.c/h` 的完整实现（本次只给设计）。
4. 据实验 4/5 重算速度 PID 与 yaw PI 增益。

---

## 10. 建议执行顺序与依赖

```
实验1 (超调补偿，小) ──┐
实验2 (舵机)         ──┤
实验3 (几何，钢尺)   ──┼──> 实验4 (电机) ──> 实验5 (差速) ──> 实验6 (阿克曼)
                      │                              （依赖 v、δ 标定）
                      └──────────────────────────────┘
实验1/2/3 互不依赖，可先做；实验6 依赖 2 和 4。
```

- 实验 1/2/3 **无需新代码**：1 用现有 `MOTOR,B`/`STOP`/`ENC,GET,CM` 命令；2 用 `STEER,US` + 量角器；3 用钢尺。
- 实验 4/5/6 需要我写 `test_calib.c`，但**先做 1/2/3 把数据给我**，我边写代码边填参数，最稳。

---

## 11. 注意事项

1. **陀螺标定**：每次开机跑 4/5/6 前必须 `Gyro_Calibrate`（静止 5s），否则 yaw 漂。
2. **编码器 16 位回绕**：±32767，约 21 圈回绕；长距离测速用 `test_distance.c` 同款"每周期读 delta 累加"法，别直接读总计数。
3. **PWM 钳位**：左 TIM4 ARR=7199、右 TIM9 ARR=7199，扫描别超 7000。
4. **舵机堵转**：找极限时听到堵转声立即回退，勿久堵烧舵机。
5. **安全**：实验 4/5/6 车会动，预留空间，随时断电。
6. **地面一致性**：所有动态实验（4/5/6）用**同一地面**，否则 k_m/k_diff 失真。
