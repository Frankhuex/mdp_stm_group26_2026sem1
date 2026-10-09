# STM32 firmware changes — Group 24 (mdp21_8)

Everything we changed in the firmware since the pulled `mdp21_8` zip
(git baseline `0e08586`, `ACTIVE_TEST=50` assessment command interface) up to
the current build. For the RPi/algo side none of this changes how you build or
flash — it changes the **UART command contract** and **motion accuracy**. Read
sections 1 and 2 first; they affect anyone talking to the board.

> Baseline: `0e08586` · Current: see `git log --oneline -- stm/mdp21_8/`
> All turns use the **rear-axle midpoint** as the reference; LEFT min radius 20 cm,
> RIGHT min radius 35 cm (asymmetric steering). UART: USART3, 115200 8-N-1.

---

## 1. UART command interface — contract changes (read this)

### New commands

| Command | Reply | What it does |
|---|---|---|
| `CALIBRATE,CENTER` | `CAL,START,...` → `CAL,trial=...` (per trial) → `CAL,DONE,center=<us>,was=<us> ...` or `ERR,CAL_NO_CROSSING` / `ERR,CAL_FLAT` | Auto servo-centre calibration. **Drives forward/back ~1.5 m several times** (needs clear space), fits yaw-vs-pulse, applies the found straight-ahead centre **live for this session** (not persisted — to persist, set `SERVO_CENTER_US`). |
| `SERVO,CENTER,<us>` | `OK,SERVO,CENTER,<us>` or `ERR,BAD_VALUE` | Manually set + apply the straight-ahead servo centre (µs), for bisecting centre by hand. Live only. |
| `US,GET` | `US,mm=<sensor→object mm>,status=<rc>` (`mm=0` = no echo / out of range) or `ERR,BUSY` / `ERR,NO_DEVICE` | **Static single front-ultrasonic read, NO motion.** The Pi uses this to decide the forward standoff without driving. `mm` is sensor→object; nose→face ≈ `mm/10 − 8 cm` (sensor sits ~8 cm behind the bumper). |

### Changed replies on existing commands

- **`TURN,...` now reports measured motion on completion:**
  ```
  DONE,TURN,state=DONE,dir=LEFT,r_cm=20,a=90,yaw_meas=+90.5,peak_dps=+98.3
  ```
  - `yaw_meas` = gyro angle actually swept this turn, **signed: + = LEFT, − = RIGHT**.
  - `peak_dps` = peak yaw rate during the turn (sanity/saturation check).
  The RPi relies on `yaw_meas` for turn calibration and for the heading-closed
  recenter, so keep this field.

- **`STRAIGHT,UNTILUS,<cm>`** (drive forward until ultrasonic threshold) reports:
  ```
  DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=<cm>,us_mm=<mm>,us_status=<v>
  ```
  The user threshold `<cm>` is treated as **nose/bumper→object clearance**; the
  firmware adds the ~8 cm sensor set-back internally. `reason=US_THRESHOLD` =
  stopped on the echo; anything else (e.g. `DISTANCE_LIMIT`) = never saw it.

> The RPi side flushes stale input and resends once on a framing error
> (`UNKNOWN_CMD` etc.), but that's a second layer — section 2 is the real fix.

---

## 2. UART RX is now interrupt-driven (reliability — important)

**Problem:** the command parser read UART **one byte at a time, polled in the
main loop** (`HAL_UART_Receive(..., 1, timeout)`), interleaved with blocking I2C
gyro reads. At 115200 a byte arrives every ~87 µs, but a gyro I2C read blocks
200 µs+, so a byte arriving during a gyro read **overran and was lost** →
corrupted command → intermittent `ERR,UNKNOWN_CMD`, worst on the first command
after an idle gap (e.g. after the Pi's camera/recognition pause).

**Fix:** RX is now **interrupt-driven with a ring buffer**.
- `USART3_IRQHandler` (`stm32f4xx_it.c`) calls `CommandInterface_UartIrq()`,
  which reads `DR` on `RXNE` (clearing RXNE **and** overrun) and pushes the byte
  into a 512-byte ring — it can't be starved by the main loop.
- `CommandInterface_Init()` enables `UART_IT_RXNE`.
- `CommandInterface_Poll()` just drains the ring into the existing line parser.
- **TX is unchanged** (still polled `HAL_UART_Transmit`).

If your team re-generates from CubeMX, preserve: the `UART_IT_RXNE` enable, the
ring buffer + `CommandInterface_UartIrq()` in `command_interface.c`, and the
`CommandInterface_UartIrq()` call in the `USART3_IRQHandler` USER CODE section.

---

## 3. Motion control & tuning

### Turns (`Core/Inc/control/turn_tuning.h`, `Core/Src/control/turn_control.c`)
- **Gyro-feedback stop with coast prediction** (not a PID mid-turn): the servo is
  held at a fixed steering angle; the turn stops when integrated yaw hits
  `target = commanded_angle × *_TARGET_SCALE`, cutting the motor a coast-time
  early so it settles onto target. Expect ~±0.5–1° scatter (normal).
- **Staged slowdown** as remaining yaw shrinks (CRUISE→MEDIUM→SLOW) before the
  coast stop. (A final "crawl" gear was trialled and reverted — the coast-brake
  trips before it engages; no net change.)
- **Calibrated values on our floor** (re-check when grip/tyres change — see
  `docs/TURN_CALIBRATION.md`):

  | knob | value | | knob | value |
  |---|---|---|---|---|
  | `TURN_LEFT_TARGET_SCALE` | 0.996 | | `TURN_LEFT_RADIUS_SCALE` | 0.954 |
  | `TURN_LEFT_REVERSE_TARGET_SCALE` | 0.999 | | `TURN_LEFT_REVERSE_RADIUS_SCALE` | 0.900 |
  | `TURN_RIGHT_TARGET_SCALE` | 1.013 | | `TURN_RIGHT_RADIUS_SCALE` | 1.000 |
  | `TURN_RIGHT_REVERSE_TARGET_SCALE` | 1.008 | | `TURN_LEFT_STEER_SCALE` | 1.00 |
  | | | | `TURN_RIGHT_STEER_SCALE` | 0.965 |

  Both sides land ~90° and a forward+reverse retrace returns ~to the spot;
  180° RIGHT chord ≈ 70 cm confirms R=35.

### Straight drive (`Core/Inc/control/straight_tuning.h`)
- **Heading-hold via gyro→servo** during straights, plus a small steering trim to
  cancel residual drift: `STRAIGHT_FORWARD_TRIM_US = 42`, `STRAIGHT_REVERSE_TRIM_US = 8`
  (applied as `SERVO_CENTER_US + trim`).

### Servo centre & backlash (`Core/Inc/drivers/robot_tuning.h`, `robot_hw.c`)
- `SERVO_CENTER_US = 1725` (the straight-ahead centre; also settable live via
  `CALIBRATE,CENTER` / `SERVO,CENTER`).
- **Backlash take-up:** `RobotHW_ServoSetUsTakeup()` approaches a target pulse
  from a consistent side (`SERVO_BACKLASH_US = 120`, `SERVO_TAKEUP_SETTLE_MS = 120`)
  so the centre is repeatable after a turn. Used by straight-centring and the
  calibration/center path. Fixed the "same pulse gave different heading before vs
  after a turn" problem.
- Runtime servo-centre support in `robot_hw.c` (`RobotHW_ServoSetCenterUs`,
  `RobotHW_ServoSetUsTakeup`) so calibration can apply without a reflash.

---

## 4. Instrumentation (`turn_control.c`, `command_interface.c`)
- Per-turn peak |yaw rate| tracked at the command tick (`Turn_PeakRateDps`) and
  settled swept yaw (`Turn_SettledYawDeg`) → surfaced as `peak_dps` / `yaw_meas`
  on `DONE,TURN` (section 1).
- `CALIBRATE,CENTER` streams per-trial `CAL,trial=...` lines (yaw vs candidate
  centre) for debugging the fit.

---

## 5. Build & flash
```bash
cd Debug
make all -j$(nproc)
arm-none-eabi-objcopy -O binary mdp21_8.elf mdp21_8.bin
st-flash --reset write mdp21_8.bin 0x08000000
```
(If `st-info --probe` shows `Found 0` → replug the ST-Link USB. `chipid 0x000`
→ board unpowered/still settling.)

---

## 6. Commit map (newest → oldest)
| commit | change |
|---|---|
| `f4933eb` | **Interrupt-driven UART RX** (ring buffer) — fixes dropped-byte `UNKNOWN_CMD` |
| `c67fe3a` | **`US,GET`** static ultrasonic read (no motion) |
| `b358bf8` | `docs/TURN_CALIBRATION.md` guide |
| `ea76618` | tune RIGHT-turn target scales to land 90° (gyro) |
| `afdf2a7` | tighten reverse-LEFT radius (0.930→0.900) for retrace position |
| `9a5f858` | tune LEFT-turn target scales + fix `peak_dps` for command turns |
| `4868ebc` | **turn instrumentation** (`yaw_meas` + `peak_dps` on `DONE,TURN`) |
| `e14eb0a` | calibrate with forward trim (centre matches the real drive pulse) |
| `5a18fba` | **steering backlash take-up** (repeatable centre after turns) |
| `0f64564` | **`SERVO,CENTER`** command + robust auto-cal |
| `13396e2` | **`CALIBRATE,CENTER`** auto servo-centre calibration |
| `0e08586` | baseline (pulled `mdp21_8` zip, `ACTIVE_TEST=50`) |

See also: `docs/TURN_CALIBRATION.md` (how to re-tune turns when the floor changes).
