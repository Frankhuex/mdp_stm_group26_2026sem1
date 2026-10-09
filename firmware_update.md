# Firmware update — Group 24 (RPi-side tuning)

Changes we made to `mdp21_8_SmallCalibration_Task2CmdGroup` while integrating it
with the RPi. All are small constant tweaks (no logic/structure changes). Diffed
against the zip you sent; this documents every source change.

---

## 1. Faster per-command timing  (the main one)

The board recalibrated the gyro **before every command** (~5.5 s each) and
settled for 1 s after every motion, so there was a long dead gap between each
`STRAIGHT`/`TURN`. On a ~40-command mission that's minutes of just waiting. We
shortened these — still a **fresh gyro zero every command**, just quicker.

**`Core/Inc/drivers/robot_tuning.h`**
| constant | was | now | effect |
|---|---|---|---|
| `GYRO_CALIB_SAMPLES` | 500 | **150** | per-command gyro recal ~5.5 s → ~1.7 s |
| `GYRO_CALIB_WARMUP_MS` | 500 | **200** | shorter settle before sampling |

**`Core/Src/control/turn_control.c`**
| constant | was | now | effect |
|---|---|---|---|
| `SERVO_SETTLE_MS` | 800 | **500** | wait for the steering servo to reach the turn angle |
| `POST_STOP_SETTLE_MS` | 1000 | **500** | post-motion settle before reporting DONE |

Net: ~4 s shaved off **every** command. After flashing we re-checked 90° turns —
`yaw_meas` still lands ~90 and repeats tightly, so the shorter calibration didn't
hurt angle accuracy. If you ever see the angle get noisy, raise
`GYRO_CALIB_SAMPLES` back toward ~250 (a middle ground).

---

## 2. Turn radius tune  `Core/Inc/control/turn_tuning.h`

| constant | was | now |
|---|---|---|
| `TURN_LEFT_REVERSE_RADIUS_SCALE` | 0.900 | **0.9200** |

(Left reverse arc made slightly wider so a forward+reverse retrace lands closer
to the start spot on our floor. Re-tune per your surface.)

---

## 3. Run mode  `Core/Inc/test_config.h`  — set for the RPi

| | was | now |
|---|---|---|
| `ACTIVE_TEST` | `TEST_HARDCODE_COMMAND_EXECUTOR` | **`ASSESSMENT_COMMAND_INTERFACE`** |

We set this so the board listens to the Pi over UART (the command interface)
instead of running the hardcoded command group. **Set it back to whatever you
need** for your own standalone tests — it's the "change only this line" selector.

---

## Build note (not a firmware change)
The `Debug/makefile` shipped with a hardcoded Windows path to
`STM32F407VETX_FLASH.ld` (`C:\Users\...`), which breaks a command-line `make` on
Linux. We repointed it to `../STM32F407VETX_FLASH.ld`. CubeIDE regenerates this
file from the project on build, so you don't need to do anything.

---

*Everything else is identical to the zip you sent. The interrupt-driven UART RX,
`CALIBRATE,CENTER`, `US,GET` and `yaw_meas` on `DONE,TURN` (which you'd already
merged) are untouched and working well with the Pi.*
