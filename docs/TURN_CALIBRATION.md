# Turn calibration guide

How to re-calibrate the robot's left/right turns when the floor, tyres, or
mechanics change. Two independent things get tuned, in this order:

1. **Angle** — does a commanded 90° actually sweep 90°? → `*_TARGET_SCALE`
2. **Position / radius** — does the arc have the right radius (so a fwd+rev
   retrace returns to the same spot, and R is correct)? → `*_RADIUS_SCALE`

All knobs live in `Core/Inc/control/turn_tuning.h`.

## How a turn works (so the numbers make sense)
- The steering servo is set to a **fixed** angle for the radius and held
  (open-loop steering — no PID during the turn, unlike straight drives).
- The gyro integrates the swept yaw and the turn **stops when yaw hits the
  target** (`target = commanded_angle × TARGET_SCALE`), with a coast-time
  prediction that cuts the motor early so it settles onto the target.
- So angle control is **gyro feedback**; expect ~±1° scatter around the target
  (coast/settle residual). That's normal and fine.

## The instrument
`DONE,TURN` reports the measured sweep:
```
DONE,TURN,state=DONE,dir=LEFT,r_cm=20,a=90,yaw_meas=+90.5,peak_dps=+98.3
```
- `yaw_meas` = gyro angle actually swept, **signed: + = LEFT, − = RIGHT**.
- `peak_dps` = peak yaw rate (near ±250 would mean the gyro saturated; we see
  ~100 left / ~77 right, so no saturation).

## Step 1 — angle (`*_TARGET_SCALE`)
Forward and reverse are tuned separately. For each side, send a forward and a
reverse 90° turn and read `yaw_meas`:
```
TURN,LEFT,R=20,A=90      -> note yaw_meas  (want ≈ 90)
TURN,LEFT,R=20,A=-90     -> note yaw_meas  (want ≈ 90 magnitude)
```
Correct each scale with:
```
new_scale = old_scale × 90 / |yaw_meas|
```
- forward  → `TURN_<SIDE>_TARGET_SCALE`
- reverse  → `TURN_<SIDE>_REVERSE_TARGET_SCALE`

Rebuild + flash, re-measure, repeat until both read ≈ ±90. When forward and
reverse both land on 90, a fwd+rev retrace **cancels in heading** (no angled
finish).

## Step 2 — position / radius (`*_RADIUS_SCALE`)
Only after the angles match. Do the retrace and measure where it lands:
```
TURN,<SIDE>,R=<r>,A=90
TURN,<SIDE>,R=<r>,A=-90   -> mark rear-axle midpoint before/after; how far off?
```
- lands **to the RIGHT** → reverse arc too wide → **lower** the reverse radius
  scale (tighter). lands **LEFT** → **raise** it.
- `TURN_LEFT_REVERSE_RADIUS_SCALE` is the knob for left; right currently shares
  one scale (`TURN_RIGHT_RADIUS_SCALE`) for both directions.
- Iterate ±0.01–0.02.

## Verifying the absolute radius
`R` is defined at the **rear-axle midpoint**. The clean check is a **180°**
turn — the midpoint traces a half-circle, so start→end = diameter:
```
TURN,RIGHT,R=35,A=180    -> rear-axle-midpoint chord should be ≈ 2R = 70 cm
```
General chord of an arc: `chord = 2·R·sin(angle/2)` (so a 90° turn is
`R·√2 ≈ 49.5 cm` at R=35, not 70). If the 180° chord ≠ 2R, adjust
`TURN_<SIDE>_RADIUS_SCALE`.

## Current tuned values (2026-10-04, this floor)
| knob | value |
|---|---|
| `TURN_LEFT_TARGET_SCALE` | 0.996 |
| `TURN_LEFT_REVERSE_TARGET_SCALE` | 0.999 |
| `TURN_RIGHT_TARGET_SCALE` | 1.013 |
| `TURN_RIGHT_REVERSE_TARGET_SCALE` | 1.008 |
| `TURN_LEFT_RADIUS_SCALE` | 0.954 |
| `TURN_LEFT_REVERSE_RADIUS_SCALE` | 0.900 |
| `TURN_RIGHT_RADIUS_SCALE` (both dirs) | 1.000 |

Left: R=20. Right: R=35 (right's physical min radius is larger — asymmetric
steering). Both land ~90° and retrace to the spot.

## Build + flash
```bash
cd Debug
make all -j$(nproc)
arm-none-eabi-objcopy -O binary mdp21_8.elf mdp21_8.bin
st-flash --reset write mdp21_8.bin 0x08000000
```
(If `st-info --probe` shows `Found 0` → replug the ST-Link USB. If `chipid
0x000` → the board isn't powered / still settling.)
