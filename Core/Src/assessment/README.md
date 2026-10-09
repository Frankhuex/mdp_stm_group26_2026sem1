# Assessment-facing STM32 code

This folder is for integrated robot behaviour, not individual hardware diagnostics.

Current active module:

- `command_interface.c` - USART3 command parser and integrated motion control interface.
  It calls the shared `straight_control`, `turn_control`, and `robot_hw` modules rather than reimplementing them.

Assessment mapping:

- A.1 STM32 <-> RPi USB/Serial movement communication: `command_interface.c`
- A.3 accurate 80-120 cm straight motion: `straight_control.c` (donor long-distance logic integrated; final floor retest/tuning still required)
- A.4 controlled 90-360 degree arc rotation: `turn_control.c`
- A.5 obstacle navigation: not finalized yet; IR/ultrasonic calibration should be completed first.

Do not add duplicate motor, encoder, gyro, straight, or turn implementations here. Assessment code should call the reusable modules.

## Lateral displacement commands

| Command | Final lateral displacement |
| --- | --- |
| `LATERAL,LEFT,IR` | Fresh left IR measurement + 10 cm |
| `LATERAL,RIGHT,IR` | Fresh right IR measurement + 10 cm |
| `LATERAL,LEFT,n` | n cm left; integer n from 1 to 55 |
| `LATERAL,RIGHT,n` | n cm right; integer n from 1 to 55 |

`LATERAL,STATUS` reports the source, target distance, rounded angle, stage,
turn result and start error. `STOP` or `LATERAL,STOP` cancels the whole action;
`TURN,STOP` and `STRAIGHT,STOP` also cancel an active lateral action. Hardcoded sequences wait
for both arcs and stop the sequence on either arc's failure.

The controller executes two forward arcs of equal angle and opposite steering:
LEFT runs left then right, RIGHT runs right then left. It reuses TURN's normal
calibration, braking and post-turn actions. The left soft radius is 20 cm and
the right minimum radius is 35 cm; the left hard minimum remains 18 cm.
All dimensions are defined in `Core/Inc/drivers/robot_tuning.h`.

With R = 20 + 35 = 55 cm, angle = acos(1 - d/R), converted to degrees and
rounded to the nearest integer for the existing TURN interface. Each arc is
at most 90 degrees, so d must be positive and at most 55 cm. The ideal forward
advance is sqrt(2*R*d - d*d); the manoeuvre needs room ahead as well as sideways.
For d = 30 cm the command angle is 63 degrees; IR = 30 cm adds 10 cm and uses
74 degrees. Integer angle rounding and real turn calibration affect the actual
displacement; verify on the floor.

IR mode reads `IRSensor_ReadDistance` once at command start and uses the
selected side's calibrated distance for both arcs. After validating the target
and computing the angle, it first drives forward until that side's IR is
strictly greater than 50 cm, then executes the two arcs immediately after the
normal straight stop/brake. No extra vehicle-length advance is added.
`stage=0` is the forward IR approach; `stage=1/2` are the two turns.
`ir_cm_x10` retains the command-start reading throughout all stages.
Only `IR_RANGE_VALID`
readings in the calibrated 10..50 cm range are accepted. The original
STRAIGHT condition-stop caches remain independent and are not used here.
The 10 cm is half the 10 cm obstacle width plus the 5 cm IR-to-centre offset.
A measured IR distance above 45 cm produces a displacement above 55 cm and
returns `ERR,BAD_VALUE`. ADC failure, FAR, NEAR or an invalid selected-side
reading at command start returns `ERR,IR_MEASUREMENT_INVALID` without moving.
During the forward approach, FAR counts as greater than 50 cm; ADC/selected-side
errors abort with STRAIGHT_ERROR. Approach timeout/travel limit/STOP also prevent
both turns. Numeric mode starts its two arcs directly and has no IR approach.
Completion emits one `DONE,LATERAL` for the composite, including the failing
stage when applicable; individual arcs do not emit `DONE,TURN`.

Host validation: compile `tests/host/test_lateral.c` together with
`Core/Src/control/lateral_control.c`, using `Core/Inc` headers and the math
library. The test simulates STRAIGHT and TURN and checks geometry, side selection, live IR
measurement/offset, approach-before-turn order, limits, busy handling, STOP and
straight/turn/start failures.

## Bidirectional UNTILUS

`STRAIGHT,UNTILUS,n` waits for consecutive fresh valid ultrasound measurements.
If distance is above n cm it drives forward until consecutive measurements are
strictly below n; if below n it reverses until consecutive measurements are
strictly above n. Direction stays fixed after starting. Initial confirmed
exact equality completes without driving. Approach speed reduction applies
in either direction. Initial persistent missing/invalid readings produce
SENSOR_ERROR without moving; invalid readings during motion cannot confirm a
crossing. Existing STOP, timeout and maximum travel limits remain active.
