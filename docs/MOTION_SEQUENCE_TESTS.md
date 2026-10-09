# Route heading: implementation and floor tests

Current completion behaviour supersedes the turn-to-straight carry examples
below: a successful RIGHT command performs the stopped 1500 -> 1725 us return
and clears heading history; LEFT applies HEADING,OFF. Each action completes
before DONE and before the next array row. TURN,STATUS keeps the measured turn
yaw, while HEADING,STATUS shows the reset reference. See the
[automatic turn procedure](TURN_REVERSE_DIAGNOSIS.md#automatic-actions-after-each-turn).
The original carry examples remain background for straight-to-straight behaviour
and turn-start correction; they no longer describe heading after a successful turn.

The selected firmware test is `TEST_MOTION_SEQUENCE`. Build and flash before
testing; edits on the computer do not change the firmware already on the car.
The default run is **forward 40 cm, left arc of radius 20 cm through 90 degrees,
reverse 10 cm**. It runs once, starts after 5 seconds plus sensor preparation,
and pauses 10 seconds between legs. Leave the robot in place during pauses.
RESET repeats the selected case. No UART cable is required while moving.

## What changed

The command interface now keeps two headings: the planned route heading and
the gyro estimate. The first motion defines zero. Straights in either gear
keep the planned heading; turns add their signed requested angle to it.
For example, if forward ends at +2 degrees, the following left 90-degree turn
still aims for route heading +90, using a nominal local rotation of 88 degrees
before the existing left-turn calibration factor. Reverse after that turn
keeps +90 as its chassis-heading target; it does not change the target by 180.
A left turn driven in reverse instead changes heading negatively.

Incoming turn correction is limited to 5 degrees (and half the requested angle
for small turns). Existing side-specific turn calibration is preserved. Yaw
continues to accumulate through braking, stopped command gaps and turn settling.
Stationary gyro bias recalibration resets the sensor's local yaw, but does not
erase the route estimate or target. PID integral state is reset for each segment;
the retained heading error is what carries across commands.

The same tracking is used by GOCM, turns, IR straights and ultrasonic approaches.
`STOP`, explicit heading/yaw reset, and manual motor/steering or sensor-reset
commands start a new reference. Use a reset when physically repositioning the car
to begin a new independent route, not between legs of one route.

A historical sampling gap over 500 ms or an isolated failed gyro read sets
`degraded=1` / `Q:LOW`; it does **not** reject the next command or silently
redefine zero. The controller retains its last estimate and retries samples.
Existing persistent live-sensor-failure, timeout and invalid-command protections
still apply. Lost measurements cannot be reconstructed, so Q:LOW results are not
reliable evidence for calibration.

Distance compensation, speed profiles, PID gains and steering trim are unchanged
by this implementation. The fastest-car file and existing obstacle-course array
are unchanged. This is heading-error correction: it does not measure sideways
position, retrace a curved path, or guarantee a return to the same floor point.

## Selecting the tests

Edit `Core/Inc/test_config.h`, build and flash for each configuration:

```c
#define ACTIVE_TEST TEST_MOTION_SEQUENCE
#define MOTION_SEQUENCE_CASE 1
#define MOTION_SEQUENCE_HEADING_ON 1
```

`HEADING_ON=1` carries route error; `0` uses a fresh local heading for each
straight/turn, with the same gyro PID, motor synchronization, speeds and distance
calibration. Both modes still report cumulative planned/estimated heading, making
their OLED errors comparable. An OFF build is a baseline test, not gyro-off mode.

| Case | Moves, in order | Planned headings after each leg |
| --- | --- | --- |
| 1 (default) | F40; left R20 A90; B10 | 0, +90, +90 degrees |
| 2 | F40; right R35 A90; B10 | 0, -90, -90 degrees |
| 3 | F30; left R20 A90; F20; right R35 A90; B10 | 0, +90, +90, 0, 0 degrees |
| 4 | F40; B40 | 0, 0 degrees |

F/B values are centimetres. Turns are arcs, not pivots; R is radius in cm.
Right uses the existing minimum supported radius of 35 cm. Place the starting
point so the entire chassis and turning arc fit within the clear 2 m by 2 m area.

Suggested order: case 4 checks heading inheritance in reverse; cases 1 and 2
check both turn signs; case 3 checks a mixed route. For each case, run ON three
times and OFF three times, using the same marked start position and body heading,
floor, load and similar battery state. Do not change servo trim, PID gains or
distance compensation between ON and OFF runs.

## What to record

Mark the rear-axle midpoint and chassis direction at the start. At each pause,
measure the endpoint without moving the car. Record the rear-axle midpoint's
position and body heading using the floor marks; do not infer body heading just
from the front wheels, which can be steered. Measure each straight leg from its
own start to its end. A round trip can hide equal distance errors in both gears.

After a leg completes, the observer waits another 300 ms while sampling yaw,
then stores a snapshot. The OLED shows:

```text
2 L20/90 H1 NEXT
T+90.0 M+91.2
E+1.20 Q:OK
Turn:+90.7deg
```

- First row: leg, move, H1=route correction ON/H0=OFF. NEXT means another leg
  will start after the pause; END means saved results after the test.
- T: planned cumulative heading. M: estimated cumulative heading.
- E: M minus T, wrapped to -180..180 degrees. Positive is left; negative is right.
- Q:OK: no detected measurement gap/failure in this route. It is not a guarantee
  that the gyro scale, mechanical alignment or floor position is accurate.
- Last row: local turn yaw, or left/right encoder travel for a straight.
  Encoder travel includes braking but is not a ruler measurement; residual coast
  after release may still add physical travel.

When the route finishes, saved pages cycle every 8 seconds. Photograph them then;
handling the robot afterward does not change the stored values. A failed route
also shows a STOP page with the failing step. No leg resumes automatically.

Use this record for each repetition:

| Case / ON-OFF / run | Leg | OLED T / M / E / Q | Actual straight travel | Rear axle position or lateral offset | Actual body heading |
| --- | --- | --- | --- | --- | --- |
| 1 / ON / 1 | F40 | | | | |
| 1 / ON / 1 | L20/90 | | n/a | | |
| 1 / ON / 1 | B10 | | | | |

Look for smaller absolute heading errors and less variation after turns and
reverse with ON. Floor measurements are the independent check: a smaller OLED E
alone is insufficient. If both modes start with almost zero error, little
difference is expected. If ON is worse, capture all three runs before tuning
gains; correction sign, gyro scale, steering backlash and bias need examination.
There is no claimed millimetre/degree accuracy until these physical tests pass.

For the first feedback, send the saved pages and floor measurements for default
case 1 repeated three times. Then compare the same case with HEADING_ON=0.

## Normal command mode

Select `ASSESSMENT_COMMAND_INTERFACE` for UART operation. Route correction defaults
to ON. Commands are unchanged; wait for each DONE before sending the next motion.
`HEADING,STATUS` reports enabled, target, measured, error, tracking and degraded.
While stopped, `HEADING,RESET` starts a new origin without changing ON/OFF;
`HEADING,ON` or `HEADING,OFF` changes mode and resets the origin. These commands
are for route boundaries and A/B setup, not routine per-leg use.

## Software verification

`tests/host/run_straight_heading_tests.cmd` executes production straight and turn
controllers and the production command parser/sequencer against mocked hardware.
It covers both gears and steering sides, incoming-error correction, A/B behavior,
braking and pauses, timer/angle wrap, nonfatal historical gaps, persistent live
failures, explicit resets, STOP cancellation, mixed routes and IR regression.
The host harness uses Windows MSVC plus system DLL imports; no Windows SDK is
needed. The STM32 firmware must also build. Neither check proves floor accuracy.
