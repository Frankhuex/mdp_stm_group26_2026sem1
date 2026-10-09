# Straight distance control - integrated production logic

There is now only **one active straight-distance algorithm**:

```text
Core/Src/control/straight_control.c
```

Both the assessment UART command interface and the straight calibration test call this same controller.

## Command path

```text
STRAIGHT,GOCM,<cm>
        |
        v
assessment/command_interface.c
        |
        v
Straight_StartCm(cm)
Straight_Step()
        |
        v
straight_control.c
```

## Heading and wheel-speed control

The integrated controller ports the donor distance-test strategy:

- ICM-20948 gyro yaw and rate control the steering servo with PID correction.
- Servo center is the shared `RobotHW_ServoCenterUs()` value (1725 us),
  plus the straight-only trim in `straight_tuning.h` (currently +8 us).
- Left/right encoder delta mismatch trims the two motor PWM values.
- The accelerometer roll/pitch code from the donor file is not part of the
  motion loop because it does not feed yaw, steering, motor PWM, or stopping.
  Accelerometer diagnostics remain under the component-test folder.

The controller keeps planned and measured route heading across forward, reverse,
turns, IR and ultrasonic straight commands. Straights correct the retained route
error; turns update the planned heading by their signed requested angle and
adjust local travel for bounded incoming error before existing calibration.
Stationary sensor resets do not erase the route frame. H1 means route correction
is ON; command status reports `route_heading=1`, and `HEADING,STATUS` exposes
planned/measured/error/quality. Historical gaps flag degraded confidence without
rejecting a move. This holds heading, not sideways position. See
[route test guide](MOTION_SEQUENCE_TESTS.md) for ON/OFF tests and reset behavior.

## Distance profiles

### 10 to 20 cm - SHORT profile

Current selection rule: **10-20 cm uses SHORT**.

The controller uses the donor short-distance behavior:

- 100 ms kickstart at PWM 3200.
- Triangular motor ramp: 2950 -> 2980 -> 2350.
- Short-distance calibration offset: -2.5 cm.
- Additional overshoot compensation: 1.5 cm forward, 0.5 cm reverse.

Together these subtract 4 cm forward and 3 cm reverse from the encoder stop
target. They are empirical early-stop allowances, not final ruler distances.

### More than 20 cm - LONG profile

The controller uses the donor long-distance behavior:

- Base PWM 3300.
- Slow PWM 2400 after 0.80 wheel revolution of progress.
- Distance stop target uses wheel circumference 20.4 cm and empirical
  compensation of 0.310 revolutions forward, 0.260 reverse.

This is the profile intended for the Week 7 80-120 cm straight-line assessment.

## Important retesting

The final robot must be retested on the actual assessment floor and battery
state in both directions. Recommended forward tests:

```text
STRAIGHT,GOCM,10
STRAIGHT,GOCM,20
STRAIGHT,GOCM,30
STRAIGHT,GOCM,40
STRAIGHT,GOCM,50
STRAIGHT,GOCM,80
STRAIGHT,GOCM,100
STRAIGHT,GOCM,120
```

Record physical distance, lateral deviation, final yaw, and left/right encoder
travel for at least three runs at each assessment distance.
