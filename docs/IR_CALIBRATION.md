# Two Sharp IR Sensors - Test and Calibration

Sensors: **2 x Sharp GP2Y0A21YK / GP2Y0A21YK0F**

Current STM32 mapping:

| Sensor | Supply | Signal | ADC |
|---|---|---|---|
| Left IR | 5V + GND | PC0 | ADC1_IN10 |
| Right IR | 5V + GND | PC1 | ADC1_IN11 |

The ADC is configured as a two-channel scan sequence: Rank 1 = PC0/IN10 and Rank 2 = PC1/IN11, 12-bit, 144-cycle sampling.

## Stage 1 - Verify both sensors work

In `Core/Inc/test_config.h` select:

```c
#define ACTIVE_TEST TEST_IR
```

Flash the STM32 and open USART3 at **115200 8-N-1**.

The output is similar to:

```text
IR_L raw=1631 mV=1314 | IR_R raw=473 mV=381
```

Test one sensor at a time:

1. Put a flat target in front of the LEFT sensor and move it from far to near. LEFT should change clearly while RIGHT remains comparatively stable.
2. Repeat for the RIGHT sensor.
3. If one side remains almost fixed, check its 5V, GND, VO wiring, and the PC0/PC1 signal connection before calibration.

For this bench test keep the **motor switch OFF**.

## Stage 2 - Collect calibration points

Select:

```c
#define ACTIVE_TEST TEST_IR_CALIBRATION
```

Available UART commands:

```text
L,<cm>   capture LEFT sensor at a known distance
R,<cm>   capture RIGHT sensor at a known distance
B,<cm>   capture BOTH at the same known distance
LIVE     continuous raw/mV display
HELP     show help
```

Example:

```text
L,20
R,20
B,30
```

Each capture averages 64 ADC scans and prints a CSV-friendly record, for example:

```text
IR_CAL,L,cm=20,raw=1512,mV=1218
IR_CAL,R,cm=20,raw=1489,mV=1200
IR_CAL,B,cm=30,L_raw=1032,L_mV=831,R_raw=1017,R_mV=819
```

Suggested calibration distances:

```text
80, 60, 50, 40, 30, 25, 20, 15, 10 cm
```

At each distance:

- measure from the sensor face to a flat target,
- keep the target approximately perpendicular to the sensor,
- keep the robot and target still while sampling,
- collect LEFT and RIGHT independently,
- repeat each distance 2-3 times if time allows.

## Stage 3 - Fit each IR sensor separately

Do **not** assume the two sensors have identical voltage-distance curves. Keep separate calibration constants for LEFT and RIGHT.

The current clean project intentionally does not convert IR voltage to centimetres yet. After real measurements are collected, fit the curve and then add the final conversion to `Core/Src/sensors/ir_sensor.c`.

A blank table is provided in `docs/IR_CALIBRATION_TEMPLATE.csv`.
