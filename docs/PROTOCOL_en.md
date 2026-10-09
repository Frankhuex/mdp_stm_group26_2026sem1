# MDP21 STM32 ↔ RPi Communication Protocol (English)

**Version**: v1.10  
**Date**: 2026-09-23  
**Peer**: Raspberry Pi / PuTTY  
**Implementation**: [command_interface.c](../Core/Src/assessment/command_interface.c)

## 1. Connection and Run Mode

- USART3: PD8=TX, PD9=RX; 115200 baud, 8N1, no flow control.
- Use the onboard USB-UART bridge or cross-connect the RPi UART with common ground.
- Select `ASSESSMENT_COMMAND_INTERFACE` in `Core/Inc/test_config.h` for commands;
  the current selection is `TEST_HARDCODE_COMMAND_EXECUTOR` for a command-array run.
- Startup emits `HELLO,mdp21_v8\r\n`; the firmware identity is independent of the document version.

## 2. Message Format

- Uppercase ASCII, comma-separated, without spaces between fields.
- Requests end with `\r\n`, standalone `\r`, or `\n`; responses end with `\r\n`.
- The 128-byte RX buffer accepts at most 127 payload bytes, excluding the terminator. Overflow emits `ERR,RX_LINE_OVERFLOW` and discards input until the next line ending.
- Empty fields, integer overflow and unparseable numbers return `ERR,BAD_ARGS`; values outside a command's range return `ERR,BAD_VALUE`.
- Currently `CMD_ECHO_ENABLE=0`: STM does not echo input. Enable PuTTY local echo if typed characters should be visible.
- Numeric telemetry generally uses signed three-decimal formatting, such as `+12.340`; target centimeters, angles and ultrasonic millimeters are integers.

| Type | Format |
|---|---|
| Accepted/success | `OK[,key=value...]` |
| Rejected | `ERR,<reason>` |
| Query response | `<NAME>,key=value,...` |
| Motion terminal event | `DONE,<motion>,[mode=UNTILUS,]state=<S>,...` |

`OK` means accepted, not motion completion. Each accepted motion emits exactly one `DONE` when it reaches a terminal state. The `DONE` prefix also covers failure and abort: always inspect `state`. Immediately rejected commands have no corresponding completion event.

| Error | Meaning |
|---|---|
| `BAD_ARGS` | Invalid argument format/count, including integer overflow |
| `BAD_VALUE` | Value outside the supported range |
| `BUSY` | Motion owns the resource or the operation conflicts with motion |
| `NO_DEVICE` | Required device initialization failed |
| `UNKNOWN_CMD` | Unknown command while idle; unknown commands during motion return `BUSY` |
| `RX_LINE_OVERFLOW` | Payload exceeds RX buffer capacity |

`TIMEOUT`, `SENSOR_ERROR` and `DISTANCE_LIMIT` are motion terminal states in completion/status messages, not standalone `ERR` reasons emitted by this interface.

## 3. Straight Commands

### 3.1 Fixed Travel: GOCM

```text
STRAIGHT,GOCM,<cm>
OK,target_cm=<cm>
DONE,STRAIGHT,state=<S>,target_cm=<cm>
```

- `cm` is a signed integer with magnitude 10..200; positive drives forward, negative backward.
- Magnitude 10..20 selects SHORT; 21..200 selects LONG.
- SHORT: PWM 3200 for the first 100 ms, then a progress-based 2950→2980→2350 curve; target distance compensation is −4 cm forward, −3 cm reverse.
- LONG: PWM 3300, reduced to 2400 once either wheel has accumulated 0.8 revolutions; target revolutions are reduced by 0.310 forward or 0.260 reverse. These PWM values are before global scaling. Slowdown depends on distance already traveled, not remaining distance.
- Both profiles share yaw/rate-to-servo PID and wheel encoder-delta synchronization; both wheels must reach their compensated targets, then complete an 80 ms brake pulse. Yaw/encoder samples include braking.
- Straights and turns track planned/estimated heading. With heading ON, straight-to-straight moves retain error and incoming turn error adjusts nominal local rotation by at most 5 degrees (half the angle for very small turns), before side calibration. Reverse arc turns invert yaw direction. Successful turns now reset the reference as part of the post-turn actions in section 4; LEFT also switches heading OFF. Sensor preparation alone does not erase route error.
- Manual motor/steering commands, STOP, encoder/yaw resets and gyro reconfiguration reset the route origin. Historical gaps or isolated read failures flag degraded confidence without rejecting a move; persistent live sensor failures still fault. Holding heading does not correct sideways displacement.
- `HEADING,STATUS` returns `HEADING,enabled=<0|1>,target=<deg>,measured=<deg>,error=<deg>,tracking=<0|1>,degraded=<0|1>`. Positive is left. While stopped, `HEADING,RESET` resets the origin; `HEADING,ON`/`HEADING,OFF` changes mode and resets the origin. OFF is the startup default for normal UART and hardcoded runs. OFF retains local PID and reports cumulative heading for comparison, but does not carry correction between commands. Gyro-based turn stopping remains active. Explicit HEADING,ON or the dedicated A/B diagnostic can still opt into carry. See [test guide](MOTION_SEQUENCE_TESTS.md).
- `STRAIGHT,TRACE` dumps the last straight motion's buffered gyro/steering samples while stopped (`ERR,BUSY` during any motion). Reply starts with `TRACE,STRAIGHT,count=<n>,dropped=<n>,fixed_us=<us-or-0>`, followed by sample rows and `TRACE,STRAIGHT,END`. Samples include `ms`, `dir`, `phase`, `local_yaw`, `control_error`, `rate_dps`, `servo_us` (commanded, not measured), and `gyro_ok`. Phases: S=start, W=ultrasonic wait, D=drive, B=brake, E=end. The next accepted straight clears the trace; queries preserve heading. See [forward baseline trial](TURN_REVERSE_DIAGNOSIS.md#previous-test-forward-baseline-with-gyro-correction).
- Terminal states: `DONE`, `ABORTED`, `TIMEOUT`; timeout is `abs(cm) × 300 + 5000` ms.

GOCM status:

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,elapsed=<ms>,target_cm=<int>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>,route_heading=<0|1>,fixed_us=<us-or-0>
```

### 3.1.1 Fixed Steering Calibration: CAL

```text
STRAIGHT,CAL,35,1770
OK,mode=CAL,target_cm=35,servo_us=1770,gyro_steering=0
DONE,STRAIGHT,state=DONE,target_cm=35
```

Forward only: distance 10..50 cm, pulse within 100 us of the configured centre
(currently 1625..1825 us). The pulse stays fixed during driving and braking;
completion centres the servo. This command bypasses gyro steering, but retains
gyro measurement, encoder synchronization, normal distance compensation, braking,
timeout, sensor faults and STOP. No heading mode or permanent tuning is changed.
`HEADING,OFF` alone does not bypass gyro steering in normal GOCM.

Status and trace header report `fixed_us` for the last accepted straight:
nonzero identifies CAL; zero identifies normal gyro control. The next normal
GOCM, IR or ultrasonic straight automatically restores gyro steering. CAL uses
the same terminal event as GOCM; clients should retain the accepted mode.
Reset the heading origin while stopped before independent calibration trials.
CAL does not hold the route heading during the trial; accumulated yaw is still
tracked and can affect a later move if its origin is not reset.

### 3.2 Ultrasonic Threshold: UNTILUS

```text
STRAIGHT,UNTILUS,<cm>
OK,mode=UNTILUS,threshold_cm=<cm>
```

`cm` is a **positive integer from 5 to 200**, specifying bumper/object clearance, not travel distance. The firmware adds 8 cm internally because the ultrasonic sensor sits behind the bumper, so the controller compares against sensor/object clearance. Motion is forward only. Stop on the first fresh valid reading **strictly below** the internal threshold; equality does not complete the command.

- Wait for startup confirmation. Two consecutive valid readings already below threshold complete without motor power; two consecutive valid readings above threshold start driving. Three consecutive unknown readings also start driving and are treated as infinity.
- PB15=TRIG, PC8=ECHO via a 5V-to-3.3V divider, matching the working `mdp21_8_ultrasonic` example. TIM8_CH3 captures both edges at 1 MHz (ARR=65535); the driver starts input capture interrupts and reads CCR3.
- Triggers are at least 60 ms apart; each measurement times out after 60 ms. Trigger/capture follows the reference. UNTILUS treats zero or invalid results as infinity without changing the standalone sensor test.
- No echo, invalid/zero readings and readings older than 200 ms mean infinity: continue at cruise PWM and keep measuring. There is no 500 ms startup failure or stale-sample stop. STOP, 30-second timeout and 200 cm travel limit still apply.
- Base PWM: kickstart 4000, cruise 3500; at or below threshold + 25 cm, reduce to 2600, with existing wheel synchronization corrections applied.
- Total runtime is limited to 30 seconds; either wheel reaching 200 cm encoder travel stops motion. Clearance threshold and travel limit are distinct.
- GOCM travel compensation is not used. Stopping applies a short both-inputs-high brake pulse, then zeros motor PWM and centers steering. Stationary clearance may be smaller than the reading that triggered the stop.

Completion/failure event, in field order:

```text
DONE,STRAIGHT,mode=UNTILUS,state=<S>,reason=<R>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>
```

| `state` | `reason` | Meaning |
|---|---|---|
| `DONE` | `US_THRESHOLD` | Fresh valid measurement strictly below threshold |
| `ABORTED` | `ABORTED` | User sent a stop command |
| `SENSOR_ERROR` | `SENSOR_ERROR` | Sensor driver unavailable; missing echoes do not cause this state |
| `TIMEOUT` | `TIMEOUT` | Total runtime limit reached |
| `DISTANCE_LIMIT` | `DISTANCE_LIMIT` | Encoder travel limit reached |

UNTILUS status replaces the GOCM status format:

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,mode=UNTILUS,elapsed=<ms>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,us_age_ms=<uint>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>
```

`RUNNING` includes both initial measurement wait and actual driving. `us_mm` is millimeters, `us_age_ms` is the age of the latest sample, and `elapsed` is command runtime in milliseconds. `us_status=0` means valid and fresh, `-1` means unavailable/no rise/stale, and `-2` means an invalid or excessively long echo. Do not use `us_mm` as a valid clearance when status is nonzero. The last sample is retained after termination, but its age keeps increasing, so later queries may report it as stale.

### 3.3 Side IR Threshold: LEFTIR / RIGHTIR

GT / LT

新格式：`STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`，n 为整数厘米。
例如 `STRAIGHT,LEFTIR,GT,50`（大于 50 cm 停车）、
`STRAIGHT,RIGHTIR,LT,20`（小于 20 cm 停车）。等于阈值继续。
左侧范围 10～50 cm，右侧 10～60 cm。旧三段格式保留为 GT。
所有 IR 回报增加 `op=GT`／`op=LT`。

LT 遇到 NEAR（小于标定下限 10 cm）也停车；FAR 继续；
ADC／转换故障则停车并报告 SENSOR_ERROR，中止硬编码序列。
起步前已满足条件则不驱动车轮。采样、纠偏及超时／行程限制不变。
下文“大于阈值、FAR／ADC 故障正常 DONE”规则仅适用于 GT。

New syntax: `STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`. GT stops strictly above n;
LT stops strictly below n. Equality continues. Legacy three-field commands
remain GT aliases. Replies include `op=GT|LT`.
LT also stops on NEAR, continues on FAR, and stops with SENSOR_ERROR on
ADC/conversion failure (aborting sequences). Bounds and motion limits are
unchanged. The greater-than and failure-as-DONE descriptions below apply
only to GT.



```text
STRAIGHT,LEFTIR,GT,50
STRAIGHT,RIGHTIR,GT,50
OK,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,threshold_cm=<int>
DONE,STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>
```

- Forward only; stop when the selected side's distance is **strictly greater** than the target. Equality continues. No ultrasonic bumper offset is added.
- Integer thresholds: LEFTIR 10..50 cm, RIGHTIR 10..60 cm, matching the current calibration tables.
- FAR, ADC acquisition failure and conversion error count as infinity and complete normally with `state=DONE,ir_cm=FAR`. They do not raise an IR sensor fault.
- Read before applying drive power; an already-satisfied condition completes without driving. While moving, sample approximately every 40 ms; one qualifying sample is sufficient.
- Initial PWM 4000 for 120 ms, then 3300, with global scaling and normal straight steering/wheel synchronization.
- STOP gives ABORTED; 30 seconds gives TIMEOUT; either wheel reaching 200 cm gives DISTANCE_LIMIT. Gyro failure can still give SENSOR_ERROR. These failures stop the sequence.
- In a hardcoded sequence, normal completion (including FAR/ADC failure) advances to the next row.

Status uses this format, including after completion:

```text
STRAIGHT,STATUS
STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>
```

See [IR_STRAIGHT.md](IR_STRAIGHT.md) for examples and validation.

### 3.4 Stopping and Timing

- `STRAIGHT,STOP` → `OK,aborted=<0|1>`; aborting running straight motion, including the initial measurement wait, is followed by one `state=ABORTED` event.
- `STOP` → `OK`; zeros motor PWM, centers steering and aborts running straight/turn motion, followed by its completion event.
- The first motion may initialize and calibrate the gyro while stationary for approximately 5.5 seconds before replying `OK`. Later calibration and waits depend on `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`; see [MOTION_SETTINGS.md](MOTION_SETTINGS.md). First-use initialization remains blocking.
- Motion control steps at intervals of at least 20 ms; UART reception/transmission can still delay the main loop. OLED command refresh is skipped during motion, and independent gyro polling pauses to avoid double integration. There is no hard real-time stopping-latency guarantee.

## 4. Turn Commands

| Request | Arguments / response |
|---|---|
| `TURN,LEFT,R=<cm>,A=<deg>` | Integer R ≥18; signed integer A with magnitude 5..360 |
| `TURN,RIGHT,R=<cm>,A=<deg>` | Integer R ≥35; same A limits |
| Acceptance | `OK,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| Terminal event | `DONE,TURN,state=<S>,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| `TURN,STOP` | `OK,aborted=<0\|1>`; followed by one completion event if aborted |
| `TURN,STATUS` | `TURN,state=<S>,dir=<LEFT\|RIGHT>,elapsed=<ms>,r_cm=<int>,target_deg=<int>,yaw=<f>` |

LEFT/RIGHT selects steering side; positive A drives forward, negative A backward. R has no additional configured upper bound but must parse as int32. Terminal text includes `DONE`, `ABORTED`, `TIMEOUT`, `WRONG_SIGN`, `NO_TURN`, `BAD_CONFIG`, and `NO_IMU` (the last two are recognized interface states). Current turn parameters are defined in [turn_tuning.h](../Core/Inc/control/turn_tuning.h) and [turn_control.c](../Core/Src/control/turn_control.c).

With `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND=1U`, each successful TURN performs
its automatic follow-up before DONE, in either gear:

- RIGHT: after normal braking/settling with motors stopped, apply 1500 us for
  500 ms, then centre (1725 us) for 500 ms. Each pulse clears heading
  history like STEER,US; the enabled ON/OFF setting is preserved.
- LEFT: apply HEADING,OFF after normal braking/settling, including the heading
  origin and gyro-yaw reset. Local gyro steering remains active on later GOCM.

At `0U`, repeated gyro calibration and these follow-up actions are skipped:
normal braking and steering centring still occur, then the turn completes
without RIGHT steering holds or LEFT heading disable/reset.

RIGHT remains RUNNING during both non-blocking holds; new motion/manual steering
is rejected as busy, and STOP cancels the sequence. Completion actions do not
run after a turn failure/abort. TURN,STATUS preserves the completed turn yaw;
HEADING,STATUS reports the reset reference. Command arrays wait for these actions
before advancing, and UART clients must wait for DONE. See the
[automatic turn procedure](TURN_REVERSE_DIAGNOSIS.md#automatic-actions-after-each-turn).

## 5. System, Manual Control and Queries

Every reply below ends with `\r\n`. `<f>` denotes three-decimal numeric formatting; `<int>` denotes an integer.

| Request | Response / notes |
|---|---|
| `HELLO` | `HELLO,mdp21_v8` |
| `PING` | `PONG` |
| `UPTIME` | `UPTIME,ms=<int>` |
| `LOOP,ONCE` | Runs one offset assessment loop: `TURN,RIGHT,R=35,A=90`, `STRAIGHT,GOCM,40`, `TURN,LEFT,R=20,A=180`, `STRAIGHT,UNTILUS,47`; emits one `OK` and one final `DONE,LOOP` line |
| `FAST,LEFT1` / `FAST,RIGHT1` | Runs the corresponding 6-command fastest-car group from test_hardcode_command_executor.c |
| `FAST,LEFT2` / `FAST,RIGHT2` | Runs the corresponding 12-command fastest-car group from test_hardcode_command_executor.c |

FAST uses the LOOP sequence executor, waiting for each motion to succeed and
stopping on failure or UART STOP. It adds no startup delay or inter-command gap;
normal calibration, steering waits and turn-end actions follow
`GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`. Heading mode is not forced.
Replies are `OK,fast=<route>,count=<n>` followed by
`DONE,FAST,route=<route>,state=DONE,count=<n>`, or on failure
`DONE,FAST,route=<route>,state=<reason>,index=<step>,count=<n>`.
Internal command/SEQ progress replies are suppressed; SEQ,LOG remains available
after stopping. Busy motions/sequences reject FAST with ERR,BUSY; invalid routes
or extra arguments return ERR,BAD_ARGS. Each group ends with stop/centring.
FAST may also appear as a row in the hardcoded command array. Each group
completes before the next outer row starts; failure or STOP cancels remaining
outer rows. Nested groups append to the same sequence log. Hardcoded test
mode still auto-starts its array; it does not become a UART-triggered task.
| `MOTOR,L,<pwm>` | `OK`; left wheel, positive forward/negative reverse, magnitude within left motor PWM limit |
| `MOTOR,R,<pwm>` | `OK`; right wheel, corresponding right motor PWM limit |
| `MOTOR,B,<l>,<r>` | `OK`; set both wheels |
| `STEER,US,<us>` | `OK`; raw servo compare value, no software range clamp |
| `STEER,CENTER` | `OK`; shared center 1725 µs (robot_tuning.h) |
| `STEER,GET` | `STEER,us=<int>,center=<int>` |
| `ENC,RESET` | `OK`; reset encoders |
| `ENC,GET` | `ENC,l=<int>,r=<int>`; forward-normalized counts |
| `ENC,GET,RAW` | `ENC,l=<int>,r=<int>`; raw signed counts |
| `ENC,GET,CM` | `ENC,l_cm=<f>,r_cm=<f>` |
| `GYRO,INIT` | `OK` or `ERR,NO_DEVICE` |
| `GYRO,CALIB` | `OK,offset=<int>`; robot must remain stationary |
| `GYRO,Z` | `GYRO,z=<f>,raw=<int>` |
| `GYRO,OFFSET` | `GYRO,offset=<int>` |
| `GYRO,SET_OFFSET,<int>` | `OK` |
| `YAW,RESET` | `OK` |
| `YAW,GET` | `YAW,deg=<f>` |
| `STATUS` | `STATUS,motor_l=<int>,motor_r=<int>,steer=<int>,enc_l=<int>,enc_r=<int>,yaw=<f>,gyro_z=<f>,uptime=<int>` |
| `STATUS,MOTOR` | `MOTOR,l=<int>,r=<int>` |
| `STATUS,ENC` | `ENC,l=<int>,r=<int>` |

During motion or a `LOOP,ONCE` sequence, new motion commands, `MOTOR,*`, `STEER,US`, `STEER,CENTER`, `ENC,RESET`, `YAW,RESET`, and all `GYRO` operations except `GYRO,OFFSET` return `ERR,BUSY`. Status queries and stop commands remain available.

**`HB`, heartbeat stopping, `RESET`, `PARAM,GET/SET`, and `STEER,ANGLE` are not currently implemented.** Do not assume a disconnected link stops manual PWM. Closed-loop motion ends using its own completion conditions or limits; neither heartbeat nor periodic status polling is required to keep it running.

## 6. LOOP,ONCE Sequence Command

`LOOP,ONCE` is a normal UART command available in `ASSESSMENT_COMMAND_INTERFACE`.
It runs one offset assessment loop through the same sequencer used by
`TEST_HARDCODE_COMMAND_EXECUTOR`. The fixed command list is:

```text
TURN,RIGHT,R=35,A=90
STRAIGHT,GOCM,40
TURN,LEFT,R=20,A=180
STRAIGHT,UNTILUS,47
```

The immediate acceptance response is:

```text
OK,loop=ONCE,count=4
```

`OK` only means the loop command was accepted and the first queued movement will
start. It does not mean any movement has completed. `LOOP,ONCE` suppresses both
`SEQ,*` progress lines and the internal movement commands' individual `OK` /
`DONE` lines.

```text
DONE,LOOP,state=DONE,count=4
```

A successful run therefore looks like:

```text
> LOOP,ONCE
< OK,loop=ONCE,count=4
< DONE,LOOP,state=DONE,count=4
```

The final `DONE,LOOP` line means all four internal movements reached their own
successful terminal state.

If a row is rejected, the user sends `STOP`, or a movement finishes with a
non-success terminal state, the loop still emits exactly one loop-level `DONE`:

```text
DONE,LOOP,state=<reason>,index=<1..4>,count=4
```

For example, if the ultrasonic step times out: `DONE,LOOP,state=TIMEOUT,index=4,count=4`.
If the loop is cancelled by `STOP`: `DONE,LOOP,state=USER_STOP,index=<current>,count=4`.

While `LOOP,ONCE` is running, UART accepts `STOP`, `PING`, `STATUS`,
`STRAIGHT,STATUS`, `TURN,STATUS`, and `HEADING,STATUS`. `STRAIGHT,TRACE` is also accepted while stopped; it returns `ERR,BUSY` during motion. Other incoming commands return `ERR,BUSY`.

## 7. UNTILUS Interaction Example

Illustrative values, not hardware measurements. `>` denotes transmitted lines and `<` received lines; these markers are not part of the protocol.

```text
> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20
> STRAIGHT,STATUS
< STRAIGHT,state=RUNNING,mode=UNTILUS,elapsed=420,threshold_cm=20,us_mm=320,us_status=0,us_age_ms=12,l_cm=+8.300,r_cm=+8.280,diff_cm=+0.020,yaw=+0.100
< DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=20,us_mm=197,us_status=0

> STRAIGHT,UNTILUS,4
< ERR,BAD_VALUE

> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20
> STOP
< OK
< DONE,STRAIGHT,mode=UNTILUS,state=ABORTED,reason=ABORTED,threshold_cm=20,us_mm=INF,us_status=-1
```

The final example aborts before the first valid sample. RPi clients should parse mode and terminal state by field name and distinguish UNTILUS clearance from the GOCM travel target.

## 8. Configuration and Verification

- [MOTION_SETTINGS.md](MOTION_SETTINGS.md): global PWM scale, calibration/transition waits and active braking. These are compile-time settings; rebuild and flash after editing.
- [IR_STRAIGHT.md](IR_STRAIGHT.md): LEFTIR/RIGHTIR commands and `IR_COMMAND_*` tuning.
- v1.10 adds side IR commands and documents FAR/ADC-failure normal completion. Firmware build/link and host IR controller checks passed during implementation; floor behavior remains unverified.

- [straight_tuning.h](../Core/Inc/control/straight_tuning.h): SHORT/LONG and `US_*` parameters.
- [ULTRASONIC_STRAIGHT.md](ULTRASONIC_STRAIGHT.md): wiring, full behavior and manual verification steps.
- [FLOOR_RETUNING.md](FLOOR_RETUNING.md): controller calibration after floor changes.
- v1.9 documents UNTILUS, failure states, motion interlocks and the current no-echo/no-heartbeat implementation; it corrects older GOCM limits and control descriptions.
- Speed, steering transitions and stopping clearance still require hardware validation.

## Hardcoded command-array test

`TEST_HARDCODE_COMMAND_EXECUTOR` runs an editable command list once, waits for each
motion to succeed, and stops on errors. During this test UART accepts STOP, PING,
STATUS, STRAIGHT,STATUS, TURN,STATUS and HEADING,STATUS, plus STRAIGHT,TRACE while stopped. SEQ,LOG replays the saved run after completion/abort and returns ERR,BUSY until the sequence finishes. WAIT,<ms> is array-only. See
[HARDCODE_COMMAND_EXECUTOR.md](HARDCODE_COMMAND_EXECUTOR.md).

`SEQ,LOG` reads a 48 KiB volatile RAM log of the latest sequence. It includes
heading before command dispatch, each completed/aborted movement's status,
straight traces copied before the next straight, and turn target/braking metrics.
Extra diagnostics are captured between motions without transmitting them live.
The reply begins `LOG,SEQ,bytes=<n>,dropped_bytes=<n>,result=<state>` and ends
`LOG,SEQ,END`. Nonzero dropped_bytes means the tail exceeded capacity; the start
is retained. Reading does not clear the log. Reset, power loss, or a new sequence
discards it. There is no flash/SD persistence. Connect UART after the run while
keeping the board powered and without resetting it.

`us_mm=INF` means unknown/infinite clearance; `us_status` remains nonzero. Clients must accept this literal in addition to numeric millimeters.
