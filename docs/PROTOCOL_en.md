# MDP21 STM32 ↔ RPi Communication Protocol (English)

**Version**: v1.11  
**Date**: 2026-10-09  
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
| `IR_MEASUREMENT_INVALID` | LATERAL initial selected-side IR is unavailable or outside calibration |

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
- SHORT: PWM 3200 for the first 100 ms, then a progress-based 2950→2980→2350 curve; normal GOCM adds a 2 cm later-braking allowance, giving net target compensation −2 cm forward and −1 cm reverse. CAL retains its previous compensation.
- LONG: PWM 3300, reduced to 2400 once either wheel has accumulated 0.8 revolutions; base target revolutions are reduced by 0.310 forward or 0.260 reverse; normal GOCM then adds 2/20.4 revolutions (2 cm), giving net deductions about 0.212 forward and 0.162 reverse. These PWM values are before global scaling. Slowdown depends on distance already traveled, not remaining distance.
- Both profiles share yaw/rate-to-servo PID and wheel encoder-delta synchronization; both wheels must reach their compensated targets, then complete a 150 ms brake pulse. Yaw/encoder samples include braking.
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

`cm` is a positive integer from 5 to 200 specifying sensor-to-object distance, in the same reference as `US,GET`. No installation offset is added in either direction. `OK`, completion and status all report the requested threshold. For input 35, compare ultrasound directly against 350 mm and expect `threshold_cm=35` throughout.

- Wait stationary for two consecutive fresh valid readings on the same side of the internal target. Above it selects forward; below it selects reverse. Two initial readings within ±1 cm of the target complete without driving. A side change resets startup confirmation.
- After starting, lock direction and use an early-brake threshold. Forward triggers at target + lead, reverse at target − lead, after two consecutive qualifying valid readings. Configured lead is 5 cm independently for each direction, capped at half the initial target error for small corrections. A nonqualifying or invalid reading resets confirmation.
- Three consecutive invalid/zero/stale startup samples finish with SENSOR_ERROR without driving. A trigger-driver failure also faults. During motion, missing echoes do not confirm a crossing; sampling continues subject to STOP, timeout and travel limits.
- PB15=TRIG, PC8=ECHO through a 5V-to-3.3V divider; TIM8_CH3 captures both edges at 1 MHz. Triggers are at least 60 ms apart; measurements time out after 60 ms. Valid samples must be no older than 200 ms.
- Base PWM: distant-start kick 4000, cruise 3500, approach 2600. Starts within 25 cm use approach PWM 2600 instead of 4000 and end kick mode as soon as both wheels move. Approach reduction is immediate; acceleration remains slew-limited. Both directions slow within 25 cm of the internal target, with 5 cm speed hysteresis and existing wheel synchronization. These are configured values before global PWM scaling.
- Total runtime is limited to 30 seconds; either wheel reaching 200 cm encoder travel stops motion. GOCM travel compensation is not used. Normal completion applies braking and steering centring; final physical clearance may differ from the triggering reading.

Completion/failure event, in field order:

```text
DONE,STRAIGHT,mode=UNTILUS,state=<S>,reason=<R>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,dir=<FORWARD|REVERSE|NONE|UNKNOWN>,us_age_ms=<uint>
```

| `state` | `reason` | Meaning |
|---|---|---|
| `DONE` | `US_THRESHOLD` | Confirmed early-brake threshold, or initial arrival tolerance |
| `ABORTED` | `ABORTED` | User sent a stop command |
| `SENSOR_ERROR` | `SENSOR_ERROR` | Trigger-driver failure or persistent invalid initial measurements |
| `TIMEOUT` | `TIMEOUT` | Total runtime limit reached |
| `DISTANCE_LIMIT` | `DISTANCE_LIMIT` | Encoder travel limit reached |

On successful completion, `us_mm` and `us_status=0` report the saved condition-triggering sample before braking. They are preserved even if the DONE transmission is delayed beyond 200 ms. `dir` reports FORWARD or REVERSE, or NONE for stationary arrival tolerance; `us_age_ms` is the sample age at detection. Failure/abort use current sample validity/age and `dir=UNKNOWN`. These fields describe the triggering measurement, not a new measurement after stopping. Live `STRAIGHT,STATUS` still reports stale samples as INF/-1.

UNTILUS status replaces the GOCM status format:

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,mode=UNTILUS,elapsed=<ms>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,us_age_ms=<uint>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>
```

`RUNNING` includes both initial measurement wait and actual driving. `us_mm` is millimeters, `us_age_ms` is the age of the latest sample, and `elapsed` is command runtime in milliseconds. `us_status=0` means valid and fresh, `-1` means unavailable/no rise/stale, and `-2` means an invalid or excessively long echo. Do not use `us_mm` as a valid clearance when status is nonzero. The last sample is retained after termination, but its age keeps increasing, so later queries may report it as stale.

### 3.3 Side IR Threshold: LEFTIR / RIGHTIR

Syntax: `STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`; n is integer centimetres. GT stops strictly above n, LT strictly below n; equality continues. Legacy three-field commands remain GT aliases. LEFTIR supports 10..50 cm and RIGHTIR 10..60 cm. Replies include `op=GT|LT`. LT also completes on NEAR, continues on FAR, and faults with SENSOR_ERROR on ADC/conversion error. The legacy infinity/failure-as-DONE rules below apply only to GT.

```text
STRAIGHT,LEFTIR,GT,50
STRAIGHT,RIGHTIR,GT,50
OK,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,threshold_cm=<int>
DONE,STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>
```

- Forward only; stop when the selected side's distance is **strictly greater** than the target. Equality continues. No distance offset is added.
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

Successful condition stops retain the selected-side reading in `Straight_LeftIRStopCmX10` / `Straight_RightIRStopCmX10` (0.1 cm units; `UINT32_MAX` for uninitialized/FAR/GT read failure). Each side updates independently; STOP, timeout and other failures preserve the cache. LATERAL IR mode takes its own fresh measurement.

### 3.4 Stopping and Timing

- `STRAIGHT,STOP` → `OK,aborted=<0|1>`; aborting running straight motion, including the initial measurement wait, is followed by one `state=ABORTED` event.
- `STOP` → `OK`; zeros motor PWM, centers steering and aborts running straight/turn motion, followed by its completion event.
- The first motion may initialize and calibrate the gyro while stationary for approximately 1.7 seconds plus initialization/read overhead before replying `OK`. Repeat bias calibration uses `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`; other waits/actions use independent switches; see [MOTION_SETTINGS.md](MOTION_SETTINGS.md). First-use initialization remains blocking.
- Motion control steps at intervals of at least 20 ms; UART reception/transmission can still delay the main loop. OLED command refresh is skipped during motion, and independent gyro polling pauses to avoid double integration. There is no hard real-time stopping-latency guarantee.

## 4. Turn Commands

Command-turn steering wait is always enabled, independently of `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`. After setting the target steering pulse, motor drive stays zero until the estimated positioning time expires. The estimate is `80 + ceil(420 × abs(target_us − previous_us) / 1600)` ms, bounded to 80..500 ms; previous commands outside configured endpoints use 500 ms. These values are configurable in `turn_tuning.h`. This uses commanded PWM travel, with no physical position feedback. It needs validation with the loaded servo; small changes wait less, large changes wait longer. STOP remains available during the non-blocking wait.

| Request | Arguments / response |
|---|---|
| `TURN,LEFT,R=<cm>,A=<deg>` | Integer R ≥18; signed integer A with magnitude 5..360 |
| `TURN,RIGHT,R=<cm>,A=<deg>` | Integer R ≥35; same A limits |
| Acceptance | `OK,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| Terminal event | `DONE,TURN,state=<S>,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| `TURN,STOP` | `OK,aborted=<0\|1>`; followed by one completion event if aborted |
| `TURN,STATUS` | `TURN,state=<S>,dir=<LEFT\|RIGHT>,elapsed=<ms>,r_cm=<int>,target_deg=<int>,yaw=<f>` |

LEFT/RIGHT selects steering side; positive A drives forward, negative A backward. R has no additional configured upper bound but must parse as int32. Terminal text includes `DONE`, `ABORTED`, `TIMEOUT`, `WRONG_SIGN`, `NO_TURN`, `BAD_CONFIG`, and `NO_IMU` (the last two are recognized interface states). Current turn parameters are defined in [turn_tuning.h](../Core/Inc/control/turn_tuning.h) and [turn_control.c](../Core/Src/control/turn_control.c).

Independent switches in `robot_tuning.h` control turn-end actions, all defaulting to 0: `TURN_POST_STOP_SETTLE_ENABLE` adds 500 ms after braking; `TURN_RIGHT_RETURN_ENABLE` performs the stopped 1500 us / 500 ms then centre / 500 ms holds; `TURN_LEFT_HEADING_OFF_ENABLE` applies HEADING,OFF on successful LEFT; `TURN_END_HEADING_RESET_ENABLE` resets successful-turn heading history independently of steering holds. These work in either gear. `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` controls only repeated bias refresh, with mandatory stationary calibration preparation. Target braking and estimated turn-start servo waiting always remain. See [MOTION_SETTINGS.md](MOTION_SETTINGS.md) for preparation/stillness and sequence-gap switches.

RIGHT remains RUNNING during both non-blocking holds; new motion/manual steering
is rejected as busy, and STOP cancels the sequence. Completion actions do not
run after a turn failure/abort. TURN,STATUS preserves the completed turn yaw;
HEADING,STATUS reports the reference according to the enabled heading policies. Command arrays wait for these actions
before advancing, and UART clients must wait for DONE. See the
[automatic turn procedure](TURN_REVERSE_DIAGNOSIS.md#automatic-actions-after-each-turn).

### 4.1 Lateral Displacement: LATERAL

| Request | Behaviour |
|---|---|
| `LATERAL,LEFT,IR` / `LATERAL,RIGHT,IR` | Save a fresh selected-side IR reading, drive forward until selected IR >50 cm, then execute two opposite turns using saved IR +10 cm |
| `LATERAL,LEFT,n` / `LATERAL,RIGHT,n` | Direct lateral displacement of integer n cm, 1..55; no IR approach or offset |
| `LATERAL,STATUS` | Query the composite motion, including the saved initial IR and current stage |
| `LATERAL,STOP` | `OK`; cancel the entire active lateral motion |

IR compensation is 5 cm from the IR sensor to the vehicle centreline plus half the 10 cm obstacle width (5 cm), totalling 10 cm. No vehicle-length compensation or extra straight distance is added. Validate the initial reading and final target before moving: initial IR must have VALID status and a calibrated distance 10..50 cm, and saved IR +10 must be at most 55 cm (thus usable initial IR is 10..45 cm). Initial ADC/FAR/NEAR/invalid results return `ERR,IR_MEASUREMENT_INVALID`; a final target beyond 55 cm returns `ERR,BAD_VALUE`. Invalid numeric input also returns `ERR,BAD_VALUE`; malformed fields/count return `ERR,BAD_ARGS`.

IR approach uses the normal straight control and braking, with a strict acquisition-error policy: equality at 50 cm continues; a distance >50 cm or FAR completes the approach, while ADC/selected-side conversion error aborts it. STOP, 30-second timeout, 200 cm travel limit and gyro faults prevent the subsequent turns. The saved initial IR never changes when later measurements or straight-stop caches change.

LEFT executes a left arc then a right arc; RIGHT executes right then left. Left radius uses the 20 cm soft limit, right radius 35 cm; TURN's left hard minimum remains 18 cm. For lateral target d, both nominal angles are `acos(1 − d/55) × 180/π`, rounded to the nearest integer degree. Each is at most 90°. Ideal forward advance during the arcs is `sqrt(110d − d²)`; IR mode also includes its preceding variable-length straight approach. Numeric `LATERAL,LEFT,20` commands two 50° turns, giving ideal rounded-angle forward advance about 42.13 cm and left displacement about 19.65 cm. Existing TURN calibration, braking and post-turn actions still apply; physical results require floor verification.

Acceptance emits `OK,LATERAL`, followed by the current LATERAL status. Completion emits exactly one composite event, without internal `DONE,STRAIGHT` / `DONE,TURN` events:

`LATERAL,state=<S>,dir=<LEFT|RIGHT>,source=<IR|CM>,d_cm=<one-decimal>,a=<int>,stage=<0|1|2>,ir_cm_x10=<uint>,straight_state=<S>,turn_state=<S>,start_rc=<int>`

`DONE,LATERAL,state=<S>,dir=<LEFT|RIGHT>,source=<IR|CM>,d_cm=<one-decimal>,a=<int>,stage=<0|1|2>,ir_cm_x10=<uint>,straight_state=<S>,turn_state=<S>,start_rc=<int>`

Stages: 0=IR approach, 1=first turn, 2=second turn. Numeric mode starts at stage 1 and reports `ir_cm_x10=0`. `ir_cm_x10` uses 0.1 cm units (300 means 30 cm); `d_cm` includes the IR offset only for source IR. States: IDLE, RUNNING, DONE, ABORTED, STRAIGHT_ERROR, TURN_ERROR, START_ERROR. Errors retain the failed stage and underlying controller state or start return code. `a` is the positive integer commanded arc angle, not measured yaw.

`STOP`, `STRAIGHT,STOP`, `TURN,STOP` and `LATERAL,STOP` cancel the entire action, including between stages; no later arc starts after cancellation. Other motions are BUSY throughout all stages. Hardcoded/LOOP/FAST sequences wait for the complete action and stop on failure; their reply suppression still applies. During sequences, use global `STOP` to cancel; `LATERAL,STATUS` is allowed, while incoming subcommand STOP forms are rejected as BUSY by the sequence gate.

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
the independent switches in robot_tuning.h. Heading mode is not forced.
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
< DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=20,us_mm=197,us_status=0,dir=FORWARD,us_age_ms=0

> STRAIGHT,UNTILUS,4
< ERR,BAD_VALUE

> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20
> STOP
< OK
< DONE,STRAIGHT,mode=UNTILUS,state=ABORTED,reason=ABORTED,threshold_cm=20,us_mm=INF,us_status=-1,dir=UNKNOWN,us_age_ms=0
```

The final example aborts before the first valid sample. RPi clients should parse mode and terminal state by field name and distinguish UNTILUS clearance from the GOCM travel target.

## 8. Configuration and Verification

- v1.11 synchronizes bidirectional UNTILUS, per-side IR stop caches and both LATERAL modes, including the IR approach and 10 cm offset.

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
STATUS, STRAIGHT,STATUS, TURN,STATUS, LATERAL,STATUS and HEADING,STATUS, plus STRAIGHT,TRACE while stopped. SEQ,LOG replays the saved run after completion/abort and returns ERR,BUSY until the sequence finishes. WAIT,<ms> is array-only. See
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
