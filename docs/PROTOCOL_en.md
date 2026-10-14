# MDP21 STM32 ↔ RPi Communication Protocol (English)

**Version**: v1.15  
**Date**: 2026-10-10  
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
- The 512-byte RX buffer accepts at most 511 payload bytes, excluding the terminator. Each individual command remains limited to 127 bytes. Overflow emits `ERR,RX_LINE_OVERFLOW` and discards input until the next line ending.
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

### Multiple commands in one line

Separate commands with semicolons, for example `FAST,APPROACH;STRAIGHT,GOCM,30;LEFTIR,GET`. The batch uses the existing sequence executor, waiting for each managed motion to finish before dispatching the next command. FAST groups are supported; `WAIT,<ms>` inserts a 0..60000 ms delay. Queries return immediately. There is no extra startup or inter-command delay.

Acceptance is `OK,batch=<count>`. Individual command replies remain visible, followed by `DONE,BATCH,state=DONE,count=<count>` on success, or `DONE,BATCH,state=<reason>,index=<1-based failed step>,count=<count>` on failure. A rejected command or failed motion stops the remaining commands. Send global `STOP` on a separate line to cancel; permitted status queries remain available and other incoming commands return BUSY while the batch runs.

Maximum 16 commands and 511 total payload bytes; each command is at most 127 bytes and 7 comma-separated fields. Spaces/tabs around each command are trimmed, but spaces inside command fields remain unsupported. Leading/trailing semicolons and empty segments return BAD_ARGS before execution. Oversized segments, excess commands or excess fields also reject the whole batch before execution; other argument errors are reported when their step executes. A single command without a semicolon retains its normal behavior.

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
STRAIGHT,FORWARD,UNTILUS,<cm>
STRAIGHT,BACKWARD,UNTILUS,<cm>
OK,mode=UNTILUS,threshold_cm=<cm>,dir=<FORWARD|BACKWARD>
```

`cm` is a positive integer from 5 to 200 specifying sensor-to-object distance, in the same reference as `US,GET`. No installation offset is added in either direction. `OK`, completion and status all report the requested threshold. For input 35, compare ultrasound directly against 350 mm and expect `threshold_cm=35` throughout.

- `STRAIGHT,UNTILUS,n` defaults to FORWARD. `STRAIGHT,FORWARD,UNTILUS,n` explicitly moves forward toward smaller clearance; `STRAIGHT,BACKWARD,UNTILUS,n` moves backward toward greater clearance. Direction never changes according to the measured distance. Command and response direction names match.
- If the initial distance already satisfies the requested condition (forward: strictly below target; backward: strictly above target), two consistent fresh readings finish without movement. Equality does not select another direction. Otherwise two initial valid readings start the specified direction.
- Moving commands retain early braking: forward at target + lead, backward at target − lead, with two valid confirmations differing by at most 50 mm. Lead is 5 cm per direction, capped at half the initial target error when known. Unknown-start commands use the configured lead. Physical stopping clearance needs vehicle validation.
- Invalid, zero, stale or missing echoes do not complete or fault the command: begin/continue 1800 PWM creep in the specified direction and keep measuring. Valid recovered readings resume normal control and confirmation. Missing echoes can have multiple causes; recovery is not guaranteed. Driver initialization/trigger failures and gyro faults remain errors. STOP, stall, 30-second timeout and 200 cm travel limits remain active.
- PB15=TRIG, PC8=ECHO through a 5V-to-3.3V divider; TIM8_CH3 captures both edges at 1 MHz. Triggers are at least 60 ms apart; measurements time out after 60 ms. Valid samples must be no older than 200 ms.
- Base PWM: distant-start kick 4000, cruise 3500, approach 2400. Starts within 35 cm use startup PWM 2600 instead of 4000 and end kick mode as soon as both wheels move. Approach reduction is immediate; acceleration remains slew-limited. Both directions slow within 35 cm of the internal target, with 5 cm speed hysteresis and existing wheel synchronization. These are configured values before global PWM scaling.
- Within the last 20 cm, base PWM drops to 1800 after wheel-start confirmation. A missing echo at this speed does not cause acceleration. Every threshold crossing, including crossing the actual target, requires two consecutive valid echoes differing by at most 50 mm. An inconsistent echo restarts confirmation; an invalid echo clears it. Successful UNTILUS braking holds for at least 150 ms and until both encoders remain quiet for 60 ms, bounded to 600 ms. Continued wheel motion at the limit reports STALL.
- Total runtime is limited to 30 seconds; either wheel reaching 200 cm encoder travel stops motion. GOCM travel compensation is not used. Normal completion applies braking and steering centring; final physical clearance may differ from the triggering reading.

Completion/failure event, in field order:

```text
DONE,STRAIGHT,mode=UNTILUS,state=<S>,reason=<R>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,dir=<FORWARD|BACKWARD|NONE|UNKNOWN>,us_age_ms=<uint>
```

| `state` | `reason` | Meaning |
|---|---|---|
| `DONE` | `US_THRESHOLD` | Confirmed early-brake threshold, or initial condition already satisfied |
| `ABORTED` | `ABORTED` | User sent a stop command |
| `SENSOR_ERROR` | `SENSOR_ERROR` | Trigger-driver or gyro failure |
| `TIMEOUT` | `TIMEOUT` | Total runtime limit reached |
| `DISTANCE_LIMIT` | `DISTANCE_LIMIT` | Encoder travel limit reached |

On successful completion, `us_mm` and `us_status=0` report the saved condition-triggering sample before braking. They are preserved even if the DONE transmission is delayed beyond 200 ms. `dir` reports FORWARD or BACKWARD, or NONE when the initial condition was already satisfied; `us_age_ms` is the sample age at detection. Failure/abort use current sample validity/age and `dir=UNKNOWN`. These fields describe the triggering measurement, not a new measurement after stopping. Live `STRAIGHT,STATUS` still reports stale samples as INF/-1.

UNTILUS status replaces the GOCM status format:

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,mode=UNTILUS,elapsed=<ms>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,us_age_ms=<uint>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>
```

`RUNNING` includes both initial measurement wait and actual driving. `us_mm` is millimeters, `us_age_ms` is the age of the latest sample, and `elapsed` is command runtime in milliseconds. `us_status=0` means valid and fresh, `-1` means unavailable/no rise/stale, and `-2` means an invalid or excessively long echo. Do not use `us_mm` as a valid clearance when status is nonzero. The last sample is retained after termination, but its age keeps increasing, so later queries may report it as stale.

### 3.3 Side IR Threshold: LEFTIR / RIGHTIR

Edge commands: `STRAIGHT,FORWARD/BACKWARD,LEFTIR/RIGHTIR,RISING/FALLING,n`. RISING first observes a distance strictly below n, records that phase without stopping, then stops on a later reading strictly above n. FALLING first observes strictly above n, then stops on a later reading strictly below n. Equality does not arm or finish; it preserves an already recorded phase. An initial reading can record the first phase; starting on the final side alone cannot finish. Each new command clears its phase. NEAR counts as below and FAR as above; acquisition/conversion errors terminate with SENSOR_ERROR and cannot act as an edge. Successful edge stops cache the final crossing reading, including FAR as UINT32_MAX. Directions, range limits, STOP, timeout and travel limits match other IR commands. Omitting direction defaults to forward. Both phases require two consecutive qualifying samples. Equality or a sample on the other side resets the current count; the armed phase persists. GT/LT also require two consecutive qualifying samples. Each new command clears the counters. No extra distance hysteresis is added.

Explicit direction goes immediately after STRAIGHT: `STRAIGHT,FORWARD,LEFTIR/RIGHTIR,GT/LT,n` or `STRAIGHT,BACKWARD,LEFTIR/RIGHTIR,GT/LT,n`. Omitting direction defaults to FORWARD. Direction is independent of side and comparison; BACKWARD reports `dir=BACKWARD`. All IR fault rules and per-side caches apply in both gears.

Syntax: `STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`; n is integer centimetres. GT stops strictly above n, LT strictly below n; equality continues. Legacy three-field commands remain GT aliases. LEFTIR supports 10..50 cm and RIGHTIR 10..60 cm. Replies include `op=GT|LT`. LT also completes on NEAR, continues on FAR, and faults with SENSOR_ERROR on ADC/conversion error. The legacy infinity/failure-as-DONE rules below apply only to GT.

```text
STRAIGHT,LEFTIR,GT,50
STRAIGHT,RIGHTIR,GT,50
OK,mode=<LEFTIR|RIGHTIR>,op=<GT|LT|RISING|FALLING>,threshold_cm=<int>,dir=<FORWARD|BACKWARD>
DONE,STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT|RISING|FALLING>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>,dir=<FORWARD|BACKWARD>
```

- In the requested direction, GT stops when the selected side's distance is **strictly greater** than the target. Equality continues. No distance offset is added.
- Integer thresholds: LEFTIR 10..50 cm, RIGHTIR 10..60 cm, matching the current calibration tables.
- FAR, ADC acquisition failure and conversion error count as infinity and complete normally with `state=DONE,ir_cm=FAR`. They do not raise an IR sensor fault.
- Read before applying drive power; an already-satisfied condition completes without driving. While moving, sample approximately every 40 ms; two consecutive qualifying samples are required. If the initial GT/LT condition already holds, confirmation occurs 40 ms apart while stationary before drive is applied.
- All active IR comparison stages use base PWM 1900 from startup, with global scaling and normal straight steering/wheel synchronization. This covers both gears, both sensors, GT/LT/RISING/FALLING, and commands with or without GOCM/LATERAL. GOCM retains its normal profile before the IR gate and reduces PWM immediately at the gate.
- STOP gives ABORTED; 30 seconds gives TIMEOUT; either wheel reaching 200 cm gives DISTANCE_LIMIT. Gyro failure can still give SENSOR_ERROR. These failures stop the sequence.
- In a hardcoded sequence, normal completion (including FAR/ADC failure) advances to the next row.

Status uses this format, including after completion:

```text
STRAIGHT,STATUS
STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT|RISING|FALLING>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>,dir=<FORWARD|BACKWARD>
```

See [IR_STRAIGHT.md](IR_STRAIGHT.md) for examples and validation.

Successful condition stops retain the selected-side reading in `Straight_LeftIRStopCmX10` / `Straight_RightIRStopCmX10` (0.1 cm units; `UINT32_MAX` for uninitialized/FAR/GT read failure). Each side updates independently; STOP, timeout and other failures preserve the cache. LATERAL IR mode takes its own fresh measurement.

### 3.3.1 Continuous GOCM followed by IR

`STRAIGHT,GOCM,x,LEFTIR/RIGHTIR,GT/LT/RISING/FALLING,n` drives the encoder-measured distance x, then enables IR detection without stopping or restarting. Signed integer x follows GOCM: positive forward, negative backward, magnitude 10..200 cm. IR n follows the existing side ranges (LEFT 10..50, RIGHT 10..60 cm). IR is not sampled before both wheels reach |x|; edge phases and confirmation start only after that point. The boundary uses raw calibrated encoder distance without GOCM brake-distance offsets because it has no intermediate stop. The GOCM speed profile transitions to the existing IR cruise control while preserving heading and encoder totals.

Acceptance reports `OK,mode=GOCM_IR,gocm_cm=x,ir=LEFTIR/RIGHTIR,op=...,threshold_cm=n,dir=FORWARD/BACKWARD`. Status and the single final IR completion event append `gocm_cm=x,phase=GOCM/IR`. Both IR edge phases and GT/LT require two consecutive confirmations. Per-side stop caches and sensor fault rules remain. STOP cancels either phase; the whole command has a 30-second timeout and a 200 cm total travel limit, including the initial distance. A gate at the travel limit cannot leave room for subsequent IR motion.

### 3.3.2 Edge-triggered lateral maneuver

`STRAIGHT,GOCM,x,LEFTIR/RIGHTIR,RISING/FALLING,n,LATERAL` extends the continuous distance-gated IR command. After the encoder gate, RISING starts caching at the first valid reading below the threshold and maintains the minimum across every subsequent valid IR reading until the confirmed exit stops motion. Resetting a confirmation counter does not clear this minimum. FALLING retains its first confirmed phase snapshot and discards isolated unconfirmed candidates. Both phases require two consecutive confirmations. The straight motion brakes at the completed edge, then performs LATERAL toward the selected IR side using saved IR +10 cm (5 cm IR-to-centre +5 cm obstacle half-width). It does not run the separate LATERAL,IR approach or remeasure IR.

The saved reading must be finite and calibrated; NEAR/FAR cannot supply the displacement snapshot. Saved distance +10 must be at most 55 cm, so saved distance must be at most 45 cm. If unavailable or out of range, stop after the edge and emit LATERAL START_ERROR without turning (`start_rc=-5` or `-2`). Earlier straight faults also prevent turning. STOP cancels every phase. The whole maneuver remains one pending command in LOOP and command arrays, ending with a single `DONE,LATERAL,...` (`source=EDGE_IR`, `ir_cm_x10` is the RISING minimum or FALLING first-phase snapshot, `d_cm` includes compensation). Stage 0 is gated straight/IR, stages 1/2 are lateral arcs. Acceptance uses `mode=GOCM_IR_LATERAL`.

### 3.4 Stopping and Timing

- `STRAIGHT,STOP` → `OK,aborted=<0|1>`; aborting running straight motion, including the initial measurement wait, is followed by one `state=ABORTED` event.
- `STOP` → `OK`; zeros motor PWM, centers steering and aborts running straight/turn motion, followed by its completion event.
- The first motion may initialize and calibrate the gyro while stationary for approximately 1.7 seconds plus initialization/read overhead before replying `OK`. Repeat bias calibration uses `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`; other waits/actions use independent switches; see [MOTION_SETTINGS.md](MOTION_SETTINGS.md). First-use initialization remains blocking.
- Motion control steps at intervals of at least 20 ms; UART reception/transmission can still delay the main loop. OLED command refresh is skipped during motion, and independent gyro polling pauses to avoid double integration. There is no hard real-time stopping-latency guarantee.

## 4. Turn Commands

Command-turn steering wait is always enabled, independently of `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`. After setting the target steering pulse, motor drive stays zero until the estimated positioning time expires. The estimate is `60 + ceil(340 × abs(target_us − previous_us) / 1600)` ms, bounded to 60..400 ms; previous commands outside configured endpoints use 400 ms. These values are configurable in `turn_tuning.h`. This uses commanded PWM travel, with no physical position feedback. It needs validation with the loaded servo; small changes wait less, large changes wait longer. STOP remains available during the non-blocking wait.

| Request | Arguments / response |
|---|---|
| `TURN,LEFT,R=<cm>,A=<deg>` | Integer R ≥18; signed integer A with magnitude 5..360 |
| `TURN,RIGHT,R=<cm>,A=<deg>` | Integer R ≥35; same A limits |
| Acceptance | `OK,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| Terminal event | `DONE,TURN,state=<S>,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| `TURN,STOP` | `OK,aborted=<0\|1>`; followed by one completion event if aborted |
| `TURN,STATUS` | `TURN,state=<S>,dir=<LEFT\|RIGHT>,elapsed=<ms>,r_cm=<int>,target_deg=<int>,yaw=<f>` |

LEFT/RIGHT selects steering side; positive A drives forward, negative A backward. R has no additional configured upper bound but must parse as int32. Terminal text includes `DONE`, `ABORTED`, `TIMEOUT`, `WRONG_SIGN`, `NO_TURN`, `BAD_CONFIG`, and `NO_IMU` (the last two are recognized interface states). Current turn parameters are defined in [turn_tuning.h](../Core/Inc/control/turn_tuning.h) and [turn_control.c](../Core/Src/control/turn_control.c).

Independent switches in `robot_tuning.h` control turn-end actions, currently enabled: `TURN_POST_STOP_SETTLE_ENABLE` adds 150 ms after braking; `TURN_RIGHT_RETURN_ENABLE` performs the stopped 1500 us / 100 ms then centre / 100 ms holds; `TURN_LEFT_HEADING_OFF_ENABLE` applies HEADING,OFF on successful LEFT; `TURN_END_HEADING_RESET_ENABLE` resets successful-turn heading history independently of steering holds. These work in either gear. `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` controls only repeated bias refresh, with mandatory stationary calibration preparation. Target braking and estimated turn-start servo waiting always remain. See [MOTION_SETTINGS.md](MOTION_SETTINGS.md) for preparation/stillness and sequence-gap switches.

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

### 4.2 Obstacle Bypass: BYPASS

- `BYPASS,LEFT,n` / `BYPASS,RIGHT,n`: forward-only three-arc maneuver, with integer `n` from 1 to 55 cm. `n` is the maximum lateral excursion; the ideal endpoint returns to the original line and heading. No automatic 10 cm compensation is added. IR mode is currently unsupported.
- RIGHT executes right → left → right; LEFT executes left → right → left. Each left arc uses the existing 20 cm soft minimum radius (18 cm hard limit remains unchanged); each right arc uses the existing 35 cm minimum.
- Outer angle: `a = round(acos(1 - n / (20 + 35)) × 180 / π)`. Middle angle is exactly `2a`, so integer rounding preserves the net heading. Ideal forward displacement is `2 × (20 + 35) × sin(a)`. For `n=20`, commanded angles are 50°, 100°, 50°.
- `BYPASS,STATUS` reports `state,dir,d_cm,a,middle_a,stage,turn_state,start_rc`; stages 1–3 identify the active arc. Accepted motion returns `OK,BYPASS` and a status line, then one `DONE,BYPASS,...` after completion or interruption. Internal arcs do not emit individual command completion messages.
- `BYPASS,STOP` or global `STOP` cancels the remaining arcs. Turn failures abort the maneuver. Concurrent motion returns `ERR,BUSY`; out-of-range distances return `ERR,BAD_VALUE`. Supports `LOOP,ONCE` and hardcoded command arrays.
- This geometry requires suitable starting distance and obstacle length clearance. Maximum lateral excursion alone does not guarantee clearance for the full vehicle body; validate the path on the vehicle.

## 5. System, Manual Control and Queries

Every reply below ends with `\r\n`. `<f>` denotes three-decimal numeric formatting; `<int>` denotes an integer.

| Request | Response / notes |
|---|---|
| `HELLO` | `HELLO,mdp21_v8` |
| `PING` | `PONG` |
| `UPTIME` | `UPTIME,ms=<int>` |
| `LOOP,ONCE` | Runs one offset assessment loop: `TURN,RIGHT,R=35,A=90`, `STRAIGHT,GOCM,40`, `TURN,LEFT,R=20,A=180`, `STRAIGHT,UNTILUS,47`; emits one `OK` and one final `DONE,LOOP` line |
| `FAST,APPROACH` | Executes only `STRAIGHT,UNTILUS,28` (default FORWARD); returns `OK,fast=APPROACH,count=1` and `DONE,FAST,route=APPROACH,state=DONE,count=1` on success |
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
| `SERVO,SETCENTER,<us>` | Integer microseconds strictly between the configured endpoints (currently 1000 < us < 2600). Updates the shared runtime centre and positions the servo using backlash take-up; replies `OK,SERVO,SETCENTER,<us>`. Motion returns BUSY. `SERVO,CENTER,<us>` remains an alias with its original reply. Read back with `STEER,GET`; resets to the compile-time default on restart. Straight direction trims still apply. |
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
< OK,mode=UNTILUS,threshold_cm=20,dir=FORWARD
> STRAIGHT,STATUS
< STRAIGHT,state=RUNNING,mode=UNTILUS,elapsed=420,threshold_cm=20,us_mm=320,us_status=0,us_age_ms=12,l_cm=+8.300,r_cm=+8.280,diff_cm=+0.020,yaw=+0.100
< DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=20,us_mm=197,us_status=0,dir=FORWARD,us_age_ms=0

> STRAIGHT,UNTILUS,4
< ERR,BAD_VALUE

> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20,dir=FORWARD
> STOP
< OK
< DONE,STRAIGHT,mode=UNTILUS,state=ABORTED,reason=ABORTED,threshold_cm=20,us_mm=INF,us_status=-1,dir=UNKNOWN,us_age_ms=0
```

The final example aborts before the first valid sample. RPi clients should parse mode and terminal state by field name and distinguish UNTILUS clearance from the GOCM travel target.

## 8. Configuration and Verification

- v1.15 synchronizes bidirectional UNTILUS, per-side IR stop caches and both LATERAL modes, including the IR approach and 10 cm offset.

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

`LEFTIR,GET` / `RIGHTIR,GET` acquire one fresh measurement of the selected IR side while stationary. Reply: `LEFTIR/RIGHTIR,cm=<decimal|NEAR|FAR|INVALID>,raw=<ADC|NA>,mv=<millivolts|NA>,status=<VALID|NEAR|FAR|ERROR>,rc=<driver return code>`. Out-of-range samples report NEAR/FAR rather than a fabricated numerical distance; ADC failures return ERROR and rc!=0. Motion returns ERR,BUSY. Reading does not change IR edge phase, confirmation counters or cached stopping readings.

IR status and completion append `ir_raw,ir_status,ir_age_ms,confirm,armed` for the selected side. These are the last control sample, not a fresh acquisition. An unavailable ADC raw is UINT32_MAX; gated commands have no IR sample before activation.

### Bounded edge search: SEARCHLATERAL

`SEARCHLATERAL,LEFT/RIGHT,x,n` drives forward x cm (10..200), then continuously detects the selected IR RISING n (LEFT 10..50, RIGHT 10..60). Successful edge completion performs lateral using the minimum finite IR +10 cm. Invalid/oversized lateral displacement and acquisition faults terminate the command rather than selecting fallback.

At acceptance, freeze the mm limit `Bypass_ForwardDistanceMm + Straight_AutoUSStartMm - Straight_ForwardUSEndMm + 300` using signed arithmetic. Missing caches, nonpositive results or limits not representable as uint32_t mm select the default total search distance of 100 cm (1000 mm), without rejecting startup. A valid cached combination takes precedence. Total search travel includes the initial x cm. Every poll checks either encoder before edge completion; reaching the bound brakes and selects fallback, including before the IR gate. Distance wins if both conditions occur in one poll. The trigger bound does not guarantee zero braking overrun.

RIGHT fallback: `STRAIGHT,RIGHTIR,GT,50` → `TURN,RIGHT,R=35,A=90` → `STRAIGHT,GOCM,-20` → `STRAIGHT,RIGHTIR,LT,50` → `TURN,LEFT,R=20,A=90`. LEFT mirrors sensor/steering sides, retaining left radius 20 and right radius 35. Fallback GT uses strict acquisition-error handling; both fallback IR thresholds are fixed at 50.

STATUS and STOP are supported. One terminal DONE,SEARCHLATERAL reports state, stage, path=LATERAL/FALLBACK, limit_mm, ir_cm_x10 and start_rc. Stage 0 searches, 1 performs normal lateral, 2..6 execute fallback. Any child failure or STOP cancels later stages. FAST/batches wait for the whole composite. LEFT2 uses SEARCHLATERAL,RIGHT,50,50 and RIGHT2 uses LEFT, then executes its original STRAIGHT,UNTILUS,15 after either successful path. Use global STOP during sequences; SEARCHLATERAL,STATUS remains available.

Fallback routes are actual executable string arrays in Core/Src/control/search_lateral_control.c: search_right_fallback_commands for FAST LEFT2, search_left_fallback_commands for FAST RIGHT2. Order and parameters drive execution; child completion and STOP follow the parsed motion type. Supported rows are STRAIGHT,GOCM, STRAIGHT,LEFTIR/RIGHTIR,GT/LT/RISING/FALLING and TURN,LEFT/RIGHT,R=...,A=....

### AUTO ultrasonic approach

STRAIGHT,AUTO,UNTILUS,n waits for two valid starting echoes on the same target side, differing by at most 50 mm. Above target selects forward, below selects backward, equality completes stationary. Gear remains fixed after selection. Existing slowdown, braking leads, brake hold and bounds apply. An invalid/missing starting echo selects backward creep and caches AUTO start clearance as zero. Recovered echoes retain reverse direction and resume stop confirmation; later missing echoes permit creep in the selected gear. The implicit UNTILUS command still defaults forward.

Straight_AutoUSStartMm / Straight_AutoUSEndMm are independent mm caches, reset on AUTO acceptance. They hold the first valid initial echo (zero for invalid starting echo) and successful final condition sample; unavailable/failed values use UINT32_MAX. AUTO does not update fixed-gear caches. OK reports dir=AUTO; successful DONE reports the actual gear or NONE. SEARCHLATERAL now uses the AUTO start cache, retaining the source +300 mm allowance and 100 cm default. FAST LEFT1/RIGHT1 use AUTO with their existing 32/34 cm targets.

Drive-scale exception: active IR comparisons and UNTILUS approach/final/near-start/no-echo creep use configured PWM without the straight multiplier. Distant UNTILUS startup/cruise and GOCM remain scaled; turn scaling is unchanged. Wheel synchronization and PWM saturation remain active.

### Current UNTILUS policy: initial measurement then fixed travel

This supersedes continuous ultrasonic control on the normal path. Cache one fresh initial sample, subtract the requested distance and drive the difference using GOCM preparation, steering PID, wheel synchronization, profiles, drive scaling, distance compensation and 150 ms braking. AUTO chooses gear; explicit/default FORWARD and BACKWARD remain fixed. Already-reached/equal conditions complete stationary. No ultrasonic reads occur during normal fixed travel. Millimetre differences are retained; sub-minimum corrections are supported, falling back to the raw difference if normal short-distance compensation would be nonpositive. UNTILUS has no accumulated travel cap; the 30-second timeout remains.

Invalid initial AUTO/BACKWARD echoes cache zero and execute target-distance reverse travel. Invalid initial FORWARD/default echoes retain the entire legacy continuous-measurement/forward-creep strategy. On success, end caches directly store target mm without a final measurement; faults/STOP leave end unavailable. Cache ownership is unchanged. DONE includes us_source=TARGET/MEASURED: fixed-travel completion reports calculated target us_mm, while legacy fallback reports its measured stopping sample; both successful end caches store the target.

SEARCHLATERAL removes the 200 cm computed-limit cap. Bounded searches use limit_mm rather than the ordinary IR 200 cm travel limit. Missing/invalid caches retain the 100 cm default; 30-second timeout, STOP and fault handling remain. Ordinary IR commands retain their travel limit.

### Motion cache and search logs

All diagnostic lines start with LOG and remain visible during FAST reply suppression; sequence RAM logging also records them. CACHE,BYPASS reports planned forward mm. CACHE,UNTILUS reports START/END, FORWARD/BACKWARD/AUTO and mm; END uses source=TARGET. Initialization/reset sentinels are not logged.

SEARCHLATERAL,LIMIT prints bypass_mm, auto_start_mm, forward_end_mm, offset_mm, sum_mm, selected limit_mm and source=CACHE/DEFAULT. Missing values display INVALID. SEARCHLATERAL,FALLBACK prints limit_mm and both accumulated wheel distances after braking, before the first fallback step. Controllers enqueue logs; UART draining occurs in command polling, including FAST. Queue overflow reports LOG,DROPPED. All distances use mm.

UNTILUS removes both computed-distance and accumulated-travel caps, including legacy fallback. Ordinary GOCM argument limits, UNTILUS target limits and the 30-second timeout remain.

US,GET caches its valid reading in Ultrasonic_LastGetMm (mm, declared in ultrasonic_sensor.h). Unmeasured/invalid/init-failed results use UINT32_MAX; BUSY preserves the previous cache. This is independent of UNTILUS caches and other ultrasonic reads. Updates emit LOG,CACHE,US_GET. LATERAL,RIGHT,20;US,GET;LATERAL,LEFT,20 provides a stopped measurement between two numeric lateral moves, without IR compensation. BYPASS retains its three-turn behavior.

LEFTIR,GET / RIGHTIR,GET independently cache valid distance in IRSensor_LeftLastGetCmX10 / IRSensor_RightLastGetCmX10 (0.1 cm, equivalent to mm). Unmeasured/NEAR/FAR/error values use UINT32_MAX. Only the requested side updates; BUSY preserves both. Other IR reads and motion caches remain independent. Updates emit LOG,CACHE,IR_GET.
