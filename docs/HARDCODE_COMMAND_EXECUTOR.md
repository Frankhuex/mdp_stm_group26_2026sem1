# Hardcoded command executor

## Current mock task 1 and heading mode

The editable array contains the mock task 1 route. Its active strings select
which legs run; commented blocks and the older course inside #if 0 are excluded.
The logging/default-heading update preserves the selected movement values and
order. Edit the array to shorten a run when diagnosing a particular leg.

Normal command-interface and hardcoded runs now start with heading carry OFF.
Each GOCM still uses local gyro steering, and turns still use gyro yaw to stop.
Previous movement errors do not change the next turn's target. Explicit
HEADING,ON remains available for a separate test; the dedicated A/B diagnostic
still uses its configured heading mode.

## Collect data after running without USART connected

1. Flash, then let the sequence run without the USART cable attached.
2. Once it finishes, keep the board powered and connect USART without resetting.
3. Send `SEQ,LOG` and save everything through `LOG,SEQ,END`.

The latest sequence is saved in a 48 KiB RAM buffer. This includes existing UART
messages plus heading before each command, final movement status, every straight's
buffered trace, and turn metrics. Extra diagnostics are copied between movements
without live UART transmission or added waits. A new sequence replaces the log;
reset or power loss erases it. Reading the log repeatedly does not clear it.
The dump is rejected with ERR,BUSY while a sequence or movement is running.

`LOG,BEFORE,index=...` precedes the heading at command dispatch.
`LOG,AFTER,index=...` precedes final status and saved straight trace, if applicable.
`LOG,TURN_METRICS` reports the positive calibrated local target magnitude,
signed yaw at braking, positive braking yaw rate, and final commanded servo pulse.
The TURN status yaw is the final measured turn angle; the HEADING status after a
turn can be zero because of the existing automatic reset. Servo pulses report
commands, not measured wheel positions. On failed turns, braking metrics may be
zero if braking was never reached; interpret them with the terminal state.

The header reports byte count, dropped_bytes and result. If dropped_bytes is
nonzero, the log's end exceeded capacity; shorten the test to capture the missing
section. Straight trace headers also report their own sample overflow separately.

## Earlier 2 October 2026 turn/reverse return test (currently commented out)

ACTIVE_TEST is TEST_HARDCODE_COMMAND_EXECUTOR and TURN_REVERSE_DIAGNOSTIC_CASE
is 0, selecting the array directly in Core/Src/tests/test_hardcode_command_executor.c.
The following describes the earlier route, retained for reference. That route first
drives forward 45 cm, then runs the turn/reverse loop below. Table coordinates
use the position after that initial straight as the loop origin, so the ideal
finish is 45 cm ahead of the original starting position.

| Turn/reverse pair | Forward turn | Next move | Ideal rear-axle midpoint (cm) | Facing |
| --- | --- | --- | --- | --- |
| 1 | RIGHT R35 / 90 | Reverse 35 cm | (0,35) | Right |
| 2 | LEFT R20 / 90 | Reverse 35 cm | (20,20) | Forward |
| 3 | LEFT R20 / 90 | Reverse 35 cm | (35,40) | Left |
| 4 | LEFT R20 / 90 | Reverse 35 cm | (15,55) | Backward |
| 5 | LEFT R20 / 90 | Reverse 35 cm | (0,35) | Right |
| 6 | RIGHT R35 / 90 | Reverse 35 cm | (35,35) | Backward |
| 7 | RIGHT R35 / 90 | Reverse 35 cm | (35,0) | Left |
| 8 | RIGHT R35 / 90 | Reverse 35 cm | (0,0) | Forward |

Mark the starting rear-axle midpoint and the body's initial direction. Coordinates
above use +X right and +Y forward from the loop origin. The ideal rear-axle loop
fits in 35 by 55 cm, but the whole car sweeps beyond it; allow about 1.5 by 1.5 m
of clear floor. This is a geometrically closed test, not position feedback:
actual final offset/angle reveal accumulated steering, distance and slip errors.

After reset, initialization is followed by a ten-second startup wait, then the
route runs automatically once. First gyro preparation can add about 5.5 seconds
before motion. Each motion in the loop is followed by status output and an explicit two-second
pause. Each reverse also dumps its buffered trace before the next move. RIGHT's
automatic 1500 -> 1725 us sequence adds one second before DONE. LEFT automatically
applies HEADING,OFF. No manual steering/reset steps are inserted between movements.
The route begins with HEADING,OFF to hold each straight's own starting direction.

Save the UART log, observe the steering return after every RIGHT, and measure the
final rear-axle offset and body angle without repositioning the car during the run.
UART STOP aborts and cancels remaining rows, including during a pause or servo
return. SEQ,DONE marks completion; reset starts the route again. Flashing firmware
in this mode and allowing it to run will start the test automatically.

## Executor configuration and other examples

Select in `Core/Inc/test_config.h` (currently selected):

```c
#define ACTIVE_TEST TEST_HARDCODE_COMMAND_EXECUTOR
```

With TURN_REVERSE_DIAGNOSTIC_CASE=0, edit the `commands[]` array in
`Core/Src/tests/test_hardcode_command_executor.c` to define your sequence:

```c
static const char *const commands[] =
{
    "PING",
    "STRAIGHT,GOCM,20",
    "WAIT,500",
    "STRAIGHT,GOCM,-20",
    "STOP",
};
```

The example above drives forward 20 cm, waits 500 ms, then reverses 20 cm and
stops. Startup and per-row gaps come from the constants below the active array
(currently 10000 ms and 0 ms). A configured automatic gap applies between
commands only when `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND=1U`. At `0U` the
automatic gap is skipped, but startup and explicit WAIT remain. Change
`EXECUTOR_STARTUP_DELAY_MS` and `EXECUTOR_COMMAND_GAP_MS` in the
same file as needed. These are initial timings; actual motion needs floor testing.

Each string is one command without CR/LF, at most 127 bytes. Existing commands
are dispatched through `command_interface.c`; no second motion implementation
or UART loopback is used. Motion rows wait for the controller's successful DONE
state, not just its immediate OK response. UNTILUS, LEFTIR and RIGHTIR wait for threshold completion;
TURN waits for its own terminal state, including automatic post-turn actions:
RIGHT performs stopped 1500 us / 500 ms then 1725 us / 500 ms, clearing
heading history at each pulse; LEFT applies HEADING,OFF. These apply in either
gear. The following row cannot start early, and UART STOP cancels the holds.
Do not add duplicate manual STEER/HEADING,OFF rows for these actions.
Ordinary status/configuration commands
finish immediately. `WAIT,<ms>` is a local executor extension (integer 0..60000),
not a new UART command. Invalid entries stop at that row; validation occurs when
the row is reached, so earlier rows may already have executed.

The sequence runs once. After success, rejection, motion failure or cancellation,
motor PWM is zeroed and steering centered. The executor stays in its service loop
and does not restart. Resetting the board runs the sequence again.

During startup delay, motion and WAIT/gap intervals, UART supports:

- `STOP`: immediately abort the current motion and cancel all remaining rows.
- `PING`, `STATUS`, `STRAIGHT,STATUS`, `TURN,STATUS`, `HEADING,STATUS`: query without changing the sequence.
- `STRAIGHT,TRACE`: dump the last straight motion's buffered steering/gyro samples while stopped; returns `ERR,BUSY` during motion. See [forward baseline trial](TURN_REVERSE_DIAGNOSIS.md#previous-test-forward-baseline-with-gyro-correction).
- Other incoming commands: `ERR,BUSY`, including after the sequence ends.

An array entry `STOP` only stops motors at that row and then continues the array.
Since each motion row is awaited, use external UART STOP to interrupt an ongoing
motion. `MOTOR,*` is an immediate PWM demand subject to global scaling; use an explicit `WAIT` followed
by `STOP` to run it for a duration. Final shutdown also zeros raw PWM. The normal
command loop still has blocking initialization/calibration and UART operations;
first gyro calibration can delay OK and STOP processing by about 5.5 seconds.

UART adds the following sequence messages alongside normal command replies:

```text
SEQ,START,count=5,startup_ms=3000
SEQ,CMD,index=1,text=PING
PONG
SEQ,STEP_DONE,index=1
...
SEQ,DONE,count=5
```

Failure example:

```text
DONE,STRAIGHT,mode=UNTILUS,state=TIMEOUT,reason=TIMEOUT,threshold_cm=20,us_mm=INF,us_status=-1
SEQ,STOPPED,index=2,reason=TIMEOUT
```

Indices are one-based. `SEQ,STEP_DONE` means that array row finished; `SEQ,DONE`
means the entire sequence finished. Failure reasons include COMMAND_REJECTED,
BAD_ARRAY, NULL_COMMAND, BAD_LINE, BAD_ARGS, BAD_WAIT, USER_STOP and the existing
motion terminal failure states. For COMMAND_REJECTED, the preceding ERR reply
contains the underlying parser/device reason. A pre-execution stop references
index 1 when the list is nonempty; index 0 indicates no row exists.

Switch ACTIVE_TEST back to ASSESSMENT_COMMAND_INTERFACE for normal UART control.
This test uses the shared ultrasonic command driver, not the independent
TEST_ULTRASONIC ground-truth test.

## Side IR rows

## IR GT / LT 更新

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



```c
"STRAIGHT,LEFTIR,GT,50",
"TURN,RIGHT,R=35,A=60",
"STRAIGHT,RIGHTIR,GT,50",
```

These are illustrative rows, not a replacement for the current array. LEFTIR
accepts 10..50 cm; RIGHTIR accepts 10..60 cm. Drive forward until the selected
IR distance is strictly greater than the threshold. Equality continues. FAR,
ADC failure and conversion error complete normally with DONE, so the executor
advances. If the first sample already qualifies, no drive power is applied.
STOP, gyro faults, 30-second timeout and 200 cm travel limit still terminate
the sequence. See [IR_STRAIGHT.md](IR_STRAIGHT.md) for status and replies.

## Speed and transitions

See [MOTION_SETTINGS.md](MOTION_SETTINGS.md). Current configuration is 120% drive
PWM and `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND=0U`. That skips repeated calibration,
preparation stops/stillness checks, turn servo/settling waits and automatic gaps.
First-use calibration, target braking, explicit WAIT and startup delay remain.
This reduces pauses but does not blend consecutive motions or eliminate restart
kick. Change settings in `Core/Inc/drivers/robot_tuning.h`, then rebuild and flash.

## Validation

Firmware compilation/linking and simulated IR controller checks passed during
implementation. Physical motion and executor sequencing still need floor tests.

1. Run an array with PING, WAIT, STATUS and STOP; check indices and single completion.
2. Run two motion rows; confirm the second begins only after the first DONE.
3. Send STOP during startup, motion, WAIT and the inter-command gap; confirm no later row runs.
4. Put an invalid command/value in the middle; verify ERR then SEQ,STOPPED and no later motion.
5. Force an UNTILUS motion timeout; verify failure propagation and motor cutoff. Missing echoes alone now mean infinity and must not abort the sequence.
6. Check the example executes once only. Check normal UART mode still dispatches commands.
7. Test LEFTIR/RIGHTIR below, equal to and above threshold; vary only the other side.
8. Verify FAR/ADC failure emits normal DONE and advances; test both before drive and during motion.
9. Test both wait-switch settings. Explicit WAIT and STOP must work with either setting.

Automatic commands now share planned route heading. The existing course array is
unchanged. Use the separate [motion sequence test](MOTION_SEQUENCE_TESTS.md) for
ON/OFF demonstrations; do not insert heading resets between route legs.
