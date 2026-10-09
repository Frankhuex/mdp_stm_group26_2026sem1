# Turn then reverse: floor test guide

Test the observed sequence: forward 90-degree turn, then straight reverse.
There is no reverse turn in these tests. Keep tuning fixed within each comparison.

## Automatic actions after each turn

Successful TURN commands now include the user's working manual follow-up:

- RIGHT: after braking and the normal settling phase, keep drive PWM zero;
  reset heading history and apply 1500 us for 500 ms; reset heading
  history again and apply the shared centre, 1725 us, for 500 ms.
  The heading ON/OFF setting itself is preserved, exactly as with STEER,US.
- LEFT: after braking/settling, apply HEADING,OFF, which disables route error
  carry and resets its origin and local gyro yaw. There is no extra servo sweep.

The turn remains RUNNING until these steps finish. DONE,TURN and the next
command-array row wait for completion. The UART interface returns ERR,BUSY for
an early movement command; it does not queue it. STOP cancels either steering
hold and centres the servo. Failed/aborted turns do not run a pending successful
completion action. The added waits use controller steps, not blocking delays.
These actions follow the LEFT/RIGHT command side for all supported angles and
both signs of A. They leave the turn's steering mapping, speed and calibration
unchanged. Floor evidence supplied by the user was for forward 90-degree turns.
The calibration/minimum-radius blocking test does not use this command completion.

After flashing, send one line at a time:

```text
TURN,RIGHT,R=35,A=90
```

Watch the stopped steering return through 1500 then 1725 us. Wait for successful
DONE,TURN, then send `STRAIGHT,GOCM,-30`. The extra return takes about one
second in addition to the turn's normal braking/settling. No manual STEER rows
are needed. In a separate floor trial, send `TURN,LEFT,R=20,A=90`, wait for
DONE,TURN, then send `STRAIGHT,GOCM,-30`; heading OFF is applied automatically.

Use TURN,STATUS to inspect the saved completed-turn yaw. HEADING,STATUS instead
shows the intentionally reset reference after a turn: target/measured/error zero,
tracking=0; LEFT also shows enabled=0. The next straight still uses gyro feedback.
This implements the manual procedure; the integrated sequence needs floor testing.
The earlier heading-carry comparisons below are historical diagnostics: successful
turns now clear that history automatically, so those old ON/OFF comparisons no
longer preserve the pre-turn route reference across completion.

## Current test: left 90-degree endpoint

The user confirmed physical overshoot after tightening the servo. Two fresh
LEFT R20/A90 turns reported +93.258 and +93.111 degrees. The following forward
35 cm with heading OFF held its starting direction well: peak recorded local
yaw about +0.127 degrees, final -0.001 degrees. Straight tuning is retained.

The first adjustment from TURN_LEFT_TARGET_SCALE 1.017 to 0.982 reduced
the internal target from 91.53 to 88.38 degrees. The follow-up TURN,STATUS
reported final yaw +89.085, short of the requested 90 by 0.915 degrees.
The current refinement changes only that scale from 0.982 to 0.990. Linear
interpolation between the earlier mean 93.1845 at 1.017 and 89.085 at 0.982
suggests 0.98981, rounded to 0.990. The new internal target is 89.10 degrees.
These gyro observations estimate the next trial; they do not guarantee its
physical endpoint or repeatability.
The command remains A=90, with a planned 90-degree heading. This target
compensates for stopping behaviour; it is not the desired physical angle.
Radius/servo mapping, braking lead and speed are unchanged, as are right-turn
and reverse calibration. The scale affects all forward LEFT turns, but only
R20/A90 supplied this evidence. Other radii/angles need separate validation.
The gyro-based estimate must be checked against the actual 90-degree floor line.

Flash the rebuilt firmware, align the car to the same starting line and send
one command at a time:

```text
HEADING,OFF
TURN,LEFT,R=20,A=90
```

After DONE,TURN, keep the car still and send:

```text
TURN,STATUS
HEADING,STATUS
```

Record whether the body is aligned with the perpendicular floor line, still
past it, or short of it. Measure the physical angle if possible. Repeat from
the same physical starting orientation, sending HEADING,OFF before each
independent trial. Test only the turn first; no subsequent straight is needed.
Heading OFF avoids inherited error but gyro turn stopping remains active.
Do not send manual STEER or a heading reset before saving the readings.
TURN,STATUS retains the measured turn yaw after the automatic actions above;
HEADING,STATUS is reset by design and cannot measure the completed turn angle.

## Previous test: forward baseline with gyro correction

The fixed-1767 us / 35 cm run ended at +0.700 degrees left with about 1 cm
reported left displacement. The command used gyro_steering=0, so no gyro
correction was applied. Other fixed-pulse runs ended +0.832 degrees at 1765 us
and -1.291 degrees at 1770 us. These are individual trials, not a verified
repeatability result or proof of an exact neutral pulse.

Forward trim is now +42 us: normal forward GOCM starts at 1725 + 42 = 1767 us
at zero heading error, with its existing P=30, I=0.6, D=2 gyro correction.
The gyro can adjust the pulse in both directions during movement. Reverse
retains its 1733 us baseline and P=14; the stopped centre remains 1725 us.
Forward IR and ultrasonic movement also share the revised forward baseline.
Speed, filtering, gains and distance compensation are unchanged in this trial.

Flash the updated firmware, use the same wheel preparation as the last trial,
then align the car with the floor line. Send one line at a time:

```text
HEADING,ON
STRAIGHT,GOCM,35
```

After DONE,STRAIGHT, send:

```text
STRAIGHT,STATUS
STRAIGHT,TRACE
```

Expect fixed_us=0, an S-row pulse of 1767 at zero error, changing D-row pulses
as gyro correction responds, and an E-row pulse of 1725. Compare final local
yaw and front/rear displacement with the fixed-1767 run. Record any left/right
oscillation. This is a trial requiring physical validation, not a confirmed fix.
CAL still holds exactly the requested pulse; it does not save a new centre.
HEADING,ON resets the reference for this independent test. For a later turn ->
straight route test, send it only before the route so turn error stays tracked.

## Earlier test: fixed steering calibration

The latest fresh-origin 50 cm run starts at zero error, yet ends at +1.778
degrees (left). The front moved about 1.7 cm left and rear about 1.2 cm left.
Servo demand increased from 1737 to 1789 us. Measured yaw rate fell from about
+2.02 deg/s to +0.278 deg/s before braking, but the left rotation was not removed
before distance completion. The trace measures commanded PWM, not wheel angle;
it cannot establish the correct neutral pulse or prove a mechanical cause.

At that time normal tuning was forward trim +12 / P=30 and reverse trim +8 / P=14.
The new CAL command holds a requested pulse throughout one forward run, so
the steering response can be measured without the gyro changing that pulse.
This is a diagnostic, not a physically verified fix. Motor synchronization,
gyro measurement, distance stopping, braking, timeouts and STOP remain active.
Limits are forward 10..50 cm and centre +/-100 us (currently 1625..1825).

Build and flash the updated firmware. Place the car along a marked straight
line. Use the same initial wheel preparation for each comparison. If using the
previous centring procedure, send STEER,US,1500, wait two seconds, then
STEER,US,1725, wait two seconds. Do this before resetting the heading below.
Send only the command text, one line at a time:

```text
HEADING,ON
STRAIGHT,CAL,35,1770
```

Wait for `DONE,STRAIGHT,state=DONE,target_cm=35`, then send:

```text
STRAIGHT,STATUS
STRAIGHT,TRACE
```

Expect `OK,mode=CAL,...,gyro_steering=0` and `fixed_us=1770` in status/trace.
The S/D/B rows should all report `servo_us=1770`; E normally reports 1725
because stopping centres the servo. Record front and rear drift from the line,
the final local_yaw, and film actual front-wheel direction during driving.
1770 is a trial value between the previous baseline and final correction;
it is not a measured neutral. A separate fixed-1737 run from the same position
and wheel preparation provides a matched baseline if needed.

Less left rotation supports testing the forward baseline; right rotation means
the trial pulse overcorrects under those conditions. Similar drift despite a
changed command requires checking the actual loaded steering response. Gyro
heading alone cannot measure or recover lateral offset from a marked line.
Use the results before making another gain/trim change.

Normal GOCM automatically restores gyro steering. For a new independent test,
reposition first and send HEADING,ON to discard the calibration run's accumulated
heading. HEADING,OFF also resets the reference but still uses local gyro steering
in normal GOCM; it is not equivalent to CAL.

## Earlier trim trial

Manual 1500 -> 1725 us steering return after a right
turn made reverse look straight, but forward still drifted left. Forward trim
was set to +12 us; reverse was independently held at +8 us. That earlier trial used
the same manual sequence with a two-second wait at each steering
position, followed by GOCM,30 (or GOCM,-30 in a separate repeat). The manual
STEER commands reset route heading. This trim trial does not automate the
steering return and still needs floor validation. It also affects forward
IR/ultrasonic travel through the shared straight controller.

## Earlier forward gyro-response trial

Forward 35 cm at P=20, trim +12 produced a saved final yaw of +1.435 degrees
and a later idle heading of +1.606 degrees. The user observed the front about
1.5 cm left and rear about 1 cm left. During the run, servo demand increased
from 1737 to 1766 us; yaw rate peaked near +1.98 deg/s and fell to +1.073 deg/s
at braking (1101 ms). Correction was in the expected right direction, but left
rotation had not stopped before distance completion. This does not measure the
actual wheel angle or prove how much of the rate change was caused by steering.

Forward proportional gain was increased to 30 us/degree (previously 20), with I=0.6 and
D=2 unchanged. Reverse retained P=14, I=0.6, D=2 and trim +8. Forward trim was then
+12. The 20 ms control period, motor speed, distance stopping and heading ON/OFF
behaviour are unchanged. Forward IR and ultrasonic travel share the new gains.
This is a stronger-response trial, not a claim of a physically verified fix.
Compare the E-row local_yaw across trials, rather than a later HEADING reading
that includes post-stop movement or gyro drift. Heading control cannot measure
or directly remove sideways displacement from a floor line.

Build and flash. To repeat the latest forward-only test, use the same initial
wheel position and send one line at a time:

```text
HEADING,ON
STRAIGHT,GOCM,35
```

Wait for DONE,STRAIGHT, then send:

```text
STRAIGHT,STATUS
HEADING,STATUS
STRAIGHT,TRACE
```

For the turn-to-forward comparison, begin a fresh HEADING,ON route, run
TURN,RIGHT,R=35,A=90 and wait for DONE,TURN. Reproduce the successful manual
centring: STEER,US,1500, wait two seconds; STEER,US,1725, wait two seconds.
Send HEADING,ON then STRAIGHT,GOCM,35 and collect the same three diagnostics.
Here HEADING,ON defines the post-turn direction as the new reference; manual
STEER already reset the old route. This older trial preceded automatic turn completion actions.
Repeat a separate reverse run to confirm the previously successful behaviour.

Trace is buffered in RAM during motion, with no automatic UART printing in
the controller. STRAIGHT,TRACE prints it only while all motion is stopped;
otherwise it returns ERR,BUSY. Collection is about every 100 ms plus start,
brake and end transitions. Capacity is 128 rows: if full, intermediate rows
are dropped and counted, while one slot is reserved for the terminal sample.
The next accepted straight motion replaces the buffer. Query it before starting
another straight. Reading the trace does not reset heading.

Each row contains milliseconds since command start, FWD/REV, phase (S=start,
W=ultrasonic wait, D=drive, B=brake, E=end), local_yaw, control_error, rate_dps,
servo_us and gyro_ok. Positive yaw is left. control_error is the route error
with ON and local yaw with OFF. servo_us is the commanded pulse, not a measured
wheel angle. The E row normally shows 1725 because stopping centres the servo;
inspect D rows to see the correction during driving. Gyro readings may keep
changing after completion, but the saved trace remains fixed.

Compare repeated runs: less left yaw without left/right oscillation supports
the gain trial. Sustained growing left error while D rows command increasing
right steering points to insufficient steering response under load, and does
not justify assuming more gain will cure it. If the car starts oscillating,
return FORWARD_KP_STEER_US_PER_DEG to 20.0f and rebuild/reflash. Preserve the
trace and film the actual wheels so command and physical response can be compared.

## 1. Build and connect

The manual tests below require `ACTIVE_TEST ASSESSMENT_COMMAND_INTERFACE` in
`Core/Inc/test_config.h`. The current build selects the hardcoded closed return
route instead; see [executor instructions](HARDCODE_COMMAND_EXECUTOR.md).
Build and flash using STM32CubeIDE. An existing ELF
in Debug is not evidence that these edits have been built or flashed.

Connect UART USART3 (PD8 TX, PD9 RX), 115200 baud, 8N1, no flow control.
Send each command as one line terminated with newline. Send `PING` and expect
`PONG`. Save the terminal log. Keep the robot stationary during gyro calibration;
the first motion can take about 5.5 seconds to calibrate before it starts.
Use an open floor area and have STOP or power cutoff available.

## 2. First test: wheels returning to centre, motors stopped

Send one line at a time:

```text
STOP
STEER,US,2600
STEER,CENTER
STEER,GET
STEER,US,1000
STEER,CENTER
STEER,GET
```

Wait two seconds after each steering change. Film the front wheels and compare
centre after right with centre after left. Repeat three times on the floor,
then with the front tyres lifted and the robot supported, motors stopped.
If the linkage binds or the servo strains, stop that endpoint test.

Current centre is 1725 us; larger pulse means right. STEER,GET reports the
commanded pulse, not measured wheel position. A reading of 1725 cannot prove
the wheels are physically centred. Look for different final angles or slow return.
If the difference disappears when lifted, steering load is implicated; if it
remains, inspect linkage/horn play and servo repeatability. Recheck power under load.

## 3. Baseline: reverse without a preceding turn

Place the robot on a marked line. Send:

```text
STOP
HEADING,OFF
STRAIGHT,GOCM,-30
HEADING,STATUS
STRAIGHT,STATUS
```

Wait for successful DONE,STRAIGHT before sending the status commands.
Repeat three times from the same position. Film the first 5-10 cm and measure
sideways displacement of the rear midpoint and change in body direction.
If reverse already curves here, turn history is not required to produce it.
OFF retains local gyro PID; this is not a mechanical-only reverse test.

## 4. Right turn with inherited heading ON

Reset the robot to the same marked starting position. Send:

```text
STOP
HEADING,ON
TURN,RIGHT,R=35,A=90
TURN,STATUS
HEADING,STATUS
STEER,GET
STRAIGHT,GOCM,-30
HEADING,STATUS
STRAIGHT,STATUS
```

Wait for successful DONE,TURN before the first status queries. After the turn,
wait five seconds before reverse in every comparison. During that pause, film
wheel return and mark the body's ACTUAL direction after the turn. Do not touch
or reposition the robot between turn and reverse. Save HEADING,STATUS just
before reversing. Wait for successful DONE,STRAIGHT before final status queries.
Do not continue after a failure/timeout; resolve it first.

Watch whether the wheels move again as reverse begins. Compare the body's
direction against its post-turn line, not just the original pre-turn line.
Rear sideways displacement alone does not establish body rotation.

## 5. Repeat right turn, OFF only before reverse

Repeat step 4 from the same starting position, with the same five-second pause.
Insert `HEADING,OFF` immediately before `STRAIGHT,GOCM,-30`, after recording
HEADING,STATUS. Do not send HEADING,ON again before reverse.

ON corrects the stored route error. OFF accepts the direction at the start of
reverse and corrects new local drift. ON and OFF reset heading when sent, so
save the pre-OFF reading. Positive reported gyro heading is left.

Do not insert STOP, STEER,CENTER, STEER,US, YAW,RESET or HEADING,RESET between
turn and reverse: they change the experiment. STOP remains the abort command.
If needed, abort and start the trial again from the beginning.

## 6. Repeat both comparisons after a left turn

Use `TURN,LEFT,R=20,A=90` in steps 4 and 5. Repeat each of the four combinations
three times. If your actual problem uses a different valid turn radius, use
that radius consistently for that side in both ON/OFF trials and record it.

| Observation | Next action |
| --- | --- |
| Centre differs after right/left, particularly on the floor | Address steering load, binding/play or power; then calibrate repeatable centre |
| ON bends, OFF consistently reduces the bend | Inherited heading correction contributes; compare physical turn angle with gyro reading before changing turn calibration |
| Both bend, wheels visibly retain previous steering | Investigate steering return; gyro PID can still contribute in OFF |
| Body is already crooked after the turn and reverse follows that direction | Investigate turn endpoint accuracy |
| Reverse-alone also curves | Investigate reverse trim, wheel drive balance and gyro behaviour |

Several causes can coexist. Current straight feed-forward is 1767 us forward
(1725 + 42) and 1733 us reverse (1725 + 8),
so a small change from stopped centre before reverse is expected even with OFF.
Do not treat that commanded change alone as evidence of faulty centring.

## Optional: ready automatic code sequences

Manual mode above is recommended for the first observations. To run one
prepared sequence without sending motion commands, edit `Core/Inc/test_config.h`:

```c
#define ACTIVE_TEST TEST_HARDCODE_COMMAND_EXECUTOR
#define TURN_REVERSE_DIAGNOSTIC_CASE 1
```

Build and flash after every case change. Diagnostic arrays are in
`Core/Inc/tests/turn_reverse_diagnostic_commands.h`, included by the existing
`Core/Src/tests/test_hardcode_command_executor.c`; no new source registration is needed.

| Case | Sequence |
| --- | --- |
| 1 | Motors stopped: right -> centre, left -> centre; five seconds to observe each centre |
| 2 | Reverse alone 30 cm, heading OFF |
| 3 | Right 90 at R35 -> reverse 30 cm, heading ON |
| 4 | Same right turn with ON; OFF only before reverse |
| 5 | Left 90 at R20 -> reverse 30 cm, heading ON |
| 6 | Same left turn with ON; OFF only before reverse |
| 0 | Editable executor array, currently the closed turn/reverse return route |

Cases 1-6 wait ten seconds after initialization, then run once. Cases 3-6
pause five seconds after turn status queries before reversing. The executor
waits for successful motion completion and records status before and after
reverse. UART STOP cancels remaining rows. Reset runs the selected case again;
flashing/resetting a moving case can therefore initiate motion.
Automatic mode accepts only its supported status queries and STOP over UART;
switch ACTIVE_TEST back to ASSESSMENT_COMMAND_INTERFACE for manual steering.

Current tuning is retained: 120% drive PWM, per-command preparation enabled.
These paused tests diagnose behaviour with time to observe. If only the original
rapid sequence drifts, a later matched short-pause test is needed to investigate
transition timing. Do not change that timing and heading mode simultaneously.

## Record each trial

| Trial | Side / ON or OFF | Wheel centre repeatable? | Heading before OFF/reverse | Body angle before/after reverse | Rear displacement | Wheel direction at reverse start |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |

Keep the video and full UART log together. Judge left/right relative to the
robot's direction immediately after the turn, and record that convention.
