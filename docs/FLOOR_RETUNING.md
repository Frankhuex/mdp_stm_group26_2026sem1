# Floor retuning guide

The final assessment command interface and the calibration tests use the SAME production controllers.
This is deliberate: if a calibration test is correct, the UART/RPi command uses the same logic.

## Straight control

Select in `Core/Inc/test_config.h`:

```c
#define ACTIVE_TEST TEST_STRAIGHT_CONTROL
```

Then edit `TEST_STRAIGHT_DISTANCE_CM` near the top of:

`Core/Src/tests/calibration/test_straight_control.c`

A 40 cm forward/reverse comparison is configured as follows:

```c
#define TEST_STRAIGHT_DISTANCE_CM       40
#define TEST_STRAIGHT_ROUND_TRIP        1U
#define TEST_STRAIGHT_RETURN_DELAY_MS   10000U
```

After the initial 3-second delay and sensor preparation, it runs +40 cm,
stops for a 10-second OLED countdown, then runs -40 cm after sensor preparation.
Do not reposition the robot during the pause. A failed first leg cancels the
return. After both legs, the OLED cycles forward result/trace and reverse
result/trace, holding each page for 8 seconds. Both records are retained.
Trace marks adapt to one-quarter, one-half and three-quarters of the requested
distance (rounded down to whole centimetres): 40 cm uses 10/20/30 cm.

Set `TEST_STRAIGHT_ROUND_TRIP` to `0U` for a single run. In single-run mode,
use a negative distance to test reverse alone; round-trip mode requires a
positive distance. Commands still use the same `Straight_StartCm` controller.

Mark the rear-axle midpoint and chassis heading at the start, measure actual
forward travel during the pause, then measure the final displacement and
heading relative to those marks. Returning to the start alone does not prove
distance accuracy: equal over-travel or under-travel in both legs can cancel.
The heading controller now preserves the planned route heading across forward,
reverse, turns and sensor-terminated straights. H1 means route correction is ON
for both legs; yaw is the estimated error from the planned heading. Sampling
continues through braking and the stopped pause. Encoder travel includes braking
but is not an independent floor measurement; coast after release may add travel.

A historical sampling gap or an isolated gyro read failure flags degraded
confidence without blocking the next movement. Persistent live sensor failures
still report faults. Manual motion, STOP and explicit heading/sensor resets start
a new reference. Direct API users must keep stepping the controllers while idle
and use MotionHeading_Reset before beginning a new independent route.

This holds heading, not sideways position or exact path retracing. For the new
mixed-route tests, OLED fields, ON/OFF comparison and full measurement procedure,
see [MOTION_SEQUENCE_TESTS.md](MOTION_SEQUENCE_TESTS.md). The selected build now
uses TEST_MOTION_SEQUENCE; select TEST_STRAIGHT_CONTROL to run the pair above.

The reported 20 cm test positions 0 -> 21 -> 1 cm mean forward travelled 21 cm
and reverse travelled 20 cm. The short-profile trial increased only
`SHORT_FORWARD_OVERSHOOT_COMP_CM` from 0.5 to 1.5 cm, moving the forward
encoder stop target nominally 1 cm earlier. Reverse remains 0.5 cm. Repeat the
20 cm round trip three times with the same chassis reference point, recording
both the forward endpoint and final position. Then check 10 cm in both
directions, since the forward change applies to the whole short profile.

The subsequent 40 cm test positions 0 -> 41.5 -> 1 cm mean forward travelled
41.5 cm (+1.5 cm error) and reverse travelled 40.5 cm (+0.5 cm error).
The long profile now has separate forward/reverse stop compensation:
`LONG_FORWARD_REV_ERROR_COMPENSATION = 0.310f` and
`LONG_REVERSE_REV_ERROR_COMPENSATION = 0.260f`, previously 0.235 in both gears.
These move the stop targets nominally 1.53 cm and 0.51 cm earlier.
Repeat the 40 cm round trip three times, recording both ruler endpoints.
Then recheck 30 and 60 cm in both directions: these settings apply to all
distances above 20 cm, and this one 40 cm observation does not validate the
whole profile. The short-profile and steering PID settings are unchanged
by this long-profile trial.

### Why the two profiles differ

There is one shared gyro-steering PID and encoder motor synchronizer, with two
speed/stop profiles selected by the absolute requested distance:

- 10..20 cm: 100 ms starting pulse, then a ramp based on move progress.
  The current short calibration subtracts 4 cm forward and 3 cm reverse
  before converting to counts (`SHORT_CALIBRATION_OFFSET_CM` minus
  the direction's `SHORT_*_OVERSHOOT_COMP_CM`).
- Above 20 cm: cruise PWM, then slow PWM after 0.80 wheel revolutions from
  the start (it is not a remaining-distance threshold). Long compensation
  subtracts 0.310 revolutions forward (nominally 6.324 cm) or 0.260 reverse
  (nominally 5.304 cm) before braking.

These are early-stop allowances. They do not specify the final ruler distance.
30 cm and 60 cm both use LONG, but spend different amounts of time at the slow
speed. Calibrating 60 cm does not validate every shorter move. A unified speed
profile is possible, but changing profiles also invalidates the current floor
calibration; do not remove the short branch as a substitute for calibration.
Use repeated floor measurements at 10, 20, 30 and 60 cm in both directions.

Recommended checks after changing floor / tyres / battery / payload:

- 10, 20 cm: SHORT profile
- 30, 40, 50, 80, 100, 120 cm: LONG profile

Main values are all together in:

`Core/Inc/control/straight_tuning.h`

Most floor-sensitive values:

- SHORT: `SHORT_*PWM`, `SHORT_*COMP*`
- LONG: `LONG_BASE_PWM`, `LONG_SLOW_PWM`, `LONG_*_REV_ERROR_COMPENSATION`
- straightness: `SYNC_KP`, `KP_STEER_US_PER_DEG`, `KI_STEER_US_PER_DEG_S`, deadbands

Record actual ruler/tape distance and sideways deviation, not only encoder output.

### Software verification

Run `tests/host/run_straight_heading_tests.cmd` on Windows with the installed
Visual Studio 2017 C compiler. The small `no_sdk` test runtime allows this
test to execute without Windows SDK/UCRT headers. It links the production
straight controller to simulated hardware and checks both return correction
signs, heading carry through sensor re-zeroing, 80 ms brake timing and samples,
idle sampling, timer wrap, explicit resets, STOP, invalid references, preparation
failure, and the existing IR regression cases. It does not validate physical
steering response, gyro bias or floor accuracy.

### Residual left drift (initial trial, hardware validation required)

The current straight-control trial uses
`STRAIGHT_FORWARD_TRIM_US = 42` relative to the 1725 us mechanical centre,
giving a 1767 us forward baseline with gyro feedback. A fixed-1767 calibration
run ended +0.700 degrees left; the revised baseline still needs validation
with feedback active. See [current trial](TURN_REVERSE_DIAGNOSIS.md#previous-test-forward-baseline-with-gyro-correction).
Reverse trim is independently +8 us; changing forward trim does not change it.
Positive trim steers the wheels right; reverse yaw response is opposite.
Trim is applied before starting the motors and retained under PID feedback.
Forward UNTILUS shares this trim; turn-controller calibration is unchanged.
The minimum nonzero PID correction is reduced from 12 to 4 us to reduce
abrupt correction reversals near zero yaw. Servo response must be checked:
if 4 us cannot overcome steering friction, revisit this minimum separately.

On the same floor, mark the initial wheel alignment and run 50 cm five times,
then 100 cm five times. Record actual distance, signed sideways deviation,
final yaw, and encoder L-R difference. Increase forward trim by 2 us if the
mean deviation remains left; decrease by 2 us if it becomes right. Change
one parameter at a time. Also check short travel, reverse, and UNTILUS.
Set forward trim to 0 and the minimum correction to 12 us to restore the
previous steering behaviour. These values are trial settings, not a measured
conversion from lateral error to servo pulse width. If yaw remains near zero
while physical drift persists, investigate wheel alignment/slip and gyro bias
before increasing PID gains or changing encoder calibration.

## Turn angle

Select:

```c
#define ACTIVE_TEST TEST_TURN_ANGLE
```

The current `turn_cases[]` in `Core/Src/tests/calibration/test_turn_angle.c`
contains LEFT R20/A90 and RIGHT R35/A90, starting on RIGHT (case 2).
Align the chassis and mark the
rear-axle midpoint S. Send `G`, measure the outbound angle and chord, then
send `B` without repositioning. Record return heading, lateral offset and
forward/back offset separately. Reposition and send `R` before another pair;
`N`/`P` selects a case. The printed chord formula assumes a 90-degree arc.

An earlier RIGHT R35/A90 run measured 180-84 = 96 degrees physically and a
50 cm chord, with a close rear-midpoint return but a left heading error.
The current trial uses forward stopping lead 0.165 s, based on post-stop
yaw/rate 13.49/82.41 = 0.164 s. Its forward target scale is 0.978, calculated
jointly with that lead change: desired gyro finish = 93.77*90/96 = 87.91;
new target = 90.99 + (87.91-93.77) + 82.41*(0.165-0.130) = 88.02 degrees.
This local linear estimate is a floor trial, not a guaranteed correction.

The new `TURN_RIGHT_REVERSE_COAST_TIME_S` is 0.155 s, increasing the old lead
by about 2.20/86.19 seconds to address the +2.20-degree left return heading.
The next physical test reported an accurate-looking outbound 90 degrees,
but the returning nose was about 0.5 cm right and rear midpoint 1.5 cm
forward of S. No gyro log was supplied for that observation. The current
reverse-only trial changes `TURN_RIGHT_REVERSE_TARGET_SCALE` from 1.000 to
1.018 after route-heading correction: about 1.6 degrees extra on a nominal
90-degree return, or about 1 cm more arc travel at R35. The centimetre nose
offset is not a measured angle; verify both heading in degrees and position
after this small trial. Restore reverse scale 1.000 to undo it.

Route-heading feedback still adjusts the local target using the preceding
outbound measurement. Steering scale
remains 0.965 (R35 pulse 2569 us) for this angle/braking trial. A 50 cm chord
at 96 degrees implies an effective circular radius of 33.64 cm, so remeasure
the radius after correcting the angle. A physical 90-degree R35 arc should
have a 49.50 cm chord. Record angle, chord, return heading and both offsets.

RIGHT turns below 90 degrees retain their separate 0.17 s stopping lead.
The target/steering scales apply across RIGHT radii/angles; the new long-turn
leads apply to both 90-degree and larger turns. Only R35/A90 supplied these
measurements. To restore the settings used in the earlier 96-degree run,
set forward/reverse target scales to 1.011/1.000 and both RIGHT long-turn
leads to 0.130 s, with steering 0.965.

Main production turn values are in:

`Core/Inc/control/turn_tuning.h`

Most floor-sensitive values:

- `TURN_LEFT_COAST_TIME_S` / `TURN_RIGHT_COAST_TIME_S` and `TURN_RIGHT_SHORT_LEAD_TIME_S`: command stopping lead based on measured yaw rate; increase for overshoot, decrease for undershoot. `TURN_*_STOP_RATIO` affects only the legacy minimum-radius test, not command turns.
- `TURN_*PWM` and global `MOTOR_SPEED_SCALE_PERCENT`: turn drive effort / inertia. Retest 15/30/45/60/90/180 degrees after changing either. See [MOTION_SETTINGS.md](MOTION_SETTINGS.md) for current kickstart and active-brake behavior.
- `TURN_LEFT_TARGET_SCALE` / `TURN_RIGHT_TARGET_SCALE`: command yaw scaling.
  If a right `A=90` command physically measures 95 degrees, use `90/95 = 0.947`.
- left/right minimum-radius values if your measured safe radius changes

For a return position error after matching the heading, tune the reverse
steering independently using `TURN_LEFT_REVERSE_RADIUS_SCALE`. The current
0.930 trial tightens LEFT reverse steering relative to the forward scale
0.954: R20 requests 1013 us on return versus 1024 us outbound. This tests
whether a tighter reverse arc reduces the reported 1 cm right offset;
it is not a measured conversion from offset to servo pulse. It applies to
all LEFT reverse turns, but only R20/A-90 supplied the floor evidence.
Set it back to 0.954 to restore the previous reverse steering. The existing
endpoint clamp still applies. `TURN_CAL,STEERING` reports the commanded pulse.

Keep `TURN_LEFT_REVERSE_TARGET_SCALE` at 1.008 for this radius trial. A lower
reverse radius scale tightens steering; a higher one widens it. After flashing,
repeat three `G`/`B` pairs from the same marked start and report heading plus
both signed position offsets. Recheck the angle because steering changes
also change yaw rate and stopping behaviour; accurate angle alone does not
guarantee a return to S.

## Assessment mode

After tuning, switch only:

```c
#define ACTIVE_TEST ASSESSMENT_COMMAND_INTERFACE
```

No algorithm is copied. `command_interface.c` calls the same `straight_control.c` and `turn_control.c` that the calibration tests use.

## Shared calibration and drift adjustment

Edit `Core/Inc/drivers/robot_tuning.h` for the single source of servo center,
left/right endpoints, wheelbase, endpoint radius assumptions, encoder calibration
and stationary gyro calibration. It is used by hardware, straight/turn control,
command telemetry and servo tests. Controller-specific filters/gains remain in
their own tuning files; different loop rates should not share filter gains blindly.

Current initial field changes (not hardware-validated):
- SERVO_CENTER_US: 1726 us (previous straight 1716, turn 1712); positive shift
  biases steering right to counter reported left drift.
- TURN_LEFT_STEER_SCALE: 1.00; TURN_RIGHT_STEER_SCALE: 0.965, in turn_tuning.h.
  These multiply the turn pulse excursion from center. Lower values relax the
  steering and increase radius; they do not mean the radius is exactly 2%/5% larger.
- TURN_LEFT_STOP_RATIO=0.80 and TURN_RIGHT_STOP_RATIO=0.92 are retained for final
  angle compensation. Radius and final angle must be checked separately.
- Wheel synchronization now compares calibrated travel normalized to a 20 ms
  interval. SYNC_KP=1.8 means PWM per left-equivalent count per nominal interval.
  The old extra x1000 forced saturation even for one count; that factor is removed.
  MAX_SYNC_CORRECTION remains 250. The gain's effective behavior therefore changes
  substantially and requires field retuning; a left bias is not proven to arise
  solely from this previous saturation.

Validate one straight segment, then LEFT R20/A90 and RIGHT R35/A180 independently
before the four-cycle route. Measure final heading and the rear axle midpoint path.
If a turn remains too tight, reduce that side's STEER_SCALE slightly. If its final
angle overshoots, reduce that side's STOP_RATIO. These fix different observations.
The route still uses relative yaw resets; accumulated route error is not solved
by these mechanical/gain adjustments alone. No compile or hardware test was run.
