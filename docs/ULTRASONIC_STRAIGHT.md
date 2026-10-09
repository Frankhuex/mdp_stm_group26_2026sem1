# STRAIGHT,UNTILUS command

Send a newline-terminated USART3 command:

```text
STRAIGHT,UNTILUS,20
```

The robot drives forward with the shared gyro steering PI and wheel encoder
synchronization until a fresh valid ultrasonic reading is strictly less than
the command threshold plus the 8 cm sensor-to-bumper offset. Equality does not
complete the command. The command value is bumper/object clearance, while the
controller uses sensor/object clearance. Accepted integer thresholds are 5..200
cm. Negative values, decimals and out-of-range values are rejected.

Select ASSESSMENT_COMMAND_INTERFACE in Core/Inc/test_config.h to use commands.
The hardcoded executor can run the same command. TEST_ULTRASONIC is a separate reference test.

## Replies and control

```text
OK,mode=UNTILUS,threshold_cm=20
DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=20,us_mm=197,us_status=0
```

OK means accepted. One DONE notification follows for success or failure.
Use STRAIGHT,STATUS to query mode, threshold, last sample, sample age,
elapsed time, encoder travel and yaw. us_status=0 means valid and fresh;
-1 means unavailable/stale/no rise; -2 means invalid pulse/stuck/long echo.
Do not interpret us_mm as a valid distance when us_status is nonzero.
After termination the last sample is retained and its age keeps increasing.

STOP and STRAIGHT,STOP abort both initial measurement wait and driving.
Terminal states/reasons are DONE/US_THRESHOLD, ABORTED, SENSOR_ERROR,
TIMEOUT, and DISTANCE_LIMIT. An immediate initialization failure returns
ERR,NO_DEVICE without accepting a motion. Malformed arguments return
ERR,BAD_ARGS; values outside the supported range return ERR,BAD_VALUE.
Other movement/motor/steering commands return ERR,BUSY while running.
Encoder/yaw reset and gyro mutation/read-update commands are also blocked.

## Measurement and control behavior

- Wait for startup confirmation. Two consecutive valid readings below threshold
  complete without motor power. Two consecutive valid readings above threshold
  start moving. Three consecutive NO_ECHO/invalid readings start moving and are
  treated as infinity.
- Wiring matches the working mdp21_8_ultrasonic example: PB15=TRIG and
  PC8=ECHO via a 5V-to-3.3V divider. TIM8_CH3 captures both edges at 1 MHz
  with ARR=65535. PC6/PC7 are no longer used by this sensor driver.
- Trigger spacing is at least 60 ms. Echo waits are asynchronous, with a 60 ms
  deadline for the whole measurement. Only the 2+10 us trigger pulse uses a
  DWT busy wait; TIM6 and the TIM8 capture counter are not reset.
- Acquisition now follows the ground-truth example: pulses above 25000 us are
  out of range; no additional minimum-pulse or edge-phase rejection is applied.
  UNTILUS maps invalid or zero readings to infinity.
- No echo or a sample older than 200 ms means infinity: continue at cruise PWM.
  Missing echoes do not stop motion. STOP, 30 seconds and 200 cm still stop it.
  SENSOR_ERROR is retained only for driver-unavailable faults.
- Kickstart PWM is 4000. Cruise PWM is 3500; within 25 cm above the threshold it is 2600.
  These are baseline PWM values, with the existing wheel synchronization applied.
- Either wheel reaching 200 cm measured travel stops with DISTANCE_LIMIT.
  Total command time of 30 seconds stops with TIMEOUT.
- Threshold success, abort and faults all apply a short both-inputs-high brake
  pulse, then zero motor PWM and center the servo. Actual stationary clearance
  can still be smaller than the reading that triggered stopping.
- GOCM retains its existing SHORT/LONG distance profiles and completion behavior.
- TEST_ULTRASONIC uses the shared blocking Read wrapper and the reference
  cadence: acquire, print UART/OLED, then delay 250 ms. UNTILUS uses the same
  acquisition primitives through Trigger/Step/GetSample without blocking.
  Do not call Read concurrently with an asynchronous consumer.
- Startup gyro initialization/calibration still follows the existing controller
  behavior and can block before OK (calibration approximately 5.5 seconds).
  Measurement and motion stepping after acceptance are asynchronous. Existing
  UART polling/transmit still adds main-loop latency; this is not a hard real-time
  stopping guarantee. Display refresh is skipped during motion.

All UNTILUS limits and PWM values are grouped under US_* in
Core/Inc/control/straight_tuning.h. They are initial settings, not validated
physical stopping accuracy. Existing GOCM compensation is not used in UNTILUS.
GPIO/interrupt settings are also reflected in mdp21_8.ioc.

## Manual verification after building

1. With motors disabled, run TEST_ULTRASONIC and verify valid readings, no echo,
   and that reconnecting the sensor restores readings. Check TIM8_CC_IRQHandler -> HAL_TIM_IRQHandler -> HAL_TIM_IC_CaptureCallback
   for TIM8 channel 3. NO_ECHO means no complete measurement within the timeout;
   OUT_OF_RANGE means pulse >25000 us. Failure output includes edges, last_ccr
   and last_pin (255 means no capture callback has recorded an edge).
2. In command mode, test invalid text, empty/extra fields, integer overflow,
   thresholds 4 and 201, and accepted boundaries 5 and 200.
3. With an obstacle initially below 20 cm, send UNTILUS,20: OK followed by exactly
   one successful DONE; motors must never energize.
4. At exactly 20 cm the command must not succeed until the measured value is
   below 200 mm. Verify with controlled readings; physical sensor noise matters.
5. Approach from farther away; verify cruise/approach behavior and final measured
   clearance on the actual floor, battery and payload.
6. Test no echo at startup (starts moving), signal loss while moving (continues),
   STOP during startup wait/driving, duplicate movement commands, and STATUS.
7. Verify encoder/yaw reset and gyro calibration return BUSY during motion.
8. Verify TIMEOUT and DISTANCE_LIMIT each stop and issue one failure notification.
9. Repeat a new command after success, abort and sensor failure to check recovery.
10. Recheck existing GOCM short/long forward/reverse commands and TURN commands.

No compilation, flashing or hardware tests were performed for this change.

## Repair against the working example (2026-09-09)

The copied/generated CubeMX configuration already provides PB15 output, PC8 AF3,
TIM8 channel 3 direct capture on both edges, and its capture/compare interrupt.
The previous shared driver still triggered PC7 and listened on PC6 EXTI; it only
started the timer base, never input capture. The generated PC6 was a plain input,
so the old callback could not obtain the expected echo.

The shared driver now starts HAL_TIM_IC_Start_IT on channel 3, reads CCR3 in the
HAL capture callback, handles 16-bit wrap, and uses DWT for the trigger pulse as
in the working example. The old EXTI callback has been removed. The reference
folder and regenerated CubeMX pin/timer setup have not been changed. The normal
project source entries include Core, Drivers and Middlewares, not the nested
reference project's sources.

A second comparison found acquisition behavior had still diverged: the shared
initializer omitted the reference's explicit PB15 output initialization, and
trigger/capture paths added ECHO-high rejection, phase rejection, overcapture
rejection and a minimum pulse cutoff. These differences are now removed.
PB15 initialization, DWT timing, trigger, conversion and capture arithmetic are
copied from the reference. The only capture addition timestamps completed echoes
for command freshness. Test timing and OLED presentation also follow the reference.
UNTILUS retains asynchronous waiting and its existing stopping limits.

The generated main/MSP, clock, HAL configuration and timebase match the reference;
the existing link map includes the sensor callback. This rules out those source
mismatches, but does not prove which previous difference caused the hardware
failure. No new firmware was compiled or tested on hardware here.

## NO_ECHO with edges=0: diagnostic build diag=2

This means no TIM8 channel-3 capture callback was recorded, not that the measured
distance was out of range. The component test now prints US_GPIO, US_TIM, US_IRQ
at startup and on failure, plus US_OBS for the most recent blocking measurement.
The startup marker contains diag=2. OLED also shows EDGES / ECHO / FLAG / TIMER.

- Expected GPIO: pb15_mode=1 (output), pc8_mode=2 (AF), pc8_af=3.
- Expected TIM8: PSC=15, ARR=65535, CR1 CEN bit=1, CCMR2 CC3S bits=1,
  CCER CC3E/CC3P/CC3NP bits=1, DIER CC3IE bit=1. Other bits may also be set.
- Expected IRQ: enabled=1, primask=0; inspect basepri if interrupts are masked.
- US_OBS timer_runs=0 means no counter progress was observed during the wait.
- With edges=0, echo_high=1 shows PC8 was seen high, directing attention to
  capture configuration or a stuck-high input. cc3_flag=1 shows a hardware capture
  flag was observed; then check interrupt enabling/masking/vector routing.
- With correct configuration and a running timer, echo_high=0 and edges=0 means
  no high level/callback was observed during the wait. It does not prove absence
  of every short pulse; verify PB15 trigger at the module and the PC8 return path.
- PB15 level is read after the trigger, so a low level in US_GPIO is normal and
  does not establish whether the physical trigger pulse was generated correctly.

These diagnostics are passive: the polling test does not clear TIM8 flags or
read CCR3, and the asynchronous motion acquisition is unchanged. No runtime
result or hardware repair is claimed until the diagnostic output is available.

UNTILUS status/completion uses `us_mm=INF` for unknown clearance with nonzero
`us_status`; the C status API uses UINT32_MAX for this value. The standalone
ultrasonic test and raw sensor API are unchanged.
