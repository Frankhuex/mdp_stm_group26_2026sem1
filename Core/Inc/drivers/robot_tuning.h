#ifndef ROBOT_TUNING_H
#define ROBOT_TUNING_H

/* Global drive PWM multiplier: 100 = unchanged, 80 = 0.8x, 120 = 1.2x.
 * Applies to both wheels, forward/reverse and kickstart; excludes braking
 * and steering. Actual vehicle speed is not proportional to PWM. */
#ifndef MOTOR_SPEED_SCALE_PERCENT
#define MOTOR_SPEED_SCALE_PERCENT       120U
#endif
#if MOTOR_SPEED_SCALE_PERCENT < 1 || MOTOR_SPEED_SCALE_PERCENT > 200
#error "MOTOR_SPEED_SCALE_PERCENT must be between 1 and 200"
#endif

/* Shared mechanical calibration: edit here, never in individual controllers.
 * Larger servo pulse steers RIGHT. Hold 1725 us for the next 60 cm test
 * with gyro steering enabled and zero extra straight trim. */
#define SERVO_CENTER_US                 1725U
#define CMD_SERVO_LEFT_US               1000U
#define CMD_SERVO_RIGHT_US              2600U

/* Steering backlash take-up: discrete servo positioning (centring before a
 * straight, calibration) approaches the target from the LOW side via a pre-move
 * this far below it, so the mechanical slop is always loaded the same way and
 * the straight-ahead angle is repeatable after any turn. Set to the observed
 * hysteresis (~120us here). Does NOT apply to the per-tick gyro-PID updates. */
#define SERVO_BACKLASH_US               120U
#define SERVO_TAKEUP_SETTLE_MS          120U
#define CMD_WHEELBASE_CM                18.0f
#define CMD_LEFT_MIN_RADIUS_CM          18  /* hard steering limit */
#define CMD_LEFT_SOFT_MIN_RADIUS_CM     20  /* normal lateral manoeuvres */
#define LATERAL_OBSTACLE_WIDTH_CM       10U
#define LATERAL_IR_TO_CENTER_CM         5U
#if CMD_LEFT_SOFT_MIN_RADIUS_CM < CMD_LEFT_MIN_RADIUS_CM
#error "Left soft radius must respect the hard limit"
#endif
#define CMD_RIGHT_MIN_RADIUS_CM         35

/* Keep existing encoder calibration unchanged; all telemetry/controllers share it. */
#define LEFT_COUNTS_PER_REV             1560.13f
#define RIGHT_COUNTS_PER_REV            1559.37f
#define LEFT_COUNTS_PER_CM              74.29f
#define RIGHT_COUNTS_PER_CM             74.26f
#define WHEEL_CIRCUMFERENCE_CM          20.4f

/* Shared stationary gyro calibration. Filter coefficients remain controller-specific.
 * Shortened (500->150 samples, 500->200ms warmup) so the per-command recal is
 * ~1.7s instead of ~5.5s - still a fresh zero every command, just quicker. */
#define GYRO_CALIB_SAMPLES              150U
#define GYRO_CALIB_DELAY_MS             10U
#define GYRO_CALIB_WARMUP_MS            200U

/* 1: recalibrate, use command preparation/settling waits and perform
 * successful turn-end actions (RIGHT steering holds / LEFT HEADING,OFF).
 * 0: reuse bias; skip repeated preparation stops, stillness checks, turn servo
 * settling, post-turn settling, turn-end actions and automatic sequence gaps.
 * First-use stationary calibration, target braking, faults/STOP, explicit
 * WAIT commands and per-motion yaw/encoder resets remain active.
 * Change and rebuild/reflash to select. Faster transitions alter the path
 * because steering can still be moving when drive begins. */
#ifndef GYRO_RECALIBRATE_BEFORE_EACH_COMMAND
#define GYRO_RECALIBRATE_BEFORE_EACH_COMMAND 1U
#endif
#if GYRO_RECALIBRATE_BEFORE_EACH_COMMAND != 0 && GYRO_RECALIBRATE_BEFORE_EACH_COMMAND != 1
#error "GYRO_RECALIBRATE_BEFORE_EACH_COMMAND must be 0 or 1"
#endif

/* Stationary calibration checks and optional short gyro-Z bias refresh. */
#define MOTION_STILL_SAMPLE_MS           10U
#define MOTION_STILL_REQUIRED_MS         250U
#define MOTION_STILL_TIMEOUT_MS          1000U
#define MOTION_STILL_MAX_DELTA_COUNTS    0
#define MOTOR_BRAKE_PULSE_MS             150U
#define GYRO_REZERO_WARMUP_MS            30U
#define GYRO_REZERO_SAMPLES              32U
#define GYRO_REZERO_DELAY_MS             10U

/* Correct the factory nominal gyro scale here after a measured 360-degree
 * calibration.  Example: physical 360 / reported 352 = 1.0227. */
#define GYRO_Z_SCALE                     1.0000f

#if CMD_SERVO_LEFT_US >= SERVO_CENTER_US || SERVO_CENTER_US >= CMD_SERVO_RIGHT_US
#error "Servo calibration must satisfy LEFT < CENTER < RIGHT"
#endif

#endif /* ROBOT_TUNING_H */
