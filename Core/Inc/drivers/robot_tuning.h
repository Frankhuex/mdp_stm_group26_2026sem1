#ifndef ROBOT_TUNING_H
#define ROBOT_TUNING_H

/* Independent drive multipliers, 1..200 percent. Braking/servo are unscaled. */
#ifndef STRAIGHT_SPEED_SCALE_PERCENT
#define STRAIGHT_SPEED_SCALE_PERCENT    155U
#endif
#ifndef TURN_SPEED_SCALE_PERCENT
#define TURN_SPEED_SCALE_PERCENT        100U
#endif
#if STRAIGHT_SPEED_SCALE_PERCENT < 1 || STRAIGHT_SPEED_SCALE_PERCENT > 200
#error "STRAIGHT_SPEED_SCALE_PERCENT must be between 1 and 200"
#endif
#if TURN_SPEED_SCALE_PERCENT < 1 || TURN_SPEED_SCALE_PERCENT > 200
#error "TURN_SPEED_SCALE_PERCENT must be between 1 and 200"
#endif

/* Shared mechanical calibration: edit here, never in individual controllers.
 * Larger servo pulse steers RIGHT. Hold 1725 us for the next 60 cm test
 * with gyro steering enabled and zero extra straight trim. */
#define SERVO_CENTER_US                 1717U
#define CMD_SERVO_LEFT_US               1000U
#define CMD_SERVO_RIGHT_US              2600U

/* Steering backlash take-up: discrete servo positioning (centring before a
 * straight, calibration) approaches the target from the LOW side via a pre-move
 * this far below it, so the mechanical slop is always loaded the same way and
 * the straight-ahead angle is repeatable after any turn. Set to the observed
 * hysteresis (~120us here). Does NOT apply to the per-tick gyro-PID updates. */
#define SERVO_BACKLASH_US               120U
#define SERVO_TAKEUP_SETTLE_MS          60U
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

/* Independent runtime policies. All defaults preserve the previous fast mode.
 * Target braking, turn-start servo wait, STOP and first-use calibration remain. */
/* Refresh gyro bias before each motion. First-use calibration is mandatory. */
#ifndef GYRO_RECALIBRATE_BEFORE_EACH_COMMAND
#define GYRO_RECALIBRATE_BEFORE_EACH_COMMAND 0U
#endif
#if GYRO_RECALIBRATE_BEFORE_EACH_COMMAND != 0 && GYRO_RECALIBRATE_BEFORE_EACH_COMMAND != 1
#error "GYRO_RECALIBRATE_BEFORE_EACH_COMMAND must be 0 or 1"
#endif

/* Additional motion preparation brake/centering; calibration always stops. */
#ifndef MOTION_PREPARE_STOP_ENABLE
#define MOTION_PREPARE_STOP_ENABLE 1U
#endif
#if MOTION_PREPARE_STOP_ENABLE != 0 && MOTION_PREPARE_STOP_ENABLE != 1
#error "MOTION_PREPARE_STOP_ENABLE must be 0 or 1"
#endif

/* Additional stationary check; calibration always checks stillness. */
#ifndef MOTION_STILL_CHECK_ENABLE
#define MOTION_STILL_CHECK_ENABLE 1U
#endif
#if MOTION_STILL_CHECK_ENABLE != 0 && MOTION_STILL_CHECK_ENABLE != 1
#error "MOTION_STILL_CHECK_ENABLE must be 0 or 1"
#endif

/* Wait POST_STOP_SETTLE_MS (150 ms) after turn braking. */
#ifndef TURN_POST_STOP_SETTLE_ENABLE
#define TURN_POST_STOP_SETTLE_ENABLE 1U
#endif
#if TURN_POST_STOP_SETTLE_ENABLE != 0 && TURN_POST_STOP_SETTLE_ENABLE != 1
#error "TURN_POST_STOP_SETTLE_ENABLE must be 0 or 1"
#endif

/* Successful right turn: 1500 us hold then centre hold. */
#ifndef TURN_RIGHT_RETURN_ENABLE
#define TURN_RIGHT_RETURN_ENABLE 1U
#endif
#if TURN_RIGHT_RETURN_ENABLE != 0 && TURN_RIGHT_RETURN_ENABLE != 1
#error "TURN_RIGHT_RETURN_ENABLE must be 0 or 1"
#endif

/* Successful left turn automatically applies HEADING,OFF. */
#ifndef TURN_LEFT_HEADING_OFF_ENABLE
#define TURN_LEFT_HEADING_OFF_ENABLE 1U
#endif
#if TURN_LEFT_HEADING_OFF_ENABLE != 0 && TURN_LEFT_HEADING_OFF_ENABLE != 1
#error "TURN_LEFT_HEADING_OFF_ENABLE must be 0 or 1"
#endif

/* Reset heading history after successful turns, independently of steering holds. */
#ifndef TURN_END_HEADING_RESET_ENABLE
#define TURN_END_HEADING_RESET_ENABLE 1U
#endif
#if TURN_END_HEADING_RESET_ENABLE != 0 && TURN_END_HEADING_RESET_ENABLE != 1
#error "TURN_END_HEADING_RESET_ENABLE must be 0 or 1"
#endif

/* Apply configured sequence gaps; diagnostic observers retain their waits. */
#ifndef SEQUENCE_COMMAND_GAP_ENABLE
#define SEQUENCE_COMMAND_GAP_ENABLE 1U
#endif
#if SEQUENCE_COMMAND_GAP_ENABLE != 0 && SEQUENCE_COMMAND_GAP_ENABLE != 1
#error "SEQUENCE_COMMAND_GAP_ENABLE must be 0 or 1"
#endif

/* Stationary calibration checks and optional short gyro-Z bias refresh. */
#define MOTION_STILL_SAMPLE_MS           10U
#define MOTION_STILL_REQUIRED_MS         100U
/* Keep a longer stationary window for gyro bias calibration. */
#define MOTION_CALIB_STILL_REQUIRED_MS   250U
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
