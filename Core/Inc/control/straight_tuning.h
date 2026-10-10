#ifndef STRAIGHT_TUNING_H
#define STRAIGHT_TUNING_H

#include "drivers/robot_tuning.h"
#include "test_config.h"

/*
 * STRAIGHT CONTROL TUNING
 * =======================
 * These are the main values to re-check when floor friction, tyre grip,
 * payload, battery condition, or steering behaviour changes.
 *
 * Use tests/calibration/test_straight_control.c to validate changes.
 */

/* 1: test the mechanical servo center without trim or gyro steering.
 * Only takes effect with ACTIVE_TEST == TEST_STRAIGHT_CONTROL.
 * Set to 0 for the normal straight test. Other test/command modes always
 * use the original tuning below. Gyro measurement and motor sync stay on. */
#define STRAIGHT_CENTER_CALIBRATION     0U
#define STRAIGHT_CENTER_CALIBRATION_ACTIVE \
    (STRAIGHT_CENTER_CALIBRATION && ACTIVE_TEST == TEST_STRAIGHT_CONTROL)

/* Shared encoder/servo calibration lives in drivers/robot_tuning.h. */
/* Straight-only feed-forward trim relative to the shared mechanical centre.
 * Positive pulse offset steers RIGHT in either gear (not a yaw correction).
 * Fixed-pulse forward trials: 1765 us ended +0.832 deg, 1767 us +0.700 deg;
 * an earlier 1770 us run ended -1.291 deg. Trial 1767 us (centre +42) as
 * the forward baseline WITH gyro feedback; this is not a verified neutral.
 * Apply the bias before yaw error builds. Reverse retains +8 us.
 * Forward IR/UNTILUS also share this baseline. Validate on the floor. */
#if STRAIGHT_CENTER_CALIBRATION_ACTIVE
#define STRAIGHT_FORWARD_TRIM_US        0
#define STRAIGHT_REVERSE_TRIM_US        0
#else
#define STRAIGHT_FORWARD_TRIM_US        42
#define STRAIGHT_REVERSE_TRIM_US        8
#endif

#if (SERVO_CENTER_US + STRAIGHT_FORWARD_TRIM_US <= CMD_SERVO_LEFT_US) || \
    (SERVO_CENTER_US + STRAIGHT_FORWARD_TRIM_US >= CMD_SERVO_RIGHT_US) || \
    (SERVO_CENTER_US + STRAIGHT_REVERSE_TRIM_US <= CMD_SERVO_LEFT_US) || \
    (SERVO_CENTER_US + STRAIGHT_REVERSE_TRIM_US >= CMD_SERVO_RIGHT_US)
#error "Straight trim must remain inside servo endpoints"
#endif

/* Profile selection: 10..20 cm = SHORT, 21..200 cm = LONG. */
#define SHORT_PROFILE_MAX_CM            20
#define MIN_COMMAND_DISTANCE_CM         10
#define MAX_COMMAND_DISTANCE_CM         200
/* Floor observation: GOCM stops about 2 cm short with the current brake.
 * Add travel before braking, in both gears and both distance profiles. */
#define GOCM_BRAKE_DELAY_CM              2.0f

/* SHORT profile: most floor-sensitive values for 10..20 cm. */
#define SHORT_CALIBRATION_OFFSET_CM     (-2.5f)
/* 20 cm floor trial: 0 -> 21 -> 1 cm means forward travelled 21 cm,
 * reverse 20 cm. Try a 1 cm earlier forward stop; retain reverse tuning.
 * These allowances affect the whole 10..20 cm short profile. */
#define SHORT_FORWARD_OVERSHOOT_COMP_CM 1.5f
#define SHORT_REVERSE_OVERSHOOT_COMP_CM 0.5f
#define SHORT_RAMP_START_PWM            2950U
#define SHORT_RAMP_PEAK_PWM             2980U
#define SHORT_RAMP_END_PWM              2350U
#define SHORT_KICKSTART_PWM             3200U
#define SHORT_KICKSTART_DURATION_MS     100U

/* LONG profile: all commanded distances above 20 cm. */
/* 40 cm floor trial at 0.235 rev: 0 -> 41.5 -> 1 cm means forward
 * travelled 41.5 cm and reverse 40.5 cm. Try earlier stops by nominally
 * 1.53 cm forward (0.075 rev) and 0.51 cm reverse (0.025 rev).
 * Applies to ALL distances above 20 cm: recheck 30/40/60 cm in both gears. */
#define LONG_FORWARD_REV_ERROR_COMPENSATION 0.310f
#define LONG_REVERSE_REV_ERROR_COMPENSATION 0.260f
#define LONG_BASE_PWM                   3300U
#define LONG_SLOW_PWM                   2400U
#define LONG_SLOWDOWN_START_REV         0.80f

/* Left/right speed synchronization. Error is left-equivalent encoder counts
 * per CONTROL_PERIOD_MS, normalized by calibrated cm and measured dt.
 * SYNC_KP units: PWM per count per nominal interval (no extra x1000). */
#define SYNC_KP                         2.5f
#define SYNC_KI                         10.0f
#define SYNC_ERROR_FILTER_ALPHA         0.30f
#define SYNC_INT_LIMIT_COUNTS_S         25.0f
#define MAX_SYNC_CORRECTION             350

/* Gyro / control loop. */
#define CONTROL_PERIOD_MS               20U
#define GYRO_IIR_ALPHA                  0.10f
/* Allow an inherited route-heading correction to settle before drive. */
#define STRAIGHT_HEADING_STEER_SETTLE_MS 100U

/* Gyro yaw -> steering servo PID (gyro rate is the derivative term). */
#if STRAIGHT_CENTER_CALIBRATION_ACTIVE
#define FORWARD_KP_STEER_US_PER_DEG     0.0f
#define FORWARD_KI_STEER_US_PER_DEG_S   0.0f
#define FORWARD_KD_STEER_US_PER_DPS     0.0f
#define REVERSE_KP_STEER_US_PER_DEG     0.0f
#define REVERSE_KI_STEER_US_PER_DEG_S   0.0f
#define REVERSE_KD_STEER_US_PER_DPS     0.0f
#else
/* Retain forward P=30, I=0.6, D=2 while testing the revised forward baseline.
 * Reverse retains its previous gains. Needs floor validation.
 * Shared forward IR/ultrasonic travel also uses these forward gains. */
#define FORWARD_KP_STEER_US_PER_DEG     30.0f
#define FORWARD_KI_STEER_US_PER_DEG_S   0.6f
#define FORWARD_KD_STEER_US_PER_DPS     2.0f
#define REVERSE_KP_STEER_US_PER_DEG     14.0f
#define REVERSE_KI_STEER_US_PER_DEG_S   0.6f
#define REVERSE_KD_STEER_US_PER_DPS     2.0f
#endif

/* Buffered diagnostics: no UART transmission in the control loop. */
#define STRAIGHT_TRACE_PERIOD_MS        100U
#define STRAIGHT_TRACE_CAPACITY         128U
#define YAW_INT_LIMIT_DEG_S             15.0f
#define YAW_DEADBAND_DEG                0.15f
#define GYRO_RATE_DEADBAND_DPS           0.08f
/* Minimum nonzero correction, NOT a zero-output deadband. A 12 us floor
 * made even tiny PID commands jump between opposite steering offsets. */
#define STEER_DEADBAND_US               4.0f
#define MAX_STEER_CORRECTION_US         250.0f
#define MIN_STEER_US                    CMD_SERVO_LEFT_US
#define MAX_STEER_US                    CMD_SERVO_RIGHT_US

/* One-command forward-only calibration: hold a fixed pulse while gyro logs.
 * Normal GOCM/IR/US commands continue to use gyro steering. */
#define STRAIGHT_CAL_MAX_DISTANCE_CM    50
#define STRAIGHT_CAL_MAX_OFFSET_US      100

/* UNTILUS: use the already-proven long-straight cruise PWM.  The stronger,
 * encoder-confirmed kick gets both wheels out of their static-friction region
 * before the normal wheel synchronizer takes over.
 * Threshold is sensor-to-object clearance, NOT commanded travel distance. */
#define US_MIN_THRESHOLD_CM              5
#define US_MAX_THRESHOLD_CM              200
#define US_CRUISE_PWM                    3500U
#define US_APPROACH_PWM                  2400U
#define US_FINAL_APPROACH_PWM            1800U
#define US_FINAL_APPROACH_MARGIN_MM       200U
/* Keep braking until both encoders stay quiet, with a bounded release time. */
#define US_BRAKE_QUIET_MS                  60U
#define US_BRAKE_MAX_MS                   600U
/* Floor observation: both directions travel about 5 cm after the trigger. */
#define US_FORWARD_STOP_LEAD_MM           50U
#define US_REVERSE_STOP_LEAD_MM           50U
#define US_NEAR_START_PWM                2600U
#define US_KICKSTART_PWM                 4000U
#define US_KICKSTART_MIN_MS                120U
#define US_KICKSTART_MAX_MS               400U
#define US_KICKSTART_MIN_COUNTS             6
#define US_START_CONFIRM_SAMPLES            2U
#define US_SLOW_MARGIN_MM                350U
#define US_SPEED_HYSTERESIS_MM            50U
#define US_PWM_SLEW_PER_SECOND           4200U

/* Close the accumulated left/right path error as well as the instantaneous
 * wheel-speed error.  This is only added in ultrasonic approach mode, where
 * the lower final speed makes a small motor imbalance more visible. */
#define US_PATH_SYNC_KP_PWM_PER_CM       80.0f

/* Every threshold crossing needs consecutive, mutually consistent echoes. */
#define US_STOP_CONFIRM_SAMPLES             2U
#define US_STOP_CONFIRM_MAX_SPREAD_MM       50U
/* Older readings become infinity; this is not a motion timeout. */
#define US_SAMPLE_MAX_AGE_MS             200U
#define US_MOTION_TIMEOUT_MS             30000U
#define US_MAX_TRAVEL_CM                 200U

/* Side IR commands: stop on strictly greater distance (FAR/failure = infinity). */
#define IR_COMMAND_CONFIRM_SAMPLES        2U
#define IR_COMMAND_SAMPLE_MS             40U
#define IR_COMMAND_CRUISE_PWM            3300U
#define IR_COMMAND_KICK_PWM              4000U
#define IR_COMMAND_KICK_MS               120U
#define IR_COMMAND_TIMEOUT_MS           30000U
#define IR_COMMAND_MAX_TRAVEL_CM         200U

#endif /* STRAIGHT_TUNING_H */
