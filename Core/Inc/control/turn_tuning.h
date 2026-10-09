#ifndef TURN_TUNING_H
#define TURN_TUNING_H

#include "drivers/robot_tuning.h"

/*
 * TURN CONTROL TUNING
 * ===================
 * Re-check these values when floor friction / tyre grip changes.
 * Use tests/calibration/test_turn_angle.c to validate actual turn angles.
 */

/* Initial heavier-chassis trial, not yet floor-validated. Raise drive effort
 * at every stage while keeping KICK > CRUISE > MEDIUM > SLOW. The longer
 * kick applies to all angles; short turns limit it by time and progress.
 * Stop checks always take precedence.
 * Recheck left/right 90/180 deg and small turns with the actual payload. */
#define TURN_MOTOR_PWM                 3600U
#define TURN_MEDIUM_PWM                3300U
#define TURN_SLOW_PWM                  3100U
#define TURN_KICKSTART_PWM             4000U
#define TURN_KICKSTART_DURATION_MS      120U
#define TURN_SHORT_KICK_DURATION_MS      60U
#define TURN_SHORT_KICK_PROGRESS_DEG      2.0f
/* Normal command-turn brake hold, sampled non-blockingly. */
#define TURN_BRAKE_HOLD_MS              150U

/* After a successful RIGHT command, reproduce the user's manual return:
 * 1500 us for 500 ms, then the shared centre (1725 us) for 500 ms. Drive PWM is
 * already zero. Each steering action clears heading history, like STEER,US.
 * Successful LEFT commands instead apply HEADING,OFF before DONE. */
#define TURN_RIGHT_RECENTER_US          1500U
#define TURN_RIGHT_RECENTER_HOLD_MS      500U
#define TURN_RIGHT_CENTER_HOLD_MS        500U

#if TURN_RIGHT_RECENTER_US < CMD_SERVO_LEFT_US || TURN_RIGHT_RECENTER_US >= SERVO_CENTER_US
#error "Right-turn return pulse must be left of centre and within servo endpoints"
#endif

/* Reduce speed as the remaining yaw becomes small.  The final stop predicts
 * coast from the measured gyro rate instead of assuming coast is a fixed
 * percentage of every requested angle. */
#define TURN_SLOWDOWN_REMAINING_DEG     30.0f
#define TURN_FINAL_REMAINING_DEG         9.0f
#define TURN_SMALL_ANGLE_DEG            15.0f
/* Forward LEFT R20/A90 trial: brake at 69.46 deg, rate 101.99 dps,
 * then 12.66 deg more yaw. Effective stopping lead ~= 12.66/101.99
 * = 0.124 s; try 0.13 s instead of 0.22 s. Recheck angle/radius.
 * Follow-up forward measured ~90 deg physically, 91.48 deg gyro. */
#define TURN_LEFT_COAST_TIME_S           0.13f
/* Reverse LEFT R20/A-90: brake at 69.07 deg and 112.88 dps, then
 * 13.91 deg more yaw. Effective lead ~= 0.123 s; try 0.13 instead
 * of 0.22 s to address the early-stop shortfall. Floor validation needed. */
/* 2026-10-05: imported stopping lead from the supplied turn_tuning.h. */
#define TURN_LEFT_REVERSE_COAST_TIME_S   0.12f
/* RIGHT R35/A90 at 2569 us: forward post-stop 13.49 deg / 82.41 dps
 * ~= 0.164 s. Use 0.165 with the jointly calculated target scale below.
 * These leads apply to |angle| >= 90; short RIGHT has its own lead. */
#define TURN_RIGHT_COAST_TIME_S          0.165f
/* Return ended +2.20 deg LEFT. Increasing the old 0.130 s lead by
 * 2.20/86.19 ~= 0.0255 s aims to remove this excess reverse rotation.
 * Measured post-stop/rate was 12.80/86.19 ~= 0.149 s; the 0.155 s trial
 * also includes observed stop-decision overshoot. Recheck on the floor. */
/* 2026-10-05: imported stopping lead from the supplied turn_tuning.h.
 * Applies to |angle| >= 90; short RIGHT retains its independent lead. */
#define TURN_RIGHT_REVERSE_COAST_TIME_S  0.145f
#define TURN_STOP_TOLERANCE_DEG          0.7f

/* RIGHT commands with |angle| below 90 degrees, forward and reverse. Initial trial values:
 * bounded high-power kick, earlier final speed, independent stopping lead.
 * Keep lead at the existing value until measured floor errors are available.
 * Increase lead for overshoot; decrease for undershoot. Units are seconds,
 * not the duration of the motor brake pulse. */
#define TURN_RIGHT_SHORT_LIMIT_DEG      90
#define TURN_RIGHT_SHORT_CRUISE_PWM     TURN_MOTOR_PWM
#define TURN_RIGHT_SHORT_SLOW_PWM       TURN_SLOW_PWM
#define TURN_RIGHT_SHORT_FINAL_DEG      20.0f
#define TURN_RIGHT_SHORT_LEAD_TIME_S     0.17f

/* Command-mode yaw target magnitude = corrected local angle * side/gear scale.
 * Supports undershoot (>1) and overshoot (<1) compensation.
 * Valid range: (0, TURN_TARGET_SCALE_MAX]. Steering scale is independent. */
#define TURN_TARGET_SCALE_MAX           2.000f
/* Forward LEFT R20/A90 after servo tightening: scale 1.017 gave post-turn
 * heading 93.258/93.111 deg; scale 0.982 then gave final turn yaw 89.085 deg.
 * Interpolate between those settings for 90 deg:
 * 0.982 + (90-89.085)*(1.017-0.982)/(93.1845-89.085) ~= 0.990.
 * Keep radius, speed and braking lead fixed. This is a trial estimate from
 * gyro readings; verify physical alignment and repeatability on the floor.
 * Other angles/radii need validation. Reverse uses its own scale below. */
/* 2026-10-04 instrumented: forward LEFT R20/A90 landed yaw_meas ~85.9 gyro at
 * scale 0.950 -> raise to 0.996 to land ~90 (trusting the gyro, no protractor). */
#define TURN_LEFT_TARGET_SCALE           0.996f
/* Reverse LEFT R20/A-90 trial: chassis heading was confirmed LEFT of start;
 * the rear midpoint being RIGHT was a separate position error. At scale
 * 0.963, target=87.93, progress=87.24 and route heading=+4.13 deg.
 * Add that remaining heading to the previous target, including its existing
 * stopping shortfall: 0.963 * (87.93 + 4.13) / 87.93 ~= 1.008.
 * Recheck on the floor; this does not calibrate reverse radius/position.
 * Applies to all LEFT reverse angles/radii; only R20/A-90 informed this trial. */
/* 2026-10-04 instrumented: reverse LEFT R20/A-90 landed yaw_meas ~90.8 gyro at
 * scale 1.008 -> lower to 0.999 to land ~90 (matches forward -> retrace cancels). */
#define TURN_LEFT_REVERSE_TARGET_SCALE   0.999f
/* Earlier RIGHT R35/A90 measured 180-84 = 96 deg physically, 93.77 gyro,
 * with target 90.99. Desired final gyro ~= 93.77*90/96 = 87.91 deg.
 * Account for the simultaneous lead increase (0.130 -> 0.165) once:
 * new target = 90.99 + (87.91-93.77) + 82.41*(0.165-0.130) ~= 88.02.
 * Scale = 88.02/90 = 0.978. Local linear trial; physical readings have
 * varied. Applies to all RIGHT forward turns; recheck other angles/radii.
 * Follow-up physical outbound angle looked 90 deg; retain this setting.
 * Reverse has its own trim after route-heading correction. */
/* 2026-10-04 instrumented: forward RIGHT R35/A90 landed yaw_meas ~86.9 gyro at
 * scale 0.978 -> raise to 1.013 (0.978*90/86.9) to land ~90. */
#define TURN_RIGHT_TARGET_SCALE          1.013f
/* RIGHT return trial: nose reported ~0.5 cm RIGHT and rear midpoint 1.5 cm
 * forward of S. Treat the nose displacement as a small right heading error,
 * not a measured angle. Add 1.8% (~1.6 deg on a 90-deg local target); at
 * nominal R35 this is ~1 cm more reverse arc travel near the start heading.
 * No new gyro log accompanied this observation: remeasure heading/offsets.
 * Applies to all RIGHT reverse turns; set 1.000 to undo this trim. */
/* 2026-10-04 instrumented: reverse RIGHT R35/A-90 landed yaw_meas ~90.86 gyro at
 * scale 1.018 -> lower to 1.008 (1.018*90/90.86) to land ~90 (matches forward). */
#define TURN_RIGHT_REVERSE_TARGET_SCALE  1.008f

/* Shared center/endpoints/geometry live in drivers/robot_tuning.h.
 * Pulse-span trim AFTER radius mapping. Smaller scale = gentler steering /
 * larger radius. These are initial field adjustments, not measured radii.
 * Selected by steering side for forward and reverse. Keep within (0, 1]. */
/* Radius trial: LEFT R20/A90 measured ~30 cm chord at ~90 deg (~21.2 cm R).
 * Increase 0.98 -> 1.00 to tighten steering; remeasure radius and angle. */
#define TURN_LEFT_STEER_SCALE           1.00f
/* RIGHT R35: 50 cm chord at ~89 deg implies effective R ~= 35.7 cm.
 * Small tightening trial in both gears: 0.950 -> 0.965, pulse 2556 ->
 * 2569 us at R35. This is a model-based trial, not a measured servo/radius
 * calibration. Latest 50 cm at 96 deg implies R ~= 33.64 cm if circular;
 * hold this pulse for the angle/braking trial, then remeasure the radius. */
#define TURN_RIGHT_STEER_SCALE          0.965f

/* Radius correction BEFORE servo mapping; <1 tightens, >1 widens the arc.
 * LEFT R20/A90 at steer scale 1.00 and radius scale 0.975 measured
 * 28.9 cm chord at ~90 deg (estimated R=20.435 cm). Next trial:
 * 0.975 * (20 * sqrt(2) / 28.9) ~= 0.954. Verify on the floor.
 * LEFT and RIGHT use separate forward/reverse scales.
 * These are one-radius floor trials; recheck other radii and angles.
 * Mapping saturates at the existing endpoint;
 * requested radius, command limits and yaw target are not rescaled. */
#define TURN_RADIUS_SCALE_MAX           2.000f
#define TURN_LEFT_RADIUS_SCALE          0.864f
/* Reverse-only trial for ~1 cm RIGHT return offset with near-matched yaw.
 * Try slightly tighter steering: R20 pulse 1024 -> 1013 us. Position alone
 * does not prove a radius mismatch; measure heading and both offsets again.
 * Set to 0.954 to restore the previous reverse path. Applies to all LEFT
 * reverse angles/radii; other radii have not been floor-validated. */
/* 2026-10-04: with yaw matched, fwd+rev LEFT retrace still lands ~0.7cm RIGHT
 * (reverse arc still a touch wide). 0.954->0.930 cut it ~1.0->0.7cm, so go
 * tighter again: 0.930 -> 0.900. Iterate +/-0.01; raise toward 0.93 if it
 * overshoots LEFT. */
#define TURN_LEFT_REVERSE_RADIUS_SCALE  0.9200f
#define TURN_RIGHT_RADIUS_SCALE         1.000f
/* Reverse RIGHT only: mapped radius is 10% wider, imported from the supplied
 * turn_tuning.h. Floor validation needed. */
#define TURN_RIGHT_REVERSE_RADIUS_SCALE  1.10f

/* Valid command angle range. */
#define CMD_MIN_ANGLE_DEG              5
#define CMD_MAX_ANGLE_DEG              360

/*
 * Legacy fixed motor cut-off ratios used by the minimum-radius calibration
 * routine.  Production command turns use measured yaw rate and the coast-time
 * constants above, so their stopping lead does not grow incorrectly with the
 * requested angle.
 * Select by steering side (LEFT/RIGHT), for both forward and reverse turns.
 * If turns overshoot on a slippery floor, DECREASE this ratio.
 * If turns stop short, INCREASE it slightly.
 */
#define TURN_LEFT_STOP_RATIO           0.80f
#define TURN_RIGHT_STOP_RATIO          0.92f

#endif /* TURN_TUNING_H */
