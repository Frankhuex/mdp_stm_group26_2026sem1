#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

/*
 * SELECT WHAT TO RUN
 * ==================
 * Change only ACTIVE_TEST, then Build + Flash.
 *
 * The names below match the test file names directly.
 * Example:
 *   ACTIVE_TEST TEST_MOTOR          -> tests/component/test_motor.c
 *   ACTIVE_TEST TEST_IR             -> tests/component/test_ir.c
 *   ACTIVE_TEST TEST_IR_CALIBRATION -> tests/calibration/test_ir_calibration.c
 *   ACTIVE_TEST TEST_STRAIGHT_CONTROL -> tests/calibration/test_straight_control.c
 *   ACTIVE_TEST TEST_TURN_ANGLE       -> tests/calibration/test_turn_angle.c
 *   ACTIVE_TEST TEST_HARDCODE_COMMAND_EXECUTOR -> tests/test_hardcode_command_executor.c
 *   ACTIVE_TEST TEST_FASTEST_CAR      -> tests/test_fastest_car.c
 *
 * TEST_ULTRASONIC runs the standalone working reference test unchanged.
 * Select ASSESSMENT_COMMAND_INTERFACE to run the UART motion commands.
 */

#define TEST_NONE                    0

/* Individual component tests */
#define TEST_MOTOR                   1   /* test_motor.c */
#define TEST_ENCODER                 2   /* test_encoder.c */
#define TEST_SERVO                   3   /* test_servo.c */
#define TEST_IMU                     4   /* test_imu.c */
#define TEST_ACCEL                   5   /* test_accel.c */
#define TEST_UART                    6   /* test_uart.c */
#define TEST_OLED                    7   /* test_oled.c */
#define TEST_IR                      8   /* test_ir.c: live RAW/mV/cm */
#define TEST_ULTRASONIC              9   /* test_ultrasonic.c */

/* Calibration tests */
#define TEST_GYRO_CALIBRATION       20   /* test_gyro_calibration.c */
#define TEST_IR_CALIBRATION         21   /* test_ir_calibration.c */
#define TEST_MIN_TURN_RADIUS        22   /* test_min_turn_radius.c */
#define TEST_STRAIGHT_CONTROL       23   /* test_straight_control.c */
#define TEST_TURN_ANGLE             24   /* test_turn_angle.c */

/* Hardcoded command-array test (runs once after boot). */
#define TEST_HARDCODE_COMMAND_EXECUTOR 25 /* tests/test_hardcode_command_executor.c */
#define TEST_APPROACH_CONTROL        26 /* tests/calibration/test_approach_control.c */
#define TEST_FASTEST_CAR             27 /* tests/test_fastest_car.c */
#define TEST_MOTION_SEQUENCE         28 /* tests/calibration/test_motion_sequence.c */

/* Route-heading demonstration: change CASE and ON for the A/B floor tests.
 * 1: F40,L20/90,B10; 2: F40,R35/90,B10;
 * 3: F30,L20/90,F20,R35/90,B10; 4: F40,B40.
 * ON=0 still uses local gyro PID, but does not carry error between commands. */
#ifndef MOTION_SEQUENCE_CASE
#define MOTION_SEQUENCE_CASE          1
#endif
#ifndef MOTION_SEQUENCE_HEADING_ON
#define MOTION_SEQUENCE_HEADING_ON    1
#endif
#define MOTION_SEQUENCE_STARTUP_MS    5000U
#define MOTION_SEQUENCE_PAUSE_MS      10000U

/* Dedicated ultrasonic approach test settings. */
#define APPROACH_TEST_TARGET_CM       50U
#define APPROACH_TEST_START_DELAY_MS  3000U

/* Final integrated robot command interface */
#define ASSESSMENT_COMMAND_INTERFACE 50 /* assessment/command_interface.c */

/* =========================================================
 * CHANGE ONLY THIS LINE
 * ========================================================= */
#define ACTIVE_TEST ASSESSMENT_COMMAND_INTERFACE

/* Optional turn -> reverse diagnosis using the existing one-shot executor.
 * Select ACTIVE_TEST TEST_HARDCODE_COMMAND_EXECUTOR to run automatically.
 * 0 = editable executor array (currently the closed turn/reverse route);
 * 1 = stationary steering return; 2 = reverse alone;
 * 3 = right turn/heading ON; 4 = right turn/OFF only before reverse;
 * 5 = left turn/heading ON; 6 = left turn/OFF only before reverse.
 * Diagnostic cases start 10 seconds after initialization. Reset repeats.
 * See docs/TURN_REVERSE_DIAGNOSIS.md before running a moving case. */
#ifndef TURN_REVERSE_DIAGNOSTIC_CASE
#define TURN_REVERSE_DIAGNOSTIC_CASE 0
#endif

#endif /* TEST_CONFIG_H */
