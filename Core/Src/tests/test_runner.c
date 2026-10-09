#include "tests/test_runner.h"
#include "tests/test_hardcode_command_executor.h"
#include "tests/test_fastest_car.h"
#include "test_config.h"

#include "tests/component/test_motor.h"
#include "tests/component/test_encoder.h"
#include "tests/component/test_servo.h"
#include "tests/component/test_imu.h"
#include "tests/component/test_accel.h"
#include "tests/component/test_uart.h"
#include "tests/component/test_oled.h"
#include "tests/component/test_ir.h"
#include "tests/component/test_ultrasonic.h"

#include "tests/calibration/test_gyro_calibration.h"
#include "tests/calibration/test_ir_calibration.h"
#include "tests/calibration/test_min_turn_radius.h"
#include "tests/calibration/test_straight_control.h"
#include "tests/calibration/test_turn_angle.h"
#include "tests/calibration/test_approach_control.h"
#include "tests/calibration/test_motion_sequence.h"

#include "assessment/command_interface.h"

void Test_RunSelected(void)
{
#if ACTIVE_TEST == TEST_MOTOR
    Test_Motor();

#elif ACTIVE_TEST == TEST_ENCODER
    Test_Encoder();

#elif ACTIVE_TEST == TEST_SERVO
    Test_Servo();

#elif ACTIVE_TEST == TEST_IMU
    Test_IMU();

#elif ACTIVE_TEST == TEST_ACCEL
    Test_Accel();

#elif ACTIVE_TEST == TEST_UART
    Test_UART();

#elif ACTIVE_TEST == TEST_OLED
    Test_OLED();

#elif ACTIVE_TEST == TEST_IR
    Test_IR();

#elif ACTIVE_TEST == TEST_ULTRASONIC
    Test_Ultrasonic();

#elif ACTIVE_TEST == TEST_GYRO_CALIBRATION
    Test_GyroCalibration();

#elif ACTIVE_TEST == TEST_IR_CALIBRATION
    Test_IRCalibration();

#elif ACTIVE_TEST == TEST_MIN_TURN_RADIUS
    Test_MinTurnRadius();

#elif ACTIVE_TEST == TEST_STRAIGHT_CONTROL
    Test_StraightControl();

#elif ACTIVE_TEST == TEST_TURN_ANGLE
    Test_TurnAngle();

#elif ACTIVE_TEST == TEST_APPROACH_CONTROL
    Test_ApproachControl();

#elif ACTIVE_TEST == TEST_MOTION_SEQUENCE
    Test_MotionSequence();

#elif ACTIVE_TEST == TEST_HARDCODE_COMMAND_EXECUTOR
    Test_HardcodeCommandExecutor();

#elif ACTIVE_TEST == TEST_FASTEST_CAR
    Test_FastestCar();

#elif ACTIVE_TEST == ASSESSMENT_COMMAND_INTERFACE
    CommandInterface_Run();

#elif ACTIVE_TEST == TEST_NONE
    /* No pre-RTOS test selected. Continue into normal scheduler startup. */

#else
#error "Invalid ACTIVE_TEST in Core/Inc/test_config.h"
#endif
}
