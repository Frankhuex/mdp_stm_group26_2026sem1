#include "main.h"
#include "drivers/robot_hw.h"
#include "tests/component/test_motor.h"

#include <string.h>

extern UART_HandleTypeDef huart3;

#define MOTOR_TEST_PWM       3500U
#define ONE_WHEEL_RUN_MS     1200U
#define BOTH_WHEEL_RUN_MS    1800U
#define BETWEEN_TESTS_MS      800U

static void uart_send(const char *text)
{
    if (text == NULL) return;
    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HAL_MAX_DELAY);
}

static void stop_and_wait(void)
{
    RobotHW_MotorStop();
    HAL_Delay(BETWEEN_TESTS_MS);
}

void Test_Motor(void)
{
    RobotHW_StartMotionPeripherals();
    RobotHW_MotorStop();
    RobotHW_ServoCenter();

    uart_send("\r\n========================================\r\n");
    uart_send(" COMPONENT TEST - LEFT / RIGHT MOTORS\r\n");
    uart_send("========================================\r\n");
    uart_send("Raise the drive wheels off the floor.\r\n");
    uart_send("Sequence: L FWD, L BACK, R FWD, R BACK, BOTH FWD, BOTH BACK.\r\n\r\n");

    uart_send("1/6 LEFT FORWARD\r\n");
    RobotHW_MotorSetSigned((int32_t)MOTOR_TEST_PWM, 0);
    HAL_Delay(ONE_WHEEL_RUN_MS);
    stop_and_wait();

    uart_send("2/6 LEFT BACKWARD\r\n");
    RobotHW_MotorSetSigned(-(int32_t)MOTOR_TEST_PWM, 0);
    HAL_Delay(ONE_WHEEL_RUN_MS);
    stop_and_wait();

    uart_send("3/6 RIGHT FORWARD\r\n");
    RobotHW_MotorSetSigned(0, (int32_t)MOTOR_TEST_PWM);
    HAL_Delay(ONE_WHEEL_RUN_MS);
    stop_and_wait();

    uart_send("4/6 RIGHT BACKWARD\r\n");
    RobotHW_MotorSetSigned(0, -(int32_t)MOTOR_TEST_PWM);
    HAL_Delay(ONE_WHEEL_RUN_MS);
    stop_and_wait();

    uart_send("5/6 BOTH FORWARD\r\n");
    RobotHW_MotorSetForward(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
    HAL_Delay(BOTH_WHEEL_RUN_MS);
    stop_and_wait();

    uart_send("6/6 BOTH BACKWARD\r\n");
    RobotHW_MotorSetBackward(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
    HAL_Delay(BOTH_WHEEL_RUN_MS);
    RobotHW_MotorStop();

    uart_send("MOTOR_COMPONENT_TEST_COMPLETE\r\n");
    while (1)
    {
        HAL_Delay(1000U);
    }
}
