#include "main.h"
#include "drivers/robot_hw.h"
#include "tests/component/test_servo.h"

#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart3;

#define SERVO_TEST_RIGHT_US (SERVO_CENTER_US + 100U)
#define SERVO_TEST_LEFT_US  (SERVO_CENTER_US - 100U)
#define SERVO_HOLD_MS       1500U

static void uart_send(const char *text)
{
    if (text == NULL) return;
    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HAL_MAX_DELAY);
}

static void show_servo(const char *position, uint32_t pulse_us)
{
    char line[64];
    snprintf(line, sizeof(line), "SERVO %s %luus\r\n", position, (unsigned long)pulse_us);
    uart_send(line);
    RobotHW_ServoSetUs(pulse_us);
}

void Test_Servo(void)
{
    RobotHW_StartMotionPeripherals();
    RobotHW_MotorStop();

    uart_send("\r\n========================================\r\n");
    uart_send(" COMPONENT TEST - STEERING SERVO\r\n");
    uart_send("========================================\r\n");
    uart_send("Motor switch OFF. Servo power ON.\r\n");
    uart_send("The servo will move a small amount around center.\r\n\r\n");

    while (1)
    {
        show_servo("CENTER", RobotHW_ServoCenterUs());
        HAL_Delay(SERVO_HOLD_MS);

        show_servo("RIGHT", SERVO_TEST_RIGHT_US);
        HAL_Delay(SERVO_HOLD_MS);

        show_servo("CENTER", RobotHW_ServoCenterUs());
        HAL_Delay(SERVO_HOLD_MS);

        show_servo("LEFT", SERVO_TEST_LEFT_US);
        HAL_Delay(SERVO_HOLD_MS);
    }
}
