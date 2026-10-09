#include "main.h"
#include "tests/component/test_encoder.h"

#include <stdio.h>
#include <string.h>

/*
 * MDP ENCODER TEST
 *
 * Hardware mapping:
 *
 * Left / MOTORA encoder:
 *   PA15 -> TIM2_CH1
 *   PB3  -> TIM2_CH2
 *
 * Right / MOTORB encoder:
 *   PB4 -> TIM3_CH1
 *   PB5 -> TIM3_CH2
 *
 * Confirmed physical encoder direction:
 *
 *   LEFT forward:
 *       raw TIM2 count = positive
 *
 *   RIGHT forward:
 *       raw TIM3 count = negative
 *
 * Therefore we normalize the right encoder by multiplying
 * the raw right count by -1.
 *
 * SOFTWARE CONVENTION:
 *
 *   Forward:
 *       Left  = positive
 *       Right = positive
 *
 *   Backward:
 *       Left  = negative
 *       Right = negative
 *
 * Encoder calibration from 3 x 10-revolution tests:
 *
 *   Left  ≈ 1560.13 counts / wheel revolution
 *   Right ≈ 1559.37 counts / wheel revolution
 *
 * Approximate common value:
 *   ≈ 1560 counts / wheel revolution
 *
 * UART output:
 *   USART3
 *   115200 baud
 *   8 data bits
 *   no parity
 *   1 stop bit
 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;


/* =========================================================
 * UART PRINT FUNCTION
 * =========================================================
 */

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)text,
                      (uint16_t)strlen(text),
                      HAL_MAX_DELAY);
}


/* =========================================================
 * ENCODER READING FUNCTIONS
 * =========================================================
 */

/*
 * Read LEFT encoder.
 *
 * Raw TIM2 direction already matches our software convention:
 *
 * Forward  -> positive
 * Backward -> negative
 */
static int16_t Encoder_GetLeft(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}


/*
 * Read RIGHT encoder.
 *
 * Raw TIM3 direction is opposite:
 *
 * Raw:
 *   Forward  -> negative
 *   Backward -> positive
 *
 * Multiply by -1 so our software convention becomes:
 *
 *   Forward  -> positive
 *   Backward -> negative
 */
static int16_t Encoder_GetRight(void)
{
    int16_t raw_right;

    raw_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    return -raw_right;
}


/* =========================================================
 * ENCODER TEST
 * =========================================================
 */

void Test_Encoder(void)
{
    char msg[120];

    /*
     * Start both hardware encoder timers.
     */
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Reset both encoder counters to zero.
     */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);


    /*
     * Print test information to PuTTY.
     */
    UART_Print("\r\n");
    UART_Print("========================================\r\n");
    UART_Print("        MDP ENCODER TEST\r\n");
    UART_Print("========================================\r\n");

    UART_Print("Left encoder  : TIM2\r\n");
    UART_Print("Right encoder : TIM3\r\n");

    UART_Print("\r\n");

    UART_Print("Normalized direction:\r\n");
    UART_Print("Forward  = positive\r\n");
    UART_Print("Backward = negative\r\n");

    UART_Print("\r\n");

    UART_Print("Turn the drive wheels by hand.\r\n");
    UART_Print("\r\n");


    /*
     * Continuously display encoder counts.
     */
    while (1)
    {
        int16_t left_count;
        int16_t right_count;

        /*
         * These are NORMALIZED readings.
         *
         * Therefore:
         *
         * Forward:
         * Left  +
         * Right +
         *
         * Backward:
         * Left  -
         * Right -
         */
        left_count  = Encoder_GetLeft();
        right_count = Encoder_GetRight();


        snprintf(msg,
                 sizeof(msg),
                 "Left:%7d   Right:%7d\r\n",
                 (int)left_count,
                 (int)right_count);
        UART_Print(msg);

                HAL_Delay(250);
            }
        }
