#include "main.h"
#include "tests/component/test_imu.h"

#include <stdio.h>
#include <string.h>

/*
 * AUDITED FOR THE ONBOARD ICM-20948.
 *
 * C30D wiring:
 *   PB10 -> I2C2_SCL
 *   PB11 -> I2C2_SDA
 *
 * ICM-20948:
 *   possible 7-bit addresses = 0x68 / 0x69
 *   WHO_AM_I register = 0x00
 *   expected WHO_AM_I = 0xEA
 *
 * Output: USART3, 115200 8-N-1.
 */

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart3;

#define ICM20948_WHO_AM_I_REG 0x00U
#define ICM20948_WHO_AM_I_VAL 0xEAU

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)text,
                      (uint16_t)strlen(text),
                      HAL_MAX_DELAY);
}

static HAL_StatusTypeDef Read_WhoAmI(uint8_t addr7, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            (uint16_t)(addr7 << 1),
                            ICM20948_WHO_AM_I_REG,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            200);
}

void Test_IMU(void)
{
    char msg[100];
    uint8_t who = 0;
    uint8_t addr = 0;

    UART_Print("\r\n=== ICM-20948 TEST ===\r\n");

    while (1)
    {
        addr = 0;

        if (HAL_I2C_IsDeviceReady(&hi2c2, (0x68U << 1), 3, 100) == HAL_OK)
            addr = 0x68U;
        else if (HAL_I2C_IsDeviceReady(&hi2c2, (0x69U << 1), 3, 100) == HAL_OK)
            addr = 0x69U;

        if (addr != 0U)
        {
            if (Read_WhoAmI(addr, &who) == HAL_OK)
            {
                snprintf(msg, sizeof(msg),
                         "addr=0x%02X WHO_AM_I=0x%02X %s\r\n",
                         addr, who,
                         (who == ICM20948_WHO_AM_I_VAL) ? "[OK]" : "[CHECK]");
                UART_Print(msg);
            }
            else
            {
                UART_Print("ICM-20948 responded but WHO_AM_I read failed.\r\n");
            }
        }
        else
        {
            UART_Print("No ICM-20948 response at 0x68 or 0x69.\r\n");
        }

        HAL_Delay(2000);
    }
}
