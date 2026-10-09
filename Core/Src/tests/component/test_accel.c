/*
 * test_accel.c
 *
 *  Created on: 28 Aug 2026
 *      Author: advaith003
 */


#include "main.h"
#include "tests/component/test_accel.h"

#include <stdio.h>
#include <string.h>

/*
 * ICM20948 ACCELEROMETER TEST
 *
 * ICM20948:
 *   PB10 -> I2C2_SCL
 *   PB11 -> I2C2_SDA
 *
 * Output:
 *   USART3 @ 115200 baud
 *
 * Accelerometer:
 *   X/Y/Z raw readings
 *   X/Y/Z converted to g
 */

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart3;

#define ICM_ADDR_68             0x68U
#define ICM_ADDR_69             0x69U
#define REG_BANK_SEL            0x7FU

/* =========================================================
 * BANK 0
 * ========================================================= */

#define WHO_AM_I_REG            0x00U
#define PWR_MGMT_1_REG          0x06U
#define PWR_MGMT_2_REG          0x07U

/* Accelerometer output registers */
#define ACCEL_XOUT_H_REG        0x2DU


/* =========================================================
 * BANK 2
 * ========================================================= */

/*
 * Accelerometer sample-rate divider.
 */
#define ACCEL_SMPLRT_DIV_1_REG  0x10U
#define ACCEL_SMPLRT_DIV_2_REG  0x11U

/*
 * ACCEL_CONFIG:
 *
 * FS_SEL = 0 -> +/-2g
 *
 * DLPFCFG = 0
 * ACCEL_FCHOICE = 1
 */
#define ACCEL_CONFIG_REG        0x14U

#define WHO_AM_I_EXPECTED       0xEAU

/*
 * +/-2g:
 *
 * 16384 LSB/g
 */
#define ACCEL_SENSITIVITY       16384.0f


static uint8_t icm_addr = 0;


/* =========================================================
 * UART
 * ========================================================= */

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)text,
                      (uint16_t)strlen(text),
                      HAL_MAX_DELAY);
}


/* =========================================================
 * ICM20948 LOW LEVEL
 * ========================================================= */

static HAL_StatusTypeDef ICM_SelectBank(uint8_t bank)
{
    uint8_t value = (uint8_t)(bank << 4);

    return HAL_I2C_Mem_Write(&hi2c2,
                             (uint16_t)(icm_addr << 1),
                             REG_BANK_SEL,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             100);
}


static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             (uint16_t)(icm_addr << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             100);
}


static HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg,
                                      uint8_t *data,
                                      uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            (uint16_t)(icm_addr << 1),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            100);
}


/* =========================================================
 * FIND ICM20948
 * ========================================================= */

static uint8_t ICM_FindDevice(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c2,
                              (ICM_ADDR_68 << 1),
                              3,
                              100) == HAL_OK)
    {
        icm_addr = ICM_ADDR_68;
        return 1;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c2,
                              (ICM_ADDR_69 << 1),
                              3,
                              100) == HAL_OK)
    {
        icm_addr = ICM_ADDR_69;
        return 1;
    }

    return 0;
}


/* =========================================================
 * ACCELEROMETER INITIALIZATION
 * ========================================================= */

static uint8_t Accel_Init(void)
{
    uint8_t who = 0;

    /* -----------------------------------------
     * Select Bank 0
     * ----------------------------------------- */

    if (ICM_SelectBank(0) != HAL_OK)
        return 0;

    /* Check WHO_AM_I */
    if (ICM_ReadRegs(WHO_AM_I_REG, &who, 1) != HAL_OK)
        return 0;

    if (who != WHO_AM_I_EXPECTED)
        return 0;


    /* -----------------------------------------
     * Wake device
     * ----------------------------------------- */

    /* SLEEP = 0
     * CLKSEL = 1
     */
    if (ICM_WriteReg(PWR_MGMT_1_REG, 0x01U) != HAL_OK)
        return 0;

    /*
     * Enable accelerometer axes.
     *
     * 0x00 = accel XYZ + gyro XYZ enabled
     */
    if (ICM_WriteReg(PWR_MGMT_2_REG, 0x00U) != HAL_OK)
        return 0;

    HAL_Delay(100);


    /* -----------------------------------------
     * Select Bank 2
     * ----------------------------------------- */

    if (ICM_SelectBank(2) != HAL_OK)
        return 0;


    /* -----------------------------------------
     * Accelerometer sample rate
     * ----------------------------------------- */

    /*
     * Set divider to 10.
     *
     * This is roughly 102 Hz, matching the
     * gyro setup you already have.
     */

    if (ICM_WriteReg(ACCEL_SMPLRT_DIV_1_REG, 0x00U) != HAL_OK)
        return 0;

    if (ICM_WriteReg(ACCEL_SMPLRT_DIV_2_REG, 10U) != HAL_OK)
        return 0;


    /* -----------------------------------------
     * Accelerometer configuration
     * ----------------------------------------- */

    /*
     * +/-2g
     *
     * ACCEL_FS_SEL = 0
     *
     * ACCEL_FCHOICE = 1
     *
     * DLPF disabled here so we first see the
     * actual sensor data.
     */

    if (ICM_WriteReg(ACCEL_CONFIG_REG, 0x01U) != HAL_OK)
        return 0;


    /* Return to Bank 0 for reading */
    if (ICM_SelectBank(0) != HAL_OK)
        return 0;

    HAL_Delay(100);

    return 1;
}


/* =========================================================
 * READ ACCELEROMETER
 * ========================================================= */

static uint8_t Accel_ReadRaw(int16_t *ax,
                             int16_t *ay,
                             int16_t *az)
{
    uint8_t data[6];

    /*
     * Registers:
     *
     * 0x2D = ACCEL_XOUT_H
     * 0x2E = ACCEL_XOUT_L
     * 0x2F = ACCEL_YOUT_H
     * 0x30 = ACCEL_YOUT_L
     * 0x31 = ACCEL_ZOUT_H
     * 0x32 = ACCEL_ZOUT_L
     */

    if (ICM_ReadRegs(ACCEL_XOUT_H_REG,
                     data,
                     6) != HAL_OK)
    {
        return 0;
    }

    *ax = (int16_t)(((uint16_t)data[0] << 8) |
                    (uint16_t)data[1]);

    *ay = (int16_t)(((uint16_t)data[2] << 8) |
                    (uint16_t)data[3]);

    *az = (int16_t)(((uint16_t)data[4] << 8) |
                    (uint16_t)data[5]);

    return 1;
}


/* =========================================================
 * ACCEL TEST
 * ========================================================= */

void Test_Accel(void)
{
    char msg[180];

    int16_t ax;
    int16_t ay;
    int16_t az;

    UART_Print("\r\n================================\r\n");
    UART_Print(" ICM20948 ACCELEROMETER TEST\r\n");
    UART_Print("================================\r\n");


    /* Find device */

    if (!ICM_FindDevice())
    {
        UART_Print("ERROR: ICM20948 not found!\r\n");

        while (1)
            HAL_Delay(1000);
    }


    snprintf(msg,
             sizeof(msg),
             "ICM20948 found at 0x%02X\r\n",
             icm_addr);

    UART_Print(msg);


    /* Initialize accelerometer */

    if (!Accel_Init())
    {
        UART_Print("ERROR: Accelerometer initialization failed!\r\n");

        while (1)
            HAL_Delay(1000);
    }

    UART_Print("Accelerometer initialized.\r\n\r\n");

    UART_Print("Place robot still.\r\n");
    UART_Print("Expected magnitude is approximately 1g.\r\n\r\n");


    /* Continuous readings */

    while (1)
    {
        if (Accel_ReadRaw(&ax, &ay, &az))
        {
            float ax_g = (float)ax / ACCEL_SENSITIVITY;
            float ay_g = (float)ay / ACCEL_SENSITIVITY;
            float az_g = (float)az / ACCEL_SENSITIVITY;

            /*
             * Avoid %f because newlib-nano may not
             * have floating point printf enabled.
             *
             * Convert g to milli-g.
             */

            int32_t ax_mg = (int32_t)(ax_g * 1000.0f);
            int32_t ay_mg = (int32_t)(ay_g * 1000.0f);
            int32_t az_mg = (int32_t)(az_g * 1000.0f);

            snprintf(msg,
                     sizeof(msg),
                     "RAW: X=%6d Y=%6d Z=%6d   "
                     "ACCEL: X=%ld.%03ldg Y=%ld.%03ldg Z=%ld.%03ldg\r\n",
                     (int)ax,
                     (int)ay,
                     (int)az,

                     (long)(ax_mg / 1000),
                     (long)((ax_mg < 0 ? -ax_mg : ax_mg) % 1000),

                     (long)(ay_mg / 1000),
                     (long)((ay_mg < 0 ? -ay_mg : ay_mg) % 1000),

                     (long)(az_mg / 1000),
                     (long)((az_mg < 0 ? -az_mg : az_mg) % 1000));

            UART_Print(msg);
        }
        else
        {
            UART_Print("ERROR: Accelerometer read failed!\r\n");
        }

        HAL_Delay(100);
    }
}
