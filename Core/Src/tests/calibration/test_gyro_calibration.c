#include "main.h"
#include "tests/calibration/test_gyro_calibration.h"

#include <stdio.h>
#include <string.h>

/*
 * ICM20948 GYRO-Z ZERO OFFSET CALIBRATION
 *
 * ICM20948:
 *   PB10 -> I2C2_SCL
 *   PB11 -> I2C2_SDA
 *
 * Output:
 *   USART3 @ 115200 baud
 *
 * IMPORTANT:
 *   Keep robot COMPLETELY STILL during calibration.
 */

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart3;

#define ICM_ADDR_68             0x68U
#define ICM_ADDR_69             0x69U
#define REG_BANK_SEL            0x7FU

/* Bank 0 */
#define WHO_AM_I_REG            0x00U
#define PWR_MGMT_1_REG          0x06U
#define PWR_MGMT_2_REG          0x07U
#define GYRO_ZOUT_H_REG         0x37U

/* Bank 2 */
#define GYRO_SMPLRT_DIV_REG     0x00U
#define GYRO_CONFIG_1_REG       0x01U

#define WHO_AM_I_EXPECTED       0xEAU
#define GYRO_SENSITIVITY        131
#define CALIBRATION_SAMPLES     500U
#define SAMPLE_DELAY_MS         10U

static uint8_t icm_addr = 0;

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)text,
                      (uint16_t)strlen(text),
                      HAL_MAX_DELAY);
}

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

static uint8_t ICM_FindDevice(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c2, (ICM_ADDR_68 << 1), 3, 100) == HAL_OK)
    {
        icm_addr = ICM_ADDR_68;
        return 1;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c2, (ICM_ADDR_69 << 1), 3, 100) == HAL_OK)
    {
        icm_addr = ICM_ADDR_69;
        return 1;
    }

    return 0;
}

static uint8_t Gyro_Init(void)
{
    uint8_t who = 0;

    if (ICM_SelectBank(0) != HAL_OK)
        return 0;

    if (ICM_ReadRegs(WHO_AM_I_REG, &who, 1) != HAL_OK)
        return 0;

    if (who != WHO_AM_I_EXPECTED)
        return 0;

    /* Wake device: SLEEP = 0, CLKSEL = 1 */
    if (ICM_WriteReg(PWR_MGMT_1_REG, 0x01U) != HAL_OK)
        return 0;

    /* Enable all accel/gyro axes */
    if (ICM_WriteReg(PWR_MGMT_2_REG, 0x00U) != HAL_OK)
        return 0;

    HAL_Delay(100);

    if (ICM_SelectBank(2) != HAL_OK)
        return 0;

    /* Approx. 102 Hz gyro sample rate */
    if (ICM_WriteReg(GYRO_SMPLRT_DIV_REG, 10U) != HAL_OK)
        return 0;

    /* DLPF enabled, +/-250 dps */
    if (ICM_WriteReg(GYRO_CONFIG_1_REG, 0x19U) != HAL_OK)
        return 0;

    if (ICM_SelectBank(0) != HAL_OK)
        return 0;

    HAL_Delay(100);
    return 1;
}

static int16_t Gyro_ReadZ_Raw(void)
{
    uint8_t data[2];

    if (ICM_ReadRegs(GYRO_ZOUT_H_REG, data, 2) != HAL_OK)
        return 0;

    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

void Test_GyroCalibration(void)
{
    char msg[120];
    int32_t sum = 0;
    int32_t gyro_z_offset = 0;

    UART_Print("\r\n================================\r\n");
    UART_Print(" ICM20948 GYRO-Z CALIBRATION\r\n");
    UART_Print("================================\r\n");

    if (!ICM_FindDevice())
    {
        UART_Print("ERROR: ICM20948 not found!\r\n");
        while (1) HAL_Delay(1000);
    }

    snprintf(msg, sizeof(msg), "ICM20948 found at 0x%02X\r\n", icm_addr);
    UART_Print(msg);

    if (!Gyro_Init())
    {
        UART_Print("ERROR: Gyro initialization failed!\r\n");
        while (1) HAL_Delay(1000);
    }

    UART_Print("Gyro initialized.\r\n\r\n");
    UART_Print("DO NOT MOVE THE ROBOT!\r\n");
    UART_Print("Starting calibration in 3 seconds...\r\n");
    HAL_Delay(3000);

    UART_Print("Collecting 500 samples...\r\n");

    /* Ignore first readings after startup */
    for (uint16_t i = 0; i < 50; i++)
    {
        (void)Gyro_ReadZ_Raw();
        HAL_Delay(SAMPLE_DELAY_MS);
    }

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        sum += Gyro_ReadZ_Raw();
        HAL_Delay(SAMPLE_DELAY_MS);
    }

    gyro_z_offset = sum / (int32_t)CALIBRATION_SAMPLES;

    snprintf(msg,
             sizeof(msg),
             "\r\nGyro-Z offset = %ld raw counts\r\n",
             (long)gyro_z_offset);
    UART_Print(msg);

    int32_t offset_mdps = (gyro_z_offset * 1000L) / GYRO_SENSITIVITY;

    snprintf(msg,
             sizeof(msg),
             "Gyro-Z offset = %ld.%03ld deg/s\r\n",
             (long)(offset_mdps / 1000),
             (long)((offset_mdps < 0 ? -offset_mdps : offset_mdps) % 1000));
    UART_Print(msg);

    UART_Print("\r\nCalibration finished.\r\n");
    UART_Print("Corrected gyro-Z readings:\r\n\r\n");

    while (1)
    {
        int16_t raw_z = Gyro_ReadZ_Raw();
        int32_t corrected_raw = (int32_t)raw_z - gyro_z_offset;
        int32_t corrected_mdps = (corrected_raw * 1000L) / GYRO_SENSITIVITY;

        snprintf(msg,
                 sizeof(msg),
                 "Raw: %6d   Corrected: %6ld   Z: %ld.%03ld deg/s\r\n",
                 (int)raw_z,
                 (long)corrected_raw,
                 (long)(corrected_mdps / 1000),
                 (long)((corrected_mdps < 0 ? -corrected_mdps : corrected_mdps) % 1000));

        UART_Print(msg);
        HAL_Delay(100);
    }
}
