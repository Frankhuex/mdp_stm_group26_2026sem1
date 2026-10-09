#include "main.h"
#include "sensors/ir_sensor.h"
#include "tests/component/test_ir.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define IR_TEST_AVG_SAMPLES  16U
#define IR_TEST_PERIOD_MS   250U

static void uart_send(const char *text)
{
    if (text == NULL) return;
    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HAL_MAX_DELAY);
}

static void format_cm(char *dst,
                      size_t dst_size,
                      uint16_t cm_x10,
                      IRSensorRangeStatus status)
{
    if (dst == NULL || dst_size == 0U) return;

    if (status == IR_RANGE_NEAR)
    {
        (void)snprintf(dst, dst_size, "<10cm");
    }
    else if (status == IR_RANGE_FAR)
    {
        (void)snprintf(dst, dst_size, ">50cm");
    }
    else if (status == IR_RANGE_VALID)
    {
        (void)snprintf(dst, dst_size, "%u.%ucm",
                       (unsigned)(cm_x10 / 10U),
                       (unsigned)(cm_x10 % 10U));
    }
    else
    {
        (void)snprintf(dst, dst_size, "ERR");
    }
}

void Test_IR(void)
{
    char line[220];

    uart_send("\r\n============================================================\r\n");
    uart_send(" LIVE TEST - 2x SHARP GP2Y0A21YK IR WITH CM\r\n");
    uart_send("============================================================\r\n");
    uart_send("LEFT  VO -> PC0 / ADC1_IN10\r\n");
    uart_send("RIGHT VO -> PC1 / ADC1_IN11\r\n");
    uart_send("VCC=5V, common GND. Motor switch OFF for bench testing.\r\n");
    uart_send("Calibration: separate LEFT/RIGHT lookup tables from your data.\r\n");
    uart_send("Validated distance range: 10..50 cm.\r\n");
    uart_send("Output refresh: 250 ms; each result averages 16 samples.\r\n");
    uart_send("============================================================\r\n");

    while (1)
    {
        IRSensorDistanceReading r;
        int rc = IRSensor_ReadDistance(&r, IR_TEST_AVG_SAMPLES);

        if (rc == 0)
        {
            char lcm[16];
            char rcm[16];
            format_cm(lcm, sizeof(lcm), r.left_cm_x10, r.left_status);
            format_cm(rcm, sizeof(rcm), r.right_cm_x10, r.right_status);

            (void)snprintf(line,
                           sizeof(line),
                           "IR_L raw=%4u mV=%4u cm=%-7s | IR_R raw=%4u mV=%4u cm=%-7s\r\n",
                           (unsigned)r.adc.left_raw,
                           (unsigned)r.adc.left_mv,
                           lcm,
                           (unsigned)r.adc.right_raw,
                           (unsigned)r.adc.right_mv,
                           rcm);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "IR_ADC_ERROR,rc=%d\r\n", rc);
        }

        uart_send(line);
        HAL_Delay(IR_TEST_PERIOD_MS);
    }
}
