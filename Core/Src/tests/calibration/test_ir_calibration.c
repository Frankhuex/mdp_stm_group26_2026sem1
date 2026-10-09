#include "main.h"
#include "sensors/ir_sensor.h"
#include "tests/calibration/test_ir_calibration.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define IR_CAL_LINE_SIZE       64U
#define IR_CAL_AVG_SAMPLES     64U
#define IR_CAL_RX_TIMEOUT_MS   50U

static char rx_line[IR_CAL_LINE_SIZE];
static uint32_t rx_len = 0U;

static void uart_send(const char *text)
{
    if (text == NULL) return;
    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HAL_MAX_DELAY);
}

static void print_help(void)
{
    uart_send("\r\nCommands:\r\n");
    uart_send("  L,<cm>   capture LEFT sensor at known distance\r\n");
    uart_send("  R,<cm>   capture RIGHT sensor at known distance\r\n");
    uart_send("  B,<cm>   capture BOTH sensors at same known distance\r\n");
    uart_send("  LIVE     print both sensors continuously with estimated cm\r\n");
    uart_send("  HELP     show this help\r\n");
    uart_send("\r\nSuggested points: 80,60,50,40,30,25,20,15,10 cm.\r\n");
    uart_send("Measure distance from the IR sensor face to a flat target.\r\n");
    uart_send("Keep the target perpendicular to the sensor.\r\n");
    uart_send("Copy the IR_CAL CSV lines into a spreadsheet for curve fitting later.\r\n\r\n");
}

static int read_line_blocking(char *out, uint32_t out_size)
{
    uint8_t b;

    if (out == NULL || out_size < 2U) return -1;
    rx_len = 0U;

    while (1)
    {
        HAL_StatusTypeDef st = HAL_UART_Receive(&huart3, &b, 1U, IR_CAL_RX_TIMEOUT_MS);
        if (st != HAL_OK) continue;

        if (b == '\r' || b == '\n')
        {
            if (rx_len == 0U) continue;
            out[rx_len] = '\0';
            return 0;
        }

        if (rx_len < (out_size - 1U))
        {
            out[rx_len++] = (char)b;
        }
        else
        {
            rx_len = 0U;
            uart_send("ERR,LINE_TOO_LONG\r\n");
        }
    }
}

static void capture_and_print(char side, int32_t cm)
{
    char line[180];
    IRSensorReading r;
    int rc;

    if (cm < 8 || cm > 100)
    {
        uart_send("ERR,DISTANCE_RANGE_USE_8_TO_100_CM\r\n");
        return;
    }

    uart_send("Hold target still... sampling.\r\n");
    rc = IRSensor_ReadAverage(&r, IR_CAL_AVG_SAMPLES);
    if (rc != 0)
    {
        (void)snprintf(line, sizeof(line), "ERR,ADC,rc=%d\r\n", rc);
        uart_send(line);
        return;
    }

    if (side == 'L')
    {
        (void)snprintf(line, sizeof(line),
                       "IR_CAL,L,cm=%ld,raw=%u,mV=%u\r\n",
                       (long)cm,
                       (unsigned)r.left_raw,
                       (unsigned)r.left_mv);
    }
    else if (side == 'R')
    {
        (void)snprintf(line, sizeof(line),
                       "IR_CAL,R,cm=%ld,raw=%u,mV=%u\r\n",
                       (long)cm,
                       (unsigned)r.right_raw,
                       (unsigned)r.right_mv);
    }
    else
    {
        (void)snprintf(line, sizeof(line),
                       "IR_CAL,B,cm=%ld,L_raw=%u,L_mV=%u,R_raw=%u,R_mV=%u\r\n",
                       (long)cm,
                       (unsigned)r.left_raw,
                       (unsigned)r.left_mv,
                       (unsigned)r.right_raw,
                       (unsigned)r.right_mv);
    }

    uart_send(line);
}

static void live_mode(void)
{
    char line[220];
    uart_send("LIVE mode: raw + mV + calibrated cm. Reset board to exit.\r\n");
    uart_send("CM range is validated for 10..50 cm only.\r\n");

    while (1)
    {
        IRSensorDistanceReading r;
        int rc = IRSensor_ReadDistance(&r, 16U);
        if (rc == 0)
        {
            char lcm[16];
            char rcm[16];

            if (r.left_status == IR_RANGE_NEAR)
                (void)snprintf(lcm, sizeof(lcm), "<10");
            else if (r.left_status == IR_RANGE_FAR)
                (void)snprintf(lcm, sizeof(lcm), ">50");
            else
                (void)snprintf(lcm, sizeof(lcm), "%u.%u",
                               (unsigned)(r.left_cm_x10 / 10U),
                               (unsigned)(r.left_cm_x10 % 10U));

            if (r.right_status == IR_RANGE_NEAR)
                (void)snprintf(rcm, sizeof(rcm), "<10");
            else if (r.right_status == IR_RANGE_FAR)
                (void)snprintf(rcm, sizeof(rcm), ">50");
            else
                (void)snprintf(rcm, sizeof(rcm), "%u.%u",
                               (unsigned)(r.right_cm_x10 / 10U),
                               (unsigned)(r.right_cm_x10 % 10U));

            (void)snprintf(line, sizeof(line),
                           "IR_L raw=%4u mV=%4u cm=%s | IR_R raw=%4u mV=%4u cm=%s\r\n",
                           (unsigned)r.adc.left_raw,
                           (unsigned)r.adc.left_mv,
                           lcm,
                           (unsigned)r.adc.right_raw,
                           (unsigned)r.adc.right_mv,
                           rcm);
            uart_send(line);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "ERR,ADC,rc=%d\r\n", rc);
            uart_send(line);
        }
        HAL_Delay(250U);
    }
}

void Test_IRCalibration(void)
{
    char line[IR_CAL_LINE_SIZE];

    uart_send("\r\n============================================================\r\n");
    uart_send(" CALIBRATION - 2x SHARP GP2Y0A21YK IR\r\n");
    uart_send("============================================================\r\n");
    uart_send("LEFT  VO -> PC0 / ADC1_IN10\r\n");
    uart_send("RIGHT VO -> PC1 / ADC1_IN11\r\n");
    uart_send("VCC -> 5V, GND -> common GND. Motor switch OFF.\r\n");
    uart_send("CM conversion is installed from your measured 10..50 cm data.\r\n");
    uart_send("L/R/B captures still print RAW data so you can recalibrate later.\r\n");
    print_help();

    while (1)
    {
        char *comma;
        char side;
        long cm;
        char *endptr = NULL;

        uart_send("> ");
        if (read_line_blocking(line, sizeof(line)) != 0) continue;

        if (strcmp(line, "HELP") == 0)
        {
            print_help();
            continue;
        }
        if (strcmp(line, "LIVE") == 0)
        {
            live_mode();
        }

        comma = strchr(line, ',');
        if (comma == NULL || comma == line || comma[1] == '\0')
        {
            uart_send("ERR,USE_L_cm_R_cm_OR_B_cm_EXAMPLE_L_20\r\n");
            continue;
        }

        *comma = '\0';
        side = line[0];
        if ((side != 'L' && side != 'R' && side != 'B') || line[1] != '\0')
        {
            uart_send("ERR,SIDE_MUST_BE_L_R_OR_B\r\n");
            continue;
        }

        cm = strtol(comma + 1, &endptr, 10);
        if (endptr == NULL || *endptr != '\0')
        {
            uart_send("ERR,BAD_DISTANCE\r\n");
            continue;
        }

        capture_and_print(side, (int32_t)cm);
    }
}
