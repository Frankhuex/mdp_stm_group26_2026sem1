#include "main.h"
#include "test_config.h"
#include "control/approach_control.h"
#include "drivers/robot_hw.h"
#include "tests/calibration/test_approach_control.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

/* Keep blocking UART diagnostics sparse so this test exercises nearly the
 * same 20 ms control timing as the command interface. */
#define APPROACH_STATUS_PERIOD_MS 1000U

static void UART_Print(const char *text)
{
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)text,
                            (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static const char *ApproachStateText(approach_state_t state)
{
    switch (state)
    {
        case APPROACH_IDLE:           return "IDLE";
        case APPROACH_RUNNING:        return "RUNNING";
        case APPROACH_DONE:           return "DONE";
        case APPROACH_ABORTED:        return "ABORTED";
        case APPROACH_TIMEOUT:        return "TIMEOUT";
        case APPROACH_SENSOR_ERROR:   return "SENSOR_ERROR";
        case APPROACH_DISTANCE_LIMIT: return "DISTANCE_LIMIT";
        case APPROACH_STALL:          return "STALL";
        default:                      return "UNKNOWN";
    }
}

static void PrintApproachStatus(const char *prefix)
{
    char line[192];
    uint32_t elapsed_ms = 0U;
    uint16_t target_cm = 0U;
    uint32_t front_mm = 0U;
    float left_cm = 0.0f;
    float right_cm = 0.0f;
    float yaw_deg = 0.0f;

    Approach_Status(&elapsed_ms, &target_cm, &front_mm,
                    &left_cm, &right_cm, &yaw_deg);

    if (front_mm == UINT32_MAX)
    {
        (void)snprintf(line, sizeof(line),
                       "%s state=%s elapsed=%lu target=%u front=INF "
                       "L_mm=%ld R_mm=%ld yaw_mdeg=%ld\r\n",
                       prefix, ApproachStateText(Approach_State()),
                       (unsigned long)elapsed_ms, (unsigned)target_cm,
                       (long)(left_cm * 10.0f), (long)(right_cm * 10.0f),
                       (long)(yaw_deg * 1000.0f));
    }
    else
    {
        (void)snprintf(line, sizeof(line),
                       "%s state=%s elapsed=%lu target=%u front_mm=%lu "
                       "L_mm=%ld R_mm=%ld yaw_mdeg=%ld\r\n",
                       prefix, ApproachStateText(Approach_State()),
                       (unsigned long)elapsed_ms, (unsigned)target_cm,
                       (unsigned long)front_mm,
                       (long)(left_cm * 10.0f), (long)(right_cm * 10.0f),
                       (long)(yaw_deg * 1000.0f));
    }
    UART_Print(line);
}

void Test_ApproachControl(void)
{
    char line[96];
    uint32_t last_print_ms;
    int rc;

    RobotHW_StartMotionPeripherals();
    rc = Approach_Init();
    if (rc != 0)
    {
        UART_Print("APPROACH_INIT_ERROR\r\n");
        while (1) HAL_Delay(1000U);
    }

    (void)snprintf(line, sizeof(line),
                   "APPROACH target=%u cm; keep car still; start in %lu ms\r\n",
                   (unsigned)APPROACH_TEST_TARGET_CM,
                   (unsigned long)APPROACH_TEST_START_DELAY_MS);
    UART_Print(line);
    HAL_Delay(APPROACH_TEST_START_DELAY_MS);

    rc = Approach_StartCm(APPROACH_TEST_TARGET_CM);
    if (rc != 0)
    {
        (void)snprintf(line, sizeof(line), "APPROACH_START_ERROR rc=%d\r\n", rc);
        UART_Print(line);
        while (1) HAL_Delay(1000U);
    }

    last_print_ms = HAL_GetTick();
    while (Approach_State() == APPROACH_RUNNING)
    {
        Approach_Step();
        if ((HAL_GetTick() - last_print_ms) >= APPROACH_STATUS_PERIOD_MS)
        {
            last_print_ms = HAL_GetTick();
            PrintApproachStatus("LIVE");
        }
        HAL_Delay(1U);
    }

    PrintApproachStatus("FINAL");
    while (1) HAL_Delay(1000U);
}
