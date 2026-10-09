#include "main.h"
#include "tests/calibration/test_straight_control.h"
#include "control/straight_control.h"
#include "control/motion_heading.h"
#include "control/straight_tuning.h"
#include "drivers/oled.h"
#include "drivers/robot_hw.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

/* ROUND_TRIP=1 runs +distance, pauses, then -distance automatically.
 * ROUND_TRIP=0 runs once; negative distance selects reverse only.
 * Keep the car in place during the return countdown. */
#define TEST_STRAIGHT_DISTANCE_CM       40
#define TEST_STRAIGHT_ROUND_TRIP        1U
#define TEST_STRAIGHT_RETURN_DELAY_MS   10000U
#define TEST_STRAIGHT_RESULT_PAGE_MS    8000U

#if TEST_STRAIGHT_ROUND_TRIP != 0 && TEST_STRAIGHT_ROUND_TRIP != 1
#error "TEST_STRAIGHT_ROUND_TRIP must be 0 or 1"
#endif
#if TEST_STRAIGHT_ROUND_TRIP && TEST_STRAIGHT_DISTANCE_CM < 0
#error "Use a positive distance for the forward/reverse round trip"
#endif
#if (TEST_STRAIGHT_DISTANCE_CM > -MIN_COMMAND_DISTANCE_CM && \
     TEST_STRAIGHT_DISTANCE_CM < MIN_COMMAND_DISTANCE_CM) || \
    TEST_STRAIGHT_DISTANCE_CM < -MAX_COMMAND_DISTANCE_CM || \
    TEST_STRAIGHT_DISTANCE_CM > MAX_COMMAND_DISTANCE_CM
#error "Straight test distance must have magnitude 10..200 cm"
#endif

/* RAM-only observations during motion. Trace marks adapt to each distance. */
#define TRACE_POINT_COUNT 3U
typedef struct
{
    uint8_t captured;
    float travel_cm;
    float yaw_deg;
    uint32_t servo_us;
    uint32_t elapsed_ms;
} straight_test_trace_t;

typedef struct
{
    int start_rc;
    straight_state_t state;
    int32_t target_cm;
    uint32_t elapsed_ms;
    float left_cm, right_cm, yaw_deg, diff_cm;
    uint8_t route_heading;
    uint8_t marks_cm[TRACE_POINT_COUNT];
    straight_test_trace_t trace[TRACE_POINT_COUNT];
} straight_test_result_t;

/* Preserve the forward record when the controller resets for reverse. */
static straight_test_result_t results[2];

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)text,
                      (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void FormatSignedX100(float value, char *out, size_t n)
{
    int32_t x100 = (int32_t)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
    int32_t a = (x100 < 0) ? -x100 : x100;
    snprintf(out, n, "%c%ld.%02ld", (x100 < 0) ? '-' : '+',
             (long)(a / 100), (long)(a % 100));
}

/* All OLED/UART work happens while stopped, never inside the drive loop. */
static void ShowSetup(int32_t target_cm)
{
    char line[32];
    int trim = (target_cm < 0) ? STRAIGHT_REVERSE_TRIM_US : STRAIGHT_FORWARD_TRIM_US;
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    snprintf(line, sizeof(line), "%s %ldcm", (target_cm < 0) ? "REV" : "FWD", (long)target_cm);
    OLED_ShowString(0, 0, (const u8 *)line);
#if STRAIGHT_CENTER_CALIBRATION_ACTIVE
    OLED_ShowString(0, 16, (const u8 *)"GYRO OFF SYNC ON");
#else
    OLED_ShowString(0, 16, (const u8 *)"GYRO ON SYNC ON");
#endif
    snprintf(line, sizeof(line), "C:%lu T:%+d", (unsigned long)RobotHW_ServoCenterUs(), trim);
    OLED_ShowString(0, 32, (const u8 *)line);
    OLED_ShowString(0, 48, (const u8 *)"Keep car STILL");
    OLED_Refresh_Gram();
}

/* countdown_s=-1 shows the saved result; otherwise announces automatic return. */
static void ShowResult(const straight_test_result_t *result, int countdown_s)
{
    char line[48], value[24];
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    if (result->start_rc != 0)
        snprintf(line, sizeof(line), "START ERROR %d", result->start_rc);
    else if (countdown_s >= 0)
        snprintf(line, sizeof(line), "REV in %ds", countdown_s);
    else
        snprintf(line, sizeof(line), "%c:%ldcm S:%d H%u", (result->target_cm < 0) ? 'R' : 'F',
                 (long)result->target_cm, (int)result->state, (unsigned)result->route_heading);
    OLED_ShowString(0, 0, (const u8 *)line);
    FormatSignedX100(result->yaw_deg, value, sizeof(value));
    snprintf(line, sizeof(line), "Yaw:%sdeg", value);
    OLED_ShowString(0, 16, (const u8 *)line);
    FormatSignedX100(result->left_cm, value, sizeof(value));
    snprintf(line, sizeof(line), "L:%scm", value);
    OLED_ShowString(0, 32, (const u8 *)line);
    FormatSignedX100(result->right_cm, value, sizeof(value));
    snprintf(line, sizeof(line), "R:%scm", value);
    OLED_ShowString(0, 48, (const u8 *)line);
    OLED_Refresh_Gram();
}

static void CaptureTrace(straight_test_result_t *result)
{
    float left_cm, right_cm, yaw_deg;
    uint32_t elapsed_ms;
    if (Straight_State() != STRAIGHT_RUNNING) return;
    Straight_Status(&elapsed_ms, NULL, &left_cm, &right_cm, &yaw_deg, NULL);
    float travel_cm = (left_cm + right_cm) * 0.5f;
    if (travel_cm < 0.0f) travel_cm = -travel_cm;
    for (unsigned i = 0U; i < TRACE_POINT_COUNT; ++i)
    {
        straight_test_trace_t *point = &result->trace[i];
        if (!point->captured && travel_cm >= (float)result->marks_cm[i])
        {
            point->captured = 1U;
            point->travel_cm = travel_cm;
            point->yaw_deg = yaw_deg;
            point->servo_us = RobotHW_ServoGetUs();
            point->elapsed_ms = elapsed_ms;
        }
    }
}

static void ShowTrace(const straight_test_result_t *result)
{
    char line[48], yaw[24];
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    OLED_ShowString(0, 0, (const u8 *)((result->target_cm < 0)
                    ? "REV cm:yaw/servo" : "FWD cm:yaw/servo"));
    for (unsigned i = 0U; i < TRACE_POINT_COUNT; ++i)
    {
        const straight_test_trace_t *point = &result->trace[i];
        if (point->captured)
        {
            FormatSignedX100(point->yaw_deg, yaw, sizeof(yaw));
            snprintf(line, sizeof(line), "%u:%s %lu", (unsigned)result->marks_cm[i],
                     yaw, (unsigned long)point->servo_us);
        }
        else
            snprintf(line, sizeof(line), "%u: not reached", (unsigned)result->marks_cm[i]);
        line[16] = '\0';
        OLED_ShowString(0, (u8)((i + 1U) * 16U), (const u8 *)line);
    }
    OLED_Refresh_Gram();
}

static void PrintResult(const straight_test_result_t *result)
{
    char msg[220], left[24], right[24], yaw[24], diff[24];
    FormatSignedX100(result->left_cm, left, sizeof(left));
    FormatSignedX100(result->right_cm, right, sizeof(right));
    FormatSignedX100(result->yaw_deg, yaw, sizeof(yaw));
    FormatSignedX100(result->diff_cm, diff, sizeof(diff));
    snprintf(msg, sizeof(msg),
             "\r\n--- %s RESULT ---\r\nstart_rc=%d State=%d time=%lums target=%ldcm\r\n"
             "Encoder travel L=%s cm R=%s cm\r\nL-R=%s cm yaw=%s deg\r\n",
             (result->target_cm < 0) ? "REVERSE" : "FORWARD", result->start_rc,
             (int)result->state, (unsigned long)result->elapsed_ms, (long)result->target_cm,
             left, right, diff, yaw);
    UART_Print(msg);
    if (result->route_heading)
        UART_Print("Heading reference: planned route heading (OLED H1).\r\n");
    for (unsigned i = 0U; i < TRACE_POINT_COUNT; ++i)
    {
        const straight_test_trace_t *point = &result->trace[i];
        if (!point->captured) continue;
        FormatSignedX100(point->travel_cm, left, sizeof(left));
        FormatSignedX100(point->yaw_deg, yaw, sizeof(yaw));
        snprintf(msg, sizeof(msg), "TRACE mark=%ucm encoder=%scm time=%lums yaw=%sdeg servo=%luus\r\n",
                 (unsigned)result->marks_cm[i], left, (unsigned long)point->elapsed_ms,
                 yaw, (unsigned long)point->servo_us);
        UART_Print(msg);
    }
}

static uint8_t RunLeg(straight_test_result_t *result, int32_t target_cm)
{
    uint32_t magnitude = (uint32_t)((target_cm < 0) ? -target_cm : target_cm);
    memset(result, 0, sizeof(*result));
    result->target_cm = target_cm;
    /* Quarter-distance marks: 30 cm -> 7,15,22; 60 cm -> 15,30,45. */
    for (unsigned i = 0U; i < TRACE_POINT_COUNT; ++i)
        result->marks_cm[i] = (uint8_t)(magnitude * (i + 1U) / 4U);
    ShowSetup(target_cm);
    result->start_rc = Straight_StartCm(target_cm);
    result->route_heading = result->start_rc == 0 ? MotionHeading_IsEnabled() : 0U;
    if (result->start_rc != 0)
    {
        RobotHW_MotorStop();
        RobotHW_ServoCenter();
        PrintResult(result);
        ShowResult(result, -1);
        return 0U;
    }
    while (Straight_State() == STRAIGHT_RUNNING)
    {
        Straight_Step();
        CaptureTrace(result);
        HAL_Delay(1U);
    }
    result->state = Straight_State();
    Straight_Status(&result->elapsed_ms, NULL, &result->left_cm, &result->right_cm,
                    &result->yaw_deg, &result->diff_cm);
    PrintResult(result);
    ShowResult(result, -1);
    return result->state == STRAIGHT_DONE;
}

void Test_StraightControl(void)
{
    char msg[220];
    unsigned result_count = 1U;
    RobotHW_StartMotionPeripherals();
    Straight_Init();
    OLED_Init();
    OLED_Display_On();
    ShowSetup(TEST_STRAIGHT_DISTANCE_CM);

    UART_Print("\r\n=== STRAIGHT CONTROL FLOOR TEST ===\r\n");
    snprintf(msg, sizeof(msg),
             "Distance=%dcm round_trip=%u centre=%luus trim_fwd/rev=%d/%dus\r\n"
             "Gyro steering=%s. Motor sync and distance stopping ON.\r\n"
             "Long stop compensation fwd/rev=%lu/%lu milli-rev.\r\n",
             TEST_STRAIGHT_DISTANCE_CM, (unsigned)TEST_STRAIGHT_ROUND_TRIP,
             (unsigned long)RobotHW_ServoCenterUs(),
             (int)STRAIGHT_FORWARD_TRIM_US, (int)STRAIGHT_REVERSE_TRIM_US,
             STRAIGHT_CENTER_CALIBRATION_ACTIVE ? "OFF" : "ON",
             (unsigned long)(LONG_FORWARD_REV_ERROR_COMPENSATION * 1000.0f + 0.5f),
             (unsigned long)(LONG_REVERSE_REV_ERROR_COMPENSATION * 1000.0f + 0.5f));
    UART_Print(msg);
    UART_Print("10-20 cm: SHORT; above 20 cm: LONG. First run after 3s and sensor preparation.\r\n");
    UART_Print("Mark rear-axle midpoint AND chassis heading. Keep still during calibration.\r\n");
#if TEST_STRAIGHT_ROUND_TRIP
    UART_Print("AUTOMATIC REVERSE after the forward stop and countdown. Do not reposition the car.\r\n");
#endif
    HAL_Delay(3000U);

    uint8_t forward_done = RunLeg(&results[0], TEST_STRAIGHT_DISTANCE_CM);
#if TEST_STRAIGHT_ROUND_TRIP
    if (forward_done)
    {
        uint32_t pause_start = HAL_GetTick();
        int last_seconds = -1;
        while (1)
        {
            Straight_Step(); /* Keep the original heading through the pause. */
            uint32_t elapsed = HAL_GetTick() - pause_start;
            if (elapsed >= TEST_STRAIGHT_RETURN_DELAY_MS) break;
            int seconds = (int)((TEST_STRAIGHT_RETURN_DELAY_MS - elapsed + 999U) / 1000U);
            if (seconds != last_seconds)
            {
                ShowResult(&results[0], seconds);
                last_seconds = seconds;
            }
            HAL_Delay(20U);
        }
        result_count = 2U;
        (void)RunLeg(&results[1], -TEST_STRAIGHT_DISTANCE_CM);
    }
    else
        UART_Print("Return cancelled: forward motion did not complete successfully.\r\n");
#else
    (void)forward_done;
#endif

    RobotHW_MotorStop();
    RobotHW_ServoCenter();
    UART_Print("Finished. OLED cycles saved results/traces every 8s; RESET repeats the test.\r\n");
    UART_Print("S=2 means DONE. Yaw and L/R include the brake pulse; verify distance on the floor.\r\n");
    UART_Print("H1: route correction ON for both legs. H0: segment-local correction.\r\n");
    UART_Print("Measure each leg and final position/heading against the floor marks.\r\n");
    UART_Print("Returning to the start does not prove distance accuracy: equal short/long legs can cancel.\r\n");
    MotionHeading_Prepare(); /* Freeze the saved test records. */
    while (1)
    {
        for (unsigned i = 0U; i < result_count; ++i)
        {
            ShowResult(&results[i], -1);
            HAL_Delay(TEST_STRAIGHT_RESULT_PAGE_MS);
            ShowTrace(&results[i]);
            HAL_Delay(TEST_STRAIGHT_RESULT_PAGE_MS);
        }
    }
}
