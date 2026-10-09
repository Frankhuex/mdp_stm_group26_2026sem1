#include "tests/calibration/test_motion_sequence.h"
#include "test_config.h"
#include "assessment/command_interface.h"
#include "control/motion_heading.h"
#include "control/straight_control.h"
#include "control/turn_control.h"
#include "drivers/oled.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#if MOTION_SEQUENCE_HEADING_ON != 0 && MOTION_SEQUENCE_HEADING_ON != 1
#error "MOTION_SEQUENCE_HEADING_ON must be 0 or 1"
#endif

/* Use the same parser, validation and controllers as UART and the course. */
#if MOTION_SEQUENCE_CASE == 1
static const char *const commands[] = {
    "STRAIGHT,GOCM,40", "TURN,LEFT,R=20,A=90", "STRAIGHT,GOCM,-10" };
static const char *const labels[] = { "F40", "L20/90", "B10" };
#elif MOTION_SEQUENCE_CASE == 2
static const char *const commands[] = {
    "STRAIGHT,GOCM,40", "TURN,RIGHT,R=35,A=90", "STRAIGHT,GOCM,-10" };
static const char *const labels[] = { "F40", "R35/90", "B10" };
#elif MOTION_SEQUENCE_CASE == 3
static const char *const commands[] = {
    "STRAIGHT,GOCM,30", "TURN,LEFT,R=20,A=90", "STRAIGHT,GOCM,20",
    "TURN,RIGHT,R=35,A=90", "STRAIGHT,GOCM,-10" };
static const char *const labels[] = { "F30", "L20/90", "F20", "R35/90", "B10" };
#elif MOTION_SEQUENCE_CASE == 4
static const char *const commands[] = { "STRAIGHT,GOCM,40", "STRAIGHT,GOCM,-40" };
static const char *const labels[] = { "F40", "B40" };
#else
#error "MOTION_SEQUENCE_CASE must be 1..4"
#endif

#define LEG_COUNT (sizeof(commands) / sizeof(commands[0]))
#define RESULT_PAGE_MS 8000U
typedef struct {
    motion_heading_status_t heading;
    float left_cm, right_cm, local_turn_deg;
    uint8_t is_turn;
} leg_result_t;
static leg_result_t results[LEG_COUNT];
static uint32_t result_count;

/* Integer formatting avoids enabling floating-point printf in the firmware. */
static void Format1(float value, char *text, size_t size)
{
    int n = (int)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
    int magnitude = n < 0 ? -n : n;
    snprintf(text, size, "%c%d.%d", n < 0 ? '-' : '+', magnitude / 10, magnitude % 10);
}

static void Row(uint8_t y, char *text)
{
    text[16] = '\0';
    OLED_ShowString(0, y, (const u8 *)text);
}

static void ShowResult(uint32_t index, uint8_t next)
{
    const leg_result_t *result = &results[index];
    char line[48], a[20], b[20];
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    snprintf(line, sizeof(line), "%lu %s H%u %s", (unsigned long)(index + 1U),
        labels[index], (unsigned)result->heading.enabled, next ? "NEXT" : "END");
    Row(0, line);
    Format1(result->heading.target_deg, a, sizeof(a));
    Format1(result->heading.measured_deg, b, sizeof(b));
    snprintf(line, sizeof(line), "T%s M%s", a, b);
    Row(16, line);
    int error100 = (int)(result->heading.error_deg * 100.0f +
        (result->heading.error_deg >= 0.0f ? 0.5f : -0.5f));
    int magnitude = error100 < 0 ? -error100 : error100;
    snprintf(a, sizeof(a), "%c%d.%02d", error100 < 0 ? '-' : '+', magnitude / 100, magnitude % 100);
    snprintf(line, sizeof(line), "E%s Q:%s", a, result->heading.degraded ? "LOW" : "OK");
    Row(32, line);
    if (result->is_turn)
    {
        Format1(result->local_turn_deg, a, sizeof(a));
        snprintf(line, sizeof(line), "Turn:%sdeg", a);
    }
    else
    {
        Format1(result->left_cm, a, sizeof(a));
        Format1(result->right_cm, b, sizeof(b));
        snprintf(line, sizeof(line), "L%s R%s", a, b);
    }
    Row(48, line);
    OLED_Refresh_Gram();
}

static void CaptureLeg(uint32_t index, const char *command)
{
    if (index == 0U || index > LEG_COUNT) return;
    leg_result_t *result = &results[index - 1U];
    MotionHeading_Status(&result->heading);
    result->is_turn = strncmp(command, "TURN,", 5U) == 0;
    if (result->is_turn) Turn_Status(NULL, NULL, NULL, &result->local_turn_deg, NULL);
    else Straight_Status(NULL, NULL, &result->left_cm, &result->right_cm, NULL, NULL);
    result_count = index;
    ShowResult(index - 1U, index < LEG_COUNT);
}

void Test_MotionSequence(void)
{
    uint32_t failure_index = 0U;
    result_count = 0U;
    const char *failure = CommandInterface_RunDiagnosticSequence(commands, LEG_COUNT,
        MOTION_SEQUENCE_STARTUP_MS, MOTION_SEQUENCE_PAUSE_MS,
        MOTION_SEQUENCE_HEADING_ON, CaptureLeg, &failure_index);
    /* These records are frozen. Moving the car to photograph it cannot
     * change the displayed results or start another movement. */
    MotionHeading_Prepare();
    while (1)
    {
        if (failure)
        {
            char line[48];
            memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
            snprintf(line, sizeof(line), "STOP step %lu", (unsigned long)failure_index);
            Row(0, line);
            snprintf(line, sizeof(line), "%s", failure);
            Row(16, line);
            OLED_ShowString(0, 48, (const u8 *)"RESET to repeat");
            OLED_Refresh_Gram();
            HAL_Delay(RESULT_PAGE_MS);
        }
        for (uint32_t i = 0U; i < result_count; ++i)
        {
            ShowResult(i, 0U);
            HAL_Delay(RESULT_PAGE_MS);
        }
    }
}
