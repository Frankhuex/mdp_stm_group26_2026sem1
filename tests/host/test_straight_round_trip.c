/* Run the real floor-test sequencer against simulated motion and OLED/UART.
 * No hardware movement. Verify automatic return, retained results and faults. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "../../Core/Src/tests/calibration/test_straight_control.c"

UART_HandleTypeDef huart3;
uint8_t OLED_GRAM[128][8];
static jmp_buf test_exit;
static uint32_t clock_ms, start_ticks[2], first_stop_tick;
static int starts, step_count, fault_case;
static int32_t requests[2], active_target;
static straight_state_t motion_state;

uint32_t HAL_GetTick(void) { return clock_ms; }
void HAL_Delay(uint32_t ms)
{
    if (ms == TEST_STRAIGHT_RESULT_PAGE_MS)
    {
        assert(motion_state != STRAIGHT_RUNNING);
        longjmp(test_exit, 1); /* Finished; stop the infinite display loop. */
    }
    clock_ms += ms;
}
int HAL_UART_Transmit(UART_HandleTypeDef *port, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)port; (void)data; (void)size; (void)timeout;
    assert(motion_state != STRAIGHT_RUNNING);
    return 0;
}
void OLED_Init(void) {}
void OLED_Display_On(void) {}
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *text)
{
    (void)x; (void)y; (void)text;
    assert(motion_state != STRAIGHT_RUNNING);
}
void OLED_Refresh_Gram(void) { assert(motion_state != STRAIGHT_RUNNING); }
void RobotHW_StartMotionPeripherals(void) {}
void RobotHW_MotorStop(void) { assert(motion_state != STRAIGHT_RUNNING); }
void RobotHW_ServoCenter(void) {}
uint32_t RobotHW_ServoCenterUs(void) { return SERVO_CENTER_US; }
uint32_t RobotHW_ServoGetUs(void) { return active_target > 0 ? 1750U : 1710U; }
void Straight_Init(void) { motion_state = STRAIGHT_IDLE; }
void MotionHeading_Prepare(void) {}
uint8_t MotionHeading_IsEnabled(void) { return 1U; }
int Straight_StartCm(int32_t target_cm)
{
    assert(starts < 2);
    requests[starts] = target_cm;
    start_ticks[starts] = clock_ms;
    starts++;
    if ((fault_case == 2 && starts == 1) || (fault_case == 3 && starts == 2))
        return -1;
    active_target = target_cm;
    step_count = 0;
    motion_state = STRAIGHT_RUNNING;
    return 0;
}
void Straight_Step(void)
{
    if (motion_state != STRAIGHT_RUNNING) return;
    step_count++;
    if (fault_case == 1 && starts == 1 && step_count == 2)
        motion_state = STRAIGHT_TIMEOUT;
    else if (step_count == 4)
        motion_state = STRAIGHT_DONE;
    if (starts == 1 && motion_state != STRAIGHT_RUNNING)
        first_stop_tick = clock_ms;
}
straight_state_t Straight_State(void) { return motion_state; }
void Straight_Status(uint32_t *elapsed, int32_t *target, float *left, float *right,
                     float *yaw, float *diff)
{
    float travel = (float)active_target * (float)step_count / 4.0f;
    if (elapsed) *elapsed = (uint32_t)step_count * 20U;
    if (target) *target = active_target;
    if (left) *left = travel;
    if (right) *right = travel;
    if (yaw) *yaw = (active_target > 0 ? 0.4f : -0.2f) * (float)step_count;
    if (diff) *diff = 0.0f;
}
static void run_case(int which)
{
    fault_case = which;
    starts = step_count = 0;
    clock_ms = UINT32_MAX - 5000U; /* Exercise countdown through timer wrap. */
    motion_state = STRAIGHT_IDLE;
    memset(results, 0, sizeof(results));
    if (setjmp(test_exit) == 0) Test_StraightControl();
}
int main(void)
{
    run_case(0);
#if TEST_STRAIGHT_ROUND_TRIP
    assert(starts == 2 && requests[0] == TEST_STRAIGHT_DISTANCE_CM && requests[1] == -TEST_STRAIGHT_DISTANCE_CM);
    assert(start_ticks[1] - first_stop_tick >= TEST_STRAIGHT_RETURN_DELAY_MS);
    assert(results[0].target_cm == TEST_STRAIGHT_DISTANCE_CM && results[1].target_cm == -TEST_STRAIGHT_DISTANCE_CM);
    assert(results[0].state == STRAIGHT_DONE && results[1].state == STRAIGHT_DONE);
    assert(results[0].route_heading == 1 && results[1].route_heading == 1);
    assert(results[0].left_cm == (float)TEST_STRAIGHT_DISTANCE_CM && results[1].left_cm == -(float)TEST_STRAIGHT_DISTANCE_CM);
    assert(results[0].yaw_deg > 0.0f && results[1].yaw_deg < 0.0f);
    for (unsigned i = 0; i < TRACE_POINT_COUNT; ++i)
    {
        assert(results[0].trace[i].captured && results[1].trace[i].captured);
        assert(results[0].trace[i].servo_us == 1750U);
        assert(results[1].trace[i].servo_us == 1710U);
        assert(results[0].marks_cm[i] == (uint8_t)(TEST_STRAIGHT_DISTANCE_CM * (i + 1U) / 4U));
    }
    run_case(1);
    assert(starts == 1 && results[0].state == STRAIGHT_TIMEOUT);
    run_case(2);
    assert(starts == 1 && results[0].start_rc == -1);
    run_case(3);
    assert(starts == 2 && results[0].state == STRAIGHT_DONE && results[1].start_rc == -1);
    puts("PASS: forward/reverse sequence, countdown/wrap, saved per-leg results/traces, no I/O while driving, faults cancel/stop return");
#else
    assert(starts == 1 && requests[0] == TEST_STRAIGHT_DISTANCE_CM);
    assert(results[0].state == STRAIGHT_DONE);
    run_case(1);
    assert(starts == 1 && results[0].state == STRAIGHT_TIMEOUT);
    run_case(2);
    assert(starts == 1 && results[0].start_rc == -1);
    puts("PASS: single leg, selected direction, timeout and start failure");
#endif
    return 0;
}
