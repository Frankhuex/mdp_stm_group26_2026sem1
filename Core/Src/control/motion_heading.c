#include "control/motion_heading.h"
#include "drivers/robot_hw.h"

/* A missed interval reduces confidence; it never rejects a motion command. */
#define HEADING_IDLE_PERIOD_MS       20U
#define HEADING_MAX_IDLE_GAP_MS      500U
#define HEADING_IDLE_FILTER_ALPHA    0.10f
#define HEADING_IDLE_RATE_DEADBAND   0.08f
/* Preserve the requested steering side and avoid large route changes.
 * Small requested turns additionally limit correction to half their angle. */
#define HEADING_MAX_TURN_CORRECTION  5.0f

static uint8_t enabled = 1U;
static uint8_t tracking;
static uint8_t idle_sampling;
static uint8_t degraded;
static float planned_deg;
static float measured_deg;
static float previous_local_deg;
static uint32_t last_idle_tick;

static float WrapError(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

void MotionHeading_Reset(void)
{
    tracking = idle_sampling = degraded = 0U;
    planned_deg = measured_deg = previous_local_deg = 0.0f;
    last_idle_tick = HAL_GetTick();
}

void MotionHeading_SetEnabled(uint8_t value)
{
    enabled = value ? 1U : 0U;
    MotionHeading_Reset();
}

uint8_t MotionHeading_IsEnabled(void) { return enabled; }
uint8_t MotionHeading_IsTracking(void) { return tracking; }
void MotionHeading_Degrade(void) { degraded = 1U; }
float MotionHeading_ErrorDeg(void) { return WrapError(measured_deg - planned_deg); }

float MotionHeading_ControlErrorDeg(float local_yaw_deg)
{
    return enabled ? MotionHeading_ErrorDeg() : local_yaw_deg;
}

void MotionHeading_Status(motion_heading_status_t *status)
{
    if (!status) return;
    status->target_deg = planned_deg;
    status->measured_deg = measured_deg;
    status->error_deg = MotionHeading_ErrorDeg();
    status->enabled = enabled;
    status->tracking = tracking;
    status->degraded = degraded;
}

void MotionHeading_Observe(float local_yaw_deg, uint8_t sample_ok)
{
    if (!tracking) return;
    if (!sample_ok)
    {
        MotionHeading_Degrade();
        return;
    }
    measured_deg += local_yaw_deg - previous_local_deg;
    previous_local_deg = local_yaw_deg;
}

void MotionHeading_PollIdle(void)
{
    if (!tracking || !idle_sampling) return;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_idle_tick;
    if (elapsed < HEADING_IDLE_PERIOD_MS) return;
    last_idle_tick = now;
    if (elapsed > HEADING_MAX_IDLE_GAP_MS)
    {
        /* Hold the last estimate; do not extrapolate one rate across a gap
         * or erase the error by redefining the route heading. */
        MotionHeading_Degrade();
        return;
    }
    (void)RobotHW_GyroUpdateYaw((float)elapsed / 1000.0f,
        HEADING_IDLE_FILTER_ALPHA, HEADING_IDLE_RATE_DEADBAND);
    MotionHeading_Observe(RobotHW_YawGetDeg(), RobotHW_GyroLastReadOk());
}

void MotionHeading_Prepare(void)
{
    MotionHeading_PollIdle();
    idle_sampling = 0U;
}

float MotionHeading_Begin(float signed_turn_deg)
{
    float correction = enabled ? MotionHeading_ErrorDeg() : 0.0f;
    float magnitude = signed_turn_deg < 0.0f ? -signed_turn_deg : signed_turn_deg;
    float limit = magnitude * 0.5f;
    if (limit > HEADING_MAX_TURN_CORRECTION) limit = HEADING_MAX_TURN_CORRECTION;
    if (correction > limit) correction = limit;
    if (correction < -limit) correction = -limit;
    planned_deg += signed_turn_deg;
    previous_local_deg = 0.0f;
    tracking = 1U;
    idle_sampling = 0U;
    return signed_turn_deg - correction;
}

void MotionHeading_LocalYawReset(void)
{
    /* A sensor-frame reset must not reset measured route heading or target. */
    previous_local_deg = 0.0f;
}

void MotionHeading_End(void)
{
    idle_sampling = tracking;
    last_idle_tick = HAL_GetTick();
}
