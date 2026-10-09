#include "control/approach_control.h"
#include "control/straight_control.h"
#include "control/straight_tuning.h"
#include "sensors/ultrasonic_sensor.h"

#include <stddef.h>
#include <stdint.h>

static uint8_t approach_initialized = 0U;

static approach_state_t MapStraightState(straight_state_t state)
{
    switch (state)
    {
        case STRAIGHT_IDLE:           return APPROACH_IDLE;
        case STRAIGHT_RUNNING:        return APPROACH_RUNNING;
        case STRAIGHT_DONE:           return APPROACH_DONE;
        case STRAIGHT_ABORTED:        return APPROACH_ABORTED;
        case STRAIGHT_TIMEOUT:        return APPROACH_TIMEOUT;
        case STRAIGHT_DISTANCE_LIMIT: return APPROACH_DISTANCE_LIMIT;
        case STRAIGHT_STALL:          return APPROACH_STALL;
        case STRAIGHT_SENSOR_ERROR:
        default:                      return APPROACH_SENSOR_ERROR;
    }
}

int Approach_Init(void)
{
    int rc;

    Straight_Init();
    rc = UltrasonicSensor_Init();
    approach_initialized = (rc == 0) ? 1U : 0U;
    return rc;
}

int Approach_StartCm(uint16_t target_cm)
{
    int rc;

    if (Straight_State() == STRAIGHT_RUNNING) return -3;
    if (target_cm < US_MIN_THRESHOLD_CM || target_cm > US_MAX_THRESHOLD_CM)
        return -2;

    if (!approach_initialized)
    {
        rc = UltrasonicSensor_Init();
        if (rc != 0) return -1;
        approach_initialized = 1U;
    }

    return Straight_StartUntilUltrasonicCm((int32_t)target_cm);
}

void Approach_Step(void)
{
    Straight_Step();
}

void Approach_Stop(void)
{
    Straight_Stop();
}

approach_state_t Approach_State(void)
{
    return MapStraightState(Straight_State());
}

void Approach_Status(uint32_t *elapsed_ms,
                     uint16_t *target_cm,
                     uint32_t *front_mm,
                     float *left_cm,
                     float *right_cm,
                     float *yaw_deg)
{
    int32_t straight_target = 0;
    int32_t threshold_cm = 0;
    uint32_t distance_mm = UINT32_MAX;

    Straight_Status(elapsed_ms, &straight_target, left_cm, right_cm,
                    yaw_deg, NULL);
    (void)straight_target;
    (void)Straight_UltrasonicStatus(&threshold_cm, &distance_mm, NULL, NULL);

    if (target_cm) *target_cm = (threshold_cm > 0) ? (uint16_t)threshold_cm : 0U;
    if (front_mm) *front_mm = distance_mm;
}
