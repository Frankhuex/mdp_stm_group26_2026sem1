#ifndef APPROACH_CONTROL_H
#define APPROACH_CONTROL_H

#include <stdint.h>

/* Compatibility API for the dedicated approach test.  The implementation
 * delegates to the production straight/ultrasonic controller so command mode
 * and the standalone test cannot drift apart. */
typedef enum
{
    APPROACH_IDLE = 0,
    APPROACH_RUNNING,
    APPROACH_DONE,
    APPROACH_ABORTED,
    APPROACH_TIMEOUT,
    APPROACH_SENSOR_ERROR,
    APPROACH_DISTANCE_LIMIT,
    APPROACH_STALL
} approach_state_t;

int Approach_Init(void);
int Approach_StartCm(uint16_t target_cm);
void Approach_Step(void);
void Approach_Stop(void);
approach_state_t Approach_State(void);
void Approach_Status(uint32_t *elapsed_ms,
                     uint16_t *target_cm,
                     uint32_t *front_mm,
                     float *left_cm,
                     float *right_cm,
                     float *yaw_deg);

#endif /* APPROACH_CONTROL_H */
