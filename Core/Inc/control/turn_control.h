#ifndef TURN_CONTROL_H
#define TURN_CONTROL_H

#include <stdint.h>

typedef enum
{
    TURN_IDLE = 0,
    TURN_RUNNING,
    TURN_DONE,
    TURN_ABORTED,
    TURN_TIMEOUT,
    TURN_WRONG_SIGN,
    TURN_NO_TURN,
    TURN_BAD_CONFIG,
    TURN_NO_IMU
}
turn_state_t;

void Turn_RunMinRadiusTestBlocking(void);

void Turn_Init(void);
/* Advance planned route heading by the signed requested arc rotation;
 * incoming heading error adjusts local turn travel before side calibration.
 * Successful RIGHT commands then perform the stopped 1500 -> centre return,
 * resetting heading history like manual STEER. Successful LEFT commands
 * apply HEADING,OFF. DONE includes these actions; STOP cancels any pending
 * return. Applies to either gear. TURN,STATUS retains the measured turn yaw. */
int Turn_StartRadiusAngle(int32_t radius_cm, int32_t angle_deg, int8_t turn_dir);
void Turn_Step(void);
void Turn_Stop(void);
/* Valid after a normal completed turn; signed yaw and positive turn rate. */
void Turn_StopMetrics(float *trigger_yaw_deg, float *trigger_rate_dps);
/* Actual positive local yaw target, including route correction and side scale. */
float Turn_TargetMagnitudeDeg(void);
float Turn_SettledYawDeg(void);   /* gyro angle actually swept this turn (signed, +=LEFT) */
float Turn_PeakRateDps(void);     /* peak |yaw rate| this turn (saturation check) */
turn_state_t Turn_State(void);
void Turn_Status(uint32_t *elapsed_ms,
                 int32_t *radius_cm,
                 int32_t *target_deg,
                 float *yaw_deg,
                 int8_t *turn_dir);

#endif /* TURN_CONTROL_H */
