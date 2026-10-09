#ifndef STRAIGHT_CONTROL_H
#define STRAIGHT_CONTROL_H

#include <stdint.h>

typedef enum
{
    STRAIGHT_IDLE = 0,
    STRAIGHT_RUNNING,
    STRAIGHT_DONE,
    STRAIGHT_ABORTED,
    STRAIGHT_TIMEOUT,
    STRAIGHT_SENSOR_ERROR,
    STRAIGHT_DISTANCE_LIMIT,
    STRAIGHT_STALL
} straight_state_t;

/* Production non-blocking controller used by the command interface. */
void Straight_Init(void);
/* Holds the route's planned chassis heading in either gear. The shared
 * motion_heading module preserves error across straight and turn commands. */
int Straight_StartCm(int32_t target_cm);
/* Forward-only short calibration with fixed steering; gyro measurement,
 * encoder synchronization, distance stop and fault handling remain active.
 * Pulse must be within +/-100 us of centre. Applies to this command only. */
int Straight_StartCalibrationCm(int32_t target_cm, int32_t servo_us);
/* 0 = normal gyro steering, otherwise the last accepted fixed pulse. */
uint32_t Straight_FixedSteeringUs(void);
/* Fresh confirmed distance selects forward (>target) or reverse (<target).
 * Stops after confirmed strict crossing; initial equality completes stationary.
 * Missing initial measurements fail without driving. */
int Straight_StartUntilUltrasonicCm(int32_t threshold_cm);
typedef enum { STRAIGHT_IR_GT = 0, STRAIGHT_IR_LT } straight_ir_comparison_t;
int Straight_StartUntilIRCm(int32_t threshold_cm, uint8_t side,
                            straight_ir_comparison_t comparison);
straight_ir_comparison_t Straight_IRComparison(void);
/* Forward until selected IR > threshold. side: 0=LEFT, 1=RIGHT.
 * FAR or ADC failure completes normally, per command policy. */
int Straight_StartUntilIRGreaterCm(int32_t threshold_cm, uint8_t side);
/* GT for lateral approach: FAR completes, ADC/selected-side error aborts. */
int Straight_StartUntilIRGreaterValidCm(int32_t threshold_cm, uint8_t side);
/* Returns 0=not IR, 1=LEFTIR, 2=RIGHTIR. UINT32_MAX distance means FAR. */
int Straight_IRStatus(int32_t *threshold_cm, uint32_t *cm_x10);
/* Last condition-triggered stop distance per side, in 0.1 cm (237 = 23.7 cm).
 * Updated only for that side on IR DONE; retained across later commands.
 * UINT32_MAX = no cached stop since Init, FAR, or GT read failure. */
extern uint32_t Straight_LeftIRStopCmX10;
extern uint32_t Straight_RightIRStopCmX10;
/* Returns 1 for UNTILUS mode. Unavailable clearance = UINT32_MAX (infinity).
 * Nonzero status means invalid/stale; missing echoes do not stop motion. */
int Straight_UltrasonicStatus(int32_t *threshold_cm, uint32_t *distance_mm,
                              int *status, uint32_t *age_ms);
void Straight_Step(void);
void Straight_Stop(void);
straight_state_t Straight_State(void);
void Straight_Status(uint32_t *elapsed_ms,
                     int32_t *target_cm,
                     float *l_cm,
                     float *r_cm,
                     float *yaw_deg,
                     float *diff_cm);

/* Last accepted straight motion; retained after DONE/STOP until the next one.
 * phase: S=start, W=ultrasonic wait, D=drive, B=brake, E=end.
 * Servo is a PWM command, not measured wheel position. */
typedef struct
{
    uint32_t elapsed_ms;
    float local_yaw_deg;
    float control_error_deg;
    float rate_dps;
    uint32_t servo_us;
    int8_t direction;
    uint8_t phase;
    uint8_t gyro_ok;
} straight_trace_sample_t;
uint32_t Straight_TraceCount(void);
uint32_t Straight_TraceDropped(void);
uint8_t Straight_TraceRead(uint32_t index, straight_trace_sample_t *sample);

#endif /* STRAIGHT_CONTROL_H */
