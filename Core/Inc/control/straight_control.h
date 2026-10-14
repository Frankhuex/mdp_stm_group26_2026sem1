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
    STRAIGHT_STALL,
    STRAIGHT_SEARCH_LIMIT /* bounded IR search exhausted, after braking */
} straight_state_t;

/* Production non-blocking controller used by the command interface. */
void Straight_Init(void);
/* Holds the route's planned chassis heading in either gear. The shared
 * motion_heading module preserves error across straight and turn commands. */
int Straight_StartCm(int32_t target_cm);
/* Freeze US,GET + current successful shared RISING minimum + offset (cm).
 * Optional outputs are mm. -5 means unavailable cache; -2 invalid sum. */
int Straight_StartFromUsIR(int32_t offset_cm, int8_t direction,
    uint32_t *us_mm, uint32_t *ir_mm, uint32_t *distance_mm);
/* Forward-only short calibration with fixed steering; gyro measurement,
 * encoder synchronization, distance stop and fault handling remain active.
 * Pulse must be within +/-100 us of centre. Applies to this command only. */
int Straight_StartCalibrationCm(int32_t target_cm, int32_t servo_us);
/* 0 = normal gyro steering, otherwise the last accepted fixed pulse. */
uint32_t Straight_FixedSteeringUs(void);
/* Default forward; direction is +1=FORWARD, -1=BACKWARD, 0=AUTO.
 * No echo: creep in the requested direction and continue acquisition.
 * Direction never changes; STOP, stall, timeout and travel bounds remain. */
int Straight_StartUntilUltrasonicCm(int32_t threshold_cm);
/* AUTO confirms valid starting distance; missing starting echo selects reverse creep. */
int Straight_StartUntilUltrasonicAutoCm(int32_t threshold_cm);
int Straight_StartUntilUltrasonicDirectionCm(int32_t threshold_cm, int8_t direction);
typedef enum {
    STRAIGHT_IR_GT = 0, STRAIGHT_IR_LT,
    STRAIGHT_IR_RISING,  /* below then above */
    STRAIGHT_IR_FALLING  /* above then below */
} straight_ir_comparison_t;
int Straight_StartUntilIRCm(int32_t threshold_cm, uint8_t side,
                            straight_ir_comparison_t comparison);
/* Explicit gear, independent of sensor side and comparison: +1/-1. */
int Straight_StartUntilIRDirectionCm(int32_t threshold_cm, uint8_t side,
                                     straight_ir_comparison_t comparison, int8_t direction);
int Straight_StartCmThenIR(int32_t distance_cm, uint8_t side,
                           straight_ir_comparison_t comparison, int32_t threshold_cm);
int Straight_StartCmThenIRBounded(int32_t distance_cm, uint8_t side,
    straight_ir_comparison_t comparison, int32_t threshold_cm, uint32_t limit_mm);
uint8_t Straight_IRFirstEdgeReading(uint32_t *cm_x10);
uint8_t Straight_IRGateStatus(int32_t *distance_cm, uint8_t *active);
void Straight_IRDiagnostics(uint32_t *raw, int *range_status, uint32_t *age_ms,
                            uint8_t *confirm, uint8_t *armed);
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
/* First fresh valid US sample before drive for each UNTILUS gear, mm.
 * BACKWARD caches zero when startup has no valid echo; recovery preserves it.
 * Selected cache resets on accepted start; UINT32_MAX means unavailable. */
extern uint32_t Straight_ForwardUSStartMm;
extern uint32_t Straight_BackwardUSStartMm;
/* Final condition sample on successful UNTILUS completion, mm.
 * Default UNTILUS updates FORWARD. Selected start/end pair resets together.
 * UINT32_MAX = unavailable, aborted or failed; not a post-brake remeasurement. */
extern uint32_t Straight_ForwardUSEndMm;
extern uint32_t Straight_BackwardUSEndMm;
/* AUTO owns this independent start/end pair, regardless of selected gear. */
extern uint32_t Straight_AutoUSStartMm;
extern uint32_t Straight_AutoUSEndMm;
/* Returns 1 for UNTILUS mode. Unavailable clearance = UINT32_MAX (infinity).
 * Nonzero status means invalid/stale; missing echoes do not stop motion. */
int Straight_UltrasonicStatus(int32_t *threshold_cm, uint32_t *distance_mm,
                              int *status, uint32_t *age_ms);
/* Successful UNTILUS condition sample, captured before braking. Historical
 * age at detection; direction +1=forward, -1=reverse, 0=stationary equality.
 * Returns 0 if no successful UNTILUS result is available. */
/* Completion sample is the planned target in distance mode; legacy fallback remains measured. */
uint8_t Straight_UltrasonicDistanceMode(void);
uint8_t Straight_UltrasonicStopReading(uint32_t *distance_mm, uint32_t *age_ms,
                                       int8_t *direction);
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
