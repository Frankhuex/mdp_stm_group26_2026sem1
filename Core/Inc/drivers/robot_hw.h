#ifndef ROBOT_HW_H
#define ROBOT_HW_H

#include "main.h"
#include "drivers/robot_tuning.h"
#include <stdint.h>

typedef struct
{
    uint16_t left;
    uint16_t right;
}
RobotHW_EncoderSnapshot;

typedef struct
{
    int16_t left_delta;
    int16_t right_delta;
}
RobotHW_EncoderDelta;

void RobotHW_StartMotionPeripherals(void);

void RobotHW_MotorStop(void);
/* Non-blocking brake: caller must release after its timed brake phase. */
void RobotHW_MotorBrakeBegin(void);
void RobotHW_MotorBrakeRelease(void);
void RobotHW_MotorSetSigned(int32_t left_pwm, int32_t right_pwm);
void RobotHW_MotorSetForward(uint32_t left_pwm, uint32_t right_pwm);
void RobotHW_MotorSetBackward(uint32_t left_pwm, uint32_t right_pwm);
int32_t RobotHW_MotorGetLeftSigned(void);
int32_t RobotHW_MotorGetRightSigned(void);
/* Unscaled demands, for changing one wheel without rescaling the other. */
int32_t RobotHW_MotorGetLeftRequested(void);
int32_t RobotHW_MotorGetRightRequested(void);
uint32_t RobotHW_MotorLeftMax(void);
uint32_t RobotHW_MotorRightMax(void);
uint32_t RobotHW_ClampPwm(int32_t pwm, uint32_t maximum);

void RobotHW_ServoSetUs(uint32_t pulse_us);
void RobotHW_ServoSetUsTakeup(uint32_t pulse_us);  /* position with backlash take-up */
void RobotHW_ServoCenter(void);
uint32_t RobotHW_ServoGetUs(void);
uint32_t RobotHW_ServoCenterUs(void);
void RobotHW_ServoSetCenterUs(uint32_t pulse_us);  /* runtime straight-centre override */

void RobotHW_EncoderReset(void);
RobotHW_EncoderSnapshot RobotHW_EncoderSnapshotRead(void);
RobotHW_EncoderDelta RobotHW_EncoderDeltaRead(RobotHW_EncoderSnapshot *previous);

/* First-use/repeat calibration always stops and checks stillness. Independent
 * preparation-stop/still-check switches can also request these steps. Stops
 * motors, confirms stillness and calibrates bias; otherwise reuses bias. Then
 * zeros encoder/yaw state for a new motion. Returns 0 on success,
 * -1 when the IMU is unavailable and -4 when the chassis did not become still. */
int RobotHW_PrepareMotionSensors(void);

uint8_t RobotHW_GyroFindDevice(void);
uint8_t RobotHW_GyroInit(void);
uint8_t RobotHW_GyroIsInitialized(void);
uint8_t RobotHW_GyroHasOffset(void);
uint8_t RobotHW_GyroCalibrate(uint32_t warmup_ms,
                              uint32_t delay_ms,
                              uint32_t samples);
int16_t RobotHW_GyroReadZRaw(void);
void RobotHW_GyroSetOffset(int32_t offset);
int32_t RobotHW_GyroGetOffset(void);
void RobotHW_GyroPrimeFilter(float alpha);
float RobotHW_GyroUpdateYaw(float dt_sec,
                            float alpha,
                            float deadband_dps);
uint8_t RobotHW_GyroLastReadOk(void);
void RobotHW_YawReset(void);
float RobotHW_YawGetDeg(void);
float RobotHW_GyroGetLastZDps(void);

#endif /* ROBOT_HW_H */
