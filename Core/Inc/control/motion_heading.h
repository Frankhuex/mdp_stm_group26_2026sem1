#ifndef MOTION_HEADING_H
#define MOTION_HEADING_H

#include <stdint.h>

/* Positive heading is a physical LEFT rotation. Headings are unwrapped so
 * requested 180/360-degree turns retain their intended direction and extent. */
typedef struct
{
    float target_deg;
    float measured_deg;
    float error_deg; /* measured - target, wrapped to [-180, 180] */
    uint8_t enabled;
    uint8_t tracking;
    uint8_t degraded; /* sticky until an explicit route reset */
} motion_heading_status_t;

/* Reset the origin at the current chassis orientation; preserve ON/OFF mode.
 * Call only while stopped. SetEnabled also resets the origin for an A/B run. */
void MotionHeading_Reset(void);
void MotionHeading_SetEnabled(uint8_t enabled);
uint8_t MotionHeading_IsEnabled(void);
uint8_t MotionHeading_IsTracking(void);
void MotionHeading_Status(motion_heading_status_t *status);

/* Controller lifecycle: Prepare before stationary gyro calibration, Begin
 * after the local gyro has been zeroed, Observe on every gyro sample, End
 * after braking. Begin(0) is straight travel in either direction.
 * For turns, Begin returns the signed local rotation with bounded correction;
 * the controller then applies its existing empirical side calibration. */
void MotionHeading_Prepare(void);
float MotionHeading_Begin(float signed_turn_deg);
void MotionHeading_LocalYawReset(void);
void MotionHeading_Observe(float local_yaw_deg, uint8_t sample_ok);
void MotionHeading_End(void);
void MotionHeading_Degrade(void);
void MotionHeading_PollIdle(void);
float MotionHeading_ErrorDeg(void);
float MotionHeading_ControlErrorDeg(float local_yaw_deg);

#endif
