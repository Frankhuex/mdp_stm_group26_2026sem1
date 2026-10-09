/* Host test links the production straight controller with simulated hardware. */
#include <assert.h>
#include <stdio.h>
#include "control/straight_control.h"
#include "control/motion_heading.h"
#include "drivers/robot_hw.h"
#include "sensors/ir_sensor.h"
#include "sensors/ultrasonic_sensor.h"
static uint32_t tick;
static int drives, adc_fail;
static int16_t encoder_delta;
static float mock_yaw, mock_rate, mock_brake_rate, mock_last_rate;
static uint8_t mock_gyro_ok = 1U, mock_braking;
static uint8_t mock_brake_sample_pending;
static uint32_t mock_servo, mock_drive_servo;
static uint32_t mock_gyro_delay_ms, mock_brake_begin_tick, mock_brake_end_tick;
static int mock_prepare_rc, mock_direction;
static IRSensorDistanceReading ir;
static ultrasonic_sample_t mock_ultrasonic;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void RobotHW_MotorStop(void) { mock_braking = 0U; }
void RobotHW_MotorBrakeBegin(void) { mock_braking = 1U; mock_brake_begin_tick = tick; }
void RobotHW_MotorBrakeRelease(void)
{
    mock_brake_sample_pending = mock_braking;
    mock_braking = 0U; mock_brake_end_tick = tick;
}
void RobotHW_ServoCenter(void) { mock_servo = SERVO_CENTER_US; }
void RobotHW_ServoSetUs(uint32_t x) { mock_servo = x; }
uint32_t RobotHW_ServoGetUs(void) { return mock_servo; }
uint32_t RobotHW_ServoCenterUs(void) { return SERVO_CENTER_US; }
int RobotHW_PrepareMotionSensors(void) { mock_yaw = 0.0f; return mock_prepare_rc; }
void RobotHW_GyroPrimeFilter(float a) { (void)a; }
void RobotHW_YawReset(void) { mock_yaw = mock_last_rate = 0.0f; }
void RobotHW_EncoderReset(void) {}
RobotHW_EncoderSnapshot RobotHW_EncoderSnapshotRead(void)
{ RobotHW_EncoderSnapshot s = {0, 0}; return s; }
RobotHW_EncoderDelta RobotHW_EncoderDeltaRead(RobotHW_EncoderSnapshot *s)
{ RobotHW_EncoderDelta d = {encoder_delta, encoder_delta}; (void)s; return d; }
void RobotHW_MotorSetForward(uint32_t l, uint32_t r)
{ (void)l; (void)r; drives++; mock_direction = 1; mock_drive_servo = mock_servo; }
void RobotHW_MotorSetBackward(uint32_t l, uint32_t r)
{ (void)l; (void)r; drives++; mock_direction = -1; mock_drive_servo = mock_servo; }
uint32_t RobotHW_MotorLeftMax(void) { return 7199; }
uint32_t RobotHW_MotorRightMax(void) { return 7199; }
uint32_t RobotHW_ClampPwm(int32_t x, uint32_t m) { return x < 0 ? 0 : (uint32_t)x > m ? m : (uint32_t)x; }
float RobotHW_GyroUpdateYaw(float d,float a,float b)
{
    (void)a; (void)b;
    tick += mock_gyro_delay_ms;
    /* The sample at release covers the preceding brake interval; angular
     * motion does not disappear at the instant the PWM outputs change. */
    mock_last_rate = (mock_braking || mock_brake_sample_pending) ? mock_brake_rate : mock_rate;
    mock_brake_sample_pending = 0U;
    if (mock_gyro_ok) mock_yaw += mock_last_rate * d;
    return mock_yaw;
}
float RobotHW_GyroGetLastZDps(void) { return mock_last_rate; }
float RobotHW_YawGetDeg(void) { return mock_yaw; }
uint8_t RobotHW_GyroLastReadOk(void) { return mock_gyro_ok; }
int IRSensor_ReadDistance(IRSensorDistanceReading *r,uint32_t n) { (void)n;*r=ir;return adc_fail ? -1 : 0; }
int UltrasonicSensor_Init(void) { return 0; }
void UltrasonicSensor_Cancel(void) {}
void UltrasonicSensor_Step(void) {}
void UltrasonicSensor_GetSample(ultrasonic_sample_t *s)
{ *s = mock_ultrasonic; }
int UltrasonicSensor_Trigger(void) { return 0; }
static void reset(void)
{
    IRSensorDistanceReading zero = {0};
    ultrasonic_sample_t us_zero = {0};
    mock_ultrasonic = us_zero;
    Straight_Init();MotionHeading_SetEnabled(1U);tick=0;drives=0;adc_fail=0;encoder_delta=0;
    mock_yaw = mock_rate = mock_brake_rate = mock_last_rate = 0.0f;
    mock_gyro_ok = 1U; mock_braking = mock_brake_sample_pending = 0U;
    mock_servo = mock_drive_servo = SERVO_CENTER_US;
    mock_gyro_delay_ms = mock_brake_begin_tick = mock_brake_end_tick = 0U;
    mock_prepare_rc = mock_direction = 0;
    ir=zero;ir.left_cm_x10=500;ir.right_cm_x10=500;
}
static void step(void) { tick+=40;Straight_Step(); }
int main(void)
{
    reset();assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    assert(Straight_State()==STRAIGHT_RUNNING && drives==1); /* equality */
    ir.right_status=IR_RANGE_FAR;step();assert(Straight_State()==STRAIGHT_RUNNING);
    ir.left_status=IR_RANGE_FAR;step();assert(Straight_State()==STRAIGHT_DONE);
    reset();ir.left_cm_x10=501;assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    assert(Straight_State()==STRAIGHT_DONE && drives==0);
    reset();ir.right_cm_x10=501;assert(Straight_StartUntilIRGreaterCm(50,1)==0);
    assert(Straight_State()==STRAIGHT_DONE && drives==0);
    reset();adc_fail=1;assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    assert(Straight_State()==STRAIGHT_DONE && drives==0);
    reset();assert(Straight_StartUntilIRGreaterCm(50,1)==0);adc_fail=1;step();
    assert(Straight_State()==STRAIGHT_DONE);
    reset();assert(Straight_StartUntilIRGreaterCm(50,0)==0);Straight_Stop();
    assert(Straight_State()==STRAIGHT_ABORTED);
    assert(Straight_StartCm(20)==0 && Straight_IRStatus(0,0)==0);Straight_Stop();
    assert(Straight_StartUntilIRGreaterCm(50,0)==0);Straight_Stop();
    assert(Straight_StartUntilUltrasonicCm(20)==0 && Straight_IRStatus(0,0)==0);
    reset();assert(Straight_StartUntilIRGreaterCm(9,0)==-2);
    assert(Straight_StartUntilIRGreaterCm(51,0)==-2);
    assert(Straight_StartUntilIRGreaterCm(61,1)==-2);
    assert(Straight_StartUntilIRGreaterCm(50,2)==-2);
    assert(Straight_StartUntilIRGreaterCm(60,1)==0);
    assert(Straight_StartUntilIRGreaterCm(50,0)==-3);
    tick=30000;Straight_Step();assert(Straight_State()==STRAIGHT_TIMEOUT);
    reset();assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    encoder_delta=16000;step();assert(Straight_State()==STRAIGHT_DISTANCE_LIMIT);
    for (uint8_t side = 0; side < 2; ++side)
    {
        reset();
        assert(Straight_StartUntilIRCm(20,side,STRAIGHT_IR_LT)==0);
        assert(Straight_IRComparison()==STRAIGHT_IR_LT);
        /* Opposite side must not stop this command. */
        if (side) ir.left_cm_x10=100; else ir.right_cm_x10=100;
        step();assert(Straight_State()==STRAIGHT_RUNNING);
        if (side) ir.right_cm_x10=200; else ir.left_cm_x10=200;
        step();assert(Straight_State()==STRAIGHT_RUNNING); /* equality */
        if (side) ir.right_cm_x10=199; else ir.left_cm_x10=199;
        step();assert(Straight_State()==STRAIGHT_DONE);
        reset();
        if (side) ir.right_status=IR_RANGE_FAR; else ir.left_status=IR_RANGE_FAR;
        assert(Straight_StartUntilIRCm(20,side,STRAIGHT_IR_LT)==0);
        step();assert(Straight_State()==STRAIGHT_RUNNING);
        adc_fail=1;step();assert(Straight_State()==STRAIGHT_SENSOR_ERROR);
        reset();adc_fail=1;
        assert(Straight_StartUntilIRCm(20,side,STRAIGHT_IR_LT)==0);
        assert(Straight_State()==STRAIGHT_SENSOR_ERROR && drives==0);
        reset();
        if (side) ir.right_status=IR_RANGE_ERROR; else ir.left_status=IR_RANGE_ERROR;
        assert(Straight_StartUntilIRCm(20,side,STRAIGHT_IR_LT)==0);
        assert(Straight_State()==STRAIGHT_SENSOR_ERROR && drives==0);
        reset();
        if (side) { ir.right_status=IR_RANGE_NEAR;ir.right_cm_x10=100; }
        else { ir.left_status=IR_RANGE_NEAR;ir.left_cm_x10=100; }
        assert(Straight_StartUntilIRCm(10,side,STRAIGHT_IR_LT)==0);
        assert(Straight_State()==STRAIGHT_DONE && drives==0);
        reset();ir.left_cm_x10=ir.right_cm_x10=150;
        assert(Straight_StartUntilIRCm(20,side,STRAIGHT_IR_LT)==0);
        assert(Straight_State()==STRAIGHT_DONE && drives==0);
        assert(Straight_StartUntilIRGreaterCm(20,side)==0);
        assert(Straight_IRComparison()==STRAIGHT_IR_GT && Straight_State()==STRAIGHT_RUNNING);
    }
    reset();assert(Straight_StartUntilIRCm(20,0,(straight_ir_comparison_t)2)==-2);
    assert(drives==0);
    assert(Straight_StartUntilIRCm(20,0,STRAIGHT_IR_LT)==0);
    tick=30000;Straight_Step();assert(Straight_State()==STRAIGHT_TIMEOUT);
    reset();assert(Straight_StartUntilIRCm(20,0,STRAIGHT_IR_LT)==0);
    encoder_delta=16000;step();assert(Straight_State()==STRAIGHT_DISTANCE_LIMIT);
    puts("PASS: GT/LT, NEAR, LT sensor faults; existing: IR thresholds, side selection, FAR/ADC failure, STOP, modes, bounds, busy, timeout, travel limit");
    return 0;
}
