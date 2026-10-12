/* Host test links the production straight controller with simulated hardware. */
#include <assert.h>
#include <stdio.h>
#include "control/straight_control.h"
#include "control/straight_tuning.h"
#include "control/motion_heading.h"
#include "drivers/robot_hw.h"
#include "sensors/ir_sensor.h"
#include "sensors/ultrasonic_sensor.h"
static uint32_t tick;
static int drives, adc_fail, ir_reads, mock_takeups;
static uint32_t last_left_pwm, last_right_pwm;
static int16_t encoder_delta;
static uint32_t mock_brake_motion_ms;
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
void RobotHW_ServoSetUsTakeup(uint32_t x) { mock_takeups++; mock_servo = x; }
uint32_t RobotHW_ServoGetUs(void) { return mock_servo; }
uint32_t RobotHW_ServoCenterUs(void) { return SERVO_CENTER_US; }
int RobotHW_PrepareMotionSensors(void) { mock_yaw = 0.0f; return mock_prepare_rc; }
void RobotHW_GyroPrimeFilter(float a) { (void)a; }
void RobotHW_YawReset(void) { mock_yaw = mock_last_rate = 0.0f; }
void RobotHW_EncoderReset(void) {}
RobotHW_EncoderSnapshot RobotHW_EncoderSnapshotRead(void)
{ RobotHW_EncoderSnapshot s = {0, 0}; return s; }
RobotHW_EncoderDelta RobotHW_EncoderDeltaRead(RobotHW_EncoderSnapshot *s)
{
    int16_t delta = mock_braking && mock_brake_motion_ms
        ? (tick - mock_brake_begin_tick < mock_brake_motion_ms ? 1 : 0) : encoder_delta;
    RobotHW_EncoderDelta d = {delta, delta}; (void)s; return d;
}
void RobotHW_MotorSetForward(uint32_t l, uint32_t r)
{ last_left_pwm=l; last_right_pwm=r; drives++; mock_direction = 1; mock_drive_servo = mock_servo; }
void RobotHW_MotorSetBackward(uint32_t l, uint32_t r)
{ last_left_pwm=l; last_right_pwm=r; drives++; mock_direction = -1; mock_drive_servo = mock_servo; }
void RobotHW_MotorSetTurnForward(uint32_t l,uint32_t r) { RobotHW_MotorSetForward(l,r); }
void RobotHW_MotorSetTurnBackward(uint32_t l,uint32_t r) { RobotHW_MotorSetBackward(l,r); }
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
int IRSensor_ReadDistance(IRSensorDistanceReading *r,uint32_t n) { (void)n;ir_reads++;*r=ir;return adc_fail ? -1 : 0; }
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
    Straight_Init();MotionHeading_SetEnabled(1U);tick=0;drives=0;adc_fail=0;ir_reads=0;encoder_delta=0;mock_takeups=0;
    mock_yaw = mock_rate = mock_brake_rate = mock_last_rate = 0.0f;
    mock_gyro_ok = 1U; mock_braking = mock_brake_sample_pending = 0U;
    mock_servo = mock_drive_servo = SERVO_CENTER_US;
    mock_gyro_delay_ms = mock_brake_begin_tick = mock_brake_end_tick = 0U;
    mock_prepare_rc = mock_direction = 0; mock_brake_motion_ms = 0;
    ir=zero;ir.left_cm_x10=500;ir.right_cm_x10=500;
}
static void step(void) { tick+=40;Straight_Step(); }
static void echo(uint32_t mm, int result)
{
    mock_ultrasonic.distance_mm = mm;
    mock_ultrasonic.status = result;
    mock_ultrasonic.timestamp_ms = tick + 40U;
    mock_ultrasonic.sequence++;
    step();
}
static void test_untilus_start_takeup(void)
{
    for (int dir=-1; dir<=1; dir+=2) {
        reset();
        assert(Straight_StartUntilUltrasonicDirectionCm(35,dir)==0);
        assert(mock_takeups==1 && drives==0);
        assert(Straight_ForwardUSStartMm==UINT32_MAX && Straight_BackwardUSStartMm==UINT32_MAX);
        echo(dir>0 ? 500 : 200,0); assert(drives==0);
        assert((dir>0 ? Straight_ForwardUSStartMm : Straight_BackwardUSStartMm)==(dir>0 ? 500U : 200U));
        assert((dir>0 ? Straight_BackwardUSStartMm : Straight_ForwardUSStartMm)==UINT32_MAX);
        echo(dir>0 ? 500 : 200,0);
        assert(drives>0 && mock_direction==dir && mock_takeups==1);
        Straight_Stop();
    }
    puts("PASS: UNTILUS backlash positioning precedes drive in both gears");
}

static void test_untilus_auto(void)
{
    for(int dir=-1;dir<=1;dir++) {
        reset(); Straight_ForwardUSStartMm=700;Straight_ForwardUSEndMm=600;
        Straight_BackwardUSStartMm=100;Straight_BackwardUSEndMm=200;
        assert(Straight_StartUntilUltrasonicAutoCm(35)==0 && drives==0);
        uint32_t first=dir<0?200:dir>0?500:350;
        echo(first,0);assert(drives==0 && Straight_AutoUSStartMm==first);
        echo(first,0);
        if(dir) {
            assert(drives>0 && mock_direction==dir);
            echo(0,-1);assert(Straight_State()==STRAIGHT_RUNNING && mock_direction==dir);
            echo(dir>0?390:310,0);echo(dir>0?380:320,0);
        }
        assert(Straight_State()==STRAIGHT_DONE && Straight_AutoUSEndMm==(dir>0?380U:dir<0?320U:350U));
        assert(Straight_ForwardUSStartMm==700 && Straight_ForwardUSEndMm==600);
        assert(Straight_BackwardUSStartMm==100 && Straight_BackwardUSEndMm==200);
        assert(Straight_StartUntilUltrasonicAutoCm(35)==0);
        assert(Straight_AutoUSStartMm==UINT32_MAX && Straight_AutoUSEndMm==UINT32_MAX);
        echo(0,-1);Straight_Stop();assert(Straight_AutoUSEndMm==UINT32_MAX);
    }
    reset();assert(Straight_StartUntilUltrasonicAutoCm(35)==0);
    echo(500,0);echo(200,0);assert(drives==0);echo(210,0);assert(mock_direction==-1);
    Straight_Stop();
    reset();assert(Straight_StartUntilUltrasonicAutoCm(35)==0);
    tick=30000;Straight_Step();assert(Straight_State()==STRAIGHT_TIMEOUT && drives==0);
    reset();Straight_BackwardUSStartMm=123;Straight_BackwardUSEndMm=234;
    assert(Straight_StartUntilUltrasonicAutoCm(35)==0);
    echo(0,-1);assert(drives>0 && mock_direction==-1 && Straight_AutoUSStartMm==0);
    assert(last_left_pwm==US_FINAL_APPROACH_PWM && last_right_pwm==US_FINAL_APPROACH_PWM);
    encoder_delta=6;echo(0,-1);encoder_delta=0;
    echo(200,0);assert(mock_direction==-1 && Straight_AutoUSStartMm==0);
    echo(310,0);assert(Straight_State()==STRAIGHT_RUNNING);
    echo(320,0);assert(Straight_State()==STRAIGHT_DONE && Straight_AutoUSEndMm==320);
    assert(Straight_BackwardUSStartMm==123 && Straight_BackwardUSEndMm==234);
    reset();assert(Straight_StartUntilUltrasonicAutoCm(35)==0);
    echo(0,-1);Straight_Stop();assert(Straight_AutoUSStartMm==0 && Straight_AutoUSEndMm==UINT32_MAX);
    reset();assert(Straight_StartUntilUltrasonicDirectionCm(35,-1)==0);
    echo(0,-1);assert(Straight_BackwardUSStartMm==0 && mock_direction==-1);
    encoder_delta=6;echo(0,-1);encoder_delta=0;
    echo(200,0);assert(Straight_BackwardUSStartMm==0);
    echo(310,0);echo(320,0);
    assert(Straight_State()==STRAIGHT_DONE && Straight_BackwardUSEndMm==320);
    assert(Straight_AutoUSStartMm==UINT32_MAX);
    puts("PASS: AUTO selection, zero-cache missing-echo reverse creep/recovery, independent caches, STOP and timeout");
}

static void test_untilus_direction_caches(void)
{
    reset();
    assert(Straight_StartUntilUltrasonicDirectionCm(35,-1)==0);
    echo(200,0); echo(210,0); echo(300,0); echo(310,0);
    assert(Straight_State()==STRAIGHT_DONE);
    assert(Straight_BackwardUSStartMm==200 && Straight_BackwardUSEndMm==310);
    assert(Straight_ForwardUSStartMm==UINT32_MAX && Straight_ForwardUSEndMm==UINT32_MAX);
    for(int explicit_dir=0;explicit_dir<2;explicit_dir++) {
        assert((explicit_dir ? Straight_StartUntilUltrasonicDirectionCm(35,1)
                             : Straight_StartUntilUltrasonicCm(35))==0);
        assert(Straight_ForwardUSStartMm==UINT32_MAX && Straight_ForwardUSEndMm==UINT32_MAX);
        echo(500,0); echo(490,0); echo(390,0); echo(380,0);
        assert(Straight_State()==STRAIGHT_DONE);
        assert(Straight_ForwardUSStartMm==500 && Straight_ForwardUSEndMm==380);
        assert(Straight_BackwardUSStartMm==200 && Straight_BackwardUSEndMm==310);
    }
    assert(Straight_StartUntilUltrasonicCm(35)==0);
    echo(200,0); echo(210,0); /* stationary arrival still belongs to forward */
    assert(Straight_State()==STRAIGHT_DONE && Straight_ForwardUSEndMm==210);
    assert(Straight_StartUntilUltrasonicCm(35)==0);
    echo(500,0); Straight_Stop();
    assert(Straight_ForwardUSStartMm==500 && Straight_ForwardUSEndMm==UINT32_MAX);
    assert(Straight_BackwardUSEndMm==310);
    puts("PASS: UNTILUS start/end gear pairs, implicit FORWARD, reset, isolation and cancellation");
}

static void test_untilus_bidirectional(void)
{
    reset(); assert(Straight_StartUntilUltrasonicCm(35)==0);
    echo(500,0); echo(500,0); assert(mock_direction==1);
    Straight_Stop(); assert(Straight_State()==STRAIGHT_ABORTED);
    reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,2)==-2);
    assert(Straight_StartUntilUltrasonicDirectionCm(4,1)==-2);
    assert(Straight_StartUntilUltrasonicDirectionCm(201,-1)==-2);
    for (int direction=-1; direction<=1; direction+=2)
    {
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        uint32_t initial=direction>0?500:200;
        uint32_t reached=direction>0?400:300;
        echo(initial,0); assert(drives==0);
        echo(initial,0); assert(mock_direction==direction);
        assert(Straight_StartUntilUltrasonicCm(35)==-3);
        encoder_delta=6; echo(initial,0); encoder_delta=0;
        assert(last_left_pwm==US_FINAL_APPROACH_PWM && last_right_pwm==US_FINAL_APPROACH_PWM);
        echo(0,-1); assert(Straight_State()==STRAIGHT_RUNNING && mock_direction==direction);
        assert(last_left_pwm==US_FINAL_APPROACH_PWM && last_right_pwm==US_FINAL_APPROACH_PWM);
        echo(reached,0); assert(Straight_State()==STRAIGHT_RUNNING);
        echo(reached,0); assert(Straight_State()==STRAIGHT_DONE);
        uint32_t mm; int8_t dir;
        assert(Straight_UltrasonicStopReading(&mm,0,&dir) && mm==reached && dir==direction);
        tick+=1000; assert(Straight_UltrasonicStopReading(&mm,0,&dir));

        /* The specified direction never flips when already beyond target. */
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(direction>0?240:500,0); echo(direction>0?240:500,0);
        assert(Straight_State()==STRAIGHT_DONE && drives==0);
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(350,0); echo(350,0); assert(mock_direction==direction); /* strict initial boundary */
        Straight_Stop();

        /* Missing startup echoes creep rather than report sensor error. */
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(0,-1); assert(Straight_State()==STRAIGHT_RUNNING && mock_direction==direction);
        assert(last_left_pwm==US_FINAL_APPROACH_PWM && last_right_pwm==US_FINAL_APPROACH_PWM);
        encoder_delta=6; echo(0,-2); encoder_delta=0;
        for(int i=0;i<8;i++) echo(0,-1);
        assert(Straight_State()==STRAIGHT_RUNNING && mock_direction==direction);
        echo(initial,0); assert(Straight_State()==STRAIGHT_RUNNING);
        echo(reached,0); echo(reached,0); assert(Straight_State()==STRAIGHT_DONE);
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(0,-1); encoder_delta=6; echo(0,-1); encoder_delta=0;
        tick+=30000; Straight_Step(); assert(Straight_State()==STRAIGHT_TIMEOUT);
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(0,-1); Straight_Stop(); assert(Straight_State()==STRAIGHT_ABORTED);
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(0,-1); tick+=401; Straight_Step(); assert(Straight_State()==STRAIGHT_STALL);

        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(initial,0); echo(initial,0); mock_brake_motion_ms=260;
        echo(reached,0); echo(reached,0);
        assert(Straight_State()==STRAIGHT_DONE);
        assert(mock_brake_end_tick-mock_brake_begin_tick>=300);
        reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,direction)==0);
        echo(initial,0); echo(initial,0); mock_brake_motion_ms=1000;
        echo(reached,0); echo(reached,0); assert(Straight_State()==STRAIGHT_STALL);
    }
    reset(); assert(Straight_StartUntilUltrasonicDirectionCm(35,-1)==0);
    echo(240,0); echo(240,0);
    echo(1023,0); assert(Straight_State()==STRAIGHT_RUNNING);
    echo(240,0); assert(Straight_State()==STRAIGHT_RUNNING);
    echo(300,0); echo(1023,0); assert(Straight_State()==STRAIGHT_RUNNING);
    echo(310,0); echo(320,0); assert(Straight_State()==STRAIGHT_DONE);
    reset(); assert(Straight_StartUntilUltrasonicCm(35)==0);
    echo(500,0); /* still waiting for startup confirmation */
    tick+=30001; Straight_Step();
    assert(Straight_State()==STRAIGHT_TIMEOUT && drives==0);
    puts("PASS: UNTILUS default/explicit fixed direction, startup equality/already reached, invalid-echo creep/recovery, spike rejection, STOP/stall/timeout, brake hold");
}
static void test_ir_edges(void)
{
    for (int dir=-1;dir<=1;dir+=2)
        for (uint8_t side=0;side<2;side++)
            for (int op=STRAIGHT_IR_RISING;op<=STRAIGHT_IR_FALLING;op++)
            {
                reset(); ir.left_status=ir.right_status=IR_RANGE_VALID;
                uint32_t first=op==STRAIGHT_IR_RISING?299:301;
                uint32_t finish=op==STRAIGHT_IR_RISING?301:299;
                ir.left_cm_x10=ir.right_cm_x10=finish;
                assert(Straight_StartUntilIRDirectionCm(30,side,op,dir)==0);
                step(); assert(Straight_State()==STRAIGHT_RUNNING && mock_direction==dir);
                if(side) ir.left_cm_x10=first; else ir.right_cm_x10=first;
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* other sensor cannot arm */
                if(side) ir.right_cm_x10=300; else ir.left_cm_x10=300;
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* equality does not arm */
                if(side) ir.right_cm_x10=finish; else ir.left_cm_x10=finish;
                step(); assert(Straight_State()==STRAIGHT_RUNNING);
                if(side) ir.right_cm_x10=first; else ir.left_cm_x10=first;
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* first sample does not arm */
                if(side) ir.right_cm_x10=300; else ir.left_cm_x10=300;
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* equality keeps arm */
                if(side) ir.right_cm_x10=finish; else ir.left_cm_x10=finish;
                step(); step(); assert(Straight_State()==STRAIGHT_RUNNING); /* lone first-side sample was reset */
                if(side) ir.right_cm_x10=first; else ir.left_cm_x10=first;
                step(); assert(Straight_State()==STRAIGHT_RUNNING);
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* now armed */
                if(side) ir.right_cm_x10=finish; else ir.left_cm_x10=finish;
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* lone final-side spike */
                if(side) ir.right_cm_x10=300; else ir.left_cm_x10=300;
                step(); assert(Straight_State()==STRAIGHT_RUNNING); /* reset final count, keep arm */
                if(side) ir.right_cm_x10=finish; else ir.left_cm_x10=finish;
                step(); assert(Straight_State()==STRAIGHT_RUNNING);
                step(); assert(Straight_State()==STRAIGHT_DONE);
                assert((side?Straight_RightIRStopCmX10:Straight_LeftIRStopCmX10)==finish);
                /* Starting another command must forget the old arm. */
                assert(Straight_StartUntilIRDirectionCm(30,side,op,dir)==0);
                step(); assert(Straight_State()==STRAIGHT_RUNNING);
                Straight_Stop(); assert(Straight_State()==STRAIGHT_ABORTED);
            }
    for(int op=STRAIGHT_IR_RISING;op<=STRAIGHT_IR_FALLING;op++)
    {
        reset(); ir.left_status=IR_RANGE_VALID; ir.left_cm_x10=300;
        assert(Straight_StartUntilIRDirectionCm(30,0,op,-1)==0);
        adc_fail=1; step(); assert(Straight_State()==STRAIGHT_SENSOR_ERROR);
        assert(Straight_LeftIRStopCmX10==UINT32_MAX);
        reset(); ir.left_status=op==STRAIGHT_IR_RISING?IR_RANGE_NEAR:IR_RANGE_FAR;
        ir.left_cm_x10=50;
        assert(Straight_StartUntilIRDirectionCm(30,0,op,1)==0);
        assert(Straight_State()==STRAIGHT_RUNNING); /* initial reading arms without stopping */
        step(); /* second first-phase sample arms */
        ir.left_status=op==STRAIGHT_IR_RISING?IR_RANGE_FAR:IR_RANGE_NEAR;
        step(); assert(Straight_State()==STRAIGHT_RUNNING);
        step(); assert(Straight_State()==STRAIGHT_DONE);
    }
    for(int op=STRAIGHT_IR_GT;op<=STRAIGHT_IR_LT;op++)
    {
        reset(); ir.left_status=IR_RANGE_VALID; ir.left_cm_x10=300;
        assert(Straight_StartUntilIRDirectionCm(30,0,op,-1)==0);
        ir.left_cm_x10=op==STRAIGHT_IR_GT?301:299;
        step(); assert(Straight_State()==STRAIGHT_RUNNING);
        ir.left_cm_x10=300; step(); assert(Straight_State()==STRAIGHT_RUNNING);
        ir.left_cm_x10=op==STRAIGHT_IR_GT?301:299;
        step(); assert(Straight_State()==STRAIGHT_RUNNING);
        step(); assert(Straight_State()==STRAIGHT_DONE);
    }
    puts("PASS: IR edges in both gears/sides, ordered crossings, equality, cache, reset, STOP, faults, NEAR/FAR");
}
static void test_gocm_ir(void)
{
    for(int dir=-1;dir<=1;dir+=2)
        for(uint8_t side=0;side<2;side++)
            for(int op=STRAIGHT_IR_GT;op<=STRAIGHT_IR_FALLING;op++)
            {
                reset(); ir.left_status=ir.right_status=IR_RANGE_VALID;
                ir.left_cm_x10=ir.right_cm_x10=op==STRAIGHT_IR_GT || op==STRAIGHT_IR_FALLING?301:299;
                assert(Straight_StartCmThenIR(dir*10,side,op,30)==0);
                assert(ir_reads==0 && mock_direction==dir);
                uint8_t active; int32_t distance;
                assert(Straight_IRGateStatus(&distance,&active) && distance==dir*10 && !active);
                step(); assert(ir_reads==0 && Straight_State()==STRAIGHT_RUNNING);
                encoder_delta=740; step(); encoder_delta=0;
                Straight_IRGateStatus(0,&active); assert(!active && ir_reads==0);
                encoder_delta=4; step(); encoder_delta=0;
                Straight_IRGateStatus(0,&active); assert(active && ir_reads==0);
                assert(mock_brake_begin_tick==0 && Straight_State()==STRAIGHT_RUNNING);
                step(); assert(ir_reads==1 && Straight_State()==STRAIGHT_RUNNING);
                step();
                if(op>=STRAIGHT_IR_RISING) {
                    assert(Straight_State()==STRAIGHT_RUNNING);
                    ir.left_cm_x10=ir.right_cm_x10=op==STRAIGHT_IR_RISING?301:299;
                    step(); assert(Straight_State()==STRAIGHT_RUNNING); step();
                }
                assert(Straight_State()==STRAIGHT_DONE);
            }
    reset(); assert(Straight_StartCmThenIR(1,0,STRAIGHT_IR_GT,30)==-2);
    assert(Straight_StartCmThenIR(10,0,STRAIGHT_IR_GT,30)==0); Straight_Stop();
    assert(Straight_State()==STRAIGHT_ABORTED && ir_reads==0);
    reset(); assert(Straight_StartCmThenIR(10,0,STRAIGHT_IR_GT,30)==0);
    tick+=30000; Straight_Step(); assert(Straight_State()==STRAIGHT_TIMEOUT);
    reset(); assert(Straight_StartCmThenIR(10,0,STRAIGHT_IR_GT,30)==0);
    encoder_delta=16000; step(); assert(Straight_State()==STRAIGHT_DISTANCE_LIMIT);
    reset(); assert(Straight_StartCmThenIR(0,0,STRAIGHT_IR_GT,30)==-2);
    assert(Straight_StartCmThenIR(201,0,STRAIGHT_IR_GT,30)==-2);
    assert(Straight_StartCmThenIR(10,2,STRAIGHT_IR_GT,30)==-2);
    puts("PASS: continuous GOCM+IR both gears/sides/operators, no pre-gate reads/brake, exact encoder gate, bounds/STOP/timeout/travel limit");
}
static void test_ir_low_speed(void)
{
    for (int dir=-1; dir<=1; dir+=2)
        for (uint8_t side=0; side<2; side++)
            for (int op=STRAIGHT_IR_GT; op<=STRAIGHT_IR_FALLING; op++)
                for (int gated=0; gated<2; gated++) {
                    reset(); ir.left_status=ir.right_status=IR_RANGE_VALID;
                    ir.left_cm_x10=ir.right_cm_x10=300; /* equality keeps every op running */
                    int rc=gated ? Straight_StartCmThenIR(dir*10,side,op,30)
                                 : Straight_StartUntilIRDirectionCm(30,side,op,dir);
                    assert(rc==0);
                    if (gated) { assert(last_left_pwm>IR_COMMAND_DETECT_PWM); encoder_delta=744; step(); encoder_delta=0; }
                    assert(last_left_pwm==IR_COMMAND_DETECT_PWM && last_right_pwm==IR_COMMAND_DETECT_PWM);
                    for (int i=0; i<5; i++) { step(); assert(last_left_pwm==IR_COMMAND_DETECT_PWM && last_right_pwm==IR_COMMAND_DETECT_PWM); }
                    assert(Straight_State()==STRAIGHT_RUNNING); Straight_Stop();
                }
    puts("PASS: all IR gears/sides/operators use low speed at startup and GOCM gate");
}

int main(void)
{
    test_ir_low_speed();
    test_untilus_start_takeup();
    test_untilus_direction_caches();
    test_untilus_auto();
    test_untilus_bidirectional();
    test_ir_edges();
    test_gocm_ir();
    for (int dir=-1;dir<=1;dir+=2)
        for (uint8_t side=0;side<2;side++)
            for (int op=STRAIGHT_IR_GT;op<=STRAIGHT_IR_LT;op++)
            {
                reset(); ir.left_cm_x10=ir.right_cm_x10=300;
                ir.left_status=ir.right_status=IR_RANGE_VALID;
                assert(Straight_StartUntilIRDirectionCm(30,side,op,dir)==0);
                step(); assert(mock_direction==dir && Straight_State()==STRAIGHT_RUNNING);
                uint32_t trigger=op==STRAIGHT_IR_GT?301:299;
                if(side) ir.right_cm_x10=trigger; else ir.left_cm_x10=trigger;
                step(); step(); assert(Straight_State()==STRAIGHT_DONE);
                assert((side?Straight_RightIRStopCmX10:Straight_LeftIRStopCmX10)==trigger);
            }
    reset(); assert(Straight_StartUntilIRDirectionCm(30,0,STRAIGHT_IR_GT,0)==-2);
    reset(); ir.left_status=IR_RANGE_VALID; ir.left_cm_x10=300;
    assert(Straight_StartUntilIRDirectionCm(20,0,STRAIGHT_IR_LT,-1)==0);
    step(); Straight_Stop(); assert(Straight_State()==STRAIGHT_ABORTED);
    reset(); ir.left_status=IR_RANGE_VALID; ir.left_cm_x10=300;
    assert(Straight_StartUntilIRDirectionCm(20,0,STRAIGHT_IR_LT,-1)==0);
    adc_fail=1; step(); assert(Straight_State()==STRAIGHT_SENSOR_ERROR);
    puts("PASS: IR directions x sides x comparisons, cache, STOP and sensor faults");

    reset(); assert(Straight_StartUntilIRGreaterValidCm(50, 0) == 0);
    step(); assert(Straight_State() == STRAIGHT_RUNNING); /* equality */
    adc_fail = 1; step(); assert(Straight_State() == STRAIGHT_SENSOR_ERROR);
    reset(); adc_fail = 1;
    assert(Straight_StartUntilIRGreaterValidCm(50, 1) == 0);
    assert(Straight_State() == STRAIGHT_SENSOR_ERROR && drives == 0);
    reset(); assert(Straight_StartUntilIRGreaterValidCm(50, 1) == 0);
    ir.right_status = IR_RANGE_FAR; step(); step(); assert(Straight_State() == STRAIGHT_DONE);
    reset(); assert(Straight_StartUntilIRGreaterValidCm(50, 0) == 0);
    ir.left_cm_x10 = 501; step(); step(); assert(Straight_State() == STRAIGHT_DONE);

    reset();
    assert(Straight_LeftIRStopCmX10 == UINT32_MAX && Straight_RightIRStopCmX10 == UINT32_MAX);
    ir.left_cm_x10 = 501;
    assert(Straight_StartUntilIRCm(50, 0, STRAIGHT_IR_GT) == 0);
    assert(Straight_LeftIRStopCmX10 == 501 && Straight_RightIRStopCmX10 == UINT32_MAX);
    assert(Straight_StartUntilIRCm(20, 1, STRAIGHT_IR_LT) == 0);
    assert(Straight_LeftIRStopCmX10 == 501 && Straight_RightIRStopCmX10 == UINT32_MAX);
    ir.right_cm_x10 = 199; step(); step();
    assert(Straight_State() == STRAIGHT_DONE);
    assert(Straight_LeftIRStopCmX10 == 501 && Straight_RightIRStopCmX10 == 199);
    ir.right_cm_x10 = 500;
    assert(Straight_StartUntilIRCm(20, 1, STRAIGHT_IR_LT) == 0);
    adc_fail = 1; step();
    assert(Straight_State() == STRAIGHT_SENSOR_ERROR && Straight_RightIRStopCmX10 == 199);
    adc_fail = 0;
    assert(Straight_StartCm(20) == 0); Straight_Stop();
    assert(Straight_LeftIRStopCmX10 == 501 && Straight_RightIRStopCmX10 == 199);
    assert(Straight_StartUntilIRCm(50, 1, STRAIGHT_IR_GT) == 0); Straight_Stop();
    assert(Straight_RightIRStopCmX10 == 199);
    assert(Straight_StartUntilIRCm(50, 1, STRAIGHT_IR_GT) == 0);
    tick += 30000; Straight_Step();
    assert(Straight_State() == STRAIGHT_TIMEOUT && Straight_RightIRStopCmX10 == 199);
    ir.right_status = IR_RANGE_FAR;
    assert(Straight_StartUntilIRCm(50, 1, STRAIGHT_IR_GT) == 0);
    assert(Straight_RightIRStopCmX10 == UINT32_MAX && Straight_LeftIRStopCmX10 == 501);
    reset();assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    assert(Straight_State()==STRAIGHT_RUNNING && drives==1); /* equality */
    ir.right_status=IR_RANGE_FAR;step();assert(Straight_State()==STRAIGHT_RUNNING);
    ir.left_status=IR_RANGE_FAR;step(); step();assert(Straight_State()==STRAIGHT_DONE);
    reset();ir.left_cm_x10=501;assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    assert(Straight_State()==STRAIGHT_DONE && drives==0);
    reset();ir.right_cm_x10=501;assert(Straight_StartUntilIRGreaterCm(50,1)==0);
    assert(Straight_State()==STRAIGHT_DONE && drives==0);
    reset();adc_fail=1;assert(Straight_StartUntilIRGreaterCm(50,0)==0);
    assert(Straight_State()==STRAIGHT_DONE && drives==0);
    reset();assert(Straight_StartUntilIRGreaterCm(50,1)==0);adc_fail=1;step(); step();
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
        step(); step();assert(Straight_State()==STRAIGHT_DONE);
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
    reset();assert(Straight_StartUntilIRCm(20,0,(straight_ir_comparison_t)4)==-2);
    assert(drives==0);
    assert(Straight_StartUntilIRCm(20,0,STRAIGHT_IR_LT)==0);
    tick=30000;Straight_Step();assert(Straight_State()==STRAIGHT_TIMEOUT);
    reset();assert(Straight_StartUntilIRCm(20,0,STRAIGHT_IR_LT)==0);
    encoder_delta=16000;step();assert(Straight_State()==STRAIGHT_DISTANCE_LIMIT);
    puts("PASS: GT/LT, NEAR, LT sensor faults; existing: IR thresholds, side selection, FAR/ADC failure, STOP, modes, bounds, busy, timeout, travel limit");
    return 0;
}
