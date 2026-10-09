#include "main.h"
#include "control/straight_control.h"
#include "control/straight_tuning.h"
#include "control/motion_heading.h"
#include "drivers/robot_hw.h"
#include "sensors/ultrasonic_sensor.h"
#include "sensors/ir_sensor.h"

#include <stdint.h>
#include <stdlib.h>

/*
 * Production straight-distance controller.
 *
 * This is the single source of truth used by both the assessment command
 * interface (STRAIGHT,GOCM,<cm>) and the straight calibration test.
 *
 * Control strategy ported from the accurate distance tests:
 *   - Gyro yaw/rate -> steering-servo PID correction around the calibrated centre.
 *   - Filtered per-20-ms encoder mismatch -> left/right motor PI sync.
 *   - 10 to 20 cm uses the short-distance kickstart + triangular
 *     speed ramp derived from the short-distance donor test.
 *   - Distances above 20 cm use the donor long-distance speed/stop profile.
 *
 * The donor accelerometer roll/pitch calculations are intentionally not in
 * this controller because they do not feed yaw, steering, motor PWM, or the
 * distance stop decision. Accelerometer testing remains a separate component
 * diagnostic.
 */

/* Tuning constants are kept in control/straight_tuning.h. */

typedef enum
{
    STRAIGHT_PROFILE_LONG = 0,
    STRAIGHT_PROFILE_SHORT
} straight_profile_t;

static straight_state_t sc_state = STRAIGHT_IDLE;
static straight_profile_t sc_profile = STRAIGHT_PROFILE_LONG;
static int8_t sc_dir = 1;
static uint16_t sc_fixed_steer_us;
static uint8_t sc_until_us;
static uint8_t sc_ir_mode; /* 0=off, 1=LEFTIR, 2=RIGHTIR */
static straight_ir_comparison_t sc_ir_comparison;
static int32_t sc_ir_threshold_cm;
static uint32_t sc_ir_cm_x10;
static uint32_t sc_ir_last_tick;
uint32_t Straight_LeftIRStopCmX10 = UINT32_MAX;
uint32_t Straight_RightIRStopCmX10 = UINT32_MAX;
static uint8_t sc_us_moving;
static int32_t sc_us_threshold_cm;
static ultrasonic_sample_t sc_us_sample;
static uint32_t sc_us_sequence;

static int32_t sc_target_cm_signed = 0;
static uint32_t sc_start_tick = 0U;
static uint32_t sc_drive_start_tick = 0U;
static uint32_t sc_last_step_tick = 0U;
static uint32_t sc_timeout_ms = 0U;
static uint32_t sc_final_elapsed_ms = 0U;
static RobotHW_EncoderSnapshot sc_prev_enc;
static int32_t sc_total_left_counts = 0;
static int32_t sc_total_right_counts = 0;
static float sc_target_left_counts = 0.0f;
static float sc_target_right_counts = 0.0f;
static float sc_yaw_deg = 0.0f;
static float sc_steer_integral = 0.0f;
static uint8_t sc_steer_saturated_prev = 0U;
static float sc_sync_integral = 0.0f;
static float sc_speed_error_filt = 0.0f;
static uint8_t sc_us_approach_mode = 0U;
static uint32_t sc_us_base_pwm = 0U;
static uint8_t sc_us_kick_complete = 0U;
static uint8_t sc_us_start_confirm_count = 0U;
static uint8_t sc_us_stop_confirm_count = 0U;
static uint8_t sc_us_fail_count = 0U;
static uint8_t sc_gyro_fail_count = 0U;
static uint8_t sc_braking;
static uint32_t sc_brake_start_tick;

static straight_trace_sample_t sc_trace[STRAIGHT_TRACE_CAPACITY];
static uint32_t sc_trace_count, sc_trace_dropped, sc_trace_tick;

static void TraceReset(void)
{
    sc_trace_count = sc_trace_dropped = 0U;
    sc_trace_tick = HAL_GetTick();
}

static void TraceCapture(uint8_t phase, uint8_t force)
{
    uint32_t now = HAL_GetTick();
    if (!force && now - sc_trace_tick < STRAIGHT_TRACE_PERIOD_MS) return;
    sc_trace_tick = now;
    /* Reserve one slot for the terminal sample, even on a long run. */
    if (sc_trace_count >= STRAIGHT_TRACE_CAPACITY - 1U && phase != 'E')
    { ++sc_trace_dropped; return; }
    if (sc_trace_count >= STRAIGHT_TRACE_CAPACITY) return;
    straight_trace_sample_t *sample = &sc_trace[sc_trace_count++];
    sample->elapsed_ms = now - sc_start_tick;
    sample->local_yaw_deg = RobotHW_YawGetDeg();
    sample->control_error_deg = MotionHeading_ControlErrorDeg(sample->local_yaw_deg);
    sample->rate_dps = RobotHW_GyroGetLastZDps();
    sample->servo_us = RobotHW_ServoGetUs();
    sample->direction = sc_dir;
    sample->phase = phase;
    sample->gyro_ok = RobotHW_GyroLastReadOk();
}

uint32_t Straight_TraceCount(void) { return sc_trace_count; }
uint32_t Straight_TraceDropped(void) { return sc_trace_dropped; }
uint8_t Straight_TraceRead(uint32_t index, straight_trace_sample_t *sample)
{
    if (!sample || index >= sc_trace_count) return 0U;
    *sample = sc_trace[index];
    return 1U;
}

static uint8_t ReadIRStopCondition(void)
{
    IRSensorDistanceReading reading;
    IRSensorRangeStatus status = IR_RANGE_ERROR;
    sc_ir_cm_x10 = UINT32_MAX;
    if (IRSensor_ReadDistance(&reading, 1U) == 0)
    {
        status = (sc_ir_mode == 1U) ? reading.left_status : reading.right_status;
        if (status == IR_RANGE_VALID || status == IR_RANGE_NEAR)
            sc_ir_cm_x10 = (sc_ir_mode == 1U)
                          ? reading.left_cm_x10 : reading.right_cm_x10;
    }
    sc_ir_last_tick = HAL_GetTick();
    /* Return 2 for LT sensor faults: stop, but do not advance a sequence. */
    if (sc_ir_comparison == STRAIGHT_IR_LT)
    {
        if (status == IR_RANGE_ERROR) return 2U;
        if (status == IR_RANGE_NEAR) return 1U; /* Below calibrated 10 cm. */
        return sc_ir_cm_x10 < (uint32_t)sc_ir_threshold_cm * 10U;
    }
    /* Preserve legacy GT infinity policy for FAR and failed reads. */
    return sc_ir_cm_x10 > (uint32_t)sc_ir_threshold_cm * 10U;
}

/* IR/US stops already block for the brake pulse. Sample during that existing
 * wait so their final heading transfers to the next route segment too. */
static void StopWithHeadingSamples(void)
{
    RobotHW_MotorBrakeBegin();
    uint32_t start = HAL_GetTick();
    uint32_t previous = sc_last_step_tick;
    while (1)
    {
        uint32_t now = HAL_GetTick();
        uint8_t done = now - start >= MOTOR_BRAKE_PULSE_MS;
        if (done) RobotHW_MotorBrakeRelease();
        if (done || now - previous >= CONTROL_PERIOD_MS)
        {
            RobotHW_EncoderDelta delta = RobotHW_EncoderDeltaRead(&sc_prev_enc);
            sc_total_left_counts += abs((int)delta.left_delta);
            sc_total_right_counts += abs((int)delta.right_delta);
            (void)RobotHW_GyroUpdateYaw((float)(now - previous) / 1000.0f,
                GYRO_IIR_ALPHA, GYRO_RATE_DEADBAND_DPS);
            MotionHeading_Observe(RobotHW_YawGetDeg(), RobotHW_GyroLastReadOk());
            sc_yaw_deg = MotionHeading_ErrorDeg();
            previous = now;
        }
        if (done) break;
        HAL_Delay(1U);
    }
    RobotHW_ServoCenter();
    MotionHeading_End();
}

static void FinishIR(straight_state_t state)
{
    if (state == STRAIGHT_DONE)
    {
        /* Keep the condition-triggering sample before braking. */
        if (sc_ir_mode == 1U) Straight_LeftIRStopCmX10 = sc_ir_cm_x10;
        else if (sc_ir_mode == 2U) Straight_RightIRStopCmX10 = sc_ir_cm_x10;
        StopWithHeadingSamples();
    }
    else
    {
        RobotHW_MotorStop();
        RobotHW_ServoCenter();
        MotionHeading_Degrade();
        MotionHeading_End();
    }
    sc_final_elapsed_ms = HAL_GetTick() - sc_start_tick;
    sc_state = state;
}

straight_ir_comparison_t Straight_IRComparison(void)
{
    return sc_ir_comparison;
}

int Straight_IRStatus(int32_t *threshold_cm, uint32_t *cm_x10)
{
    if (threshold_cm) *threshold_cm = sc_ir_threshold_cm;
    if (cm_x10) *cm_x10 = sc_ir_cm_x10;
    return sc_ir_mode;
}

static int32_t ClampSigned(int32_t value, int32_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void Command_MotorSet(int8_t dir, uint32_t left_pwm, uint32_t right_pwm)
{
    if (dir >= 0)
    {
        RobotHW_MotorSetForward(left_pwm, right_pwm);
    }
    else
    {
        RobotHW_MotorSetBackward(left_pwm, right_pwm);
    }
}

static uint32_t ComputeShortRampPWM(float progress)
{
    float pwm_f;

    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    if (progress < 0.5f)
    {
        float frac = progress / 0.5f;
        pwm_f = (float)SHORT_RAMP_START_PWM
              + frac * ((float)SHORT_RAMP_PEAK_PWM - (float)SHORT_RAMP_START_PWM);
    }
    else
    {
        float frac = (progress - 0.5f) / 0.5f;
        pwm_f = (float)SHORT_RAMP_PEAK_PWM
              - frac * ((float)SHORT_RAMP_PEAK_PWM - (float)SHORT_RAMP_END_PWM);
    }

    if (pwm_f < 0.0f) pwm_f = 0.0f;
    return (uint32_t)(pwm_f + 0.5f);
}

static uint32_t SlewPwm(uint32_t current, uint32_t target, float dt_sec)
{
    uint32_t step = (uint32_t)((float)US_PWM_SLEW_PER_SECOND * dt_sec + 0.5f);
    if (step == 0U) step = 1U;
    if (current < target)
    {
        uint32_t remaining = target - current;
        return current + ((remaining < step) ? remaining : step);
    }
    if (current > target)
    {
        uint32_t remaining = current - target;
        return current - ((remaining < step) ? remaining : step);
    }
    return current;
}

static uint16_t StraightCenterUs(int8_t dir)
{
    int32_t trim = (dir < 0) ? STRAIGHT_REVERSE_TRIM_US
                             : STRAIGHT_FORWARD_TRIM_US;
    return (uint16_t)((int32_t)RobotHW_ServoCenterUs() + trim);
}

static uint16_t ComputeSteeringUs(float dt_sec)
{
    if (sc_fixed_steer_us) return sc_fixed_steer_us;
    float yaw_for_control = MotionHeading_ControlErrorDeg(RobotHW_YawGetDeg());
    float yaw_rate_for_control = RobotHW_GyroGetLastZDps();
    float correction_us;
    float servo_us;
    float center_us = (float)StraightCenterUs(sc_dir);

    if (yaw_for_control > -YAW_DEADBAND_DEG &&
        yaw_for_control <  YAW_DEADBAND_DEG)
    {
        yaw_for_control = 0.0f;
    }

    if (!sc_steer_saturated_prev)
    {
        sc_steer_integral += yaw_for_control * dt_sec;
        if (sc_steer_integral > YAW_INT_LIMIT_DEG_S)
            sc_steer_integral = YAW_INT_LIMIT_DEG_S;
        if (sc_steer_integral < -YAW_INT_LIMIT_DEG_S)
            sc_steer_integral = -YAW_INT_LIMIT_DEG_S;
    }

    float kp = sc_dir < 0 ? REVERSE_KP_STEER_US_PER_DEG : FORWARD_KP_STEER_US_PER_DEG;
    float ki = sc_dir < 0 ? REVERSE_KI_STEER_US_PER_DEG_S : FORWARD_KI_STEER_US_PER_DEG_S;
    float kd = sc_dir < 0 ? REVERSE_KD_STEER_US_PER_DPS : FORWARD_KD_STEER_US_PER_DPS;
    correction_us = kp * yaw_for_control
                  + ki * sc_steer_integral
                  + kd * yaw_rate_for_control;

    if (correction_us > 0.0f && correction_us < STEER_DEADBAND_US)
        correction_us = STEER_DEADBAND_US;
    else if (correction_us < 0.0f && correction_us > -STEER_DEADBAND_US)
        correction_us = -STEER_DEADBAND_US;

    sc_steer_saturated_prev =
        (correction_us > MAX_STEER_CORRECTION_US ||
         correction_us < -MAX_STEER_CORRECTION_US) ? 1U : 0U;

    if (correction_us > MAX_STEER_CORRECTION_US)
        correction_us = MAX_STEER_CORRECTION_US;
    if (correction_us < -MAX_STEER_CORRECTION_US)
        correction_us = -MAX_STEER_CORRECTION_US;

    /* Reversing flips the steering/yaw relationship. */
    if (sc_dir < 0)
        correction_us = -correction_us;

    servo_us = center_us + correction_us;
    if (servo_us < (float)MIN_STEER_US) servo_us = (float)MIN_STEER_US;
    if (servo_us > (float)MAX_STEER_US) servo_us = (float)MAX_STEER_US;

    return (uint16_t)(servo_us + 0.5f);
}

void Straight_Init(void)
{
    Straight_LeftIRStopCmX10 = UINT32_MAX;
    Straight_RightIRStopCmX10 = UINT32_MAX;
    sc_fixed_steer_us = 0U;
    TraceReset();
    MotionHeading_Reset();
    sc_braking = 0U;
    sc_ir_mode = 0U;
    sc_state = STRAIGHT_IDLE;
    sc_until_us = 0U;
    sc_us_moving = 0U;
    sc_profile = STRAIGHT_PROFILE_LONG;
    sc_dir = 1;
    sc_target_cm_signed = 0;
    sc_start_tick = 0U;
    sc_drive_start_tick = 0U;
    sc_last_step_tick = 0U;
    sc_timeout_ms = 0U;
    sc_final_elapsed_ms = 0U;
    sc_total_left_counts = 0;
    sc_total_right_counts = 0;
    sc_target_left_counts = 0.0f;
    sc_target_right_counts = 0.0f;
    sc_yaw_deg = 0.0f;
    sc_steer_integral = 0.0f;
    sc_steer_saturated_prev = 0U;
    sc_sync_integral = 0.0f;
    sc_speed_error_filt = 0.0f;
    sc_us_approach_mode = 0U;
    sc_us_base_pwm = 0U;
    sc_us_kick_complete = 0U;
    sc_us_start_confirm_count = 0U;
    sc_us_stop_confirm_count = 0U;
    sc_us_fail_count = 0U;
    sc_gyro_fail_count = 0U;

    RobotHW_MotorStop();
    RobotHW_ServoCenter();
}

static int Straight_StartCmInternal(int32_t target_cm, uint8_t ir_mode,
                                    int32_t ir_threshold_cm, straight_ir_comparison_t comparison,
                                    uint16_t fixed_steer_us)
{
    int32_t abs_cm;
    int prep_rc;
    if (target_cm == INT32_MIN) return -2;
    abs_cm = (target_cm < 0) ? -target_cm : target_cm;
    float target_revolutions;

    if (sc_state == STRAIGHT_RUNNING) return -3;
    if (abs_cm < MIN_COMMAND_DISTANCE_CM || abs_cm > MAX_COMMAND_DISTANCE_CM)
        return -2;

    MotionHeading_Prepare();

    /* Apply trim before sensor preparation so steering settles before drive.
     * Take up the steering backlash from a consistent side so the straight
     * starts centred regardless of the previous turn direction. */
    if (fixed_steer_us)
        RobotHW_ServoSetUs(fixed_steer_us);
    else
        RobotHW_ServoSetUsTakeup(StraightCenterUs((target_cm < 0) ? -1 : 1));
    prep_rc = RobotHW_PrepareMotionSensors();
    if (prep_rc != 0) { MotionHeading_Degrade(); return prep_rc; }
    RobotHW_GyroPrimeFilter(GYRO_IIR_ALPHA);
    RobotHW_YawReset();

    (void)MotionHeading_Begin(0.0f);
    sc_braking = 0U;

    sc_until_us = 0U;
    sc_ir_mode = ir_mode;
    sc_ir_comparison = comparison;
    sc_ir_threshold_cm = ir_threshold_cm;
    sc_ir_cm_x10 = UINT32_MAX;
    sc_profile = (abs_cm <= SHORT_PROFILE_MAX_CM)
               ? STRAIGHT_PROFILE_SHORT
               : STRAIGHT_PROFILE_LONG;
    sc_dir = (target_cm < 0) ? -1 : 1;
    sc_fixed_steer_us = fixed_steer_us;
    sc_target_cm_signed = target_cm;
    sc_total_left_counts = 0;
    sc_total_right_counts = 0;
    sc_yaw_deg = MotionHeading_ErrorDeg();
    sc_steer_integral = 0.0f;
    sc_steer_saturated_prev = 0U;
    sc_sync_integral = 0.0f;
    sc_speed_error_filt = 0.0f;
    sc_us_approach_mode = 0U;
    sc_us_base_pwm = 0U;
    sc_us_kick_complete = 0U;
    sc_us_start_confirm_count = 0U;
    sc_us_stop_confirm_count = 0U;
    sc_us_fail_count = 0U;
    sc_gyro_fail_count = 0U;

    if (!sc_fixed_steer_us && MotionHeading_IsEnabled() &&
        (sc_yaw_deg > YAW_DEADBAND_DEG || sc_yaw_deg < -YAW_DEADBAND_DEG))
    {
        /* Apply inherited route error before drive in either gear. */
        RobotHW_ServoSetUs(ComputeSteeringUs(0.0f));
        uint32_t settle_start = HAL_GetTick();
        uint32_t sample_tick = settle_start;
        while (HAL_GetTick() - settle_start < STRAIGHT_HEADING_STEER_SETTLE_MS)
        {
            HAL_Delay(1U);
            uint32_t sample_now = HAL_GetTick();
            if (sample_now - sample_tick >= CONTROL_PERIOD_MS)
            {
                (void)RobotHW_GyroUpdateYaw((float)(sample_now - sample_tick) / 1000.0f,
                    GYRO_IIR_ALPHA, GYRO_RATE_DEADBAND_DPS);
                sample_tick = sample_now;
                MotionHeading_Observe(RobotHW_YawGetDeg(), RobotHW_GyroLastReadOk());
                if (!RobotHW_GyroLastReadOk())
                {
                    if (++sc_gyro_fail_count >= 5U) { MotionHeading_End(); return -1; }
                }
                else sc_gyro_fail_count = 0U;
            }
        }
        sc_yaw_deg = MotionHeading_ErrorDeg();
        if (RobotHW_GyroLastReadOk()) RobotHW_ServoSetUs(ComputeSteeringUs(0.0f));
    }

    RobotHW_EncoderReset();
    sc_prev_enc = RobotHW_EncoderSnapshotRead();

    if (sc_profile == STRAIGHT_PROFILE_SHORT)
    {
        const float overshoot_comp_cm = (sc_dir < 0)
            ? SHORT_REVERSE_OVERSHOOT_COMP_CM
            : SHORT_FORWARD_OVERSHOOT_COMP_CM;
        float compensated_target_cm =
            (float)abs_cm
            + SHORT_CALIBRATION_OFFSET_CM
            - overshoot_comp_cm;

        if (compensated_target_cm <= 0.0f) return -2;
        target_revolutions = compensated_target_cm / WHEEL_CIRCUMFERENCE_CM;
    }
    else
    {
        const float compensation_rev = (sc_dir < 0)
            ? LONG_REVERSE_REV_ERROR_COMPENSATION
            : LONG_FORWARD_REV_ERROR_COMPENSATION;
        target_revolutions =
            ((float)abs_cm / WHEEL_CIRCUMFERENCE_CM)
            - compensation_rev;

        if (target_revolutions <= 0.0f) return -2;
    }

    sc_target_left_counts = target_revolutions * LEFT_COUNTS_PER_REV;
    sc_target_right_counts = target_revolutions * RIGHT_COUNTS_PER_REV;

    sc_start_tick = HAL_GetTick();
    sc_drive_start_tick = sc_start_tick;
    sc_last_step_tick = sc_start_tick;
    sc_timeout_ms = (uint32_t)abs_cm * 300U + 5000U;
    sc_final_elapsed_ms = 0U;
    sc_state = STRAIGHT_RUNNING;

    TraceReset();
    TraceCapture('S', 1U);

    if (sc_ir_mode)
    {
        sc_timeout_ms = IR_COMMAND_TIMEOUT_MS;
        uint8_t result = ReadIRStopCondition();
        if (result)
            FinishIR(result == 2U ? STRAIGHT_SENSOR_ERROR : STRAIGHT_DONE);
        else
            Command_MotorSet(sc_dir, IR_COMMAND_KICK_PWM, IR_COMMAND_KICK_PWM);
    }
    else if (sc_profile == STRAIGHT_PROFILE_SHORT)
        Command_MotorSet(sc_dir, SHORT_KICKSTART_PWM, SHORT_KICKSTART_PWM);
    else
        Command_MotorSet(sc_dir, LONG_BASE_PWM, LONG_BASE_PWM);

    if (sc_state != STRAIGHT_RUNNING) TraceCapture('E', 1U);
    return 0;
}

int Straight_StartCm(int32_t target_cm)
{
    return Straight_StartCmInternal(target_cm, 0U, 0, STRAIGHT_IR_GT, 0U);
}

int Straight_StartCalibrationCm(int32_t target_cm, int32_t servo_us)
{
    if (target_cm < MIN_COMMAND_DISTANCE_CM || target_cm > STRAIGHT_CAL_MAX_DISTANCE_CM ||
        servo_us < (int32_t)SERVO_CENTER_US - STRAIGHT_CAL_MAX_OFFSET_US ||
        servo_us > (int32_t)SERVO_CENTER_US + STRAIGHT_CAL_MAX_OFFSET_US ||
        servo_us <= (int32_t)CMD_SERVO_LEFT_US || servo_us >= (int32_t)CMD_SERVO_RIGHT_US)
        return -2;
    return Straight_StartCmInternal(target_cm, 0U, 0, STRAIGHT_IR_GT, (uint16_t)servo_us);
}

uint32_t Straight_FixedSteeringUs(void) { return sc_fixed_steer_us; }

int Straight_StartUntilIRGreaterCm(int32_t threshold_cm, uint8_t side)
{
    return Straight_StartUntilIRCm(threshold_cm, side, STRAIGHT_IR_GT);
}

int Straight_StartUntilIRCm(int32_t threshold_cm, uint8_t side,
                            straight_ir_comparison_t comparison)
{
    if (comparison != STRAIGHT_IR_GT && comparison != STRAIGHT_IR_LT) return -2;
    /* Match the actual driver calibration: LEFT 10..50, RIGHT 10..60 cm. */
    if (side > 1U || threshold_cm < 10 || threshold_cm > (side == 0U ? 50 : 60))
        return -2;
    return Straight_StartCmInternal(MAX_COMMAND_DISTANCE_CM, side + 1U, threshold_cm, comparison, 0U);
}

/* RUNNING includes the stationary initial measurement wait, so every existing
 * motion interlock and STOP command also covers that phase. */
int Straight_StartUntilUltrasonicCm(int32_t threshold_cm)
{
    int prep_rc;
    if (sc_state == STRAIGHT_RUNNING) return -3;
    if (threshold_cm < US_MIN_THRESHOLD_CM || threshold_cm > US_MAX_THRESHOLD_CM)
        return -2;
    MotionHeading_Prepare();
    sc_braking = 0U;
    if (GYRO_RECALIBRATE_BEFORE_EACH_COMMAND) RobotHW_MotorStop();
    RobotHW_ServoSetUs(StraightCenterUs(1));
    if (UltrasonicSensor_Init() != 0) { MotionHeading_Degrade(); return -1; }
    prep_rc = RobotHW_PrepareMotionSensors();
    if (prep_rc != 0) { MotionHeading_Degrade(); return prep_rc; }
    RobotHW_GyroPrimeFilter(GYRO_IIR_ALPHA);
    RobotHW_YawReset();
    (void)MotionHeading_Begin(0.0f);
    RobotHW_EncoderReset();
    sc_prev_enc = RobotHW_EncoderSnapshotRead();
    sc_until_us = 1U;
    sc_ir_mode = 0U;
    sc_us_moving = 0U;
    sc_us_threshold_cm = threshold_cm;
    sc_dir = 1;
    sc_fixed_steer_us = 0U;
    sc_target_cm_signed = 0;
    sc_total_left_counts = sc_total_right_counts = 0;
    sc_yaw_deg = MotionHeading_ErrorDeg();
    sc_steer_integral = 0.0f;
    sc_steer_saturated_prev = 0U;
    sc_sync_integral = 0.0f;
    sc_speed_error_filt = 0.0f;
    sc_us_approach_mode = 0U;
    sc_us_base_pwm = US_KICKSTART_PWM;
    sc_us_kick_complete = 0U;
    sc_us_start_confirm_count = 0U;
    sc_us_stop_confirm_count = 0U;
    sc_us_fail_count = 0U;
    sc_gyro_fail_count = 0U;
    sc_start_tick = HAL_GetTick();
    sc_drive_start_tick = 0U;
    sc_last_step_tick = sc_start_tick;
    sc_timeout_ms = US_MOTION_TIMEOUT_MS;
    sc_final_elapsed_ms = 0U;
    UltrasonicSensor_Cancel();
    UltrasonicSensor_GetSample(&sc_us_sample);
    sc_us_sequence = sc_us_sample.sequence;
    sc_us_sample.status = -1;
    sc_us_sample.distance_mm = 0U;
    sc_us_sample.timestamp_ms = sc_start_tick;
    sc_state = STRAIGHT_RUNNING;
    TraceReset();
    TraceCapture('S', 1U);
    (void)UltrasonicSensor_Trigger();
    return 0;
}

static void FinishUltrasonic(straight_state_t state)
{
    UltrasonicSensor_Cancel();
    FinishIR(state);
}

/* Unknown clearance is infinity for UNTILUS. Keep this policy out of the
 * shared sensor driver and the standalone ground-truth sensor test. */
static uint32_t UltrasonicClearanceMm(uint32_t now)
{
    if (sc_us_sample.status != 0 || sc_us_sample.distance_mm == 0U ||
        now - sc_us_sample.timestamp_ms > US_SAMPLE_MAX_AGE_MS)
        return UINT32_MAX;
    return sc_us_sample.distance_mm;
}

static void StartUltrasonicDrive(uint32_t now)
{
    sc_us_moving = 1U;
    RobotHW_GyroPrimeFilter(GYRO_IIR_ALPHA);
    RobotHW_YawReset();
    MotionHeading_LocalYawReset();
    RobotHW_EncoderReset();
    sc_total_left_counts = sc_total_right_counts = 0;
    sc_yaw_deg = MotionHeading_ErrorDeg();
    sc_steer_integral = 0.0f;
    sc_steer_saturated_prev = 0U;
    sc_sync_integral = 0.0f;
    sc_speed_error_filt = 0.0f;
    sc_gyro_fail_count = 0U;
    sc_drive_start_tick = now;
    sc_us_base_pwm = US_KICKSTART_PWM;
    sc_us_kick_complete = 0U;
    sc_us_start_confirm_count = 0U;
    sc_us_stop_confirm_count = 0U;
    sc_us_fail_count = 0U;
    sc_last_step_tick = now;
    sc_prev_enc = RobotHW_EncoderSnapshotRead();
    RobotHW_ServoSetUs(ComputeSteeringUs(0.0f));
    Command_MotorSet(sc_dir, US_KICKSTART_PWM, US_KICKSTART_PWM);
}

static void StepUltrasonic(void)
{
    ultrasonic_sample_t sample;
    uint32_t now;
    UltrasonicSensor_Step();
    UltrasonicSensor_GetSample(&sample);
    now = HAL_GetTick();
    if (sample.sequence != sc_us_sequence)
    {
        sc_us_sequence = sample.sequence;
        sc_us_sample = sample;

        if (sample.status == 0 && sample.distance_mm > 0U)
        {
            sc_us_fail_count = 0U;

            if (sample.distance_mm < (uint32_t)sc_us_threshold_cm * 10U)
            {
                sc_us_start_confirm_count = 0U;
                if (!sc_us_moving)
                {
                    if (sc_us_stop_confirm_count < UINT8_MAX)
                        sc_us_stop_confirm_count++;
                    if (sc_us_stop_confirm_count >= US_START_CONFIRM_SAMPLES)
                    {
                        FinishUltrasonic(STRAIGHT_DONE);
                        return;
                    }
                }
                else
                {
                    if (sc_us_stop_confirm_count < UINT8_MAX)
                        sc_us_stop_confirm_count++;
                    if (sc_us_stop_confirm_count >= US_STOP_CONFIRM_SAMPLES)
                    {
                        FinishUltrasonic(STRAIGHT_DONE);
                        return;
                    }
                }
            }
            else
            {
                sc_us_stop_confirm_count = 0U;
                if (!sc_us_moving)
                {
                    if (sc_us_start_confirm_count < UINT8_MAX)
                        sc_us_start_confirm_count++;
                    if (sc_us_start_confirm_count >= US_START_CONFIRM_SAMPLES)
                    {
                        StartUltrasonicDrive(now);
                    }
                }
            }
        }
        else
        {
            sc_us_stop_confirm_count = 0U;
            sc_us_start_confirm_count = 0U;
            if (sc_us_fail_count < UINT8_MAX) sc_us_fail_count++;
            if (!sc_us_moving && sc_us_fail_count >= US_START_UNKNOWN_SAMPLES)
            {
                StartUltrasonicDrive(now);
            }
        }
    }
    /* Keep sampling after a brief no-echo result; -3 here only means
     * busy/cooldown.  A real trigger-driver failure is still fatal. */
    if (UltrasonicSensor_Trigger() == -1)
        FinishUltrasonic(STRAIGHT_SENSOR_ERROR);
}

int Straight_UltrasonicStatus(int32_t *threshold_cm, uint32_t *distance_mm,
                              int *status, uint32_t *age_ms)
{
    uint32_t age = HAL_GetTick() - sc_us_sample.timestamp_ms;
    if (threshold_cm) *threshold_cm = sc_us_threshold_cm;
    uint32_t clearance = UltrasonicClearanceMm(HAL_GetTick());
    if (distance_mm) *distance_mm = clearance;
    if (status) *status = (age > US_SAMPLE_MAX_AGE_MS) ? -1 :
                         (sc_us_sample.status != 0 ? sc_us_sample.status :
                          (clearance == UINT32_MAX ? -2 : 0));
    if (age_ms) *age_ms = age;
    return sc_until_us;
}

static void Straight_StepInternal(void)
{
    uint32_t now;
    uint32_t elapsed_ms;
    float dt_sec;
    RobotHW_EncoderDelta delta;
    int32_t left_counts;
    int32_t right_counts;
    float speed_error;
    int32_t sync_correction;
    uint32_t base_pwm;
    uint32_t left_pwm;
    uint32_t right_pwm;
    uint16_t steering_us;
    uint8_t force_equal_kick = 0U;
    uint8_t brake_done;
    uint8_t gyro_sample_ok;

    if (sc_state != STRAIGHT_RUNNING)
    {
        MotionHeading_PollIdle();
        return;
    }

    if (sc_ir_mode && HAL_GetTick() - sc_ir_last_tick >= IR_COMMAND_SAMPLE_MS)
    {
        uint8_t result = ReadIRStopCondition();
        if (result)
        {
            FinishIR(result == 2U ? STRAIGHT_SENSOR_ERROR : STRAIGHT_DONE);
            return;
        }
    }

    if (sc_until_us)
    {
        /* Retain yaw while waiting for the first ultrasound measurement,
         * before StartUltrasonicDrive resets the local gyro frame again. */
        if (!sc_us_moving && HAL_GetTick() - sc_last_step_tick >= CONTROL_PERIOD_MS)
        {
            uint32_t sample_now = HAL_GetTick();
            (void)RobotHW_GyroUpdateYaw((float)(sample_now - sc_last_step_tick) / 1000.0f,
                GYRO_IIR_ALPHA, GYRO_RATE_DEADBAND_DPS);
            sc_last_step_tick = sample_now;
            MotionHeading_Observe(RobotHW_YawGetDeg(), RobotHW_GyroLastReadOk());
            sc_yaw_deg = MotionHeading_ErrorDeg();
            if (!RobotHW_GyroLastReadOk())
            {
                if (++sc_gyro_fail_count >= 5U)
                { FinishUltrasonic(STRAIGHT_SENSOR_ERROR); return; }
            }
            else sc_gyro_fail_count = 0U;
        }
        StepUltrasonic();
        if (sc_state != STRAIGHT_RUNNING || !sc_us_moving) return;
    }
    now = HAL_GetTick();
    brake_done = sc_braking && (now - sc_brake_start_tick >= MOTOR_BRAKE_PULSE_MS);
    if (!brake_done && (now - sc_last_step_tick) < CONTROL_PERIOD_MS) return;
    /* Release at the brake deadline even between control ticks, before I2C
     * sampling can extend the electrical brake pulse. */
    if (brake_done) RobotHW_MotorBrakeRelease();

    dt_sec = (float)(now - sc_last_step_tick) / 1000.0f;
    sc_last_step_tick = now;
    elapsed_ms = now - sc_start_tick;

    if (!sc_braking && elapsed_ms >= sc_timeout_ms)
    {
        RobotHW_MotorStop();
        RobotHW_ServoCenter();
        sc_final_elapsed_ms = elapsed_ms;
        if (sc_until_us) UltrasonicSensor_Cancel();
        sc_state = STRAIGHT_TIMEOUT;
        MotionHeading_Degrade();
        MotionHeading_End();
        return;
    }

    delta = RobotHW_EncoderDeltaRead(&sc_prev_enc);
    left_counts = abs((int)delta.left_delta);
    right_counts = abs((int)delta.right_delta);
    sc_total_left_counts += left_counts;
    sc_total_right_counts += right_counts;

    if (sc_until_us &&
        ((float)sc_total_left_counts / LEFT_COUNTS_PER_CM >= US_MAX_TRAVEL_CM ||
         (float)sc_total_right_counts / RIGHT_COUNTS_PER_CM >= US_MAX_TRAVEL_CM))
    {
        FinishUltrasonic(STRAIGHT_DISTANCE_LIMIT);
        return;
    }

    if (sc_ir_mode &&
        ((float)sc_total_left_counts / LEFT_COUNTS_PER_CM >= IR_COMMAND_MAX_TRAVEL_CM ||
         (float)sc_total_right_counts / RIGHT_COUNTS_PER_CM >= IR_COMMAND_MAX_TRAVEL_CM))
    {
        FinishIR(STRAIGHT_DISTANCE_LIMIT);
        return;
    }

    (void)RobotHW_GyroUpdateYaw(dt_sec, GYRO_IIR_ALPHA,
                                GYRO_RATE_DEADBAND_DPS);
    gyro_sample_ok = RobotHW_GyroLastReadOk();
    MotionHeading_Observe(RobotHW_YawGetDeg(), gyro_sample_ok);
    if (!gyro_sample_ok)
    {
        if (++sc_gyro_fail_count >= 5U)
        {
            RobotHW_MotorStop();
            RobotHW_ServoCenter();
            if (sc_until_us) UltrasonicSensor_Cancel();
            sc_final_elapsed_ms = elapsed_ms;
            sc_state = STRAIGHT_SENSOR_ERROR;
            MotionHeading_End();
            return;
        }
    }
    else
    {
        sc_gyro_fail_count = 0U;
    }
    sc_yaw_deg = MotionHeading_ErrorDeg();

    if (sc_braking)
    {
        /* Keep the original 80 ms electrical brake and steering timing, but
         * continue measuring yaw/encoders so the return includes braking yaw. */
        if (brake_done)
        {
            RobotHW_ServoCenter();
            sc_braking = 0U;
            sc_final_elapsed_ms = elapsed_ms;
            sc_state = STRAIGHT_DONE;
            MotionHeading_End();
        }
        return;
    }

    if (!sc_until_us && !sc_ir_mode && (float)sc_total_left_counts >= sc_target_left_counts &&
        (float)sc_total_right_counts >= sc_target_right_counts)
    {
        RobotHW_MotorBrakeBegin();
        sc_brake_start_tick = HAL_GetTick();
        sc_braking = 1U;
        return;
    }

    /* A missed sample does not change steering or wind up the yaw integral.
     * Wheel synchronization/distance control continue while reads retry. */
    if (gyro_sample_ok)
    {
        steering_us = ComputeSteeringUs(dt_sec);
        RobotHW_ServoSetUs((uint32_t)steering_us);
    }

    /* Compare calibrated wheel travel, normalized to the nominal interval.
     * Previously x1000 saturated +/-250 PWM on any one-count mismatch.
     * Encoder deltas are already counts/interval, not counts/millisecond. */
    speed_error = ((float)left_counts / LEFT_COUNTS_PER_CM
                - (float)right_counts / RIGHT_COUNTS_PER_CM)
               * LEFT_COUNTS_PER_CM
               * ((float)CONTROL_PERIOD_MS / (dt_sec * 1000.0f));
    sc_speed_error_filt = SYNC_ERROR_FILTER_ALPHA * speed_error
                        + (1.0f - SYNC_ERROR_FILTER_ALPHA) * sc_speed_error_filt;
    sc_sync_integral += sc_speed_error_filt * dt_sec;
    if (sc_sync_integral > SYNC_INT_LIMIT_COUNTS_S)
        sc_sync_integral = SYNC_INT_LIMIT_COUNTS_S;
    if (sc_sync_integral < -SYNC_INT_LIMIT_COUNTS_S)
        sc_sync_integral = -SYNC_INT_LIMIT_COUNTS_S;
    {
        float sync_command = SYNC_KP * sc_speed_error_filt
                           + SYNC_KI * sc_sync_integral;

        if (sc_until_us)
        {
            float left_path_cm = (float)sc_total_left_counts / LEFT_COUNTS_PER_CM;
            float right_path_cm = (float)sc_total_right_counts / RIGHT_COUNTS_PER_CM;
            sync_command += US_PATH_SYNC_KP_PWM_PER_CM
                          * (left_path_cm - right_path_cm);
        }

        sync_correction = (int32_t)(-sync_command);
    }
    sync_correction = ClampSigned(sync_correction, MAX_SYNC_CORRECTION);

    if (sc_until_us)
    {
        uint32_t clearance = UltrasonicClearanceMm(now);
        uint32_t slow_boundary = (uint32_t)sc_us_threshold_cm * 10U
                               + US_SLOW_MARGIN_MM;
        uint32_t target_pwm;

        /* Hysteresis prevents noisy readings from repeatedly switching speed.
         * If an echo is briefly lost after slowing down, remain conservative. */
        if (clearance != UINT32_MAX)
        {
            if (!sc_us_approach_mode && clearance <= slow_boundary)
                sc_us_approach_mode = 1U;
            else if (sc_us_approach_mode &&
                     clearance > slow_boundary + US_SPEED_HYSTERESIS_MM)
                sc_us_approach_mode = 0U;
        }

        uint32_t drive_ms = now - sc_drive_start_tick;
        uint8_t both_wheels_started =
            (sc_total_left_counts >= US_KICKSTART_MIN_COUNTS &&
             sc_total_right_counts >= US_KICKSTART_MIN_COUNTS) ? 1U : 0U;

        if (!sc_us_kick_complete &&
            drive_ms >= US_KICKSTART_MIN_MS && both_wheels_started)
        {
            sc_us_kick_complete = 1U;
            /* Start normal synchronization with the measured path mismatch,
             * but without carrying kick-period speed spikes into the filter. */
            sc_sync_integral = 0.0f;
            sc_speed_error_filt = 0.0f;
        }

        if (!sc_us_kick_complete && drive_ms >= US_KICKSTART_MAX_MS)
        {
            if (!both_wheels_started)
            {
                FinishUltrasonic(STRAIGHT_STALL);
                return;
            }
            sc_us_kick_complete = 1U;
        }

        if (!sc_us_kick_complete)
        {
            sc_us_base_pwm = US_KICKSTART_PWM;
            base_pwm = US_KICKSTART_PWM;
            force_equal_kick = 1U;
        }
        else
        {
            target_pwm = sc_us_approach_mode
                       ? US_APPROACH_PWM : US_CRUISE_PWM;
            sc_us_base_pwm = SlewPwm(sc_us_base_pwm, target_pwm, dt_sec);
            base_pwm = sc_us_base_pwm;
        }
    }
    else if (sc_ir_mode)
    {
        force_equal_kick = elapsed_ms < IR_COMMAND_KICK_MS;
        base_pwm = force_equal_kick ? IR_COMMAND_KICK_PWM : IR_COMMAND_CRUISE_PWM;
    }
    else if (sc_profile == STRAIGHT_PROFILE_SHORT)
    {
        float avg_target_counts =
            (sc_target_left_counts + sc_target_right_counts) * 0.5f;
        float avg_current_counts =
            ((float)sc_total_left_counts + (float)sc_total_right_counts) * 0.5f;
        float progress = (avg_target_counts > 0.0f)
                       ? (avg_current_counts / avg_target_counts)
                       : 1.0f;

        base_pwm = ComputeShortRampPWM(progress);
        if (elapsed_ms < SHORT_KICKSTART_DURATION_MS)
        {
            base_pwm = SHORT_KICKSTART_PWM;
            force_equal_kick = 1U;
        }
    }
    else
    {
        float left_progress = (float)sc_total_left_counts / LEFT_COUNTS_PER_REV;
        float right_progress = (float)sc_total_right_counts / RIGHT_COUNTS_PER_REV;
        float progress = (left_progress > right_progress)
                       ? left_progress : right_progress;

        base_pwm = (progress >= LONG_SLOWDOWN_START_REV)
                 ? LONG_SLOW_PWM
                 : LONG_BASE_PWM;
    }

    if (force_equal_kick)
    {
        /* Break static friction on both wheels before applying differential
         * corrections; otherwise the weaker wheel can create initial yaw. */
        sc_sync_integral = 0.0f;
        sc_speed_error_filt = 0.0f;
        left_pwm = RobotHW_ClampPwm((int32_t)base_pwm,
                                    RobotHW_MotorLeftMax());
        right_pwm = RobotHW_ClampPwm((int32_t)base_pwm,
                                     RobotHW_MotorRightMax());
    }
    else
    {
        left_pwm = RobotHW_ClampPwm((int32_t)base_pwm + sync_correction,
                                    RobotHW_MotorLeftMax());
        right_pwm = RobotHW_ClampPwm((int32_t)base_pwm - sync_correction,
                                     RobotHW_MotorRightMax());
    }

    Command_MotorSet(sc_dir, left_pwm, right_pwm);
}

void Straight_Step(void)
{
    straight_state_t before = sc_state;
    uint8_t was_braking = sc_braking;
    uint8_t was_us_moving = sc_us_moving;
    Straight_StepInternal();
    if (before != STRAIGHT_RUNNING) return;
    if (sc_state != STRAIGHT_RUNNING) TraceCapture('E', 1U);
    else if (sc_braking) TraceCapture('B', !was_braking);
    else if (sc_until_us && !sc_us_moving) TraceCapture('W', 0U);
    else TraceCapture('D', sc_until_us && !was_us_moving);
}

void Straight_Stop(void)
{
    if (sc_state == STRAIGHT_RUNNING)
    {
        RobotHW_MotorStop();
        RobotHW_ServoCenter();
        sc_final_elapsed_ms = HAL_GetTick() - sc_start_tick;
        if (sc_until_us) UltrasonicSensor_Cancel();
        sc_state = STRAIGHT_ABORTED;
        MotionHeading_Degrade();
        MotionHeading_End();
        TraceCapture('E', 1U);
    }
}

straight_state_t Straight_State(void)
{
    return sc_state;
}

void Straight_Status(uint32_t *elapsed_ms,
                     int32_t *target_cm,
                     float *l_cm,
                     float *r_cm,
                     float *yaw_deg_out,
                     float *diff_cm)
{
    float sign = (sc_dir < 0) ? -1.0f : 1.0f;
    uint32_t elapsed = (sc_state == STRAIGHT_RUNNING)
                     ? (HAL_GetTick() - sc_start_tick)
                     : sc_final_elapsed_ms;
    float left_cm = (float)sc_total_left_counts / LEFT_COUNTS_PER_CM;
    float right_cm = (float)sc_total_right_counts / RIGHT_COUNTS_PER_CM;

    if (elapsed_ms) *elapsed_ms = elapsed;
    if (target_cm) *target_cm = sc_target_cm_signed;
    if (l_cm) *l_cm = sign * left_cm;
    if (r_cm) *r_cm = sign * right_cm;
    if (yaw_deg_out) *yaw_deg_out = sc_yaw_deg;
    if (diff_cm) *diff_cm = sign * (left_cm - right_cm);
}
