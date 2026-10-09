#include "test_config.h"
#if ACTIVE_TEST == TEST_FASTEST_CAR

#include "main.h"
#include "control/turn_tuning.h"
#include "drivers/oled.h"
#include "drivers/robot_hw.h"
#include "sensors/ir_sensor.h"
#include "sensors/ultrasonic_sensor.h"
#include "tests/test_fastest_car.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define FC_CONTROL_PERIOD_MS          20U
#define FC_IR_AVG_SAMPLES             4U
#define FC_US_MAX_AGE_MS            200U
#define FC_FRONT_CONFIRM_COUNT        2U
#define FC_EDGE_LOST_COUNT            3U

#define FC_SIDE_KP_NUM                6
#define FC_SIDE_KP_DEN                5
#define FC_MAX_STEER_CORRECTION_US  700
#define FC_HEADING_MAX_CORRECTION_US 250
#define FC_HEADING_DEADBAND_DEG     1.5f
#define FC_HEADING_RATE_DEADBAND_DPS 0.8f
#define FC_GYRO_IIR_ALPHA          0.10f
#define FC_GYRO_RATE_DEADBAND_DPS  0.08f
#define FC_HEADING_KP_US_PER_DEG   12.0f
#define FC_HEADING_KD_US_PER_DPS    1.5f

typedef enum
{
    FC_ARROW_LEFT = -1,
    FC_ARROW_RIGHT = 1
} fc_arrow_t;

typedef enum
{
    FC_STATE_SEEK_GOAL = 0,
    FC_STATE_READ_ARROW,
    FC_STATE_BYPASS_ENTRY,
    FC_STATE_BYPASS_FOLLOW,
    FC_STATE_BYPASS_EXIT,
    FC_STATE_RECENTER,
    FC_STATE_RETURN_ARC,
    FC_STATE_RETURN_DRIVE,
    FC_STATE_DONE,
    FC_STATE_FAIL
} fc_state_t;

typedef enum
{
    FC_FOLLOW_LEFT = 0,
    FC_FOLLOW_RIGHT
} fc_follow_side_t;

typedef struct
{
    uint8_t front_valid;
    uint32_t front_mm;
    uint8_t left_valid;
    uint8_t right_valid;
    uint16_t left_mm;
    uint16_t right_mm;
} fc_sensors_t;

static fc_state_t fc_state;
static uint32_t fc_state_tick;
static uint32_t fc_run_start_tick;
static uint32_t fc_last_control_tick;
static uint32_t fc_last_oled_tick;
static uint8_t fc_goal_index;
static uint8_t fc_front_confirm;
static uint8_t fc_edge_lost_confirm;
static uint8_t fc_follow_locked;
static fc_arrow_t fc_bypass_arrow;
static ultrasonic_sample_t fc_us_sample;
static float fc_yaw_deg;
static float fc_state_start_yaw_deg;
static float fc_heading_target_deg;
static int32_t fc_steering_correction_us;

static void UART_Print(const char *text)
{
    if (text == NULL) return;
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)text,
                            (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static const char *StateText(fc_state_t state)
{
    switch (state)
    {
        case FC_STATE_SEEK_GOAL:     return "SEEK";
        case FC_STATE_READ_ARROW:    return "ARROW";
        case FC_STATE_BYPASS_ENTRY:  return "ENTRY";
        case FC_STATE_BYPASS_FOLLOW: return "FOLLOW";
        case FC_STATE_BYPASS_EXIT:   return "EXIT";
        case FC_STATE_RECENTER:      return "RECENTER";
        case FC_STATE_RETURN_ARC:    return "RET_ARC";
        case FC_STATE_RETURN_DRIVE:  return "RET_DRV";
        case FC_STATE_DONE:          return "DONE";
        case FC_STATE_FAIL:          return "FAIL";
        default:                     return "UNKNOWN";
    }
}

static int32_t ClampI32(int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static uint32_t ClampServoUs(int32_t pulse_us)
{
    pulse_us = ClampI32(pulse_us, (int32_t)CMD_SERVO_LEFT_US,
                        (int32_t)CMD_SERVO_RIGHT_US);
    return (uint32_t)pulse_us;
}

static float FastestCar_ApproxAtan(float x)
{
    return x / (1.0f + 0.281f * x * x);
}

static int32_t FastestCar_LeftSafeCorrectionUs(void)
{
    float delta = FastestCar_ApproxAtan(
        CMD_WHEELBASE_CM / (float)FASTEST_CAR_LEFT_SAFE_RADIUS_CM);
    float max_delta = FastestCar_ApproxAtan(
        CMD_WHEELBASE_CM / (float)CMD_LEFT_MIN_RADIUS_CM);
    float us_span = (float)((int32_t)SERVO_CENTER_US - (int32_t)CMD_SERVO_LEFT_US)
                  * TURN_LEFT_STEER_SCALE;
    int32_t correction_us = (int32_t)((delta * us_span / max_delta) + 0.5f);

    if (correction_us < 0) correction_us = 0;
    if (correction_us > FC_MAX_STEER_CORRECTION_US)
        correction_us = FC_MAX_STEER_CORRECTION_US;
    return correction_us;
}

static void SetSteeringCorrection(int32_t correction_us)
{
    correction_us = ClampI32(correction_us,
                             -FastestCar_LeftSafeCorrectionUs(),
                              FC_MAX_STEER_CORRECTION_US);
    fc_steering_correction_us = correction_us;
    RobotHW_ServoSetUs(ClampServoUs((int32_t)SERVO_CENTER_US + correction_us));
}

static void SetSteeringCorrectionSmooth(int32_t target_us)
{
    int32_t delta_us = target_us - fc_steering_correction_us;

    delta_us = ClampI32(delta_us,
                        -FASTEST_CAR_STEER_SLEW_US_PER_CYCLE,
                         FASTEST_CAR_STEER_SLEW_US_PER_CYCLE);
    SetSteeringCorrection(fc_steering_correction_us + delta_us);
}

static uint32_t FrontStopMm(void)
{
    return FASTEST_CAR_FRONT_STOP_CM * 10U;
}

static uint32_t FrontEmergencyMm(void)
{
    return FASTEST_CAR_FRONT_EMERGENCY_CM * 10U;
}

static uint32_t SideTargetMm(void)
{
    return FASTEST_CAR_SIDE_TARGET_CM * 10U;
}

static uint32_t SideMinMm(void)
{
    return FASTEST_CAR_SIDE_MIN_CM * 10U;
}

static uint32_t EdgeLostMm(void)
{
    return FASTEST_CAR_EDGE_LOST_CM * 10U;
}

static uint8_t IrStatusUsable(IRSensorRangeStatus status)
{
    return (status == IR_RANGE_VALID || status == IR_RANGE_NEAR) ? 1U : 0U;
}

static void ReadSensors(fc_sensors_t *s)
{
    IRSensorDistanceReading ir;
    uint32_t now = HAL_GetTick();

    memset(s, 0, sizeof(*s));

    UltrasonicSensor_Step();
    UltrasonicSensor_GetSample(&fc_us_sample);
    (void)UltrasonicSensor_Trigger();

    if (fc_us_sample.status == 0 &&
        fc_us_sample.distance_mm > 0U &&
        (now - fc_us_sample.timestamp_ms) <= FC_US_MAX_AGE_MS)
    {
        s->front_valid = 1U;
        s->front_mm = fc_us_sample.distance_mm;
    }

    if (IRSensor_ReadDistance(&ir, FC_IR_AVG_SAMPLES) == 0)
    {
        s->left_valid = IrStatusUsable(ir.left_status);
        s->right_valid = IrStatusUsable(ir.right_status);
        s->left_mm = (ir.left_status == IR_RANGE_NEAR)
                   ? IR_SENSOR_VALID_MIN_CM_X10
                   : ir.left_cm_x10;
        s->right_mm = (ir.right_status == IR_RANGE_NEAR)
                    ? IR_SENSOR_VALID_MIN_CM_X10
                    : ir.right_cm_x10;
    }
}

static int32_t SideCorrection(const fc_sensors_t *s, fc_follow_side_t side)
{
    int32_t error_mm = 0;
    int32_t target_mm = (int32_t)SideTargetMm();
    int32_t deadband_mm = (int32_t)FASTEST_CAR_SIDE_DEADBAND_CM * 10;

    if (side == FC_FOLLOW_RIGHT && s->right_valid)
    {
        error_mm = (int32_t)s->right_mm - target_mm;
    }
    else if (side == FC_FOLLOW_LEFT && s->left_valid)
    {
        error_mm = target_mm - (int32_t)s->left_mm;
    }
    else
    {
        return 0;
    }

    if (error_mm > deadband_mm)
        error_mm -= deadband_mm;
    else if (error_mm < -deadband_mm)
        error_mm += deadband_mm;
    else
        return 0;

    return ClampI32((error_mm * FC_SIDE_KP_NUM) / FC_SIDE_KP_DEN,
                    -FASTEST_CAR_FOLLOW_MAX_CORRECTION_US,
                     FASTEST_CAR_FOLLOW_MAX_CORRECTION_US);
}

static uint32_t SpeedForFront(const fc_sensors_t *s)
{
    if (s->front_valid &&
        s->front_mm <= FASTEST_CAR_FRONT_SLOW_CM * 10U)
    {
        return FASTEST_CAR_APPROACH_PWM;
    }
    return FASTEST_CAR_CRUISE_PWM;
}

static int8_t MoreOpenTurnDirection(const fc_sensors_t *s)
{
    if (s->left_valid && s->right_valid)
        return (s->left_mm > s->right_mm) ? -1 : 1;
    if (s->left_valid) return 1;
    if (s->right_valid) return -1;
    return 1;
}

static void EmergencyRecover(const fc_sensors_t *s)
{
    int8_t turn_dir = MoreOpenTurnDirection(s);
    UART_Print("FASTEST_CAR,EMERGENCY_RECOVER\r\n");
    RobotHW_MotorStop();
    SetSteeringCorrection((turn_dir > 0) ? FC_MAX_STEER_CORRECTION_US
                                         : -FC_MAX_STEER_CORRECTION_US);
    RobotHW_MotorSetBackward(FASTEST_CAR_REVERSE_PWM,
                             FASTEST_CAR_REVERSE_PWM);
    HAL_Delay(FASTEST_CAR_EMERGENCY_REVERSE_MS);
    RobotHW_MotorStop();
}

static void DriveObstacleFollow(const fc_sensors_t *s,
                                fc_follow_side_t follow_side,
                                uint32_t pwm)
{
    int32_t correction = SideCorrection(s, follow_side);

    if (s->left_valid && s->left_mm < SideMinMm())
        correction = FASTEST_CAR_FOLLOW_MAX_CORRECTION_US;
    if (s->right_valid && s->right_mm < SideMinMm())
        correction = -FASTEST_CAR_FOLLOW_MAX_CORRECTION_US;

    SetSteeringCorrectionSmooth(correction);
    RobotHW_MotorSetForward(pwm, pwm);
}

static float AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void UpdateGyroHeading(float dt_sec)
{
    (void)RobotHW_GyroUpdateYaw(dt_sec, FC_GYRO_IIR_ALPHA,
                                FC_GYRO_RATE_DEADBAND_DPS);
    fc_yaw_deg = RobotHW_YawGetDeg();
}

static void DriveGyroHeading(float target_yaw_deg, uint32_t pwm)
{
    float yaw_error = fc_yaw_deg - target_yaw_deg;
    float rate_dps = RobotHW_GyroGetLastZDps();
    int32_t correction;

    if (AbsF(yaw_error) <= FC_HEADING_DEADBAND_DEG &&
        AbsF(rate_dps) <= FC_HEADING_RATE_DEADBAND_DPS)
    {
        correction = 0;
    }
    else
    {
        correction = (int32_t)(FC_HEADING_KP_US_PER_DEG * yaw_error
                             + FC_HEADING_KD_US_PER_DPS * rate_dps);
    }
    correction = ClampI32(correction,
                          -FC_HEADING_MAX_CORRECTION_US,
                           FC_HEADING_MAX_CORRECTION_US);

    SetSteeringCorrectionSmooth(correction);
    RobotHW_MotorSetForward(pwm, pwm);
}

static void EnterState(fc_state_t state)
{
    fc_state = state;
    fc_state_tick = HAL_GetTick();
    fc_front_confirm = 0U;
    fc_edge_lost_confirm = 0U;
    fc_follow_locked = 0U;
    fc_state_start_yaw_deg = fc_yaw_deg;
    if (state == FC_STATE_RETURN_DRIVE)
        fc_heading_target_deg = fc_yaw_deg;
}

static fc_arrow_t FastestCar_ReadArrow(uint8_t goal_index)
{
    int configured = (goal_index == 0U)
                   ? FASTEST_CAR_FIRST_OBSTACLE_TURN
                   : FASTEST_CAR_SECOND_OBSTACLE_TURN;
    return (configured == FASTEST_CAR_TURN_RIGHT)
         ? FC_ARROW_RIGHT : FC_ARROW_LEFT;
}

static void OledShowState(const fc_sensors_t *s)
{
    char line[22];
    long yaw_deg = (long)fc_yaw_deg;
    unsigned us_cm = (s != NULL && s->front_valid)
                   ? (unsigned)(s->front_mm / 10U) : 999U;
    unsigned left_cm = (s != NULL && s->left_valid)
                     ? (unsigned)(s->left_mm / 10U) : 99U;
    unsigned right_cm = (s != NULL && s->right_valid)
                      ? (unsigned)(s->right_mm / 10U) : 99U;

    OLED_Clear();
    snprintf(line, sizeof(line), "%s G%u %c L%u",
             StateText(fc_state),
             (unsigned)(fc_goal_index + 1U),
             (fc_bypass_arrow == FC_ARROW_RIGHT) ? 'R' : 'L',
             (unsigned)fc_follow_locked);
    OLED_ShowString(0, 0, (const u8 *)line);

    snprintf(line, sizeof(line), "Y:%lddeg US:%ucm", yaw_deg, us_cm);
    OLED_ShowString(0, 16, (const u8 *)line);

    snprintf(line, sizeof(line), "IR L:%u R:%u cm", left_cm, right_cm);
    OLED_ShowString(0, 32, (const u8 *)line);

    snprintf(line, sizeof(line), "t:%lus",
             (unsigned long)((HAL_GetTick() - fc_run_start_tick) / 1000U));
    OLED_ShowString(0, 48, (const u8 *)line);

    OLED_Refresh_Gram();
}

static void OledShowStateThrottled(const fc_sensors_t *s)
{
    uint32_t now = HAL_GetTick();
    if ((now - fc_last_oled_tick) < 250U) return;
    fc_last_oled_tick = now;
    OledShowState(s);
}

static uint8_t FollowSideLost(const fc_sensors_t *s, fc_follow_side_t side)
{
    if (side == FC_FOLLOW_RIGHT)
        return (!s->right_valid || s->right_mm >= EdgeLostMm()) ? 1U : 0U;
    if (side == FC_FOLLOW_LEFT)
        return (!s->left_valid || s->left_mm >= EdgeLostMm()) ? 1U : 0U;
    return 0U;
}

static uint8_t FollowSideVisible(const fc_sensors_t *s, fc_follow_side_t side)
{
    if (side == FC_FOLLOW_RIGHT)
        return (s->right_valid && s->right_mm < EdgeLostMm()) ? 1U : 0U;
    if (side == FC_FOLLOW_LEFT)
        return (s->left_valid && s->left_mm < EdgeLostMm()) ? 1U : 0U;
    return 0U;
}

static fc_follow_side_t ObstacleSideForArrow(fc_arrow_t arrow)
{
    return (arrow == FC_ARROW_LEFT) ? FC_FOLLOW_RIGHT : FC_FOLLOW_LEFT;
}

static int32_t TurnCorrectionForArrow(fc_arrow_t arrow)
{
    return (arrow == FC_ARROW_LEFT)
         ? -FASTEST_CAR_LEFT_TURN_CORRECTION_US
         :  FASTEST_CAR_RIGHT_TURN_CORRECTION_US;
}

static int32_t OppositeTurnCorrectionForArrow(fc_arrow_t arrow)
{
    return (arrow == FC_ARROW_LEFT)
         ?  FASTEST_CAR_RIGHT_TURN_CORRECTION_US
         : -FASTEST_CAR_LEFT_TURN_CORRECTION_US;
}

static float TurnProgressDeg(fc_arrow_t arrow)
{
    float yaw_delta = fc_yaw_deg - fc_state_start_yaw_deg;
    return (arrow == FC_ARROW_LEFT) ? yaw_delta : -yaw_delta;
}

static void StepFastestCar(const fc_sensors_t *s)
{
    uint32_t now = HAL_GetTick();
    uint32_t in_state_ms = now - fc_state_tick;
    fc_follow_side_t obstacle_side = ObstacleSideForArrow(fc_bypass_arrow);

    if ((now - fc_run_start_tick) >= FASTEST_CAR_MAX_RUN_MS)
    {
        EnterState(FC_STATE_FAIL);
        return;
    }

    if (s->front_valid && s->front_mm <= FrontEmergencyMm())
    {
        EmergencyRecover(s);
        return;
    }

    switch (fc_state)
    {
        case FC_STATE_SEEK_GOAL:
            DriveGyroHeading(0.0f, SpeedForFront(s));
            if (s->front_valid && s->front_mm <= FrontStopMm())
            {
                if (++fc_front_confirm >= FC_FRONT_CONFIRM_COUNT)
                {
                    RobotHW_MotorStop();
                    EnterState(FC_STATE_READ_ARROW);
                }
            }
            else
            {
                fc_front_confirm = 0U;
            }
            break;

        case FC_STATE_READ_ARROW:
            RobotHW_MotorStop();
            SetSteeringCorrection(0);
            if (in_state_ms >= FASTEST_CAR_ARROW_SETTLE_MS)
            {
                fc_bypass_arrow = FastestCar_ReadArrow(fc_goal_index);
                OledShowState(s);
                EnterState(FC_STATE_BYPASS_ENTRY);
            }
            break;

        case FC_STATE_BYPASS_ENTRY:
            SetSteeringCorrection(TurnCorrectionForArrow(fc_bypass_arrow));
            RobotHW_MotorSetForward(FASTEST_CAR_TURN_PWM,
                                    FASTEST_CAR_TURN_PWM);
            if (in_state_ms >= FASTEST_CAR_ENTRY_TURN_MIN_MS &&
                TurnProgressDeg(fc_bypass_arrow) >=
                    FASTEST_CAR_ENTRY_TURN_ANGLE_DEG &&
                (!s->front_valid || s->front_mm > FrontStopMm() + 180U))
            {
                EnterState(FC_STATE_BYPASS_FOLLOW);
            }
            else if (in_state_ms >= FASTEST_CAR_ENTRY_TURN_MAX_MS)
            {
                EnterState(FC_STATE_BYPASS_FOLLOW);
            }
            break;

        case FC_STATE_BYPASS_FOLLOW:
            if (FollowSideVisible(s, obstacle_side))
                fc_follow_locked = 1U;

            if (!fc_follow_locked &&
                in_state_ms < FASTEST_CAR_PRELOCK_TURN_MAX_MS)
            {
                SetSteeringCorrection(TurnCorrectionForArrow(fc_bypass_arrow));
                RobotHW_MotorSetForward(FASTEST_CAR_TURN_PWM,
                                        FASTEST_CAR_TURN_PWM);
            }
            else
            {
                DriveObstacleFollow(s, obstacle_side,
                                    FASTEST_CAR_TURN_PWM);
            }

            if (fc_follow_locked &&
                in_state_ms >= FASTEST_CAR_FOLLOW_MIN_MS &&
                FollowSideLost(s, obstacle_side))
            {
                if (++fc_edge_lost_confirm >= FC_EDGE_LOST_COUNT)
                    EnterState(FC_STATE_BYPASS_EXIT);
            }
            else
            {
                fc_edge_lost_confirm = 0U;
            }

            if (in_state_ms >= FASTEST_CAR_FOLLOW_MAX_MS)
                EnterState(FC_STATE_BYPASS_EXIT);
            break;

        case FC_STATE_BYPASS_EXIT:
            SetSteeringCorrection(OppositeTurnCorrectionForArrow(fc_bypass_arrow));
            RobotHW_MotorSetForward(FASTEST_CAR_TURN_PWM,
                                    FASTEST_CAR_TURN_PWM);
            if (in_state_ms >= FASTEST_CAR_EXIT_TURN_MS)
            {
                if (fc_goal_index == 0U)
                {
                    fc_goal_index = 1U;
                    EnterState(FC_STATE_RECENTER);
                }
                else
                {
                    EnterState(FC_STATE_RETURN_ARC);
                }
            }
            break;

        case FC_STATE_RECENTER:
            DriveGyroHeading(0.0f, FASTEST_CAR_APPROACH_PWM);
            if ((in_state_ms >= FASTEST_CAR_RECENTER_MIN_MS &&
                 AbsF(fc_yaw_deg) <= FASTEST_CAR_RECENTER_YAW_TOL_DEG) ||
                in_state_ms >= FASTEST_CAR_RECENTER_MAX_MS)
            {
                EnterState(FC_STATE_SEEK_GOAL);
            }
            break;

        case FC_STATE_RETURN_ARC:
            SetSteeringCorrection(TurnCorrectionForArrow(fc_bypass_arrow));
            RobotHW_MotorSetForward(FASTEST_CAR_TURN_PWM,
                                    FASTEST_CAR_TURN_PWM);
            if (in_state_ms >= FASTEST_CAR_RETURN_ARC_MS)
                EnterState(FC_STATE_RETURN_DRIVE);
            break;

        case FC_STATE_RETURN_DRIVE:
            DriveGyroHeading(fc_heading_target_deg,
                             FASTEST_CAR_APPROACH_PWM);
            if (in_state_ms >= FASTEST_CAR_RETURN_DRIVE_MS ||
                (s->front_valid && s->front_mm <= FrontStopMm()))
            {
                EnterState(FC_STATE_DONE);
            }
            break;

        case FC_STATE_DONE:
        case FC_STATE_FAIL:
        default:
            RobotHW_MotorStop();
            SetSteeringCorrection(0);
            break;
    }
}

void Test_FastestCar(void)
{
    fc_sensors_t sensors;
    int prep_rc;
    int us_rc;
    char line[128];

    RobotHW_StartMotionPeripherals();
    OLED_Init();
    OLED_Display_On();
    RobotHW_MotorStop();
    RobotHW_ServoCenter();

    UART_Print("\r\n=== FASTEST_CAR_TEST ===\r\n");
    UART_Print("Closed-loop with front ultrasonic + left/right IR.\r\n");
    UART_Print("Configured turns are in Core/Inc/tests/test_fastest_car.h.\r\n");

    us_rc = UltrasonicSensor_Init();
    prep_rc = RobotHW_PrepareMotionSensors();
    if (us_rc != 0 || prep_rc != 0)
    {
        (void)snprintf(line, sizeof(line),
                       "FASTEST_CAR_INIT_ERROR us=%d prep=%d\r\n",
                       us_rc, prep_rc);
        UART_Print(line);
        while (1) HAL_Delay(1000U);
    }

    snprintf(line, sizeof(line), "FASTEST_CAR_START_IN,%lu_ms\r\n",
             (unsigned long)FASTEST_CAR_START_DELAY_MS);
    UART_Print(line);
    HAL_Delay(FASTEST_CAR_START_DELAY_MS);

    fc_goal_index = 0U;
    fc_bypass_arrow = FC_ARROW_LEFT;
    fc_yaw_deg = 0.0f;
    fc_state_start_yaw_deg = 0.0f;
    fc_heading_target_deg = 0.0f;
    fc_steering_correction_us = 0;
    fc_run_start_tick = HAL_GetTick();
    fc_last_control_tick = fc_run_start_tick;
    fc_last_oled_tick = 0U;
    (void)UltrasonicSensor_Trigger();
    EnterState(FC_STATE_SEEK_GOAL);

    while (fc_state != FC_STATE_DONE && fc_state != FC_STATE_FAIL)
    {
        uint32_t now = HAL_GetTick();
        if ((now - fc_last_control_tick) >= FC_CONTROL_PERIOD_MS)
        {
            uint32_t prev_control_tick = fc_last_control_tick;
            fc_last_control_tick = now;
            ReadSensors(&sensors);
            UpdateGyroHeading((float)(now - prev_control_tick) / 1000.0f);
            StepFastestCar(&sensors);
            OledShowStateThrottled(&sensors);
        }
        HAL_Delay(1U);
    }

    RobotHW_MotorStop();
    SetSteeringCorrection(0);
    UltrasonicSensor_Cancel();
    snprintf(line, sizeof(line), "FASTEST_CAR_FINAL,%s,elapsed=%lu_ms\r\n",
             StateText(fc_state),
             (unsigned long)(HAL_GetTick() - fc_run_start_tick));
    UART_Print(line);
    OledShowState(&sensors);

    while (1) HAL_Delay(1000U);
}

#endif /* ACTIVE_TEST == TEST_FASTEST_CAR */
