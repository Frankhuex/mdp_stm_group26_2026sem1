#include "main.h"
#include "control/turn_control.h"
#include "control/turn_tuning.h"
#include "control/motion_heading.h"
#include "drivers/robot_hw.h"
#include "drivers/oled.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * STEP 15B - MINIMUM TURNING RADIUS TEST
 *
 * PURPOSE
 * -------
 * 1) STEER_CHECK mode:
 *      Motors stay OFF. Apply exactly one steering-servo pulse so you can
 *      inspect the steering mechanism safely with the drive wheels lifted.
 *      Increase/decrease TEST_STEER_US manually in 25 us steps until the
 *      tightest SAFE steering position is found.
 *
 * 2) RADIUS_RUN mode:
 *      Hold that steering pulse constant, drive both rear wheels forward at
 *      constant low PWM, use ICM-20948 gyro Z to stop near 180 deg, then keep
 *      the result in RAM and repeat the UART report forever.
 *
 * IMPORTANT
 * ---------
 * - This is a car-like arc turn. It is NOT an on-the-spot rotation.
 * - Minimum turning radius is measured on the FLOOR from the MIDPOINT OF THE
 *   REAR AXLE, not from one rear wheel.
 * - For a physical 180 deg turn:
 *       rear-axle-midpoint radius = start-to-end chord / 2
 * - Keep STM32 powered after the run if you unplug/replug UART for PuTTY.
 *
 * CURRENT ROBOT HARDWARE
 * ----------------------
 * Left motor  : TIM4_CH3 / TIM4_CH4
 * Right motor : TIM9_CH1 / TIM9_CH2
 * Left encoder: TIM2
 * Right enc   : TIM3
 * Steering    : TIM12_CH1, PB14, 1 us timer tick
 * Gyro        : I2C2, ICM-20948
 * UART        : USART3, 115200 8-N-1
 *
 * CONFIRMED GYRO SIGN ON THIS ROBOT
 * ---------------------------------
 * RIGHT physical turn -> negative gyro Z / yaw
 * LEFT  physical turn -> positive gyro Z / yaw
 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart3;

/* =========================================================
 * USER SETTINGS
 * ========================================================= */
#define MODE_STEER_CHECK             0
#define MODE_RADIUS_RUN              1

/* FIRST use MODE_STEER_CHECK.
 * After finding the tightest SAFE steering pulse, change to MODE_RADIUS_RUN. */
#define TEST_MODE                    MODE_RADIUS_RUN

#define TURN_LEFT                    (-1)
#define TURN_RIGHT                   (1)

#define TEST_TURN_DIRECTION          TURN_LEFT

/* THIS is the parameter represented by 1912, 1937, 1962, 1987, ...
 * It is the steering-servo PWM HIGH pulse width in MICROSECONDS.
 * TIM12 has a 1 us timer tick, so 1912U means a 1912 us pulse.
 *
 * Shared centre/endpoints: drivers/robot_tuning.h.
 * RIGHT is above the centre; LEFT is below the centre.
 */
#define TEST_STEER_US                ((TEST_TURN_DIRECTION == TURN_LEFT) ? CMD_SERVO_LEFT_US : CMD_SERVO_RIGHT_US)

/* Software guard only. Mechanical linkage may reach its safe limit earlier. */
#define SERVO_SOFT_MIN_US            CMD_SERVO_LEFT_US
#define SERVO_SOFT_MAX_US            CMD_SERVO_RIGHT_US

/* Radius-run settings. Keep constant PWM: no fast/slow transition. */
#define TARGET_ANGLE_DEG             180.0f

/* Use the same side-specific angle compensation as command-mode turns. */
#define MOTOR_STOP_ANGLE_DEG         (TARGET_ANGLE_DEG * \
    ((TEST_TURN_DIRECTION == TURN_LEFT) ? TURN_LEFT_STOP_RATIO : TURN_RIGHT_STOP_RATIO))
#define TURN_PWM                     TURN_MOTOR_PWM
#define SERVO_SETTLE_MS              500U
#define PLACE_DELAY_MS               10000U
#define CONTROL_DELAY_MS             10U
#define POST_STOP_SETTLE_MS          150U
#define TURN_TIMEOUT_MS              30000U

/* Safety checks. */
#define WRONG_SIGN_CHECK_MS          1000U
#define WRONG_SIGN_LIMIT_DEG         3.0f
#define NO_TURN_CHECK_MS             3000U
#define NO_TURN_LIMIT_DEG            3.0f

/* Encoder calibration already measured on this robot. */

/* =========================================================
 * ICM-20948
 * ========================================================= */
#define ICM_ADDR_68                  0x68U
#define ICM_ADDR_69                  0x69U
#define REG_BANK_SEL                 0x7FU

/* Bank 0 */
#define WHO_AM_I_REG                 0x00U
#define PWR_MGMT_1_REG               0x06U
#define PWR_MGMT_2_REG               0x07U
#define GYRO_ZOUT_H_REG              0x37U

/* Bank 2 */
#define GYRO_SMPLRT_DIV_REG          0x00U
#define GYRO_CONFIG_1_REG            0x01U

#define WHO_AM_I_EXPECTED            0xEAU
#define GYRO_CONFIG_1_VALUE          0x29U
#define GYRO_SENSITIVITY             131.0f

#define GYRO_IIR_ALPHA               0.35f
#define GYRO_RATE_DEADBAND_DPS       0.20f

/* =========================================================
 * RESULT STORAGE
 * ========================================================= */
typedef enum
{
    RADIUS_RESULT_OK = 0,
    RADIUS_RESULT_TIMEOUT,
    RADIUS_RESULT_WRONG_GYRO_SIGN,
    RADIUS_RESULT_NO_TURN,
    RADIUS_RESULT_BAD_CONFIG,
    RADIUS_RESULT_NO_IMU
} RadiusResult;

static uint8_t icm_addr = 0U;
static uint8_t gyro_config_readback = 0U;
static int32_t gyro_z_offset = 0;
static float gyro_z_filt = 0.0f;
static float yaw_deg = 0.0f;

static float yaw_at_motor_stop_deg = 0.0f;
static float yaw_after_settle_deg = 0.0f;
static float tc_peak_abs_dps = 0.0f;   /* instrumentation: peak |yaw rate| this turn */
static int32_t final_left_counts = 0;
static int32_t final_right_counts = 0;
static uint32_t final_drive_time_ms = 0U;
static RadiusResult final_result = RADIUS_RESULT_OK;

/* =========================================================
 * SMALL HELPERS
 * ========================================================= */
static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)text,
                      (uint16_t)strlen(text),
                      HAL_MAX_DELAY);
}

static float AbsFloat(float x)
{
    return (x < 0.0f) ? -x : x;
}

static const char *DirectionText(void)
{
    return (TEST_TURN_DIRECTION == TURN_RIGHT) ? "RIGHT" : "LEFT";
}

static const char *ResultText(RadiusResult r)
{
    switch (r)
    {
        case RADIUS_RESULT_OK:              return "OK";
        case RADIUS_RESULT_TIMEOUT:         return "TIMEOUT";
        case RADIUS_RESULT_WRONG_GYRO_SIGN: return "WRONG_GYRO_SIGN";
        case RADIUS_RESULT_NO_TURN:         return "NO_TURN_DETECTED";
        case RADIUS_RESULT_BAD_CONFIG:      return "BAD_CONFIG";
        case RADIUS_RESULT_NO_IMU:          return "NO_IMU";
        default:                            return "UNKNOWN";
    }
}

static void Servo_SetPulse(uint32_t pulse_us)
{
    RobotHW_ServoSetUs(pulse_us);
}

static void Motor_Stop(void)
{
    RobotHW_MotorStop();
}

static void Drive_Forward(uint32_t pwm)
{
    RobotHW_MotorSetTurnForward(pwm, pwm);
}

static void Drive_Backward(uint32_t pwm)
{
    RobotHW_MotorSetTurnBackward(pwm, pwm);
}

static void OLED_FourLines(const char *l1,
                           const char *l2,
                           const char *l3,
                           const char *l4)
{
    OLED_Clear();
    if (l1 != NULL) OLED_ShowString(0, 0,  (const u8 *)l1);
    if (l2 != NULL) OLED_ShowString(0, 16, (const u8 *)l2);
    if (l3 != NULL) OLED_ShowString(0, 32, (const u8 *)l3);
    if (l4 != NULL) OLED_ShowString(0, 48, (const u8 *)l4);
    OLED_Refresh_Gram();
}

static void FormatSignedFloat3(float value, char *buf, size_t n)
{
    int32_t x1000 = (int32_t)(value * 1000.0f);
    int32_t a = (x1000 < 0) ? -x1000 : x1000;
    snprintf(buf, n, "%c%ld.%03ld",
             (x1000 < 0) ? '-' : '+',
             (long)(a / 1000),
             (long)(a % 1000));
}

static void FormatUnsignedFloat3(float value, char *buf, size_t n)
{
    int32_t x1000 = (int32_t)(AbsFloat(value) * 1000.0f);
    snprintf(buf, n, "%ld.%03ld",
             (long)(x1000 / 1000),
             (long)(x1000 % 1000));
}

/* =========================================================
 * ICM-20948 LOW LEVEL
 * ========================================================= */
static HAL_StatusTypeDef ICM_SelectBank(uint8_t bank)
{
    uint8_t value = (uint8_t)(bank << 4);
    return HAL_I2C_Mem_Write(&hi2c2,
                             (uint16_t)(icm_addr << 1),
                             REG_BANK_SEL,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             100U);
}

static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             (uint16_t)(icm_addr << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             100U);
}

static HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg,
                                     uint8_t *data,
                                     uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            (uint16_t)(icm_addr << 1),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            100U);
}

static uint8_t ICM_FindDevice(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c2,
                              (uint16_t)(ICM_ADDR_68 << 1),
                              3U,
                              100U) == HAL_OK)
    {
        icm_addr = ICM_ADDR_68;
        return 1U;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c2,
                              (uint16_t)(ICM_ADDR_69 << 1),
                              3U,
                              100U) == HAL_OK)
    {
        icm_addr = ICM_ADDR_69;
        return 1U;
    }

    return 0U;
}

static uint8_t Gyro_Init(void)
{
    uint8_t who = 0U;

    if (ICM_SelectBank(0U) != HAL_OK) return 0U;
    if (ICM_ReadRegs(WHO_AM_I_REG, &who, 1U) != HAL_OK) return 0U;
    if (who != WHO_AM_I_EXPECTED) return 0U;

    if (ICM_WriteReg(PWR_MGMT_1_REG, 0x01U) != HAL_OK) return 0U;
    if (ICM_WriteReg(PWR_MGMT_2_REG, 0x00U) != HAL_OK) return 0U;
    HAL_Delay(100U);

    if (ICM_SelectBank(2U) != HAL_OK) return 0U;
    if (ICM_WriteReg(GYRO_SMPLRT_DIV_REG, 10U) != HAL_OK) return 0U;
    if (ICM_WriteReg(GYRO_CONFIG_1_REG, GYRO_CONFIG_1_VALUE) != HAL_OK) return 0U;
    if (ICM_ReadRegs(GYRO_CONFIG_1_REG, &gyro_config_readback, 1U) != HAL_OK) return 0U;

    if (ICM_SelectBank(0U) != HAL_OK) return 0U;
    HAL_Delay(100U);

    return 1U;
}

static int16_t Gyro_ReadZ_Raw(void)
{
    uint8_t data[2] = {0U, 0U};

    if (ICM_ReadRegs(GYRO_ZOUT_H_REG, data, 2U) != HAL_OK)
    {
        return 0;
    }

    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static void Gyro_Calibrate(void)
{
    int32_t sum = 0;

    for (uint32_t i = 0U;
         i < (GYRO_CALIB_WARMUP_MS / GYRO_CALIB_DELAY_MS);
         i++)
    {
        (void)Gyro_ReadZ_Raw();
        HAL_Delay(GYRO_CALIB_DELAY_MS);
    }

    for (uint32_t i = 0U; i < GYRO_CALIB_SAMPLES; i++)
    {
        sum += (int32_t)Gyro_ReadZ_Raw();
        HAL_Delay(GYRO_CALIB_DELAY_MS);
    }

    gyro_z_offset = sum / (int32_t)GYRO_CALIB_SAMPLES;
    gyro_z_filt = (float)gyro_z_offset;
}

static float Yaw_Update(float dt_sec)
{
    int16_t raw = Gyro_ReadZ_Raw();

    gyro_z_filt = GYRO_IIR_ALPHA * (float)raw
                + (1.0f - GYRO_IIR_ALPHA) * gyro_z_filt;

    float corrected_raw = gyro_z_filt - (float)gyro_z_offset;
    float gyro_z_dps = corrected_raw / GYRO_SENSITIVITY;

    if (AbsFloat(gyro_z_dps) < GYRO_RATE_DEADBAND_DPS)
    {
        gyro_z_dps = 0.0f;
    }

    {
        float abs_dps = AbsFloat(gyro_z_dps);
        if (abs_dps > tc_peak_abs_dps) tc_peak_abs_dps = abs_dps;
    }
    yaw_deg += gyro_z_dps * dt_sec;
    return gyro_z_dps;
}

/* =========================================================
 * STEERING-ONLY SAFETY CHECK
 * ========================================================= */
static void RunSteerCheck(void)
{
    char l2[20];
    char l3[20];

    Motor_Stop();
    Servo_SetPulse(SERVO_CENTER_US);
    HAL_Delay(1000U);

    Servo_SetPulse(TEST_STEER_US);

    snprintf(l2, sizeof(l2), "%s US=%u",
             DirectionText(), (unsigned)TEST_STEER_US);
    snprintf(l3, sizeof(l3), "CENTER=%u", (unsigned)SERVO_CENTER_US);

    OLED_FourLines("STEER CHECK",
                   l2,
                   l3,
                   "MOTORS OFF");

    UART_Print("\r\n=== MIN RADIUS: STEER CHECK ===\r\n");
    {
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "Direction      : %s\r\n"
                 "Servo center   : %u us\r\n"
                 "TEST_STEER_US  : %u us\r\n"
                 "Motor state    : OFF\r\n"
                 "Inspect linkage with drive wheels lifted.\r\n"
                 "If safe and still not at max steering, change TEST_STEER_US by 25 us and reflash.\r\n",
                 DirectionText(),
                 (unsigned)SERVO_CENTER_US,
                 (unsigned)TEST_STEER_US);
        UART_Print(msg);
    }

    while (1)
    {
        HAL_Delay(1000U);
    }
}

/* =========================================================
 * STORED REPORT
 * ========================================================= */
static void PrintStoredReport(void)
{
    char msg[512];
    char stop_yaw[24];
    char settle_yaw[24];
    char progress[24];
    char left_cm[24];
    char right_cm[24];

    const float expected_sign =
        (TEST_TURN_DIRECTION == TURN_RIGHT) ? -1.0f : +1.0f;

    float settled_progress = expected_sign * yaw_after_settle_deg;
    float left_cm_f = (float)final_left_counts / LEFT_COUNTS_PER_CM;
    float right_cm_f = (float)final_right_counts / RIGHT_COUNTS_PER_CM;

    FormatSignedFloat3(yaw_at_motor_stop_deg, stop_yaw, sizeof(stop_yaw));
    FormatSignedFloat3(yaw_after_settle_deg, settle_yaw, sizeof(settle_yaw));
    FormatUnsignedFloat3(settled_progress, progress, sizeof(progress));
    FormatUnsignedFloat3(left_cm_f, left_cm, sizeof(left_cm));
    FormatUnsignedFloat3(right_cm_f, right_cm, sizeof(right_cm));

    UART_Print("\r\n\r\n");
    UART_Print("=================================================\r\n");
    UART_Print(" STEP 15B MINIMUM TURNING RADIUS REPORT\r\n");
    UART_Print("=================================================\r\n");

    snprintf(msg, sizeof(msg),
             "Result                  : %s\r\n"
             "Direction               : %s\r\n"
             "Servo center            : %u us\r\n"
             "Steering pulse tested   : %u us\r\n"
             "Constant motor PWM      : %u\r\n"
             "Target final heading    : 180.000 deg\r\n"
             "Motor stop threshold    : 165.700 deg\r\n"
             "Gyro config readback    : 0x%02X\r\n"
             "Gyro Z offset           : %ld raw\r\n"
             "Drive time              : %lu ms\r\n",
             ResultText(final_result),
             DirectionText(),
             (unsigned)SERVO_CENTER_US,
             (unsigned)TEST_STEER_US,
             (unsigned)TURN_PWM,
             (unsigned)gyro_config_readback,
             (long)gyro_z_offset,
             (unsigned long)final_drive_time_ms);
    UART_Print(msg);

    snprintf(msg, sizeof(msg),
             "Yaw at motor stop        : %s deg\r\n"
             "Yaw after 1 s settle     : %s deg\r\n"
             "Settled turn progress    : %s deg\r\n"
             "Left encoder             : %ld counts (%s cm wheel travel)\r\n"
             "Right encoder            : %ld counts (%s cm wheel travel)\r\n",
             stop_yaw,
             settle_yaw,
             progress,
             (long)final_left_counts,
             left_cm,
             (long)final_right_counts,
             right_cm);
    UART_Print(msg);

    UART_Print("\r\nFLOOR MEASUREMENT REQUIRED:\r\n");
    UART_Print("1. Mark rear-axle MIDPOINT at start = S.\r\n");
    UART_Print("2. Mark same rear-axle MIDPOINT at final stop = E.\r\n");
    UART_Print("3. Measure straight S-to-E chord in cm.\r\n");
    UART_Print("4. Measure actual physical heading change.\r\n");
    UART_Print("5. If actual heading is exactly 180 deg: R_mid = chord / 2.\r\n");
    UART_Print("Keep STM32 powered; this report repeats every 1.5 s.\r\n");
}

/* =========================================================
 * MAIN TEST
 * ========================================================= */
void Turn_RunMinRadiusTestBlocking(void)
{
    uint16_t previous_left = 0U;
    uint16_t previous_right = 0U;
    int32_t total_left_counts = 0;
    int32_t total_right_counts = 0;

    /* Start peripherals used by both modes. */
    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    Motor_Stop();
    Servo_SetPulse(SERVO_CENTER_US);

    OLED_Init();
    OLED_FourLines("MIN RADIUS TEST",
                   "INIT",
                   NULL,
                   NULL);

    /* Configuration guards. */
    if ((TEST_TURN_DIRECTION != TURN_LEFT) &&
        (TEST_TURN_DIRECTION != TURN_RIGHT))
    {
        final_result = RADIUS_RESULT_BAD_CONFIG;
        OLED_FourLines("BAD CONFIG",
                       "TURN DIRECTION",
                       NULL,
                       NULL);
        UART_Print("ERROR: TEST_TURN_DIRECTION must be TURN_LEFT or TURN_RIGHT.\r\n");
        while (1) HAL_Delay(1000U);
    }

    if ((TEST_STEER_US < SERVO_SOFT_MIN_US) ||
        (TEST_STEER_US > SERVO_SOFT_MAX_US))
    {
        final_result = RADIUS_RESULT_BAD_CONFIG;
        OLED_FourLines("BAD CONFIG",
                       "STEER US RANGE",
                       NULL,
                       NULL);
        UART_Print("ERROR: TEST_STEER_US outside shared servo endpoint limits.\r\n");
        while (1) HAL_Delay(1000U);
    }

    if (TEST_MODE == MODE_STEER_CHECK)
    {
        RunSteerCheck();
        return;
    }

    if (TEST_MODE != MODE_RADIUS_RUN)
    {
        final_result = RADIUS_RESULT_BAD_CONFIG;
        UART_Print("ERROR: TEST_MODE invalid.\r\n");
        while (1) HAL_Delay(1000U);
    }

    /* Radius-run peripherals. */
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }

    UART_Print("\r\n=== STEP 15B MINIMUM TURNING RADIUS RUN ===\r\n");

    if (!ICM_FindDevice())
    {
        final_result = RADIUS_RESULT_NO_IMU;
        Motor_Stop();
        Servo_SetPulse(SERVO_CENTER_US);
        OLED_FourLines("IMU ERROR",
                       "NOT FOUND",
                       NULL,
                       NULL);
        UART_Print("ERROR: ICM-20948 not found.\r\n");
        while (1) HAL_Delay(1000U);
    }

    if (!Gyro_Init())
    {
        final_result = RADIUS_RESULT_NO_IMU;
        Motor_Stop();
        Servo_SetPulse(SERVO_CENTER_US);
        OLED_FourLines("IMU ERROR",
                       "INIT FAILED",
                       NULL,
                       NULL);
        UART_Print("ERROR: ICM-20948 init failed.\r\n");
        while (1) HAL_Delay(1000U);
    }

    OLED_FourLines("GYRO CAL",
                   "KEEP STILL",
                   "WAIT",
                   NULL);
    Gyro_Calibrate();

    /* Placement window: mark S under rear-axle midpoint during this time. */
    for (int sec = 10; sec >= 1; sec--)
    {
        char l4[20];
        snprintf(l4, sizeof(l4), "START IN %dS", sec);
        OLED_FourLines("PLACE CAR",
                       "MARK REAR MID",
                       DirectionText(),
                       l4);
        HAL_Delay(1000U);
    }

    /* Set the selected steering pulse before the motors start. */
    Servo_SetPulse(TEST_STEER_US);
    OLED_FourLines("STEERING SET",
                   DirectionText(),
                   "MOTORS OFF",
                   "WAIT 0.8S");
    HAL_Delay(SERVO_SETTLE_MS);

    /* Reset measurement immediately before motion. */
    yaw_deg = 0.0f;
    gyro_z_filt = (float)gyro_z_offset;
    final_result = RADIUS_RESULT_OK;
    yaw_at_motor_stop_deg = 0.0f;
    yaw_after_settle_deg = 0.0f;
    tc_peak_abs_dps = 0.0f;
    final_left_counts = 0;
    final_right_counts = 0;
    final_drive_time_ms = 0U;

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    previous_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    previous_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    const float expected_sign =
        (TEST_TURN_DIRECTION == TURN_RIGHT) ? -1.0f : +1.0f;

    uint32_t start_tick = HAL_GetTick();
    uint32_t last_tick = start_tick;
    uint32_t last_oled_tick = 0U;

    Drive_Forward(TURN_PWM);

    while (1)
    {
        HAL_Delay(CONTROL_DELAY_MS);

        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - start_tick;
        uint32_t delta_ms = now - last_tick;
        last_tick = now;

        /* Actual elapsed dt is used; do not assume an exact 10 ms loop. */
        float dt_sec = (float)delta_ms / 1000.0f;
        (void)Yaw_Update(dt_sec);

        /* Encoder diagnostics. */
        uint16_t current_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
        uint16_t current_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t left_delta = (int16_t)(current_left - previous_left);
        int16_t right_delta = (int16_t)(current_right - previous_right);
        previous_left = current_left;
        previous_right = current_right;
        total_left_counts += abs((int)left_delta);
        total_right_counts += abs((int)right_delta);

        float progress_deg = expected_sign * yaw_deg;

        if ((elapsed_ms >= WRONG_SIGN_CHECK_MS) &&
            (progress_deg <= -WRONG_SIGN_LIMIT_DEG))
        {
            final_result = RADIUS_RESULT_WRONG_GYRO_SIGN;
            break;
        }

        if ((elapsed_ms >= NO_TURN_CHECK_MS) &&
            (AbsFloat(yaw_deg) < NO_TURN_LIMIT_DEG))
        {
            final_result = RADIUS_RESULT_NO_TURN;
            break;
        }

        if (progress_deg >= MOTOR_STOP_ANGLE_DEG)
        {
            final_result = RADIUS_RESULT_OK;
            break;
        }

        if (elapsed_ms >= TURN_TIMEOUT_MS)
        {
            final_result = RADIUS_RESULT_TIMEOUT;
            break;
        }

        if ((now - last_oled_tick) >= 250U)
        {
            char l2[20];
            char l3[20];
            char l4[20];
            int32_t yaw_x10 = (int32_t)(progress_deg * 10.0f);
            if (yaw_x10 < 0) yaw_x10 = -yaw_x10;

            snprintf(l2, sizeof(l2), "%s US=%u",
                     DirectionText(), (unsigned)TEST_STEER_US);
            snprintf(l3, sizeof(l3), "YAW=%ld.%ld",
                     (long)(yaw_x10 / 10),
                     (long)(yaw_x10 % 10));
            snprintf(l4, sizeof(l4), "PWM=%u", (unsigned)TURN_PWM);

            OLED_FourLines("RADIUS RUN",
                           l2,
                           l3,
                           l4);
            last_oled_tick = now;
        }
    }

    /* Stop drive first, then centre steering. */
    Motor_Stop();
    yaw_at_motor_stop_deg = yaw_deg;
    final_drive_time_ms = HAL_GetTick() - start_tick;
    Servo_SetPulse(SERVO_CENTER_US);

    /* Keep integrating gyro for 1 s to record coast/settling. */
    uint32_t settle_start = HAL_GetTick();
    uint32_t settle_last = settle_start;

    while ((HAL_GetTick() - settle_start) < POST_STOP_SETTLE_MS)
    {
        HAL_Delay(CONTROL_DELAY_MS);
        uint32_t now = HAL_GetTick();
        uint32_t delta_ms = now - settle_last;
        settle_last = now;
        (void)Yaw_Update((float)delta_ms / 1000.0f);
    }

    yaw_after_settle_deg = yaw_deg;
    final_left_counts = total_left_counts;
    final_right_counts = total_right_counts;

    OLED_FourLines("RUN FINISHED",
                   "MARK REAR MID",
                   "MEASURE S-E",
                   "OPEN PUTTY");

    while (1)
    {
        PrintStoredReport();
        HAL_Delay(1500U);
    }
}

/* =========================================================
 * COMMAND-MODE NON-BLOCKING ARC TURN
 *
 * Timing and gyro filtering follow the trusted min-radius test:
 * servo settle 800 ms, gyro deadband, wrong-sign/no-turn guards,
 * motor cut before final target, then 1000 ms post-stop integration.
 * ========================================================= */
/* Floor/geometry tuning values are in control/turn_tuning.h. */
#define CMD_TURN_LEFT                (-1)
#define CMD_TURN_RIGHT               (1)

static turn_state_t tc_state = TURN_IDLE;
static int32_t tc_radius_cm = 0;
static int32_t tc_target_deg = 0;
static int8_t tc_turn_dir = CMD_TURN_LEFT;
static int8_t tc_drive_dir = 1;
static float tc_target_abs_deg = 0.0f;
static uint32_t tc_start_tick = 0U;
static uint32_t tc_last_tick = 0U;
static uint32_t tc_phase_tick = 0U;
static uint32_t tc_servo_wait_ms;
/* 0=initial settle, 1=drive, 2=post-brake settle, 3=brake,
 * 4=right return at 1500 us, 5=right return at centre. */
static uint8_t tc_phase = 0U;
static uint8_t tc_gyro_fail_count = 0U;
static float tc_stop_rate_dps = 0.0f;

static uint32_t Command_ServoWaitMs(uint32_t previous_us, uint32_t target_us)
{
    uint32_t delta = previous_us > target_us ? previous_us - target_us : target_us - previous_us;
    uint32_t span = CMD_SERVO_RIGHT_US - CMD_SERVO_LEFT_US;
    if (delta > span) delta = span;
    uint32_t travel_ms = TURN_SERVO_WAIT_FULL_SPAN_MS - TURN_SERVO_WAIT_MIN_MS;
    /* Round up; invalid/out-of-range previous commands receive maximum wait. */
    if (previous_us < CMD_SERVO_LEFT_US || previous_us > CMD_SERVO_RIGHT_US)
        return TURN_SERVO_WAIT_FULL_SPAN_MS;
    return TURN_SERVO_WAIT_MIN_MS + (uint32_t)(((uint64_t)delta * travel_ms + span - 1U) / span);
}

static uint8_t Command_IsShortRight(void)
{
    /* Select the same profile for forward/reverse on this steering side.
     * Use the requested magnitude, before target-scale compensation. */
    int32_t magnitude = (tc_target_deg < 0) ? -tc_target_deg : tc_target_deg;
    return tc_turn_dir == CMD_TURN_RIGHT && magnitude > 0 &&
           magnitude < TURN_RIGHT_SHORT_LIMIT_DEG;
}

static uint32_t Command_ShortRightPwm(float remaining_deg)
{
    return remaining_deg <= TURN_RIGHT_SHORT_FINAL_DEG
           ? TURN_RIGHT_SHORT_SLOW_PWM : TURN_RIGHT_SHORT_CRUISE_PWM;
}

static uint8_t Command_KickActive(uint32_t elapsed_ms, float progress_deg)
{
    if (Command_IsShortRight() || tc_target_abs_deg <= TURN_SMALL_ANGLE_DEG)
        return elapsed_ms < TURN_SHORT_KICK_DURATION_MS &&
               progress_deg < TURN_SHORT_KICK_PROGRESS_DEG;
    return elapsed_ms < TURN_KICKSTART_DURATION_MS;
}

static float Command_ApproxAtan(float x)
{
    return x / (1.0f + 0.281f * x * x);
}

static float Command_RadToDeg(float rad)
{
    return rad * (180.0f / 3.14159265f);
}

static int32_t Command_RadiusCmToPulseUs(int32_t radius_cm, int8_t turn_dir,
                                        uint8_t reverse)
{
    float mapped_radius_cm = (float)radius_cm *
        ((turn_dir == CMD_TURN_RIGHT)
         ? (reverse ? TURN_RIGHT_REVERSE_RADIUS_SCALE : TURN_RIGHT_RADIUS_SCALE)
         : reverse ? TURN_LEFT_REVERSE_RADIUS_SCALE : TURN_LEFT_RADIUS_SCALE);
    float min_radius_cm = (float)((turn_dir == CMD_TURN_RIGHT)
                                  ? CMD_RIGHT_MIN_RADIUS_CM
                                  : CMD_LEFT_MIN_RADIUS_CM);
    /* Saturate before the geometry calculation: a calibration factor must
     * not request steering beyond the established endpoint. Keep the nominal
     * endpoint radius in the denominator so the correction does not cancel. */
    if (mapped_radius_cm < min_radius_cm) mapped_radius_cm = min_radius_cm;
    float delta_deg = Command_RadToDeg(
        Command_ApproxAtan(CMD_WHEELBASE_CM / mapped_radius_cm));

    if (turn_dir == CMD_TURN_RIGHT)
    {
        float max_delta_deg = Command_RadToDeg(
            Command_ApproxAtan(CMD_WHEELBASE_CM / (float)CMD_RIGHT_MIN_RADIUS_CM));
        float us_span = (float)((int32_t)CMD_SERVO_RIGHT_US - (int32_t)SERVO_CENTER_US)
                      * TURN_RIGHT_STEER_SCALE;
        int32_t delta_us = (int32_t)((delta_deg * us_span / max_delta_deg) + 0.5f);
        int32_t pulse = (int32_t)SERVO_CENTER_US + delta_us;
        if (pulse > (int32_t)CMD_SERVO_RIGHT_US) pulse = (int32_t)CMD_SERVO_RIGHT_US;
        return pulse;
    }
    else
    {
        float max_delta_deg = Command_RadToDeg(
            Command_ApproxAtan(CMD_WHEELBASE_CM / (float)CMD_LEFT_MIN_RADIUS_CM));
        float us_span = (float)((int32_t)SERVO_CENTER_US - (int32_t)CMD_SERVO_LEFT_US)
                      * TURN_LEFT_STEER_SCALE;
        int32_t delta_us = (int32_t)((delta_deg * us_span / max_delta_deg) + 0.5f);
        int32_t pulse = (int32_t)SERVO_CENTER_US - delta_us;
        if (pulse < (int32_t)CMD_SERVO_LEFT_US) pulse = (int32_t)CMD_SERVO_LEFT_US;
        return pulse;
    }
}

void Turn_Init(void)
{
    MotionHeading_Reset();
    tc_state = TURN_IDLE;
    tc_phase = 0U;
    tc_radius_cm = 0;
    tc_target_deg = 0;
    tc_target_abs_deg = 0.0f;
    tc_turn_dir = CMD_TURN_LEFT;
    tc_drive_dir = 1;
    tc_gyro_fail_count = 0U;
    Motor_Stop();
    Servo_SetPulse(SERVO_CENTER_US);
}

int Turn_StartRadiusAngle(int32_t radius_cm, int32_t angle_deg, int8_t turn_dir)
{
    if (angle_deg == INT32_MIN) return -2;
    int32_t abs_a = (angle_deg < 0) ? -angle_deg : angle_deg;
    int32_t min_radius_cm;
    int prep_rc;

    if (turn_dir != CMD_TURN_LEFT && turn_dir != CMD_TURN_RIGHT) return -2;
    if (!(TURN_LEFT_STEER_SCALE > 0.0f && TURN_LEFT_STEER_SCALE <= 1.0f) ||
        !(TURN_RIGHT_STEER_SCALE > 0.0f && TURN_RIGHT_STEER_SCALE <= 1.0f)) return -2;
    if (!(TURN_LEFT_RADIUS_SCALE > 0.0f && TURN_LEFT_RADIUS_SCALE <= TURN_RADIUS_SCALE_MAX) ||
        !(TURN_LEFT_REVERSE_RADIUS_SCALE > 0.0f && TURN_LEFT_REVERSE_RADIUS_SCALE <= TURN_RADIUS_SCALE_MAX) ||
        !(TURN_RIGHT_RADIUS_SCALE > 0.0f && TURN_RIGHT_RADIUS_SCALE <= TURN_RADIUS_SCALE_MAX) ||
        !(TURN_RIGHT_REVERSE_RADIUS_SCALE > 0.0f && TURN_RIGHT_REVERSE_RADIUS_SCALE <= TURN_RADIUS_SCALE_MAX)) return -2;
    if (!(TURN_LEFT_TARGET_SCALE > 0.0f && TURN_LEFT_TARGET_SCALE <= TURN_TARGET_SCALE_MAX) ||
        !(TURN_LEFT_REVERSE_TARGET_SCALE > 0.0f && TURN_LEFT_REVERSE_TARGET_SCALE <= TURN_TARGET_SCALE_MAX) ||
        !(TURN_RIGHT_REVERSE_TARGET_SCALE > 0.0f && TURN_RIGHT_REVERSE_TARGET_SCALE <= TURN_TARGET_SCALE_MAX) ||
        !(TURN_RIGHT_TARGET_SCALE > 0.0f && TURN_RIGHT_TARGET_SCALE <= TURN_TARGET_SCALE_MAX)) return -2;

    min_radius_cm = (turn_dir == CMD_TURN_RIGHT)
                          ? CMD_RIGHT_MIN_RADIUS_CM
                          : CMD_LEFT_MIN_RADIUS_CM;

    if (tc_state == TURN_RUNNING) return -3;
    if (radius_cm < min_radius_cm) return -2;
    if (abs_a < CMD_MIN_ANGLE_DEG || abs_a > CMD_MAX_ANGLE_DEG) return -2;

    MotionHeading_Prepare();

    if (MOTION_PREPARE_STOP_ENABLE)
    {
        Servo_SetPulse(SERVO_CENTER_US);
    }
    prep_rc = RobotHW_PrepareMotionSensors();
    if (prep_rc != 0) { MotionHeading_Degrade(); return prep_rc; }
    RobotHW_GyroPrimeFilter(GYRO_IIR_ALPHA);
    RobotHW_YawReset();

    int32_t pulse = Command_RadiusCmToPulseUs(radius_cm, turn_dir, angle_deg < 0);

    yaw_deg = 0.0f;
    yaw_at_motor_stop_deg = 0.0f;
    yaw_after_settle_deg = 0.0f;
    tc_peak_abs_dps = 0.0f;

    tc_radius_cm = radius_cm;
    tc_target_deg = angle_deg;
    tc_stop_rate_dps = 0.0f;
    /* Nominal route heading follows the requested physical angle. Apply
     * incoming route error before side/gear-specific calibration. */
    float expected_sign = turn_dir == CMD_TURN_RIGHT ? -1.0f : 1.0f;
    if (angle_deg < 0) expected_sign = -expected_sign;
    float local_angle = MotionHeading_Begin(expected_sign * (float)abs_a);
    float target_scale;
    if (turn_dir == CMD_TURN_RIGHT)
        target_scale = (angle_deg < 0) ? TURN_RIGHT_REVERSE_TARGET_SCALE
                                     : TURN_RIGHT_TARGET_SCALE;
    else
        target_scale = (angle_deg < 0) ? TURN_LEFT_REVERSE_TARGET_SCALE
                                     : TURN_LEFT_TARGET_SCALE;
    tc_target_abs_deg = expected_sign * local_angle * target_scale;
    tc_turn_dir = turn_dir;
    tc_drive_dir = (angle_deg >= 0) ? 1 : -1;
    tc_gyro_fail_count = 0U;
    tc_start_tick = HAL_GetTick();
    tc_last_tick = tc_start_tick;
    tc_phase_tick = tc_start_tick;
    tc_phase = 0U;
    tc_state = TURN_RUNNING;

    RobotHW_MotorBrakeRelease(); /* Preparation already stopped the wheels. */
    tc_servo_wait_ms = Command_ServoWaitMs(RobotHW_ServoGetUs(), (uint32_t)pulse);
    Servo_SetPulse((uint32_t)pulse);
    /* Start timing after preparation stops and the actual servo command. */
    tc_phase_tick = HAL_GetTick();
    tc_last_tick = tc_phase_tick;
    return 0;
}

static void Command_CompleteTurn(void)
{
    tc_state = TURN_DONE;
    tc_phase = 0U;
    MotionHeading_End();
}

static void Command_BeginPostTurnActions(void)
{
    yaw_after_settle_deg = yaw_deg; /* Preserve measured turn result. */
    if (TURN_END_HEADING_RESET_ENABLE) MotionHeading_Reset();
    if (tc_turn_dir == CMD_TURN_RIGHT && TURN_RIGHT_RETURN_ENABLE)
    {
        Servo_SetPulse(TURN_RIGHT_RECENTER_US);
        tc_phase = 4U;
        tc_phase_tick = HAL_GetTick();
        return;
    }
    if (tc_turn_dir == CMD_TURN_LEFT && TURN_LEFT_HEADING_OFF_ENABLE)
    {
        MotionHeading_SetEnabled(0U);
        RobotHW_YawReset();
    }
    Command_CompleteTurn();
}

void Turn_Step(void)
{
    if (tc_state != TURN_RUNNING) return;

    uint32_t now = HAL_GetTick();
    uint8_t brake_done = tc_phase == 3U &&
        (now - tc_phase_tick) >= TURN_BRAKE_HOLD_MS;
    /* Release on the deadline even if the next I2C transaction fails. */
    if (brake_done)
    {
        RobotHW_MotorBrakeRelease();
        Servo_SetPulse(SERVO_CENTER_US);
    }
    if ((now - tc_last_tick) < CONTROL_DELAY_MS && !brake_done) return;

    uint32_t delta_ms = now - tc_last_tick;
    tc_last_tick = now;

    uint32_t timeout_ms = TURN_TIMEOUT_MS;
    if (tc_phase == 4U || tc_phase == 5U)
        timeout_ms += TURN_RIGHT_RECENTER_HOLD_MS + TURN_RIGHT_CENTER_HOLD_MS;
    if (now - tc_start_tick >= timeout_ms)
    {
        Motor_Stop();
        Servo_SetPulse(SERVO_CENTER_US);
        tc_state = TURN_TIMEOUT;
        MotionHeading_Degrade();
        MotionHeading_End();
        return;
    }

    (void)RobotHW_GyroUpdateYaw((float)delta_ms / 1000.0f,
                                GYRO_IIR_ALPHA,
                                GYRO_RATE_DEADBAND_DPS);
    MotionHeading_Observe(RobotHW_YawGetDeg(), RobotHW_GyroLastReadOk());
    if (!RobotHW_GyroLastReadOk())
    {
        if (++tc_gyro_fail_count >= 5U)
        {
            Motor_Stop();
            Servo_SetPulse(SERVO_CENTER_US);
            tc_state = TURN_NO_IMU;
            tc_phase = 0U;
            MotionHeading_End();
            return;
        }
        /* Stopped phases must still advance their brake/settle timers. */
        if (tc_phase <= 1U) return;
    }
    else tc_gyro_fail_count = 0U;
    /* Continue sensor fault checks while returning the steering, but keep
     * the saved completed-turn angle independent of those stationary waits. */
    if (tc_phase != 4U && tc_phase != 5U) yaw_deg = RobotHW_YawGetDeg();
    {
        float abs_dps = AbsFloat(RobotHW_GyroGetLastZDps());   /* command-turn peak rate */
        if (abs_dps > tc_peak_abs_dps) tc_peak_abs_dps = abs_dps;
    }

    if (tc_phase == 0U)
    {
        if ((now - tc_phase_tick) >= tc_servo_wait_ms)
        {
            uint32_t initial_pwm = TURN_KICKSTART_PWM;

            /* Establish zero after the always-enabled steering wait. */
            RobotHW_GyroPrimeFilter(GYRO_IIR_ALPHA);
            RobotHW_YawReset();
            MotionHeading_LocalYawReset();
            RobotHW_EncoderReset();
            yaw_deg = 0.0f;
            tc_gyro_fail_count = 0U;
            tc_last_tick = now;
            if (tc_drive_dir >= 0) Drive_Forward(initial_pwm);
            else Drive_Backward(initial_pwm);
            tc_phase = 1U;
            tc_phase_tick = now;
        }
        return;
    }

    float expected_sign = (tc_turn_dir == CMD_TURN_RIGHT) ? -1.0f : +1.0f;
    if (tc_drive_dir < 0) expected_sign = -expected_sign;
    float progress_deg = expected_sign * yaw_deg;

    if (tc_phase == 1U)
    {
        uint32_t drive_elapsed_ms = now - tc_phase_tick;
        float turn_rate_dps = expected_sign * RobotHW_GyroGetLastZDps();
        float coast_time_s = (tc_turn_dir == CMD_TURN_LEFT)
                           ? TURN_LEFT_COAST_TIME_S : TURN_RIGHT_COAST_TIME_S;
        if (tc_turn_dir == CMD_TURN_LEFT && tc_drive_dir < 0)
            coast_time_s = TURN_LEFT_REVERSE_COAST_TIME_S;
        if (tc_turn_dir == CMD_TURN_RIGHT && tc_drive_dir < 0)
            coast_time_s = TURN_RIGHT_REVERSE_COAST_TIME_S;
        if (Command_IsShortRight())
            coast_time_s = TURN_RIGHT_SHORT_LEAD_TIME_S;
        float remaining_deg = tc_target_abs_deg - progress_deg;
        float predicted_final_deg;
        uint32_t drive_pwm;

        if (turn_rate_dps < 0.0f) turn_rate_dps = 0.0f;
        predicted_final_deg = progress_deg + turn_rate_dps * coast_time_s;

        if ((drive_elapsed_ms >= WRONG_SIGN_CHECK_MS) &&
            (progress_deg <= -WRONG_SIGN_LIMIT_DEG))
        {
            Motor_Stop();
            Servo_SetPulse(SERVO_CENTER_US);
            tc_state = TURN_WRONG_SIGN;
            MotionHeading_Degrade();
            MotionHeading_End();
            return;
        }

        if ((drive_elapsed_ms >= NO_TURN_CHECK_MS) &&
            (AbsFloat(yaw_deg) < NO_TURN_LIMIT_DEG))
        {
            Motor_Stop();
            Servo_SetPulse(SERVO_CENTER_US);
            tc_state = TURN_NO_TURN;
            MotionHeading_Degrade();
            MotionHeading_End();
            return;
        }

        if (progress_deg >= tc_target_abs_deg - TURN_STOP_TOLERANCE_DEG ||
            predicted_final_deg >= tc_target_abs_deg)
        {
            yaw_at_motor_stop_deg = yaw_deg;
            tc_stop_rate_dps = turn_rate_dps;
            RobotHW_MotorBrakeBegin();
            tc_phase = 3U; /* Sample yaw throughout the brake pulse. */
            tc_phase_tick = HAL_GetTick();
        }
        else
        {
            if (Command_KickActive(drive_elapsed_ms, progress_deg))
                drive_pwm = TURN_KICKSTART_PWM;
            else if (Command_IsShortRight())
                drive_pwm = Command_ShortRightPwm(remaining_deg);
            else if (tc_target_abs_deg <= TURN_SMALL_ANGLE_DEG)
                drive_pwm = TURN_SLOW_PWM;
            else if (remaining_deg <= TURN_FINAL_REMAINING_DEG)
                drive_pwm = TURN_SLOW_PWM;
            else if (remaining_deg <= TURN_SLOWDOWN_REMAINING_DEG)
                drive_pwm = TURN_MEDIUM_PWM;
            else
                drive_pwm = TURN_MOTOR_PWM;

            if (tc_drive_dir >= 0) Drive_Forward(drive_pwm);
            else Drive_Backward(drive_pwm);
        }
        return;
    }

    if (tc_phase == 3U)
    {
        if (brake_done)
        {
            tc_phase = 2U;
            tc_phase_tick = now;
            if (!TURN_POST_STOP_SETTLE_ENABLE)
            {
                Command_BeginPostTurnActions();
            }
        }
        return;
    }

    if (tc_phase == 2U)
    {
        if ((now - tc_phase_tick) >= POST_STOP_SETTLE_MS)
        {
            Command_BeginPostTurnActions();
        }
        return;
    }

    if (tc_phase == 4U)
    {
        if ((now - tc_phase_tick) >= TURN_RIGHT_RECENTER_HOLD_MS)
        {
            if (TURN_END_HEADING_RESET_ENABLE) MotionHeading_Reset();
            Servo_SetPulse(SERVO_CENTER_US);
            tc_phase = 5U;
            tc_phase_tick = HAL_GetTick();
        }
        return;
    }

    if (tc_phase == 5U)
    {
        if ((now - tc_phase_tick) >= TURN_RIGHT_CENTER_HOLD_MS)
            Command_CompleteTurn();
    }
}

void Turn_Stop(void)
{
    if (tc_state == TURN_RUNNING)
    {
        Motor_Stop();
        Servo_SetPulse(SERVO_CENTER_US);
        tc_state = TURN_ABORTED;
        tc_phase = 0U;
        MotionHeading_Degrade();
        MotionHeading_End();
    }
}

void Turn_StopMetrics(float *trigger_yaw_deg, float *trigger_rate_dps)
{
    if (trigger_yaw_deg) *trigger_yaw_deg = yaw_at_motor_stop_deg;
    if (trigger_rate_dps) *trigger_rate_dps = tc_stop_rate_dps;
}

float Turn_TargetMagnitudeDeg(void)
{
    return tc_target_abs_deg;
}

turn_state_t Turn_State(void)
{
    return tc_state;
}

void Turn_Status(uint32_t *elapsed_ms,
                 int32_t *radius_cm,
                 int32_t *target_deg,
                 float *yaw_deg_out,
                 int8_t *turn_dir)
{
    if (elapsed_ms)
    {
        *elapsed_ms = (tc_state == TURN_IDLE) ? 0U : (HAL_GetTick() - tc_start_tick);
    }
    if (radius_cm) *radius_cm = tc_radius_cm;
    if (target_deg) *target_deg = tc_target_deg;
    if (yaw_deg_out) *yaw_deg_out = yaw_deg;
    if (turn_dir) *turn_dir = tc_turn_dir;
}

/* Instrumentation: the gyro angle actually swept this turn (settled, signed:
 * + = LEFT) and the peak |yaw rate| seen during it. */
float Turn_SettledYawDeg(void)
{
    return yaw_after_settle_deg;
}

float Turn_PeakRateDps(void)
{
    return tc_peak_abs_dps;
}
