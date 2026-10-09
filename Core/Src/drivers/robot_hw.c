#include "drivers/robot_hw.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern I2C_HandleTypeDef hi2c2;


#define ICM_ADDR_68           0x68U
#define ICM_ADDR_69           0x69U
#define REG_BANK_SEL          0x7FU
#define WHO_AM_I_REG          0x00U
#define PWR_MGMT_1_REG        0x06U
#define PWR_MGMT_2_REG        0x07U
#define GYRO_ZOUT_H_REG       0x37U
#define GYRO_SMPLRT_DIV_REG   0x00U
#define GYRO_CONFIG_1_REG     0x01U
#define WHO_AM_I_EXPECTED     0xEAU
#define GYRO_SENSITIVITY      131.0f

static uint8_t icm_addr = 0U;
static uint8_t gyro_initialized = 0U;
static uint8_t gyro_has_offset = 0U;
static int32_t gyro_z_offset = 0;
static float gyro_z_filt = 0.0f;
static float yaw_deg = 0.0f;
static float last_gyro_z_dps = 0.0f;
static int16_t last_gyro_z_raw = 0;
static uint8_t last_gyro_read_ok = 0U;
static uint32_t servo_pulse_us = SERVO_CENTER_US;
static int32_t motor_left_pwm = 0;
static int32_t motor_right_pwm = 0;
static int32_t motor_left_requested = 0;
static int32_t motor_right_requested = 0;

static void RobotHW_MotorCoast(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
}

void RobotHW_StartMotionPeripherals(void)
{
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    RobotHW_MotorStop();
    RobotHW_ServoCenter();
}

void RobotHW_MotorBrakeBegin(void)
{
    motor_left_requested = 0;
    motor_right_requested = 0;
    motor_left_pwm = 0;
    motor_right_pwm = 0;
    /* PWM1, active high, ARR=7199 on TIM4/TIM9: ARR+1 keeps each
     * output continuously high, including the last counter tick. */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, RobotHW_MotorLeftMax() + 1U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, RobotHW_MotorLeftMax() + 1U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, RobotHW_MotorRightMax() + 1U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, RobotHW_MotorRightMax() + 1U);
}

void RobotHW_MotorBrakeRelease(void)
{
    RobotHW_MotorCoast();
}

void RobotHW_MotorStop(void)
{
    RobotHW_MotorBrakeBegin();
    HAL_Delay(MOTOR_BRAKE_PULSE_MS);
    RobotHW_MotorBrakeRelease();
}

static uint32_t Motor_ScaledMagnitude(int32_t pwm, uint32_t maximum)
{
    uint64_t magnitude = (pwm < 0) ? (uint64_t)(-(int64_t)pwm) : (uint64_t)pwm;
    uint64_t scaled = (magnitude * MOTOR_SPEED_SCALE_PERCENT + 50U) / 100U;
    return (scaled > maximum) ? maximum : (uint32_t)scaled;
}

void RobotHW_MotorSetSigned(int32_t left_pwm, int32_t right_pwm)
{
    uint32_t left_abs = Motor_ScaledMagnitude(left_pwm, RobotHW_MotorLeftMax());
    uint32_t right_abs = Motor_ScaledMagnitude(right_pwm, RobotHW_MotorRightMax());
    motor_left_requested = left_pwm;
    motor_right_requested = right_pwm;
    motor_left_pwm = (left_pwm < 0) ? -(int32_t)left_abs : (int32_t)left_abs;
    motor_right_pwm = (right_pwm < 0) ? -(int32_t)right_abs : (int32_t)right_abs;

    if (left_pwm >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, left_abs);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_abs);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
    }

    if (right_pwm >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, right_abs);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, right_abs);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
    }
}

void RobotHW_MotorSetForward(uint32_t left_pwm, uint32_t right_pwm)
{
    RobotHW_MotorSetSigned((int32_t)left_pwm, (int32_t)right_pwm);
}

void RobotHW_MotorSetBackward(uint32_t left_pwm, uint32_t right_pwm)
{
    RobotHW_MotorSetSigned(-(int32_t)left_pwm, -(int32_t)right_pwm);
}

int32_t RobotHW_MotorGetLeftSigned(void)
{
    return motor_left_pwm;
}

int32_t RobotHW_MotorGetRightSigned(void)
{
    return motor_right_pwm;
}

uint32_t RobotHW_MotorLeftMax(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim4);
}

uint32_t RobotHW_MotorRightMax(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim9);
}

uint32_t RobotHW_ClampPwm(int32_t pwm, uint32_t maximum)
{
    if (pwm < 0) return 0U;
    if ((uint32_t)pwm > maximum) return maximum;
    return (uint32_t)pwm;
}

void RobotHW_ServoSetUs(uint32_t pulse_us)
{
    servo_pulse_us = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse_us);
}

/* Straight-ahead servo pulse. Defaults to the compile-time SERVO_CENTER_US but
 * can be overridden at runtime by the auto-centre calibration (CALIBRATE,CENTER)
 * so the found value takes effect immediately without a reflash. Set the
 * SERVO_CENTER_US #define to the reported value to make it persist. */
static uint32_t servo_center_us = SERVO_CENTER_US;

void RobotHW_ServoSetUsTakeup(uint32_t pulse_us)
{
    /* Approach `pulse_us` from the LOW side (a pre-move SERVO_BACKLASH_US below
     * it, then up to it) so the steering slop is always taken up the same way -
     * a repeatable wheel angle regardless of the previous turn direction. */
    uint32_t pre = (pulse_us > (uint32_t)(CMD_SERVO_LEFT_US + SERVO_BACKLASH_US))
                       ? (pulse_us - SERVO_BACKLASH_US)
                       : (uint32_t)CMD_SERVO_LEFT_US;
    RobotHW_ServoSetUs(pre);
    HAL_Delay(SERVO_TAKEUP_SETTLE_MS);
    RobotHW_ServoSetUs(pulse_us);
    HAL_Delay(SERVO_TAKEUP_SETTLE_MS);
}

void RobotHW_ServoCenter(void)
{
    RobotHW_ServoSetUsTakeup(servo_center_us);
}

uint32_t RobotHW_ServoGetUs(void)
{
    return servo_pulse_us;
}

uint32_t RobotHW_ServoCenterUs(void)
{
    return servo_center_us;
}

void RobotHW_ServoSetCenterUs(uint32_t pulse_us)
{
    servo_center_us = pulse_us;
}

void RobotHW_EncoderReset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
}

RobotHW_EncoderSnapshot RobotHW_EncoderSnapshotRead(void)
{
    RobotHW_EncoderSnapshot snap;
    snap.left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    snap.right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    return snap;
}

RobotHW_EncoderDelta RobotHW_EncoderDeltaRead(RobotHW_EncoderSnapshot *previous)
{
    RobotHW_EncoderSnapshot current = RobotHW_EncoderSnapshotRead();
    RobotHW_EncoderDelta delta;

    delta.left_delta = (int16_t)(current.left - previous->left);
    delta.right_delta = (int16_t)(current.right - previous->right);
    *previous = current;
    return delta;
}

int32_t RobotHW_MotorGetLeftRequested(void)
{
    return motor_left_requested;
}

int32_t RobotHW_MotorGetRightRequested(void)
{
    return motor_right_requested;
}

static uint8_t RobotHW_WaitForStill(void)
{
    RobotHW_EncoderSnapshot previous = RobotHW_EncoderSnapshotRead();
    uint32_t start = HAL_GetTick();
    uint32_t stable_ms = 0U;

    while ((HAL_GetTick() - start) < MOTION_STILL_TIMEOUT_MS)
    {
        RobotHW_EncoderDelta delta;
        int32_t left_delta;
        int32_t right_delta;

        HAL_Delay(MOTION_STILL_SAMPLE_MS);
        delta = RobotHW_EncoderDeltaRead(&previous);
        left_delta = (delta.left_delta < 0) ? -(int32_t)delta.left_delta
                                            : (int32_t)delta.left_delta;
        right_delta = (delta.right_delta < 0) ? -(int32_t)delta.right_delta
                                               : (int32_t)delta.right_delta;

        if (left_delta <= MOTION_STILL_MAX_DELTA_COUNTS &&
            right_delta <= MOTION_STILL_MAX_DELTA_COUNTS)
        {
            stable_ms += MOTION_STILL_SAMPLE_MS;
            if (stable_ms >= MOTION_STILL_REQUIRED_MS) return 1U;
        }
        else
        {
            stable_ms = 0U;
        }
    }

    return 0U;
}

int RobotHW_PrepareMotionSensors(void)
{
    if (GYRO_RECALIBRATE_BEFORE_EACH_COMMAND || !RobotHW_GyroHasOffset())
    {
        RobotHW_MotorStop();
        if (!RobotHW_WaitForStill()) return -4;
    }

    if (!RobotHW_GyroIsInitialized() && !RobotHW_GyroInit()) return -1;

    if (!RobotHW_GyroHasOffset())
    {
        if (!RobotHW_GyroCalibrate(GYRO_CALIB_WARMUP_MS,
                                   GYRO_CALIB_DELAY_MS,
                                   GYRO_CALIB_SAMPLES)) return -1;
    }
    else if (GYRO_RECALIBRATE_BEFORE_EACH_COMMAND)
    {
        if (!RobotHW_GyroCalibrate(GYRO_REZERO_WARMUP_MS,
                                   GYRO_REZERO_DELAY_MS,
                                   GYRO_REZERO_SAMPLES)) return -1;
    }

    RobotHW_GyroPrimeFilter(1.0f);
    RobotHW_YawReset();
    RobotHW_EncoderReset();
    return 0;
}

static HAL_StatusTypeDef ICM_SelectBank(uint8_t bank)
{
    uint8_t value = (uint8_t)(bank << 4);
    return HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(icm_addr << 1),
                             REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, 100U);
}

static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(icm_addr << 1),
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, 100U);
}

static HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(icm_addr << 1),
                            reg, I2C_MEMADD_SIZE_8BIT,
                            data, length, 100U);
}

uint8_t RobotHW_GyroFindDevice(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(ICM_ADDR_68 << 1), 3U, 100U) == HAL_OK)
    {
        icm_addr = ICM_ADDR_68;
        return 1U;
    }
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(ICM_ADDR_69 << 1), 3U, 100U) == HAL_OK)
    {
        icm_addr = ICM_ADDR_69;
        return 1U;
    }
    return 0U;
}

uint8_t RobotHW_GyroInit(void)
{
    uint8_t who = 0U;

    if (icm_addr == 0U && !RobotHW_GyroFindDevice()) return 0U;
    if (ICM_SelectBank(0U) != HAL_OK) return 0U;
    if (ICM_ReadRegs(WHO_AM_I_REG, &who, 1U) != HAL_OK) return 0U;
    if (who != WHO_AM_I_EXPECTED) return 0U;
    if (ICM_WriteReg(PWR_MGMT_1_REG, 0x01U) != HAL_OK) return 0U;
    if (ICM_WriteReg(PWR_MGMT_2_REG, 0x00U) != HAL_OK) return 0U;
    HAL_Delay(100U);
    if (ICM_SelectBank(2U) != HAL_OK) return 0U;
    if (ICM_WriteReg(GYRO_SMPLRT_DIV_REG, 10U) != HAL_OK) return 0U;
    if (ICM_WriteReg(GYRO_CONFIG_1_REG, 0x29U) != HAL_OK) return 0U;
    if (ICM_SelectBank(0U) != HAL_OK) return 0U;
    HAL_Delay(100U);
    gyro_initialized = 1U;
    return 1U;
}

uint8_t RobotHW_GyroIsInitialized(void)
{
    return gyro_initialized;
}

uint8_t RobotHW_GyroHasOffset(void)
{
    return gyro_has_offset;
}

uint8_t RobotHW_GyroCalibrate(uint32_t warmup_ms,
                              uint32_t delay_ms,
                              uint32_t samples)
{
    int32_t sum = 0;
    uint32_t warmup_loops;
    uint32_t valid_samples = 0U;
    uint32_t attempts = 0U;

    if (delay_ms == 0U || samples == 0U) return 0U;
    warmup_loops = warmup_ms / delay_ms;

    for (uint32_t i = 0U; i < warmup_loops; i++)
    {
        (void)RobotHW_GyroReadZRaw();
        HAL_Delay(delay_ms);
    }

    while (valid_samples < samples && attempts < samples * 3U)
    {
        int16_t raw = RobotHW_GyroReadZRaw();
        if (last_gyro_read_ok)
        {
            sum += raw;
            valid_samples++;
        }
        attempts++;
        HAL_Delay(delay_ms);
    }

    if (valid_samples < samples) return 0U;

    gyro_z_offset = sum / (int32_t)valid_samples;
    gyro_z_filt = (float)gyro_z_offset;
    yaw_deg = 0.0f;
    last_gyro_z_dps = 0.0f;
    gyro_has_offset = 1U;
    return 1U;
}

int16_t RobotHW_GyroReadZRaw(void)
{
    uint8_t data[2] = {0U, 0U};
    if (ICM_ReadRegs(GYRO_ZOUT_H_REG, data, 2U) != HAL_OK)
    {
        last_gyro_read_ok = 0U;
        return last_gyro_z_raw;
    }
    last_gyro_z_raw = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    last_gyro_read_ok = 1U;
    return last_gyro_z_raw;
}

void RobotHW_GyroSetOffset(int32_t offset)
{
    gyro_z_offset = offset;
    gyro_z_filt = (float)gyro_z_offset;
    gyro_has_offset = 1U;
}

int32_t RobotHW_GyroGetOffset(void)
{
    return gyro_z_offset;
}

void RobotHW_GyroPrimeFilter(float alpha)
{
    (void)alpha;
    gyro_z_filt = (float)gyro_z_offset;
}

float RobotHW_GyroUpdateYaw(float dt_sec,
                            float alpha,
                            float deadband_dps)
{
    int16_t raw = RobotHW_GyroReadZRaw();
    if (!last_gyro_read_ok)
    {
        /* A missing I2C sample is not zero angular rate minus a nonzero bias.
         * Hold yaw and discard this interval instead of creating a false turn. */
        last_gyro_z_dps = 0.0f;
        return 0.0f;
    }
    gyro_z_filt = alpha * (float)raw + (1.0f - alpha) * gyro_z_filt;

    last_gyro_z_dps = ((gyro_z_filt - (float)gyro_z_offset) /
                       GYRO_SENSITIVITY) * GYRO_Z_SCALE;
    if (deadband_dps > 0.0f)
    {
        float abs_rate = (last_gyro_z_dps < 0.0f) ? -last_gyro_z_dps : last_gyro_z_dps;
        if (abs_rate < deadband_dps) last_gyro_z_dps = 0.0f;
    }

    yaw_deg += last_gyro_z_dps * dt_sec;
    return last_gyro_z_dps;
}

uint8_t RobotHW_GyroLastReadOk(void)
{
    return last_gyro_read_ok;
}

void RobotHW_YawReset(void)
{
    yaw_deg = 0.0f;
    gyro_z_filt = (float)gyro_z_offset;
    last_gyro_z_dps = 0.0f;
}

float RobotHW_YawGetDeg(void)
{
    return yaw_deg;
}

float RobotHW_GyroGetLastZDps(void)
{
    return last_gyro_z_dps;
}
