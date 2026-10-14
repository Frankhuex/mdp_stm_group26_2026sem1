#include "main.h"
#include "sensors/ir_sensor.h"

#include <stddef.h>

/* ADC1 is configured in main.c / CubeMX. */
extern ADC_HandleTypeDef hadc1;
uint32_t IRSensor_LeftLastGetCmX10 = UINT32_MAX;
uint32_t IRSensor_RightLastGetCmX10 = UINT32_MAX;


/* ============================================================
 * CALIBRATION POINT
 *
 * cm_x10 stores centimetres multiplied by 10:
 *
 *   100 = 10.0 cm
 *   150 = 15.0 cm
 *   500 = 50.0 cm
 *   600 = 60.0 cm
 * ============================================================ */
typedef struct
{
    uint16_t raw;
    uint16_t cm_x10;
} IRCalPoint;


/* ============================================================
 * LEFT IR CALIBRATION
 *
 * Sharp GP2Y0A21YK
 * VO -> PC0 / ADC1_IN10
 *
 * Latest measured/repaired-wire calibration.
 *
 * Valid calibrated range:
 *      10 cm ... 50 cm
 *
 * ADC decreases as distance increases.
 * ============================================================ */
static const IRCalPoint k_left_cal[] =
{
    {2958U, 100U},   /* 10 cm */
    {2078U, 150U},   /* 15 cm */
    {1705U, 200U},   /* 20 cm */
    {1407U, 250U},   /* 25 cm */
    {1279U, 300U},   /* 30 cm */

    {1056U, 400U},   /* 40 cm */
    { 985U, 450U},   /* 45 cm */
    { 913U, 500U}    /* 50 cm */
};


/* ============================================================
 * RIGHT IR CALIBRATION
 *
 * Sharp GP2Y0A21YK
 * VO -> PC1 / ADC1_IN11
 *
 * Valid calibrated range:
 *      10 cm ... 60 cm
 *
 * 70 cm and 80 cm were NOT included because the measurements
 * became non-monotonic / unreliable.
 * ============================================================ */
static const IRCalPoint k_right_cal[] =
{
    {2968U, 100U},   /* 10 cm */
    {2086U, 150U},   /* 15 cm */
    {1670U, 200U},   /* 20 cm */
    {1389U, 250U},   /* 25 cm */
    {1246U, 300U},   /* 30 cm */
    {1025U, 400U},   /* 40 cm */
    { 947U, 500U},   /* 50 cm */
    { 842U, 600U}    /* 60 cm */
};


/* ============================================================
 * ENDPOINT GUARD BANDS
 * ============================================================ */

/* LEFT:
 * 10 cm readings:
 *   2987, 2944, 2942
 *
 * 50 cm latest readings:
 *   908, 897, 935
 */
#define LEFT_10CM_MAX_RAW       2987U
#define LEFT_50CM_MIN_RAW        897U


/* RIGHT:
 * 10 cm readings:
 *   2968, 2969, 2967
 *
 * 60 cm readings:
 *   831, 848, 848
 */
#define RIGHT_10CM_MAX_RAW      2969U
#define RIGHT_60CM_MIN_RAW       831U


/* ============================================================
 * SIDE-SPECIFIC VALID DISTANCE LIMITS
 * ============================================================ */

#define LEFT_MIN_CM_X10          100U    /* 10.0 cm */
#define LEFT_MAX_CM_X10          500U    /* 50.0 cm */

#define RIGHT_MIN_CM_X10         100U    /* 10.0 cm */
#define RIGHT_MAX_CM_X10         600U    /* 60.0 cm */


/* ============================================================
 * ADC RAW -> MILLIVOLTS
 * ============================================================ */
static uint16_t adc_to_mv(uint16_t raw)
{
    return (uint16_t)(
        ((uint32_t)raw * 3300U + 2047U) / 4095U
    );
}


/* ============================================================
 * PIECEWISE LINEAR INTERPOLATION
 * ============================================================ */
static uint16_t interpolate_cm_x10(const IRCalPoint *table,
                                   uint32_t count,
                                   uint16_t raw)
{
    uint32_t i;

    if (table == NULL || count == 0U)
    {
        return 0U;
    }

    for (i = 0U; i + 1U < count; i++)
    {
        uint16_t raw_near = table[i].raw;
        uint16_t raw_far  = table[i + 1U].raw;

        if (raw <= raw_near && raw >= raw_far)
        {
            uint16_t cm_near = table[i].cm_x10;
            uint16_t cm_far  = table[i + 1U].cm_x10;

            uint32_t raw_span =
                (uint32_t)raw_near - (uint32_t)raw_far;

            uint32_t raw_from_near =
                (uint32_t)raw_near - (uint32_t)raw;

            uint32_t cm_span =
                (uint32_t)cm_far - (uint32_t)cm_near;

            if (raw_span == 0U)
            {
                return cm_near;
            }

            return (uint16_t)(
                (uint32_t)cm_near +
                (
                    (raw_from_near * cm_span) +
                    (raw_span / 2U)
                ) /
                raw_span
            );
        }
    }

    return table[count - 1U].cm_x10;
}


/* ============================================================
 * READ BOTH IR SENSORS ONCE
 *
 * ADC1 scan sequence:
 *   Rank 1 = ADC1_IN10 / PC0 = LEFT
 *   Rank 2 = ADC1_IN11 / PC1 = RIGHT
 * ============================================================ */
int IRSensor_ReadPair(IRSensorReading *out)
{
    uint16_t left_raw;
    uint16_t right_raw;

    if (out == NULL)
    {
        return -1;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return -2;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 20U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return -3;
    }

    left_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 20U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return -4;
    }

    right_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

    (void)HAL_ADC_Stop(&hadc1);

    out->left_raw  = left_raw;
    out->right_raw = right_raw;

    out->left_mv  = adc_to_mv(left_raw);
    out->right_mv = adc_to_mv(right_raw);

    return 0;
}


/* ============================================================
 * AVERAGE MULTIPLE READINGS
 * ============================================================ */
int IRSensor_ReadAverage(IRSensorReading *out,
                         uint32_t samples)
{
    uint32_t left_sum  = 0U;
    uint32_t right_sum = 0U;
    IRSensorReading one;
    uint32_t i;

    if (out == NULL || samples == 0U)
    {
        return -1;
    }

    for (i = 0U; i < samples; i++)
    {
        int rc = IRSensor_ReadPair(&one);

        if (rc != 0)
        {
            return rc;
        }

        left_sum  += one.left_raw;
        right_sum += one.right_raw;
    }

    out->left_raw  = (uint16_t)(left_sum / samples);
    out->right_raw = (uint16_t)(right_sum / samples);

    out->left_mv  = adc_to_mv(out->left_raw);
    out->right_mv = adc_to_mv(out->right_raw);

    return 0;
}


/* ============================================================
 * CONVERT RAW ADC -> DISTANCE
 *
 * LEFT:
 *      valid = 10 ... 50 cm
 *
 * RIGHT:
 *      valid = 10 ... 60 cm
 * ============================================================ */
uint16_t IRSensor_RawToCmX10(IRSensorSide side,
                             uint16_t raw,
                             IRSensorRangeStatus *status)
{
    const IRCalPoint *table;
    uint32_t count;
    uint16_t near_limit;
    uint16_t far_limit;
    uint16_t min_cm_x10;
    uint16_t max_cm_x10;

    if (status == NULL)
    {
        return 0U;
    }

    if (side == IR_SENSOR_LEFT)
    {
        table = k_left_cal;
        count = (uint32_t)(sizeof(k_left_cal) / sizeof(k_left_cal[0]));

        near_limit = LEFT_10CM_MAX_RAW;
        far_limit  = LEFT_50CM_MIN_RAW;

        min_cm_x10 = LEFT_MIN_CM_X10;
        max_cm_x10 = LEFT_MAX_CM_X10;
    }
    else if (side == IR_SENSOR_RIGHT)
    {
        table = k_right_cal;
        count = (uint32_t)(sizeof(k_right_cal) / sizeof(k_right_cal[0]));

        near_limit = RIGHT_10CM_MAX_RAW;
        far_limit  = RIGHT_60CM_MIN_RAW;

        min_cm_x10 = RIGHT_MIN_CM_X10;
        max_cm_x10 = RIGHT_MAX_CM_X10;
    }
    else
    {
        *status = IR_RANGE_ERROR;
        return 0U;
    }

    if (raw > near_limit)
    {
        *status = IR_RANGE_NEAR;
        return min_cm_x10;
    }

    if (raw < far_limit)
    {
        *status = IR_RANGE_FAR;
        return max_cm_x10;
    }

    if (raw >= table[0].raw)
    {
        *status = IR_RANGE_VALID;
        return table[0].cm_x10;
    }

    if (raw <= table[count - 1U].raw)
    {
        *status = IR_RANGE_VALID;
        return table[count - 1U].cm_x10;
    }

    *status = IR_RANGE_VALID;

    return interpolate_cm_x10(
        table,
        count,
        raw
    );
}


/* ============================================================
 * READ + CONVERT BOTH SENSORS
 * ============================================================ */
int IRSensor_ReadDistance(IRSensorDistanceReading *out,
                          uint32_t samples)
{
    int rc;

    if (out == NULL)
    {
        return -1;
    }

    rc = IRSensor_ReadAverage(
        &out->adc,
        samples
    );

    if (rc != 0)
    {
        out->left_status  = IR_RANGE_ERROR;
        out->right_status = IR_RANGE_ERROR;

        out->left_cm_x10  = 0U;
        out->right_cm_x10 = 0U;

        return rc;
    }

    out->left_cm_x10 =
        IRSensor_RawToCmX10(
            IR_SENSOR_LEFT,
            out->adc.left_raw,
            &out->left_status
        );

    out->right_cm_x10 =
        IRSensor_RawToCmX10(
            IR_SENSOR_RIGHT,
            out->adc.right_raw,
            &out->right_status
        );

    return 0;
}


/* ============================================================
 * RANGE STATUS -> TEXT
 * ============================================================ */
const char *IRSensor_RangeText(IRSensorRangeStatus status)
{
    switch (status)
    {
        case IR_RANGE_VALID:
            return "VALID";

        case IR_RANGE_NEAR:
            return "NEAR";

        case IR_RANGE_FAR:
            return "FAR";

        case IR_RANGE_ERROR:
        default:
            return "ERROR";
    }
}
