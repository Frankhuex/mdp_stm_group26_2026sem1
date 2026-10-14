#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>

/* Explicit LEFTIR/RIGHTIR,GET caches, 0.1 cm (=mm).
 * Only the selected side updates; UINT32_MAX = unmeasured/NEAR/FAR/error. */
extern uint32_t IRSensor_LeftLastGetCmX10;
extern uint32_t IRSensor_RightLastGetCmX10;

/*
 * 2x Sharp GP2Y0A21YK used by this project
 *
 * LEFT  VO -> PC0 / ADC1_IN10
 * RIGHT VO -> PC1 / ADC1_IN11
 * VCC      -> 5V
 * GND      -> common GND
 *
 * Calibration data was measured on this exact robot on 2026-09-07.
 * The validated centimetre conversion is intentionally limited to 10..50 cm.
 * 60..80 cm measurements were not repeatable enough to trust yet.
 */

#define IR_SENSOR_VALID_MIN_CM_X10   100U   /* 10.0 cm */
#define IR_SENSOR_VALID_MAX_CM_X10   500U   /* 50.0 cm */

typedef enum
{
    IR_SENSOR_LEFT = 0,
    IR_SENSOR_RIGHT = 1
} IRSensorSide;

typedef enum
{
    IR_RANGE_VALID = 0,
    IR_RANGE_NEAR,       /* stronger than calibrated 10 cm endpoint */
    IR_RANGE_FAR,        /* weaker than calibrated 50 cm endpoint */
    IR_RANGE_ERROR
} IRSensorRangeStatus;

typedef struct
{
    uint16_t left_raw;
    uint16_t right_raw;
    uint16_t left_mv;
    uint16_t right_mv;
} IRSensorReading;

typedef struct
{
    IRSensorReading adc;
    uint16_t left_cm_x10;      /* 237 means 23.7 cm */
    uint16_t right_cm_x10;
    IRSensorRangeStatus left_status;
    IRSensorRangeStatus right_status;
} IRSensorDistanceReading;

/* Raw ADC acquisition. */
int IRSensor_ReadPair(IRSensorReading *out);
int IRSensor_ReadAverage(IRSensorReading *out, uint32_t samples);

/*
 * Piecewise-linear conversion using this robot's measured calibration table.
 * Result is cm x10 to avoid printf-float requirements in STM32CubeIDE.
 * Example: 253 = 25.3 cm.
 */
uint16_t IRSensor_RawToCmX10(IRSensorSide side,
                             uint16_t raw,
                             IRSensorRangeStatus *status);

/* Read averaged ADC values and convert LEFT + RIGHT to cm x10. */
int IRSensor_ReadDistance(IRSensorDistanceReading *out, uint32_t samples);

const char *IRSensor_RangeText(IRSensorRangeStatus status);

#endif /* IR_SENSOR_H */
