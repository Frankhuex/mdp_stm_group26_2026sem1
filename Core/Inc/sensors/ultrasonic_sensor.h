#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdint.h>

/*
 * HC-SR04 wiring matching mdp21_8_ultrasonic:
 *   TRIG -> PB15 GPIO output
 *   ECHO -> PC8 TIM8_CH3 through a 5V-to-3.3V divider
 *
 * TIM8: 1 MHz, ARR=65535, CH3 direct input capture on both edges.
 * Trigger pulse uses DWT; HAL TIM6 system tick is not modified.
 */
int UltrasonicSensor_Init(void);
int UltrasonicSensor_Read(uint32_t *pulse_us, uint32_t *distance_mm);
/* Last explicit US,GET result, mm; UINT32_MAX means unmeasured/invalid.
 * Separate from UNTILUS caches; other sensor reads do not update it. */
extern uint32_t Ultrasonic_LastGetMm;

typedef struct
{
    uint32_t pulse_us;
    uint32_t distance_mm;
    uint32_t timestamp_ms;
    uint32_t sequence;
    int status; /* 0 valid, -1 measurement timeout, -2 pulse >25000 us */
} ultrasonic_sample_t;

/* One measurement at a time; at least 60 ms between triggers. */
int UltrasonicSensor_Trigger(void);
void UltrasonicSensor_Step(void);
void UltrasonicSensor_GetSample(ultrasonic_sample_t *sample);
void UltrasonicSensor_Cancel(void);

typedef struct
{
    uint32_t edge_count;
    uint32_t last_capture;
    uint8_t last_pin_state;
    /* Last blocking test read only; passive observations, not a second reader. */
    uint8_t test_saw_echo_high;
    uint8_t test_saw_cc3_flag;
    uint8_t test_timer_advanced;
} ultrasonic_diagnostics_t;
void UltrasonicSensor_GetDiagnostics(ultrasonic_diagnostics_t *diag);

#endif /* ULTRASONIC_SENSOR_H */
