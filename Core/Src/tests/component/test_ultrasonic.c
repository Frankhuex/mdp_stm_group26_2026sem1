#include "test_config.h"
#if ACTIVE_TEST == TEST_ULTRASONIC

/*
 * HC-SR04 ultrasonic test using TIM8 Channel 3 (PC8) in Input Capture mode.
 *
 * Wiring:
 *   TRIG -> PB15 (GPIO output)
 *   ECHO -> PC8  (TIM8_CH3, AF, via a 5V-to-3.3V divider)
 *
 * CubeMX configuration used by this project:
 *   - PC8 = TIM8_CH3, Input Capture direct mode, Polarity = Both Edges
 *   - TIM8 Prescaler = 15, Period = 65535 (1 tick = 1 us at 16 MHz)
 *   - TIM8 NVIC capture/compare interrupt enabled
 *
 * TIM6 is this project's HAL system tick source, so the trigger pulse uses
 * the DWT cycle counter for microsecond timing instead of borrowing TIM6.
 */

#include "tests/component/test_ultrasonic.h"
#include "main.h"
#include "drivers/oled.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart3;

#define TRIG_GPIO_PORT   GPIOB
#define TRIG_GPIO_PIN    GPIO_PIN_15

#define US_TIMEOUT_MS          60U
#define US_MAX_VALID_PULSE_US  25000U
#define US_SAMPLE_PERIOD_MS    250U

static volatile uint32_t g_rising_capture = 0;
static volatile uint32_t g_echo_pulse_us  = 0;
static volatile uint8_t  g_echo_ready     = 0;
static volatile uint32_t g_edge_count     = 0;
static volatile uint32_t g_last_captured  = 0;
static volatile uint8_t  g_last_pin_state = 0xFFU;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us_dwt(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks) { /* spin */ }
}

static uint32_t ultrasonic_pulse_to_cm_x10(uint32_t pulse_us)
{
    /* distance_cm = pulse_us * 0.0343 / 2, then scaled by 10. */
    return (pulse_us * 1715U + 5000U) / 10000U;
}

static void uart_send(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HAL_MAX_DELAY);
}

static void oled_show_ultrasonic(const char *status,
                                 uint32_t pulse_us,
                                 int32_t distance_cm_x10)
{
    char line[22];

    OLED_Clear();
    OLED_ShowString(0, 0, (const u8 *)"ULTRASONIC");

    snprintf(line, sizeof(line), "STATUS:%s", status);
    OLED_ShowString(0, 16, (const u8 *)line);

    snprintf(line, sizeof(line), "P:%lu us", (unsigned long)pulse_us);
    OLED_ShowString(0, 32, (const u8 *)line);

    if (distance_cm_x10 >= 0)
    {
        snprintf(line,
                 sizeof(line),
                 "D:%ld.%ld cm",
                 (long)(distance_cm_x10 / 10),
                 (long)(distance_cm_x10 % 10));
    }
    else
    {
        snprintf(line, sizeof(line), "D:--.- cm");
    }
    OLED_ShowString(0, 48, (const u8 *)line);

    OLED_Refresh_Gram();
}

static void ultrasonic_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = TRIG_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG_GPIO_PORT, &gpio);

    dwt_init();
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
}

static void ultrasonic_trigger(void)
{
    g_echo_ready = 0;

    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
    delay_us_dwt(2);
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_SET);
    delay_us_dwt(10);
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM8)
    {
        return;
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint8_t pin_now = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;

        g_edge_count++;
        g_last_captured = captured;
        g_last_pin_state = pin_now;

        if (pin_now)
        {
            g_rising_capture = captured;
        }
        else
        {
            if (captured >= g_rising_capture)
            {
                g_echo_pulse_us = captured - g_rising_capture;
            }
            else
            {
                g_echo_pulse_us = (0xFFFFU - g_rising_capture) + captured + 1U;
            }
            g_echo_ready = 1;
        }
    }
}

void Test_Ultrasonic(void)
{
    char line[96];

    OLED_Init();
    OLED_Display_On();
    oled_show_ultrasonic("INIT", 0U, -1);

    ultrasonic_init();

    uart_send("US_TEST_INIT_OK\r\n");
    oled_show_ultrasonic("READY", 0U, -1);

    while (1)
    {
        ultrasonic_trigger();

        uint32_t t0 = HAL_GetTick();
        while (!g_echo_ready)
        {
            if ((HAL_GetTick() - t0) > US_TIMEOUT_MS)
            {
                uart_send("US=NO_ECHO (timeout)\r\n");
                oled_show_ultrasonic("NO_ECHO", 0U, -1);
                goto next_reading;
            }
        }

        {
            uint32_t pulse_us = g_echo_pulse_us;

            if (pulse_us > US_MAX_VALID_PULSE_US)
            {
                uart_send("US=OUT_OF_RANGE\r\n");
                oled_show_ultrasonic("OUT_OF_RANGE", pulse_us, -1);
            }
            else
            {
                uint32_t distance_cm_x10 = ultrasonic_pulse_to_cm_x10(pulse_us);

                snprintf(line,
                         sizeof(line),
                         "pulse=%lu us  dist=%lu.%lu cm\r\n",
                         (unsigned long)pulse_us,
                         (unsigned long)(distance_cm_x10 / 10U),
                         (unsigned long)(distance_cm_x10 % 10U));
                uart_send(line);
                oled_show_ultrasonic("OK", pulse_us, (int32_t)distance_cm_x10);
            }
        }

next_reading:
        HAL_Delay(US_SAMPLE_PERIOD_MS);
    }
}

#endif /* ACTIVE_TEST == TEST_ULTRASONIC */
