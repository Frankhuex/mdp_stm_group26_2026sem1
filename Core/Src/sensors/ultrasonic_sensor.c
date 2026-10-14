/* Acquisition primitives below are copied from the working
 * mdp21_8_ultrasonic/Core/Src/tests/test_ir_ultrasonic.c.
 * The public API only adds nonblocking result/timeout delivery for UNTILUS. */
#include "main.h"
#include "test_config.h"
#include "sensors/ultrasonic_sensor.h"

uint32_t Ultrasonic_LastGetMm = UINT32_MAX;
extern TIM_HandleTypeDef htim8;
#define TRIG_GPIO_PORT GPIOB
#define TRIG_GPIO_PIN GPIO_PIN_15
#define US_TIMEOUT_MS 60U
#define US_MAX_VALID_PULSE_US 25000U
#define US_MIN_TRIGGER_PERIOD_MS 60U

static volatile uint32_t g_rising_capture = 0;
static volatile uint32_t g_echo_pulse_us = 0;
static volatile uint8_t g_echo_ready = 0;
static volatile uint32_t g_echo_capture_ms = 0;
static volatile uint32_t g_edge_count = 0;
static volatile uint32_t g_last_captured = 0;
static volatile uint8_t g_last_pin_state = 0xFFU;

static uint8_t initialized;
static uint8_t waiting;
static uint8_t triggered;
static uint32_t trigger_tick;
static ultrasonic_sample_t latest;
static uint8_t test_saw_echo_high;
static uint8_t test_saw_cc3_flag;
static uint8_t test_timer_advanced;


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

static HAL_StatusTypeDef ultrasonic_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = TRIG_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG_GPIO_PORT, &gpio);

    dwt_init();
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
    return HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
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

/* The standalone reference test owns the callback in test mode. */
#if ACTIVE_TEST != TEST_ULTRASONIC
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
            g_echo_capture_ms = HAL_GetTick(); /* Metadata for UNTILUS freshness. */
            g_echo_ready = 1;
        }
    }
}
#endif /* ACTIVE_TEST != TEST_ULTRASONIC */


int UltrasonicSensor_Init(void)
{
    if (initialized) return 0;
    if (ultrasonic_init() != HAL_OK) return -1;
    latest.status = -1;
    initialized = 1U;
    return 0;
}

int UltrasonicSensor_Trigger(void)
{
    uint32_t now = HAL_GetTick();
    if (!initialized) return -1;
    if (waiting || (triggered && now - trigger_tick < US_MIN_TRIGGER_PERIOD_MS))
        return -3;
    /* As in the reference: always send the trigger, regardless of ECHO level.
     * No flag clearing, counter reset or phase-gated capture callback. */
    ultrasonic_trigger();
    trigger_tick = HAL_GetTick();
    triggered = 1U;
    waiting = 1U;
    return 0;
}

void UltrasonicSensor_Step(void)
{
    uint32_t pulse;
    uint32_t capture_ms;
    uint8_t ready;
    uint32_t irq;
    if (!waiting) return;
    irq = __get_PRIMASK();
    __disable_irq();
    ready = g_echo_ready;
    pulse = g_echo_pulse_us;
    capture_ms = g_echo_capture_ms;
    __set_PRIMASK(irq);

    if (ready)
    {
        latest.pulse_us = pulse;
        latest.status = pulse > US_MAX_VALID_PULSE_US ? -2 : 0;
        /* cm x10 equals mm: use the reference conversion unchanged. */
        latest.distance_mm = latest.status == 0 ? ultrasonic_pulse_to_cm_x10(pulse) : 0U;
    }
    else if (HAL_GetTick() - trigger_tick > US_TIMEOUT_MS)
    {
        latest.pulse_us = 0U;
        latest.distance_mm = 0U;
        latest.status = -1;
    }
    else return;
    latest.timestamp_ms = ready ? capture_ms : HAL_GetTick();
    latest.sequence++;
    waiting = 0U;
}

void UltrasonicSensor_GetSample(ultrasonic_sample_t *sample)
{
    if (sample) *sample = latest; /* Written only by foreground Step(). */
}

void UltrasonicSensor_Cancel(void)
{
    waiting = 0U;
    /* Keep capture running exactly as in the reference. The next trigger
     * clears g_echo_ready; cancelled results cannot complete a new command. */
}

void UltrasonicSensor_GetDiagnostics(ultrasonic_diagnostics_t *diag)
{
    uint32_t irq;
    if (!diag) return;
    irq = __get_PRIMASK();
    __disable_irq();
    diag->edge_count = g_edge_count;
    diag->last_capture = g_last_captured;
    diag->last_pin_state = g_last_pin_state;
    diag->test_saw_echo_high = test_saw_echo_high;
    diag->test_saw_cc3_flag = test_saw_cc3_flag;
    diag->test_timer_advanced = test_timer_advanced;
    __set_PRIMASK(irq);
}

/* Test-only blocking wait, matching the reference's wait/read cadence. */
int UltrasonicSensor_Read(uint32_t *pulse_us, uint32_t *distance_mm)
{
    uint32_t sequence;
    uint32_t initial_counter;
    int rc;
    if (!pulse_us || !distance_mm) return -3;
    *pulse_us = 0U;
    *distance_mm = 0U;
    if (!initialized) return -1;
    if (waiting) return -3; /* Do not steal an asynchronous measurement. */
    sequence = latest.sequence;
    test_saw_echo_high = 0U;
    test_saw_cc3_flag = 0U;
    test_timer_advanced = 0U;
    do { rc = UltrasonicSensor_Trigger(); } while (rc == -3);
    if (rc != 0) return rc;
    initial_counter = TIM8->CNT;
    do
    {
        /* Read-only diagnostics: never clear SR or read CCR3 here, since doing
         * so could consume the hardware event before the real callback. */
        if (GPIOC->IDR & GPIO_PIN_8) test_saw_echo_high = 1U;
        if (TIM8->SR & TIM_SR_CC3IF) test_saw_cc3_flag = 1U;
        if (TIM8->CNT != initial_counter) test_timer_advanced = 1U;
        UltrasonicSensor_Step();
    } while (latest.sequence == sequence);
    *pulse_us = latest.pulse_us;
    *distance_mm = latest.distance_mm;
    return latest.status;
}
