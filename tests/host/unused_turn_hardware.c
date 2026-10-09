/* The legacy min-radius diagnostic shares the turn-controller source file.
 * Its hardware entry points must never run in these host tests. */
#include "main.h"
#include <assert.h>
#include "control/straight_control.h"
#include "control/turn_control.h"
#include <string.h>
TIM_HandleTypeDef htim2, htim3, htim4, htim9, htim12;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;
uint8_t OLED_GRAM[128][8];
char host_oled_rows[4][17];
unsigned long host_uart_tx_bytes;
int HAL_UART_Transmit(UART_HandleTypeDef *p, uint8_t *s, uint16_t n, uint32_t t)
{ (void)p; (void)s; (void)t; host_uart_tx_bytes += n; return 0; }
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *p, uint16_t a, uint16_t b, uint16_t c, uint8_t *d, uint16_t n, uint32_t t)
{ (void)p;(void)a;(void)b;(void)c;(void)d;(void)n;(void)t;assert(0);return -1; }
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *p, uint16_t a, uint16_t b, uint16_t c, uint8_t *d, uint16_t n, uint32_t t)
{ (void)p;(void)a;(void)b;(void)c;(void)d;(void)n;(void)t;assert(0);return -1; }
int HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *p, uint16_t a, uint32_t b, uint32_t c)
{ (void)p;(void)a;(void)b;(void)c;assert(0);return -1; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *p, uint32_t c)
{ (void)p;(void)c;assert(0);return -1; }
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *p, uint32_t c)
{ (void)p;(void)c;assert(0);return -1; }
void Error_Handler(void) { assert(0); }
void OLED_Init(void) {}
void OLED_Display_On(void) {}
void OLED_Clear(void) {}
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *s)
{
    (void)x;
    assert(Straight_State() != STRAIGHT_RUNNING && Turn_State() != TURN_RUNNING);
    assert(strlen((const char *)s) <= 16U);
    if (y / 16U < 4U) memcpy(host_oled_rows[y / 16U], s, strlen((const char *)s) + 1U);
}
void OLED_Refresh_Gram(void)
{ assert(Straight_State() != STRAIGHT_RUNNING && Turn_State() != TURN_RUNNING); }
