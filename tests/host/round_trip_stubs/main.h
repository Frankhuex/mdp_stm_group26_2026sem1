#ifndef ROUND_TRIP_HOST_MAIN_H
#define ROUND_TRIP_HOST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
#define HAL_MAX_DELAY UINT32_MAX
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
