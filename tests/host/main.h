#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_MAX_DELAY UINT32_MAX
#define I2C_MEMADD_SIZE_8BIT 1
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_2 2
#define TIM_CHANNEL_3 3
#define TIM_CHANNEL_4 4
#define TIM_CHANNEL_ALL 15
#define __HAL_TIM_SET_COUNTER(t,n) ((void)0)
#define __HAL_TIM_GET_COUNTER(t) 0
int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
int HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
int HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
void Error_Handler(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
