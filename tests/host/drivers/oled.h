#ifndef HOST_OLED_H
#define HOST_OLED_H
#include <stdint.h>
#define u8 uint8_t
extern uint8_t OLED_GRAM[128][8];
void OLED_Init(void);
void OLED_Display_On(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *text);
void OLED_Refresh_Gram(void);
#endif
