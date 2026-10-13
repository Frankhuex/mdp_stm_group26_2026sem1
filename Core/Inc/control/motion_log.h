#ifndef MOTION_LOG_H
#define MOTION_LOG_H
#include <stdint.h>
/* Foreground-only queue; UART output is drained by command polling. */
void MotionLog_Init(void);
void MotionLog_Write(const char *format, ...);
uint8_t MotionLog_Read(char *out, uint32_t size);
#endif
