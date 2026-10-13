#include "control/motion_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define LOG_CAPACITY 16U
#define LOG_LINE_SIZE 256U
static char lines[LOG_CAPACITY][LOG_LINE_SIZE];
static uint32_t head, count, dropped;
void MotionLog_Init(void) { head = count = dropped = 0; }
void MotionLog_Write(const char *format, ...)
{
    if (!format) return;
    if (count == LOG_CAPACITY) { dropped++; return; }
    char *line = lines[(head + count) % LOG_CAPACITY];
    memcpy(line,"LOG,",4);
    va_list args; va_start(args,format);
    vsnprintf(line+4, LOG_LINE_SIZE-6, format,args);
    va_end(args);
    size_t length=strlen(line);memcpy(line+length,"\r\n",3);
    count++;
}
uint8_t MotionLog_Read(char *out, uint32_t size)
{
    if (!out || size < LOG_LINE_SIZE) return 0;
    if (count) {
        strcpy(out,lines[head]);head=(head+1)%LOG_CAPACITY;count--;return 1;
    }
    if (dropped) { snprintf(out,size,"LOG,DROPPED,count=%lu\r\n",(unsigned long)dropped);dropped=0;return 1; }
    return 0;
}
