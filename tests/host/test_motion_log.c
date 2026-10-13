#include "control/motion_log.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void) {
 char line[256];MotionLog_Init();assert(!MotionLog_Read(line,sizeof(line)));
 MotionLog_Write("CACHE,UNTILUS,kind=START,mode=AUTO,mm=%u",0U);
 MotionLog_Write("SEARCHLATERAL,LIMIT,sum_mm=%ld,limit_mm=%u,source=CACHE",1234L,1234U);
 MotionLog_Write("SEARCHLATERAL,FALLBACK,limit_mm=%u",1234U);
 assert(MotionLog_Read(line,sizeof(line))&&!strcmp(line,"LOG,CACHE,UNTILUS,kind=START,mode=AUTO,mm=0\r\n"));
 assert(MotionLog_Read(line,sizeof(line))&&strstr(line,"sum_mm=1234"));
 assert(MotionLog_Read(line,sizeof(line))&&strstr(line,"FALLBACK"));
 for(int i=0;i<17;i++)MotionLog_Write("TEST,%d",i);
 for(int i=0;i<16;i++){assert(MotionLog_Read(line,sizeof(line)));assert(!strncmp(line,"LOG,",4));}
 assert(MotionLog_Read(line,sizeof(line))&&!strcmp(line,"LOG,DROPPED,count=1\r\n"));
 assert(!MotionLog_Read(line,sizeof(line)));
 puts("PASS: motion log cache/limit/fallback format, FIFO and overflow reporting");
}
