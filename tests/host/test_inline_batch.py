"""Compile production batch/RX/executor functions against a simulated command runner."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
source = (root / 'Core/Src/assessment/command_interface.c').read_text()
def function(signature):
    start = source.index(signature)
    begin = source.index('{', start)
    depth = 1
    end = begin + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]
executor_signature = 'static const char *CommandInterface_RunSequenceRows('
# Skip the forward declaration.
executor_start = source.index(executor_signature, source.index('static int Sequence_Wait('))
executor = source[executor_start:]
source_without_declaration = source
source = executor
executor = function(executor_signature)
source = source_without_declaration
prefix = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#define CMD_LINE_SIZE 128U
#define CMD_RX_LINE_SIZE 512U
#define CMD_BATCH_MAX_COMMANDS 16U
#define CMD_ECHO_ENABLE 0
#define SEQUENCE_COMMAND_GAP_ENABLE 1
#define STRAIGHT_DONE 2
#define TURN_DONE 2
#define LATERAL_DONE 2
#define BYPASS_DONE 2
static char rx_line[CMD_RX_LINE_SIZE], inline_batch_line[CMD_RX_LINE_SIZE];
static uint32_t rx_line_len; static uint8_t rx_discard,previous_byte;
static uint8_t sequence_mode,sequence_cancelled,sequence_row_dispatch,sequence_log_active;
static uint8_t pending_motion_done_suppressed,command_output_suppressed,command_rejected;
static const char *sequence_row_failure,*sequence_log_result;
typedef enum {PENDING_MOTION_NONE,PENDING_MOTION_STRAIGHT,PENDING_MOTION_UNTILUS,PENDING_MOTION_TURN,PENDING_MOTION_LATERAL,PENDING_MOTION_BYPASS} pending_motion_t;
typedef void (*command_motion_observer_t)(uint32_t,const char*);
static pending_motion_t pending_motion;
static int moving,steps,executed,stop_at,fail_move; static uint32_t now;
static char output[12000], rows[32][128]; static const char *inject;
static void ProcessRxByte(uint8_t b);
static void DispatchReceivedLine(char *line);
static const char *CommandInterface_RunSequenceRows(const char *const[],uint32_t,uint32_t,uint32_t,uint8_t,uint8_t,uint8_t,uint32_t*,command_motion_observer_t);
static void Cmd_Send(const char*s){strcat(output,s);} static void Cmd_Reject(const char*s){command_rejected=1;Cmd_Send(s);Cmd_Send("\r\n");}
static uint8_t AnyMotionRunning(void){return moving!=0;}
static int Straight_State(void){return fail_move?3:2;} static int Turn_State(void){return 2;}
static int Lateral_State(void){return 2;} static int Bypass_State(void){return 2;}
static const char *StraightStateText(int s){return s==2?"DONE":"SENSOR_ERROR";}
static const char *TurnStateText(int s){(void)s;return "DONE";} static const char *LateralStateText(int s){(void)s;return "DONE";} static const char *BypassStateText(int s){(void)s;return "DONE";}
static uint32_t HAL_GetTick(void){return now;}
static void SequenceLog_Begin(void){sequence_log_active=1;}
static void SequenceLog_CaptureBefore(uint32_t i,const char*c){(void)i;(void)c;}
static void SequenceLog_CaptureAfter(uint32_t i,pending_motion_t m){(void)i;(void)m;}
static void NotifyPendingMotionDone(void){if(!moving && pending_motion){Cmd_Send(fail_move?"DONE,STRAIGHT,state=SENSOR_ERROR\r\n":"DONE,STRAIGHT,state=DONE\r\n");pending_motion=PENDING_MOTION_NONE;}}
static void Sequence_StopMotors(void){moving=0;NotifyPendingMotionDone();}
static int ParseIntStrict(const char*s,int32_t*n){char*end;long v=strtol(s,&end,10);if(!*s||*end)return 0;*n=v;return 1;}
static void DispatchLine(char*line){
 if(!strcmp(line,"STOP")){moving=0;Cmd_Send("OK\r\n");return;}
 if(!strcmp(line,"PING")||!strcmp(line,"STATUS")){Cmd_Send("PONG\r\n");return;}
 if(!strcmp(line,"BAD")){Cmd_Reject("ERR,UNKNOWN_CMD");return;}
 if(!strcmp(line,"FAST,APPROACH")){
  const char *nested[]={"STRAIGHT,UNTILUS,28"};uint32_t index;
  sequence_row_dispatch=0;sequence_row_failure=CommandInterface_RunSequenceRows(nested,1,0,0,0,0,0,&index,NULL);return;
 }
 assert(!moving);strcpy(rows[executed++],line);moving=1;steps=3;pending_motion=PENDING_MOTION_STRAIGHT;Cmd_Send("OK\r\n");
}
static void CommandInterface_Poll(void){
 now++; if(inject){const char*p=inject;inject=NULL;while(*p)ProcessRxByte(*p++);}
 if(moving && stop_at && now>=(uint32_t)stop_at){stop_at=0;const char*p="STOP\n";while(*p)ProcessRxByte(*p++);}
 if(moving && --steps==0)moving=0;
 NotifyPendingMotionDone();
}
'''
wait = function('static int Sequence_Wait(')
functions = '\n'.join(function(s) for s in ['static uint8_t SplitInlineCommands(', 'static void DispatchInlineBatch(', 'static void DispatchReceivedLine(', 'static void ProcessRxByte('])
main = r'''
static void reset(void){moving=steps=executed=stop_at=fail_move=0;now=rx_line_len=0;sequence_mode=sequence_cancelled=sequence_row_dispatch=sequence_log_active=0;pending_motion=0;output[0]=0;inject=NULL;rx_discard=previous_byte=0;}
static void send(const char*s){while(*s)ProcessRxByte(*s++);}
int main(void){
 reset();send("STRAIGHT,GOCM,10; STRAIGHT,UNTILUS,28\r\n");assert(executed==2&&!strcmp(rows[1],"STRAIGHT,UNTILUS,28"));assert(strstr(output,"DONE,BATCH,state=DONE,count=2"));
 reset();inject="PING\nSTRAIGHT,GOCM,99;PING\n";send("STRAIGHT,GOCM,10;STRAIGHT,GOCM,20\n");assert(executed==2&&!strcmp(rows[1],"STRAIGHT,GOCM,20"));assert(strstr(output,"PONG")&&strstr(output,"ERR,BUSY"));
 reset();stop_at=2;send("STRAIGHT,GOCM,10;STRAIGHT,GOCM,20\n");assert(executed==1&&strstr(output,"state=USER_STOP,index=1"));assert(!sequence_mode);
 reset();send("STRAIGHT,GOCM,10;BAD;STRAIGHT,GOCM,20\n");assert(executed==1&&strstr(output,"COMMAND_REJECTED,index=2"));
 reset();fail_move=1;send("STRAIGHT,GOCM,10;STRAIGHT,GOCM,20\n");assert(executed==1&&strstr(output,"SENSOR_ERROR,index=1"));
 const char*bad[]={";PING\n","PING;\n","PING;;PING\n","PING; \t ;PING\n"};
 for(unsigned i=0;i<4;i++){reset();send(bad[i]);assert(!executed&&strstr(output,"ERR,BAD_ARGS")&&!strstr(output,"OK,batch"));}
 reset();send("FAST,APPROACH;STRAIGHT,GOCM,20\n");assert(executed==2&&strstr(output,"DONE,BATCH,state=DONE"));
 reset();send("WAIT,3;STRAIGHT,GOCM,20\n");assert(executed==1&&now>=3);
 reset();send("STRAIGHT,GOCM,10\n");assert(executed==1&&!strstr(output,"batch"));
 reset();moving=1;send("PING;PING\n");assert(strstr(output,"ERR,BUSY"));
 reset();char longline[600];memset(longline,'A',512);longline[512]='\n';longline[513]=0;send(longline);assert(strstr(output,"RX_LINE_OVERFLOW")&&!executed);send("PING\n");assert(strstr(output,"PONG"));
 reset();char*cmds[16];const char*parts[16];uint32_t count;char buf[512];(void)cmds;
 buf[0]=0;for(int i=0;i<16;i++)strcat(buf,i?";PING":"PING");assert(SplitInlineCommands(buf,parts,&count)&&count==16);
 buf[0]=0;for(int i=0;i<17;i++)strcat(buf,i?";PING":"PING");assert(!SplitInlineCommands(buf,parts,&count));
 memset(buf,'A',128);buf[128]=';';strcpy(buf+129,"PING");assert(!SplitInlineCommands(buf,parts,&count));
 puts("PASS: ordered motion, nested FAST, WAIT, STOP, faults, BUSY, RX reentry, empty segments, limits and overflow recovery");
}
'''
with tempfile.TemporaryDirectory() as tmp:
    c = Path(tmp) / 'test.c'
    c.write_text(prefix + wait + executor + functions + main)
    binary = Path(tmp) / 'test'
    subprocess.run(['clang','-std=c11','-fsanitize=address,undefined',str(c),'-o',str(binary)], check=True)
    subprocess.run([str(binary)],check=True)
