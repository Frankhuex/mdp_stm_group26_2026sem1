#include "control/motion_log.h"
#include "control/search_lateral_control.h"
#include "control/bypass_control.h"
#include "control/lateral_control.h"
#include "control/straight_tuning.h"
#include "control/turn_tuning.h"
#include <stdio.h>
#include <string.h>
static search_lateral_status_t sl;

#define SEARCH_LATERAL_FALLBACK_OFFSET_MM -160
/* Edit these actual command rows to change the fallback route.
 * RIGHT is used by FAST LEFT2; LEFT is used by FAST RIGHT2. */
static const char *const search_right_fallback_commands[] = {
    // "TURN,RIGHT,R=35,A=90",
    // "STRAIGHT,GOCM,-20",
    // "TURN,LEFT,R=20,A=90",
    "LATERAL,RIGHT,ENCODER",
};
static const char *const search_left_fallback_commands[] = {
    // "TURN,LEFT,R=20,A=90",
    // "STRAIGHT,GOCM,-20",
    // "TURN,RIGHT,R=35,A=90",
    "LATERAL,LEFT,ENCODER",
};
typedef enum { FALLBACK_STRAIGHT, FALLBACK_TURN, FALLBACK_LATERAL } fallback_child_t;
static fallback_child_t fallback_child;
static uint32_t FallbackCount(void)
{
    return sl.side ? sizeof(search_right_fallback_commands)/sizeof(search_right_fallback_commands[0])
                   : sizeof(search_left_fallback_commands)/sizeof(search_left_fallback_commands[0]);
}

void SearchLateral_Init(void) { search_lateral_status_t empty = {0}; sl = empty; fallback_child = FALLBACK_STRAIGHT; }
search_lateral_state_t SearchLateral_State(void) { return sl.state; }
void SearchLateral_Status(search_lateral_status_t *out) { if (out) *out = sl; }
/* newlib-nano may omit long-long printf support; retain exact signed sums. */
static void FormatSearchSum(char *out, int64_t value)
{
    char digits[24]; uint32_t n=0, pos=0;
    uint64_t magnitude=value<0 ? (uint64_t)(-value) : (uint64_t)value;
    do { digits[n++]=(char)('0'+magnitude%10U);magnitude/=10U; } while(magnitude);
    if(value<0) out[pos++]='-';
    while(n) out[pos++]=digits[--n];
    out[pos]='\0';
}
int SearchLateral_Start(uint8_t side, int32_t gate_cm, int32_t threshold_cm)
{
    if (sl.state == SEARCH_RUNNING || Straight_State() == STRAIGHT_RUNNING ||
        Turn_State() == TURN_RUNNING || Lateral_State() == LATERAL_RUNNING ||
        Bypass_State() == BYPASS_RUNNING) return -3;
    if (side > 1U || gate_cm < MIN_COMMAND_DISTANCE_CM || gate_cm > MAX_COMMAND_DISTANCE_CM ||
        threshold_cm < 10 || threshold_cm > (side ? 60 : 50)) return -2;
    int64_t limit = SEARCH_LATERAL_DEFAULT_LIMIT_MM;
    const int32_t offset_mm = SEARCH_LATERAL_FALLBACK_OFFSET_MM; /* Search-distance allowance, mm. */
    // int64_t sum = (int64_t)Bypass_ForwardDistanceMm + Straight_AutoUSStartMm - Straight_ForwardUSEndMm + offset_mm;
    int64_t sum = (int64_t)Bypass_ForwardDistanceMm + Straight_AutoUSStartMm + offset_mm;
    uint8_t caches_valid = Bypass_ForwardDistanceMm != UINT32_MAX && Straight_AutoUSStartMm != UINT32_MAX && Straight_ForwardUSEndMm != UINT32_MAX;
    uint8_t use_cache = caches_valid && sum > 0 && sum <= UINT32_MAX;
    if (use_cache) limit = sum;
    char bypass[16], initial[16], end[16], total[32];
    if (Bypass_ForwardDistanceMm == UINT32_MAX) strcpy(bypass,"INVALID");
    else snprintf(bypass,sizeof(bypass),"%lu",(unsigned long)Bypass_ForwardDistanceMm);
    if (Straight_AutoUSStartMm == UINT32_MAX) strcpy(initial,"INVALID");
    else snprintf(initial,sizeof(initial),"%lu",(unsigned long)Straight_AutoUSStartMm);
    if (Straight_ForwardUSEndMm == UINT32_MAX) strcpy(end,"INVALID");
    else snprintf(end,sizeof(end),"%lu",(unsigned long)Straight_ForwardUSEndMm);
    if (!caches_valid) strcpy(total,"INVALID");
    else FormatSearchSum(total,sum);
    MotionLog_Write("SEARCHLATERAL,LIMIT,dir=%s,bypass_mm=%s,auto_start_mm=%s,forward_end_mm=%s,offset_mm=%ld,sum_mm=%s,limit_mm=%lu,source=%s",
        side ? "RIGHT" : "LEFT",bypass,initial,end,(long)offset_mm,total,(unsigned long)limit,use_cache ? "CACHE" : "DEFAULT");
    int rc = Straight_StartCmThenIRBounded(gate_cm, side, STRAIGHT_IR_RISING, threshold_cm, (uint32_t)limit);
    if (rc) return rc;
    SearchLateral_Init(); sl.state = SEARCH_RUNNING; sl.side = side;
    sl.gate_cm = gate_cm; sl.threshold_cm = threshold_cm; sl.limit_mm = (uint32_t)limit;
    return 0;
}
/* Parse supported motion rows and use the same production controllers.
 * Child type and final step come from the array, not fixed stage numbers. */
static int StartFallback(void)
{
    uint32_t index = sl.stage - 2U;
    if (sl.stage < 2U || index >= FallbackCount()) return -2;
    const char *command = sl.side ? search_right_fallback_commands[index] : search_left_fallback_commands[index];
    char side[16], op[16]; int value, angle, end = 0;
    if (sscanf(command,"STRAIGHT,GOCM,%d%n",&value,&end)==1 && end && !command[end]) {
        fallback_child = FALLBACK_STRAIGHT;
        return Straight_StartCm(value);
    }
    end = 0;
    if (sscanf(command,"LATERAL,%15[^,],ENCODER%n",side,&end)==1 && end && !command[end]) {
        if (strcmp(side,"LEFT") && strcmp(side,"RIGHT")) return -2;
        fallback_child = FALLBACK_LATERAL;
        return Lateral_StartEncoder(!strcmp(side,"RIGHT"));
    }
    end = 0;
    if (sscanf(command,"LATERAL,%15[^,],%d%n",side,&value,&end)==2 && end && !command[end]) {
        if ((strcmp(side,"LEFT") && strcmp(side,"RIGHT")) || value <= 0 ||
            value > CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM) return -2;
        fallback_child = FALLBACK_LATERAL;
        return Lateral_Start(!strcmp(side,"RIGHT"), 0U, (uint32_t)value * 10U);
    }
    end = 0;
    if (sscanf(command,"TURN,%15[^,],R=%d,A=%d%n",side,&value,&angle,&end)==3 && end && !command[end]) {
        if (strcmp(side,"LEFT") && strcmp(side,"RIGHT")) return -2;
        fallback_child = FALLBACK_TURN;
        return Turn_StartRadiusAngle(value, angle, !strcmp(side,"RIGHT") ? 1 : -1);
    }
    end = 0;
    if (sscanf(command,"STRAIGHT,%15[^,],%15[^,],%d%n",side,op,&value,&end)==3 && end && !command[end]) {
        if (strcmp(side,"LEFTIR") && strcmp(side,"RIGHTIR")) return -2;
        uint8_t right = !strcmp(side,"RIGHTIR");
        fallback_child = FALLBACK_STRAIGHT;
        if (!strcmp(op,"GT")) return Straight_StartUntilIRGreaterValidCm(value,right);
        if (!strcmp(op,"LT")) return Straight_StartUntilIRCm(value,right,STRAIGHT_IR_LT);
        if (!strcmp(op,"RISING")) return Straight_StartUntilIRCm(value,right,STRAIGHT_IR_RISING);
        if (!strcmp(op,"FALLING")) return Straight_StartUntilIRCm(value,right,STRAIGHT_IR_FALLING);
    }
    return -2;
}
void SearchLateral_Step(void)
{
    if (sl.state != SEARCH_RUNNING) return;
    if (sl.stage == 0) {
        straight_state_t state = Straight_State();
        if (state == STRAIGHT_RUNNING) return;
        if (state == STRAIGHT_SEARCH_LIMIT) {
            float left, right; Straight_Status(0,0,&left,&right,0,0);
            MotionLog_Write("SEARCHLATERAL,FALLBACK,dir=%s,limit_mm=%lu,left_mm=%lu,right_mm=%lu",
                sl.side ? "RIGHT" : "LEFT",(unsigned long)sl.limit_mm,
                (unsigned long)(left*10.0f+0.5f),(unsigned long)(right*10.0f+0.5f));
            sl.fallback = 1; sl.stage = 2; sl.start_rc = StartFallback();
        } else if (state == STRAIGHT_DONE) {
            if (!Straight_IRFirstEdgeReading(&sl.ir_cm_x10)) sl.start_rc = -5;
            else {
                sl.stage = 1;
                sl.start_rc = Lateral_Start(sl.side, 0, sl.ir_cm_x10 +
                    LATERAL_OBSTACLE_WIDTH_CM * 5U + LATERAL_IR_TO_CENTER_CM * 10U);
            }
        } else { sl.state = state == STRAIGHT_ABORTED ? SEARCH_ABORTED : SEARCH_STRAIGHT_ERROR; return; }
        if (sl.start_rc) sl.state = SEARCH_START_ERROR;
        return;
    }
    if (sl.stage == 1) {
        lateral_state_t state = Lateral_State();
        if (state == LATERAL_RUNNING) return;
        sl.state = state == LATERAL_DONE ? SEARCH_DONE : state == LATERAL_ABORTED ? SEARCH_ABORTED : SEARCH_LATERAL_ERROR;
        return;
    }
    if (fallback_child == FALLBACK_LATERAL) {
        lateral_state_t state = Lateral_State();
        if (state == LATERAL_RUNNING) return;
        if (state != LATERAL_DONE) { sl.state = state == LATERAL_ABORTED ? SEARCH_ABORTED : SEARCH_LATERAL_ERROR; return; }
    } else if (fallback_child == FALLBACK_TURN) {
        turn_state_t state = Turn_State();
        if (state == TURN_RUNNING) return;
        if (state != TURN_DONE) { sl.state = state == TURN_ABORTED ? SEARCH_ABORTED : SEARCH_TURN_ERROR; return; }
    } else {
        straight_state_t state = Straight_State();
        if (state == STRAIGHT_RUNNING) return;
        if (state != STRAIGHT_DONE) { sl.state = state == STRAIGHT_ABORTED ? SEARCH_ABORTED : SEARCH_STRAIGHT_ERROR; return; }
    }
    if ((uint32_t)sl.stage - 2U + 1U >= FallbackCount()) { sl.state = SEARCH_DONE; return; }
    sl.stage++; sl.start_rc = StartFallback();
    if (sl.start_rc) sl.state = SEARCH_START_ERROR;
}
void SearchLateral_Stop(void)
{
    if (sl.state != SEARCH_RUNNING) return;
    sl.state = SEARCH_ABORTED; /* Cancel before a completed child can launch another. */
    if (sl.stage == 1 || (sl.stage >= 2 && fallback_child == FALLBACK_LATERAL)) Lateral_Stop();
    else if (sl.stage >= 2 && fallback_child == FALLBACK_TURN) Turn_Stop();
    else Straight_Stop();
}
