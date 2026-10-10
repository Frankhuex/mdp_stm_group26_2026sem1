#define main straight_regression_main
#include "test_ir_straight.c"
#undef main
#include "control/lateral_control.h"
static turn_state_t turn_state;
static int turn_calls;
static int32_t turn_radii[2];
static int8_t turn_dirs[2];
turn_state_t Turn_State(void) { return turn_state; }
void Turn_Stop(void) { if(turn_state==TURN_RUNNING) turn_state=TURN_ABORTED; }
int Turn_StartRadiusAngle(int32_t radius,int32_t angle,int8_t dir)
{
    assert(turn_calls<2 && angle>=5 && angle<=90);
    turn_radii[turn_calls]=radius; turn_dirs[turn_calls++]=dir;
    turn_state=TURN_RUNNING; return 0;
}
static void setup(void)
{
    reset(); Lateral_Init(); turn_state=TURN_IDLE; turn_calls=0;
    ir.left_status=ir.right_status=IR_RANGE_VALID;
}
static void read_ir(uint32_t value)
{ ir.left_cm_x10=ir.right_cm_x10=value; step(); Lateral_Step(); }
static void gate(void)
{ encoder_delta=744; step(); encoder_delta=0; Lateral_Step(); assert(ir_reads==0 && turn_calls==0); }
int main(void)
{
    for(int dir=-1;dir<=1;dir+=2)
        for(uint8_t side=0;side<2;side++)
            for(int op=STRAIGHT_IR_RISING;op<=STRAIGHT_IR_FALLING;op++) {
                setup();
                assert(Lateral_StartAfterIREdge(dir*10,side,op,40)==0); gate();
                uint32_t first=op==STRAIGHT_IR_RISING?300:430;
                uint32_t finish=op==STRAIGHT_IR_RISING?410:390;
                read_ir(first); assert(turn_calls==0);
                read_ir(first+10); assert(turn_calls==0);
                uint32_t cached; assert(Straight_IRFirstEdgeReading(&cached) && cached==first);
                read_ir(finish); assert(turn_calls==0);
                read_ir(finish); assert(turn_calls==1 && Straight_State()==STRAIGHT_DONE);
                lateral_status_t st; Lateral_Status(&st);
                assert(st.initial_ir_cm_x10==first && st.distance_cm_x10==first+100);
                assert(st.from_edge && st.stage==1);
                assert(turn_radii[0]==(side?35:20) && turn_dirs[0]==(side?1:-1));
                turn_state=TURN_DONE; Lateral_Step(); assert(turn_calls==2);
                turn_state=TURN_DONE; Lateral_Step(); assert(Lateral_State()==LATERAL_DONE);
            }
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_RISING,40)==0);
    gate(); read_ir(200); read_ir(410); /* first below-threshold sample starts the minimum cache */
    read_ir(300); read_ir(310); read_ir(410); read_ir(420);
    lateral_status_t st; Lateral_Status(&st); assert(st.initial_ir_cm_x10==200);
    /* Minimum updates before arming, after arming, and between exit confirmations. */
    setup(); assert(Lateral_StartAfterIREdge(10,1,STRAIGHT_IR_RISING,50)==0);
    gate(); read_ir(480); read_ir(420); read_ir(350); read_ir(510);
    read_ir(280); read_ir(520); read_ir(530);
    Lateral_Status(&st); assert(st.initial_ir_cm_x10==280 && st.distance_cm_x10==380);
    assert(turn_calls==1 && st.stage==1);
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_RISING,40)==0);
    Lateral_Stop(); Lateral_Step(); assert(turn_calls==0 && Lateral_State()==LATERAL_ABORTED);
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_FALLING,40)==0);
    gate(); read_ir(500); read_ir(510); read_ir(390); read_ir(380);
    assert(turn_calls==0 && Lateral_State()==LATERAL_START_ERROR);
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_RISING,40)==0);
    gate(); adc_fail=1; read_ir(300);
    assert(turn_calls==0 && Lateral_State()==LATERAL_STRAIGHT_ERROR);
    /* STOP in the stopped handoff must cancel the first turn. */
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_RISING,40)==0);
    gate(); read_ir(300); read_ir(310); read_ir(410);
    ir.left_cm_x10=420; step(); assert(Straight_State()==STRAIGHT_DONE);
    Lateral_Stop(); Lateral_Step(); assert(turn_calls==0 && Lateral_State()==LATERAL_ABORTED);
    /* STOP after the first arc must cancel the second. */
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_RISING,40)==0);
    gate(); read_ir(300); read_ir(310); read_ir(410); read_ir(420);
    assert(turn_calls==1); turn_state=TURN_DONE;
    Lateral_Stop(); Lateral_Step(); assert(turn_calls==1 && Lateral_State()==LATERAL_ABORTED);
    /* FAR cannot supply a finite lateral displacement. */
    setup(); assert(Lateral_StartAfterIREdge(10,0,STRAIGHT_IR_FALLING,40)==0);
    gate(); ir.left_status=IR_RANGE_FAR;
    read_ir(0); read_ir(0); ir.left_status=IR_RANGE_VALID;
    read_ir(390); read_ir(380);
    assert(turn_calls==0 && Lateral_State()==LATERAL_START_ERROR);
    Lateral_Status(&st); assert(st.start_rc==-5);
    puts("PASS: GOCM+edge+LATERAL all directions/sides/edges, RISING minimum / FALLING snapshot +10cm, debounce, STOP, faults and range");
    return 0;
}
