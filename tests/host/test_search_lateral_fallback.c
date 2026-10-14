#define main straight_regression_main
#include "test_ir_straight.c"
#undef main
#include "control/search_lateral_control.h"
#include "control/lateral_control.h"
#include "control/bypass_control.h"
uint32_t Bypass_ForwardDistanceMm;
bypass_state_t Bypass_State(void) { return BYPASS_IDLE; }
static turn_state_t turn_state;
static int turn_calls, fail_turn;
static int32_t radii[4], angles[4];
static int8_t directions[4];
turn_state_t Turn_State(void) { return turn_state; }
void Turn_Stop(void) { if (turn_state==TURN_RUNNING) turn_state=TURN_ABORTED; }
int Turn_StartRadiusAngle(int32_t r,int32_t a,int8_t d)
{
    assert(turn_calls<4); radii[turn_calls]=r; angles[turn_calls]=a; directions[turn_calls]=d;
    turn_calls++; if(fail_turn) return -1; turn_state=TURN_RUNNING; return 0;
}
static void setup(uint32_t limit)
{
    reset(); Lateral_Init(); SearchLateral_Init(); turn_state=TURN_IDLE; turn_calls=fail_turn=0;
    Bypass_ForwardDistanceMm=limit; Straight_AutoUSStartMm=400; Straight_ForwardUSEndMm=240;
    ir.left_status=ir.right_status=IR_RANGE_VALID;
}
static void poll(void) { step(); Lateral_Step(); SearchLateral_Step(); }
static search_lateral_status_t status(void) { search_lateral_status_t s;SearchLateral_Status(&s);return s; }
static void reading(uint32_t n) { ir.left_cm_x10=ir.right_cm_x10=n;poll(); }
static void reach_gate(void) { encoder_delta=3715; poll();encoder_delta=0;assert(status().stage==0); }
static void trigger(void) {
    assert(SearchLateral_Start(1,50,50)==0);
    encoder_delta=16000;poll();encoder_delta=0;
    assert(status().fallback && status().stage==2 && SearchLateral_State()==SEARCH_RUNNING);
}
int main(void) {
    for(uint8_t side=0;side<2;side++) {
        setup(1000);assert(SearchLateral_Start(side,50,50)==0);
        encoder_delta=16000;poll();encoder_delta=0;
        assert(status().fallback && status().stage==2 && turn_calls==1);
        assert(radii[0]==(side?35:20) && angles[0]==35);
        SearchLateral_Step();assert(SearchLateral_State()==SEARCH_RUNNING);
        turn_state=TURN_DONE;poll();assert(turn_calls==2);
        assert(SearchLateral_State()==SEARCH_RUNNING && radii[1]==(side?20:35));
        turn_state=TURN_DONE;poll();assert(SearchLateral_State()==SEARCH_DONE);
    }
    setup(1000);fail_turn=1;
    assert(SearchLateral_Start(1,50,50)==0);
    encoder_delta=16000;poll();encoder_delta=0;
    assert(SearchLateral_State()==SEARCH_START_ERROR && status().start_rc==-1);
    for(int arc=1;arc<=2;arc++) {
        setup(1000);trigger();
        if(arc==2) { turn_state=TURN_DONE;poll();assert(turn_calls==2); }
        SearchLateral_Stop();assert(Lateral_State()==LATERAL_ABORTED && turn_state==TURN_ABORTED);
        poll();assert(SearchLateral_State()==SEARCH_ABORTED && turn_calls==arc);
    }
    setup(1000);trigger();turn_state=TURN_NO_IMU;poll();
    assert(SearchLateral_State()==SEARCH_LATERAL_ERROR);
    puts("PASS: fallback LATERAL both sides, both arcs, completion, startup fault, runtime fault and STOP");
}
