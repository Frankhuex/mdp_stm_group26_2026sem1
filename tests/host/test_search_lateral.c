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
    Bypass_ForwardDistanceMm=limit; Straight_AutoUSStartMm=400; Straight_ForwardUSEndMm=1000;
    ir.left_status=ir.right_status=IR_RANGE_VALID;
}
static void poll(void) { step(); Lateral_Step(); SearchLateral_Step(); }
static search_lateral_status_t status(void) { search_lateral_status_t s;SearchLateral_Status(&s);return s; }
static void reading(uint32_t n) { ir.left_cm_x10=ir.right_cm_x10=n;poll(); }
static void reach_gate(void) { encoder_delta=3715; poll();encoder_delta=0;assert(status().stage==0); }
static void complete_back(void)
{
    encoder_delta=2400;poll();encoder_delta=0;
    for(int i=0;i<10&&status().stage==4;i++)poll();
    assert(status().stage==5);
}
int main(void)
{
    for(uint8_t side=0;side<2;side++) {
        setup(1000);assert(SearchLateral_Start(side,50,50)==0);
        reach_gate();reading(400);reading(350);reading(300);reading(510);reading(520);
        assert(status().stage==1 && status().ir_cm_x10==300 && turn_calls==1);
        lateral_status_t l;Lateral_Status(&l);assert(l.distance_cm_x10==400);
        turn_state=TURN_DONE;Lateral_Step();SearchLateral_Step();assert(turn_calls==2);
        assert(radii[0]==(side?35:20)&&radii[1]==(side?20:35));
        turn_state=TURN_DONE;Lateral_Step();SearchLateral_Step();assert(SearchLateral_State()==SEARCH_DONE);
        setup(600);assert(SearchLateral_Start(side,50,50)==0);reach_gate();reading(400);reading(350);
        /* Distance wins over a completing edge in the same poll. */
        reading(510);encoder_delta=745;reading(520);encoder_delta=0;
        assert(status().fallback && status().stage==2 && turn_calls==0);
        assert(status().limit_mm==600);
        reading(520);assert(status().stage==3 && turn_calls==1);
        assert(radii[0]==(side?35:20)&&angles[0]==90&&directions[0]==(side?1:-1));
        turn_state=TURN_DONE;SearchLateral_Step();assert(status().stage==4);
        int32_t target;Straight_Status(NULL,&target,NULL,NULL,NULL,NULL);assert(target==-30);
        complete_back();assert(Straight_IRComparison()==STRAIGHT_IR_LT);
        reading(400);reading(390);assert(status().stage==6 && turn_calls==1);
        Straight_Status(NULL,&target,NULL,NULL,NULL,NULL);assert(target==(side?-20:-35));
        encoder_delta=3000;poll();encoder_delta=0;
        for(int i=0;i<10&&status().stage==6;i++)poll();
        assert(status().stage==7 && turn_calls==2);
        assert(radii[1]==(side?20:35)&&directions[1]==(side?-1:1));
        turn_state=TURN_DONE;SearchLateral_Step();assert(SearchLateral_State()==SEARCH_DONE);
    }
    setup(1000);assert(SearchLateral_Start(1,50,50)==0);reach_gate();
    reading(470);reading(480);reading(510);reading(520);
    assert(SearchLateral_State()==SEARCH_START_ERROR && status().start_rc==-2 && turn_calls==0);
    setup(100);assert(SearchLateral_Start(1,50,50)==0);
    /* Bound applies before the 50cm gate and remains frozen if caches change. */
    Bypass_ForwardDistanceMm=1500;encoder_delta=745;poll();encoder_delta=0;
    assert(status().stage==2&&status().limit_mm==100&&ir_reads==1); /* fallback startup read only */
    SearchLateral_Stop();reading(520);assert(SearchLateral_State()==SEARCH_ABORTED&&turn_calls==0);
    setup(1000);assert(SearchLateral_Start(1,50,50)==0);adc_fail=1;reach_gate();poll();
    assert(SearchLateral_State()==SEARCH_STRAIGHT_ERROR&&turn_calls==0);
    for(int stage=0;stage<=7;stage++) {
        setup(600);assert(SearchLateral_Start(1,50,50)==0);
        if(stage>0) {
            if(stage==1) { reach_gate();reading(300);reading(310);reading(510);reading(520); }
            else {
                encoder_delta=4460;poll();encoder_delta=0;
                if(stage>=3) { reading(520);reading(530); }
                if(stage>=4) { turn_state=TURN_DONE;SearchLateral_Step(); }
                if(stage>=5) complete_back();
                if(stage>=6) { reading(300);reading(310); }
                if(stage>=7) { encoder_delta=3000;poll();encoder_delta=0;for(int i=0;i<10&&status().stage==6;i++)poll(); }
            }
        }
        assert(status().stage==stage);SearchLateral_Stop();int before=turn_calls;
        turn_state=TURN_DONE;Lateral_Step();SearchLateral_Step();assert(SearchLateral_State()==SEARCH_ABORTED&&turn_calls==before);
    }
    for(int missing=0;missing<4;missing++) {
        setup(1000);
        if(missing==0) Bypass_ForwardDistanceMm=UINT32_MAX;
        if(missing==1) Straight_AutoUSStartMm=UINT32_MAX;
        if(missing==2) Straight_ForwardUSEndMm=UINT32_MAX;
        if(missing==3) Straight_ForwardUSEndMm=2500;
        assert(SearchLateral_Start(0,50,50)==0 && drives>0);
        assert(status().limit_mm==1000 && SearchLateral_State()==SEARCH_RUNNING);
        /* Default is frozen and triggers fallback at 100cm total travel. */
        Bypass_ForwardDistanceMm=500; encoder_delta=7430;poll();encoder_delta=0;
        assert(status().stage==2 && status().fallback && status().limit_mm==1000);
        SearchLateral_Stop();
    }
    setup(2300);assert(SearchLateral_Start(1,50,50)==0 && status().limit_mm==2300);
    encoder_delta=15000;poll();encoder_delta=0;
    assert(SearchLateral_State()==SEARCH_RUNNING && status().stage==0);
    SearchLateral_Stop();
    setup(600);assert(SearchLateral_Start(1,50,50)==0);encoder_delta=4460;poll();encoder_delta=0;
    fail_turn=1;reading(510);reading(520);assert(SearchLateral_State()==SEARCH_START_ERROR);
    puts("PASS: SEARCHLATERAL both paths/sides, cache validation/freeze, full-distance bound, edge race, mirrors, STOP every stage and errors");
}
