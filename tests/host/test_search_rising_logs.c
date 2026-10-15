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
int main(void) {
    for(uint8_t side=0;side<2;side++) {
        setup(3000);assert(SearchLateral_Start(side,50,50)==0);
        reach_gate();reading(400);reading(350);reading(300);reading(510);reading(520);
        assert(status().stage==1 && status().ir_cm_x10==300);
        turn_state=TURN_DONE;poll();turn_state=TURN_DONE;poll();
        assert(SearchLateral_State()==SEARCH_DONE);
        char log[256];int minimum=0,armed=0,edge=0,calc=0,start=0,end=0;
        while(MotionLog_Read(log,sizeof(log))) {
            assert(!strstr(log,"DROPPED"));
            if(strstr(log,"IR,RISING_MIN,") && strstr(log,"min_mm=300")) minimum=1;
            if(strstr(log,"IR,EDGE_ARMED,")) armed=1;
            if(strstr(log,"IR,EDGE_CONFIRMED,") && strstr(log,"first_mm=300")) edge=1;
            if(strstr(log,"SEARCHLATERAL,EDGE,") && strstr(log,"ir_min_mm=300,offset_mm=100,target_mm=400")) calc=1;
            if(strstr(log,"SEARCHLATERAL,LATERAL_START,") && strstr(log,"start_rc=0,angle_valid=1")) start=1;
            if(strstr(log,"SEARCHLATERAL,LATERAL_END,") && strstr(log,"result=DONE")) end=1;
        }
        assert(minimum && armed && edge && calc && start && end);
    }
    puts("PASS: live RISING minimum, arm/edge, SEARCHLATERAL formula/start/end logs, both sides");
}
