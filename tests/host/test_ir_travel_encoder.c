/* Link with search_lateral_control.c whose fallback rows are numeric ENCODER
 * commands; see test_ir_travel_encoder.py for the isolated fixture runner. */
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
    for(int op=STRAIGHT_IR_GT;op<=STRAIGHT_IR_FALLING;op++) for(int gate=0;gate<2;gate++) {
        setup(1000);Straight_IRTravelMm=77;
        assert((gate?Straight_StartCmThenIR(10,1,op,50):Straight_StartUntilIRDirectionCm(50,1,op,-1))==0);
        assert(Straight_IRTravelMm==77);
        if(gate) {encoder_delta=800;poll();encoder_delta=0;}
        if(op==STRAIGHT_IR_RISING || op==STRAIGHT_IR_FALLING) {
            encoder_delta=74;reading(op==STRAIGHT_IR_RISING?300:550);
            reading(op==STRAIGHT_IR_RISING?310:540);
        }
        encoder_delta=74;reading(op==STRAIGHT_IR_GT || op==STRAIGHT_IR_RISING?550:300);
        reading(op==STRAIGHT_IR_GT || op==STRAIGHT_IR_RISING?540:310);encoder_delta=0;
        assert(Straight_State()==STRAIGHT_DONE && Straight_IRTravelMm>77);
        assert(Straight_IRTravelMm<400);
        if(gate) assert(Straight_IRTravelMm>100);
        uint32_t cache=Straight_IRTravelMm;
        assert(Lateral_StartEncoder(1)==0);lateral_status_t l;Lateral_Status(&l);
        assert(l.from_encoder && l.distance_cm_x10==(cache+1)/2+350);
        turn_state=TURN_DONE;Lateral_Step();turn_state=TURN_DONE;Lateral_Step();
        assert(Lateral_State()==LATERAL_DONE && Straight_IRTravelMm==cache);
    }
    setup(1000);Straight_IRTravelMm=123;
    assert(Straight_StartUntilIRCm(50,0,STRAIGHT_IR_LT)==0);Straight_Stop();assert(Straight_IRTravelMm==123);
    assert(Lateral_StartEncoder(0)==0);Lateral_Stop();assert(Straight_IRTravelMm==123);
    setup(1000);Straight_IRTravelMm=UINT32_MAX;assert(Lateral_StartEncoder(0)==-5);
    Straight_IRTravelMm=401;assert(Lateral_StartEncoder(0)==-2);
    for(uint8_t side=0;side<2;side++) {
    setup(1000);Straight_IRTravelMm=200;
    uint32_t before = tick;
    assert(SearchLateral_Start(side,50,50)==0);
    assert(tick-before >= SEARCH_SERVO_PRE_HOLD_MS+SEARCH_SERVO_CENTER_HOLD_MS);
    assert(mock_takeups==0);
    char log[256];int centre_log=0;
    while(MotionLog_Read(log,sizeof(log))) if(strstr(log,"SEARCHLATERAL,CENTER,")) {
        centre_log=1;assert(strstr(log,"pre_ms=100,center_ms=100"));
    }
    assert(centre_log);
    encoder_delta=16000;poll();encoder_delta=0;
    assert(status().fallback && SearchLateral_State()==SEARCH_RUNNING);
    lateral_status_t l;Lateral_Status(&l);assert(l.from_encoder && l.distance_cm_x10==450);
    turn_state=TURN_DONE;poll();turn_state=TURN_DONE;poll();assert(SearchLateral_State()==SEARCH_DONE);
    assert(Straight_IRTravelMm==200);
    }
    puts("PASS: IR travel all comparisons/gates, ENCODER geometry/invalid cache/range/STOP and fallback");
}
