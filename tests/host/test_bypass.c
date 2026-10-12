#include <assert.h>
#include <stdio.h>
#include "control/bypass_control.h"
#include "control/straight_control.h"
#include "control/lateral_control.h"
static turn_state_t turn;
static straight_state_t straight;
static lateral_state_t lateral;
static int calls, fail_call;
static int32_t radii[3], angles[3];
static int8_t dirs[3];
turn_state_t Turn_State(void) { return turn; }
straight_state_t Straight_State(void) { return straight; }
lateral_state_t Lateral_State(void) { return lateral; }
void Turn_Stop(void) { if (turn == TURN_RUNNING) turn = TURN_ABORTED; }
int Turn_StartRadiusAngle(int32_t r, int32_t a, int8_t d)
{
    assert(calls < 3); radii[calls]=r; angles[calls]=a; dirs[calls]=d; calls++;
    if (calls == fail_call) return -1;
    turn=TURN_RUNNING; return 0;
}
static void reset(void)
{
    Bypass_Init(); turn=TURN_IDLE; straight=STRAIGHT_IDLE; lateral=LATERAL_IDLE;
    calls=fail_call=0;
}
int main(void)
{
    for (int side=0; side<2; side++) {
        reset(); assert(Bypass_Start(side,20)==0);
        assert(Bypass_ForwardDistanceMm==843U);
        Bypass_Step(); assert(calls==1);
        turn=TURN_DONE; Bypass_Step(); turn=TURN_DONE; Bypass_Step();
        for (int i=0;i<3;i++) {
            int right=(i==1)?!side:side;
            assert(radii[i]==(right?35:20)); assert(dirs[i]==(right?1:-1));
            assert(angles[i]==(i==1?100:50));
        }
        assert(dirs[0]*angles[0]+dirs[1]*angles[1]+dirs[2]*angles[2]==0);
        turn=TURN_DONE; Bypass_Step(); assert(Bypass_State()==BYPASS_DONE);
        Bypass_Step(); assert(calls==3);
    }
    reset(); assert(Bypass_Start(0,55)==0); assert(angles[0]==90);
    turn=TURN_DONE; Bypass_Step(); assert(angles[1]==180);
    reset(); assert(Bypass_Start(1,1)==0);
    for (int n=-1;n<=0;n++) { reset(); assert(Bypass_Start(0,n)==-2); }
    reset(); assert(Bypass_Start(0,56)==-2); assert(Bypass_Start(2,20)==-2);
    reset(); straight=STRAIGHT_RUNNING; assert(Bypass_Start(0,20)==-3);
    reset(); lateral=LATERAL_RUNNING; assert(Bypass_Start(0,20)==-3);
    reset(); turn=TURN_RUNNING; assert(Bypass_Start(0,20)==-3);
    for (int stage=1;stage<=3;stage++) {
        reset(); assert(Bypass_Start(0,20)==0);
        for(int i=1;i<stage;i++) { turn=TURN_DONE; Bypass_Step(); }
        assert(Bypass_Start(1,20)==-3);
        Bypass_Stop(); assert(Bypass_State()==BYPASS_ABORTED);
        turn=TURN_DONE; Bypass_Step(); assert(calls==stage);
        reset(); fail_call=stage;
        int rc=Bypass_Start(0,20);
        if(stage==1) assert(rc==-1); else assert(rc==0);
        for(int i=1;i<stage;i++) { turn=TURN_DONE; Bypass_Step(); }
        assert(Bypass_State()==BYPASS_START_ERROR); Bypass_Step(); assert(calls==stage);
        reset(); assert(Bypass_Start(0,20)==0);
        for(int i=1;i<stage;i++) { turn=TURN_DONE; Bypass_Step(); }
        turn=TURN_TIMEOUT; Bypass_Step(); assert(Bypass_State()==BYPASS_TURN_ERROR);
        Bypass_Step(); assert(calls==stage);
    }
    reset(); assert(Bypass_Start(0,20)==0); turn=TURN_DONE;
    Bypass_Stop(); Bypass_Step(); assert(calls==1);
    puts("BYPASS tests passed"); return 0;
}
