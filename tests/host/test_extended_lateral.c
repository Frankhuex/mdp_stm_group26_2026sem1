#define main original_lateral_main
#include "test_lateral.c"
#undef main
int main(void) {
    original_lateral_main();
    for(uint8_t side=0;side<2;side++) for(unsigned i=0;i<4;i++) {
        uint32_t targets[]={550,551,836,2650};reset();uint32_t mm=targets[i];
        assert(Lateral_Start(side,0,mm)==0);
        assert(angles[0]==90 && status().middle_mm==mm-550);
        turn=TURN_DONE;Lateral_Step();
        if(mm>550) {
            assert(status().stage==3 && straight==STRAIGHT_RUNNING && calls==1);
            assert(middle_distance_mm==mm-550);
            Lateral_Step();assert(calls==1);
            straight=STRAIGHT_DONE;Lateral_Step();
        }
        assert(calls==2 && angles[1]==90 && dirs[1]==-dirs[0]);
        turn=TURN_DONE;Lateral_Step();assert(Lateral_State()==LATERAL_DONE);
    }
    reset();Straight_IRTravelMm=1072;assert(Lateral_StartEncoderOffset(1,5)==0);
    assert(angles[0]==95 && status().middle_mm==286);
    turn=TURN_DONE;Lateral_Step();straight=STRAIGHT_DONE;Lateral_Step();assert(angles[1]==90);
    for(int phase=1;phase<=3;phase++) {
        reset();assert(Lateral_Start(1,0,836)==0);
        if(phase>=2) {turn=TURN_DONE;Lateral_Step();}
        if(phase==3) {straight=STRAIGHT_DONE;Lateral_Step();}
        Lateral_Stop();Lateral_Step();assert(Lateral_State()==LATERAL_ABORTED);
        if(phase==2) assert(straight==STRAIGHT_ABORTED);else assert(turn==TURN_ABORTED);
    }
    reset();assert(Lateral_Start(0,0,836)==0);straight_rc=-4;turn=TURN_DONE;Lateral_Step();
    assert(Lateral_State()==LATERAL_START_ERROR && status().start_rc==-4 && calls==1);
    reset();assert(Lateral_Start(0,0,836)==0);turn=TURN_DONE;Lateral_Step();straight=STRAIGHT_TIMEOUT;Lateral_Step();
    assert(Lateral_State()==LATERAL_STRAIGHT_ERROR && calls==1);
    puts("PASS: extended lateral boundaries, 83.6cm geometry, signed first arc, middle precision, faults and STOP");
}
