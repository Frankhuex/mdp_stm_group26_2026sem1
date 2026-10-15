#define main original_lateral_tests
#include "test_lateral.c"
#undef main
#include <limits.h>
int main(void) {
    original_lateral_tests();
    for(uint8_t side=0;side<2;side++) for(int offset=-10;offset<=10;offset+=10) {
        reset();Straight_IRTravelMm=300;
        assert(Lateral_StartEncoderOffset(side,offset)==0);
        lateral_status_t s=status();
        assert(s.first_angle_offset_deg==offset && s.from_encoder);
        assert(angles[0]==s.angle_deg+offset);
        turn=TURN_DONE;Lateral_Step();
        assert(angles[1]==s.angle_deg && dirs[1]==-dirs[0]);
        turn=TURN_DONE;Lateral_Step();assert(Lateral_State()==LATERAL_DONE);
        reset();assert(Lateral_StartEncoder(side)==0);assert(status().first_angle_offset_deg==0);
    }
    int offsets[]={INT_MIN,INT_MAX,-360,360};
    for(unsigned i=0;i<sizeof(offsets)/sizeof(offsets[0]);i++) {
        reset();Straight_IRTravelMm=300;
        assert(Lateral_StartEncoderOffset(0,offsets[i])==-2 && calls==0);
    }
    reset();Straight_IRTravelMm=UINT32_MAX;assert(Lateral_StartEncoderOffset(1,5)==-5);
    puts("PASS: encoder first-angle signed offset, mirrored sides, unchanged arc 2, defaults and overflow bounds");
}
