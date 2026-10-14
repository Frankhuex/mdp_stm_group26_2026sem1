#define main straight_regression_main
#include "test_ir_straight.c"
#undef main
int main(void) {
    for(int mode=0;mode<4;mode++) {
        reset();Straight_AutoUSStartMm=123;Straight_AutoUSEndMm=456;
        Straight_ForwardUSStartMm=111;Straight_ForwardUSEndMm=222;
        Straight_BackwardUSStartMm=333;Straight_BackwardUSEndMm=444;
        assert(Straight_StartUntilUltrasonicAutoNoCacheCm(32)==0);
        assert(Straight_AutoUSStartMm==123 && Straight_AutoUSEndMm==456);
        echo(mode==0?500:mode==1?200:mode==2?320:0,mode==3?-1:0);
        if(mode!=2) finish_distance();
        assert(Straight_State()==STRAIGHT_DONE);
        assert(Straight_AutoUSStartMm==123 && Straight_AutoUSEndMm==456);
        assert(Straight_ForwardUSStartMm==111 && Straight_ForwardUSEndMm==222);
        assert(Straight_BackwardUSStartMm==333 && Straight_BackwardUSEndMm==444);
        char log[256];while(MotionLog_Read(log,sizeof(log))) assert(!strstr(log,"CACHE,UNTILUS"));
        assert(Straight_StartUntilUltrasonicAutoCm(32)==0);
        echo(400,0);finish_distance();
        assert(Straight_AutoUSStartMm==400 && Straight_AutoUSEndMm==320);
    }
    reset();Straight_AutoUSStartMm=123;Straight_AutoUSEndMm=456;
    assert(Straight_StartUntilUltrasonicAutoNoCacheCm(32)==0);echo(200,0);Straight_Stop();
    assert(Straight_AutoUSStartMm==123 && Straight_AutoUSEndMm==456);
    puts("PASS: AUTO NOCACHE forward/backward/equal/no-echo/STOP and subsequent normal caching");
}
