#define main previous_regression_main
#include "test_ir_straight.c"
#undef main
int main(void)
{
    for(int dir=-1;dir<=1;dir+=2) for(int distance=20;distance<=60;distance+=40) {
        reset();assert(Straight_StartCm(dir*distance)==0);
        uint32_t previous=(last_left_pwm+last_right_pwm)/2U;
        int reductions=0, saw_final=0;
        for(int i=0;i<300 && Straight_State()==STRAIGHT_RUNNING;i++) {
            encoder_delta=50;step();
            uint32_t current=(last_left_pwm+last_right_pwm)/2U;
            if(current<previous) reductions++;
            /* Per 40ms control interval: at most 100 base PWM, plus rounding. */
            assert(current+102U>=previous);
            if(current<=GOCM_FINAL_PWM+20U) saw_final=1;
            previous=current;
        }
        encoder_delta=0;
        assert(Straight_State()==STRAIGHT_DONE && reductions>5 && saw_final);
        assert(mock_brake_end_tick-mock_brake_begin_tick>=MOTOR_BRAKE_PULSE_MS);
    }
    reset();assert(Straight_StartUntilUltrasonicCm(35)==0);echo(950,0);
    int reductions=0;uint32_t previous=(last_left_pwm+last_right_pwm)/2U;
    for(int i=0;i<300 && Straight_State()==STRAIGHT_RUNNING;i++) {
        encoder_delta=50;step();uint32_t current=(last_left_pwm+last_right_pwm)/2U;
        if(current<previous) reductions++;assert(current+102U>=previous);previous=current;
    }
    encoder_delta=0;assert(Straight_State()==STRAIGHT_DONE && reductions>5 && Straight_ForwardUSEndMm==350);
    puts("PASS: smooth deceleration, bounded PWM changes, final crawl, both gears/profiles, brake hold and UNTILUS reuse");
}
