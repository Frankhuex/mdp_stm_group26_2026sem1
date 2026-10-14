#include "control/bypass_control.h"
uint32_t Bypass_ForwardDistanceMm = UINT32_MAX;
uint32_t Straight_IRTravelMm = UINT32_MAX;
void Straight_SetIRTravelCacheEnabled(uint8_t enabled) { (void)enabled; }
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control/lateral_control.h"
#include "control/straight_control.h"
#include "drivers/robot_tuning.h"
#include "sensors/ir_sensor.h"

static IRSensorDistanceReading ir;
static int ir_reads, ir_fail;
int IRSensor_ReadDistance(IRSensorDistanceReading *out, uint32_t samples)
{
    assert(samples == 1U);
    ir_reads++;
    *out = ir;
    return ir_fail ? -1 : 0;
}
int Straight_StartUntilIRDirectionCm(int32_t cm,uint8_t side,straight_ir_comparison_t op,int8_t dir) { (void)cm;(void)side;(void)op;(void)dir;return -2; }
uint8_t Straight_IRFirstEdgeReading(uint32_t *out) { (void)out; return 0; }
int Straight_StartCmThenIR(int32_t cm, uint8_t side, straight_ir_comparison_t op, int32_t threshold)
{ (void)cm; (void)side; (void)op; (void)threshold; return -2; }
static turn_state_t turn;
static straight_state_t straight;
static int calls, fail_call, fail_rc;
static int straight_calls, straight_rc;
static uint8_t approach_side;
int Straight_StartUntilIRGreaterValidCm(int32_t threshold, uint8_t side)
{
    assert(threshold == 50); approach_side = side; straight_calls++;
    if (straight_rc) return straight_rc;
    straight = STRAIGHT_RUNNING; return 0;
}
void Straight_Stop(void) { if (straight == STRAIGHT_RUNNING) straight = STRAIGHT_ABORTED; }
static int32_t radii[2], angles[2];
static int8_t dirs[2];
turn_state_t Turn_State(void) { return turn; }
straight_state_t Straight_State(void) { return straight; }
void Turn_Stop(void) { if (turn == TURN_RUNNING) turn = TURN_ABORTED; }
int Turn_StartRadiusAngle(int32_t r, int32_t a, int8_t d)
{
    assert(calls < 2);
    radii[calls] = r; angles[calls] = a; dirs[calls] = d;
    calls++;
    if (calls == fail_call) return fail_rc;
    turn = TURN_RUNNING;
    return 0;
}
static void reset(void)
{
    Lateral_Init(); turn = TURN_IDLE; straight = STRAIGHT_IDLE;
    calls = fail_call = 0; fail_rc = -1;
    straight_calls = straight_rc = 0;
    IRSensorDistanceReading empty = {0};
    ir = empty;
    ir.left_status = ir.right_status = IR_RANGE_ERROR;
    ir_reads = ir_fail = 0;
}
static lateral_status_t status(void)
{ lateral_status_t st; Lateral_Status(&st); return st; }
int main(void)
{
    for (uint8_t side = 0; side < 2; side++) {
        reset();
        Bypass_ForwardDistanceMm = UINT32_MAX;
        assert(Lateral_Start(side, 0, 270) == 0);
        assert(Bypass_ForwardDistanceMm == 943U);
        reset();
        fail_call = 1;
        assert(Lateral_Start(side, 0, 270) != 0);
        assert(Bypass_ForwardDistanceMm == 943U);
    }
    for (uint8_t side = 0; side < 2; side++)
    {
        reset();
        assert(Lateral_Start(side, 0, 300) == 0);
        assert(calls == 1 && angles[0] == 63 && dirs[0] == (side ? 1 : -1));
        assert(radii[0] == (side ? CMD_RIGHT_MIN_RADIUS_CM : CMD_LEFT_SOFT_MIN_RADIUS_CM));
        Lateral_Step(); assert(calls == 1 && Lateral_State() == LATERAL_RUNNING);
        assert(Lateral_Start(!side, 0, 200) == -3);
        turn = TURN_DONE; Lateral_Step();
        assert(calls == 2 && Lateral_State() == LATERAL_RUNNING && status().stage == 2);
        assert(dirs[1] == -dirs[0] && angles[1] == angles[0]);
        assert(radii[1] == (side ? CMD_LEFT_SOFT_MIN_RADIUS_CM : CMD_RIGHT_MIN_RADIUS_CM));
        turn = TURN_DONE; Lateral_Step();
        assert(Lateral_State() == LATERAL_DONE); Lateral_Step(); assert(calls == 2);
        float actual = (CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM) *
                       (1.0f - cosf(angles[0] * 3.14159265358979323846f / 180.0f));
        assert(fabsf(actual - 30.0f) < 0.5f);
        assert(ir_reads == 0); /* numeric mode never reads IR */
        reset(); ir.left_cm_x10 = 300; ir.right_cm_x10 = 200;
        ir.left_status = ir.right_status = IR_RANGE_VALID;
        assert(Lateral_Start(side, 1, 0) == 0);
        assert(status().distance_cm_x10 == (side ? 300U : 400U));
        assert(status().angle_deg == (side ? 63 : 74));
        assert(ir_reads == 1 && straight_calls == 1 && approach_side == side);
        assert(calls == 0 && status().stage == 0 && status().initial_ir_cm_x10 == (side ? 200U : 300U));
        Lateral_Step(); assert(calls == 0);
        ir.left_cm_x10 = ir.right_cm_x10 = 100;
        assert(status().distance_cm_x10 == (side ? 300U : 400U));
        straight = STRAIGHT_DONE; Lateral_Step();
        assert(calls == 1 && status().stage == 1);
        assert(status().distance_cm_x10 == (side ? 300U : 400U));
        turn = TURN_DONE; Lateral_Step();
        assert(ir_reads == 1); /* second arc uses the command-start measurement */
        Lateral_Stop(); assert(Lateral_State() == LATERAL_ABORTED);
        Lateral_Step(); assert(calls == 2 && turn == TURN_ABORTED);
    }
    reset(); assert(Lateral_Start(0, 1, 0) == -5 && calls == 0);
    ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 451;
    assert(Lateral_Start(0, 1, 0) == -2 && calls == 0);
    ir.left_cm_x10 = 450;
    assert(Lateral_Start(0, 1, 0) == 0 && status().angle_deg == 90);
    for (uint8_t side = 0; side < 2; side++)
    {
        for (int range = IR_RANGE_NEAR; range <= IR_RANGE_ERROR; range++)
        {
            reset(); ir.left_status = ir.right_status = IR_RANGE_VALID;
            ir.left_cm_x10 = ir.right_cm_x10 = 300;
            if (side) ir.right_status = range; else ir.left_status = range;
            assert(Lateral_Start(side, 1, 0) == -5 && calls == 0);
        }
        reset(); ir_fail = 1;
        assert(Lateral_Start(side, 1, 0) == -5 && calls == 0);
        reset(); ir.left_status = ir.right_status = IR_RANGE_ERROR;
        if (side) { ir.right_status = IR_RANGE_VALID; ir.right_cm_x10 = 300; }
        else { ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 300; }
        assert(Lateral_Start(side, 1, 0) == 0); /* opposite sensor error ignored */
    }
    for (int outcome = STRAIGHT_DONE; outcome <= STRAIGHT_STALL; outcome++)
    {
        if (outcome == STRAIGHT_DONE) continue;
        reset(); ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 300;
        assert(Lateral_Start(0, 1, 0) == 0);
        straight = outcome; Lateral_Step();
        assert(calls == 0 && Lateral_State() == (outcome == STRAIGHT_ABORTED ? LATERAL_ABORTED : LATERAL_STRAIGHT_ERROR));
    }
    reset(); ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 300;
    assert(Lateral_Start(0, 1, 0) == 0); Lateral_Stop(); Lateral_Step();
    assert(calls == 0 && straight == STRAIGHT_ABORTED && Lateral_State() == LATERAL_ABORTED);
    reset(); ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 300;
    assert(Lateral_Start(0, 1, 0) == 0); straight = STRAIGHT_DONE;
    Lateral_Stop(); Lateral_Step(); assert(calls == 0 && Lateral_State() == LATERAL_ABORTED);
    reset(); ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 300;
    straight_rc = -4;
    assert(Lateral_Start(0, 1, 0) == -4 && calls == 0 && Lateral_State() == LATERAL_START_ERROR);
    reset(); ir.left_status = IR_RANGE_VALID; ir.left_cm_x10 = 300;
    assert(Lateral_Start(0, 1, 0) == 0); fail_call = 1;
    straight = STRAIGHT_DONE; Lateral_Step();
    assert(Lateral_State() == LATERAL_START_ERROR && calls == 1);
    reset();
    assert(Lateral_Start(2, 0, 300) == -2);
    assert(Lateral_Start(0, 2, 300) == -2);
    assert(Lateral_Start(0, 0, 0) == -2);
    assert(Lateral_Start(0, 0, 551) == -2);
    assert(Lateral_Start(0, 0, UINT32_MAX) == -2 && calls == 0);
    assert(Lateral_Start(1, 0, 10) == 0 && status().distance_cm_x10 == 10); /* n=1 cm */
    reset(); straight = STRAIGHT_RUNNING;
    assert(Lateral_Start(0, 0, 300) == -3 && calls == 0);
    straight = STRAIGHT_IDLE; turn = TURN_RUNNING;
    assert(Lateral_Start(0, 0, 300) == -3 && calls == 0);
    reset(); fail_call = 1;
    assert(Lateral_Start(0, 0, 300) == -1 && Lateral_State() == LATERAL_START_ERROR);
    Lateral_Step(); assert(calls == 1);
    reset(); fail_call = 2; fail_rc = -4;
    assert(Lateral_Start(0, 0, 300) == 0);
    turn = TURN_DONE; Lateral_Step();
    assert(Lateral_State() == LATERAL_START_ERROR && status().start_rc == -4 && status().stage == 2);
    Lateral_Step(); assert(calls == 2);
    reset(); assert(Lateral_Start(0, 0, 300) == 0);
    turn = TURN_TIMEOUT; Lateral_Step();
    assert(Lateral_State() == LATERAL_TURN_ERROR && calls == 1 && status().turn_state == TURN_TIMEOUT);
    reset(); assert(Lateral_Start(0, 0, 300) == 0);
    turn = TURN_DONE; Lateral_Stop(); Lateral_Step();
    assert(Lateral_State() == LATERAL_ABORTED && calls == 1); /* stop between arcs */
    reset(); assert(Lateral_Start(1, 0, 300) == 0);
    turn = TURN_DONE; Lateral_Step(); Lateral_Stop(); Lateral_Step();
    assert(Lateral_State() == LATERAL_ABORTED && calls == 2 && turn == TURN_ABORTED);
    puts("PASS: lateral geometry, both directions/modes, IR snapshot, approach-before-arcs, approach errors/STOP, bounds, busy, both-stage STOP and faults");
}
