#include "control/bypass_control.h"
#include "control/straight_control.h"
#include "control/lateral_control.h"
#include "control/turn_tuning.h"
#include <math.h>
static bypass_status_t bc;
static int StartArc(void)
{
    uint8_t side = bc.stage == 2U ? !bc.side : bc.side;
    return Turn_StartRadiusAngle(side ? CMD_RIGHT_MIN_RADIUS_CM : CMD_LEFT_SOFT_MIN_RADIUS_CM,
                                bc.angle_deg * (bc.stage == 2U ? 2 : 1), side ? 1 : -1);
}
void Bypass_Init(void) { bypass_status_t empty = {0}; bc = empty; }
int Bypass_Start(uint8_t side, int32_t distance_cm)
{
    if (bc.state == BYPASS_RUNNING || Straight_State() == STRAIGHT_RUNNING ||
        Turn_State() == TURN_RUNNING || Lateral_State() == LATERAL_RUNNING) return -3;
    int32_t radius_sum = CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM;
    if (side > 1U || distance_cm <= 0 || distance_cm > radius_sum) return -2;
    int32_t angle = (int32_t)lroundf(acosf(1.0f - (float)distance_cm / radius_sum) *
                                    (180.0f / 3.14159265358979323846f));
    if (angle < CMD_MIN_ANGLE_DEG || angle > 90 || 2 * angle > CMD_MAX_ANGLE_DEG) return -2;
    bc.side = side; bc.stage = 1U; bc.distance_cm = distance_cm; bc.angle_deg = angle;
    bc.start_rc = StartArc(); bc.turn_state = Turn_State();
    bc.state = bc.start_rc ? BYPASS_START_ERROR : BYPASS_RUNNING;
    return bc.start_rc;
}
void Bypass_Step(void)
{
    if (bc.state != BYPASS_RUNNING) return;
    bc.turn_state = Turn_State();
    if (bc.turn_state == TURN_RUNNING) return;
    if (bc.turn_state != TURN_DONE) {
        bc.state = bc.turn_state == TURN_ABORTED ? BYPASS_ABORTED : BYPASS_TURN_ERROR;
        return;
    }
    if (bc.stage == 3U) { bc.state = BYPASS_DONE; return; }
    bc.stage++;
    bc.start_rc = StartArc(); bc.turn_state = Turn_State();
    if (bc.start_rc) bc.state = BYPASS_START_ERROR;
}
void Bypass_Stop(void)
{
    if (bc.state != BYPASS_RUNNING) return;
    bc.state = BYPASS_ABORTED; Turn_Stop(); bc.turn_state = Turn_State();
}
bypass_state_t Bypass_State(void) { return bc.state; }
void Bypass_Status(bypass_status_t *out) { if (out) *out = bc; }
