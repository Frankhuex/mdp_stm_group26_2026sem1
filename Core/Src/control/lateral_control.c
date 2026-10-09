#include "control/lateral_control.h"
#include "control/straight_control.h"
#include "control/turn_tuning.h"
#include "sensors/ir_sensor.h"
#include <math.h>

static lateral_status_t lc;

static int StartArc(uint8_t side)
{
    /* Existing TURN convention: -1=LEFT, +1=RIGHT; positive angle=forward. */
    return Turn_StartRadiusAngle(side ? CMD_RIGHT_MIN_RADIUS_CM : CMD_LEFT_SOFT_MIN_RADIUS_CM,
                                 lc.angle_deg, side ? 1 : -1);
}
void Lateral_Init(void)
{
    lateral_status_t empty = {0};
    lc = empty;
}
int Lateral_Start(uint8_t side, uint8_t from_ir, uint32_t distance_cm_x10)
{
    if (lc.state == LATERAL_RUNNING || Straight_State() == STRAIGHT_RUNNING ||
        Turn_State() == TURN_RUNNING) return -3;
    if (side > 1U || from_ir > 1U) return -2;
    if (from_ir)
    {
        IRSensorDistanceReading reading;
        if (IRSensor_ReadDistance(&reading, 1U) != 0) return -5;
        IRSensorRangeStatus range = side ? reading.right_status : reading.left_status;
        /* A calibrated finite distance is required for the displacement math. */
        if (range != IR_RANGE_VALID) return -5;
        distance_cm_x10 = side ? reading.right_cm_x10 : reading.left_cm_x10;
        if (distance_cm_x10 < IR_SENSOR_VALID_MIN_CM_X10 ||
            distance_cm_x10 > IR_SENSOR_VALID_MAX_CM_X10) return -5;
        /* Half obstacle width plus half vehicle width, converted to 0.1 cm. */
        uint32_t offset = (LATERAL_OBSTACLE_WIDTH_CM + LATERAL_VEHICLE_WIDTH_CM) * 5U;
        if (distance_cm_x10 > UINT32_MAX - offset) return -2;
        distance_cm_x10 += offset;
    }
    float sum_radius = (float)(CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM);
    if (distance_cm_x10 == 0U || (float)distance_cm_x10 > sum_radius * 10.0f) return -2;
    float angle = acosf(1.0f - (float)distance_cm_x10 / (10.0f * sum_radius)) *
                  (180.0f / 3.14159265358979323846f);
    int32_t rounded = (int32_t)lroundf(angle);
    if (rounded < CMD_MIN_ANGLE_DEG || rounded > 90 || rounded > CMD_MAX_ANGLE_DEG) return -2;
    lc.side = side;
    lc.from_ir = from_ir;
    lc.stage = 1U;
    lc.distance_cm_x10 = distance_cm_x10;
    lc.angle_deg = rounded;
    lc.start_rc = StartArc(side);
    lc.turn_state = Turn_State();
    lc.state = lc.start_rc == 0 ? LATERAL_RUNNING : LATERAL_START_ERROR;
    return lc.start_rc;
}
void Lateral_Step(void)
{
    if (lc.state != LATERAL_RUNNING) return;
    lc.turn_state = Turn_State();
    if (lc.turn_state == TURN_RUNNING) return;
    if (lc.turn_state != TURN_DONE)
    {
        lc.state = lc.turn_state == TURN_ABORTED ? LATERAL_ABORTED : LATERAL_TURN_ERROR;
        return;
    }
    if (lc.stage == 2U) { lc.state = LATERAL_DONE; return; }
    lc.stage = 2U;
    lc.start_rc = StartArc(!lc.side);
    lc.turn_state = Turn_State();
    if (lc.start_rc != 0) lc.state = LATERAL_START_ERROR;
}
void Lateral_Stop(void)
{
    if (lc.state != LATERAL_RUNNING) return;
    /* Cancel the composite first so a completed first arc cannot launch arc 2. */
    lc.state = LATERAL_ABORTED;
    Turn_Stop();
    lc.turn_state = Turn_State();
}
lateral_state_t Lateral_State(void) { return lc.state; }
void Lateral_Status(lateral_status_t *out) { if (out) *out = lc; }
