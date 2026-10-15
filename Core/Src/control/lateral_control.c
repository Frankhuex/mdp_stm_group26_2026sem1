#include "control/lateral_control.h"
#include "control/bypass_control.h"
#include "control/motion_log.h"
#include "control/straight_control.h"
#include "control/turn_tuning.h"
#include "control/straight_tuning.h"
#include "sensors/ir_sensor.h"
#include <math.h>

#define LATERAL_ENCODER_OFFSET_MM 300U

static lateral_status_t lc;

static int StartArc(uint8_t side)
{
    /* Existing TURN convention: -1=LEFT, +1=RIGHT; positive angle=forward. */
    return Turn_StartRadiusAngle(side ? CMD_RIGHT_MIN_RADIUS_CM : CMD_LEFT_SOFT_MIN_RADIUS_CM,
                                 lc.angle_deg + (lc.stage == 1U ? lc.first_angle_offset_deg : 0), side ? 1 : -1);
}
void Lateral_Init(void)
{
    lateral_status_t empty = {0};
    lc = empty;
}
static int StartWithOffset(uint8_t side, uint8_t from_ir, uint32_t distance_cm_x10, int32_t first_offset)
{
    if (lc.state == LATERAL_RUNNING || Straight_State() == STRAIGHT_RUNNING ||
        Turn_State() == TURN_RUNNING) return -3;
    if (side > 1U || from_ir > 1U) return -2;
    uint32_t initial_ir = 0U;
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
        initial_ir = distance_cm_x10;
        /* Half obstacle width plus IR-to-centre offset, converted to 0.1 cm. */
        uint32_t offset = LATERAL_OBSTACLE_WIDTH_CM * 5U + LATERAL_IR_TO_CENTER_CM * 10U;
        if (distance_cm_x10 > UINT32_MAX - offset) return -2;
        distance_cm_x10 += offset;
    }
    float sum_radius = (float)(CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM);
    uint32_t arc_limit_mm = (uint32_t)sum_radius * 10U;
    if (distance_cm_x10 == 0U || distance_cm_x10 > arc_limit_mm + MAX_COMMAND_DISTANCE_CM * 10U) return -2;
    uint32_t arc_mm = distance_cm_x10 > arc_limit_mm ? arc_limit_mm : distance_cm_x10;
    float angle = acosf(1.0f - (float)arc_mm / (10.0f * sum_radius)) *
                  (180.0f / 3.14159265358979323846f);
    int32_t rounded = (int32_t)lroundf(angle);
    if (rounded < CMD_MIN_ANGLE_DEG || rounded > 90 || rounded > CMD_MAX_ANGLE_DEG) return -2;
    int64_t first_angle = (int64_t)rounded + first_offset;
    if (first_angle < CMD_MIN_ANGLE_DEG || first_angle > CMD_MAX_ANGLE_DEG) return -2;
    lc.first_angle_offset_deg = first_offset;
    lc.side = side;
    lc.from_ir = from_ir;
    lc.from_edge = 0U;
    lc.from_encoder = 0U;
    lc.stage = from_ir ? 0U : 1U;
    lc.initial_ir_cm_x10 = initial_ir;
    lc.distance_cm_x10 = distance_cm_x10;
    lc.middle_mm = distance_cm_x10 - arc_mm;
    lc.angle_deg = rounded;
    MotionLog_Write("LATERAL,PLAN,dir=%s,target_mm=%lu,first_deg=%ld,middle_mm=%lu,second_deg=%ld",
        side ? "RIGHT" : "LEFT",(unsigned long)distance_cm_x10,(long)(rounded+first_offset),
        (unsigned long)lc.middle_mm,(long)rounded);
    lc.start_rc = from_ir ? Straight_StartUntilIRGreaterValidCm(50, side) : StartArc(side);
    lc.straight_state = Straight_State();
    lc.turn_state = Turn_State();
    if (!lc.start_rc && !from_ir) {
        /* Two mirrored LATERAL moves have twice this arc pair's forward travel.
         * Use the rounded commanded angle, matching BYPASS geometry. */
        Bypass_ForwardDistanceMm = (uint32_t)lroundf(2.0f * sum_radius * 10.0f *
            sinf(rounded * (3.14159265358979323846f / 180.0f)));
        MotionLog_Write("CACHE,BYPASS,source=LATERAL,dir=%s,forward_mm=%lu",
            side ? "RIGHT" : "LEFT", (unsigned long)Bypass_ForwardDistanceMm);
    }
    lc.state = lc.start_rc == 0 ? LATERAL_RUNNING : LATERAL_START_ERROR;
    return lc.start_rc;
}
int Lateral_Start(uint8_t side, uint8_t from_ir, uint32_t distance_cm_x10)
{
    return StartWithOffset(side, from_ir, distance_cm_x10, 0);
}
int Lateral_StartEncoder(uint8_t side)
{
    return Lateral_StartEncoderOffset(side, 0);
}
int Lateral_StartEncoderOffset(uint8_t side, int32_t first_angle_offset_deg)
{
    if (Straight_IRTravelMm == UINT32_MAX) return -5;
    uint32_t displacement_mm = (Straight_IRTravelMm + 1U) / 2U + LATERAL_ENCODER_OFFSET_MM;
    MotionLog_Write("LATERAL,ENCODER,dir=%s,travel_mm=%lu,target_mm=%lu,first_offset_deg=%ld",
        side ? "RIGHT" : "LEFT",(unsigned long)Straight_IRTravelMm,(unsigned long)displacement_mm,(long)first_angle_offset_deg);
    int rc = StartWithOffset(side, 0U, displacement_mm, first_angle_offset_deg);
    if (!rc) lc.from_encoder = 1U;
    return rc;
}
int Lateral_StartAfterIREdge(int32_t distance_cm, uint8_t side,
                             straight_ir_comparison_t comparison, int32_t threshold_cm)
{
    if (lc.state == LATERAL_RUNNING || Straight_State() == STRAIGHT_RUNNING ||
        Turn_State() == TURN_RUNNING) return -3;
    if (side > 1U || (comparison != STRAIGHT_IR_RISING && comparison != STRAIGHT_IR_FALLING)) return -2;
    int rc = Straight_StartCmThenIR(distance_cm, side, comparison, threshold_cm);
    if (rc) return rc;
    Straight_SetIRTravelCacheEnabled(0U);
    lateral_status_t empty = {0}; lc = empty;
    lc.side = side; lc.from_ir = lc.from_edge = 1U; lc.stage = 0U;
    lc.state = LATERAL_RUNNING; lc.straight_state = Straight_State();
    return 0;
}

int Lateral_StartOnIREdge(uint8_t side, straight_ir_comparison_t comparison,
    int32_t threshold_cm, int8_t direction)
{
    if (lc.state == LATERAL_RUNNING || Straight_State() == STRAIGHT_RUNNING ||
        Turn_State() == TURN_RUNNING) return -3;
    if (side > 1U || (comparison != STRAIGHT_IR_RISING && comparison != STRAIGHT_IR_FALLING)) return -2;
    int rc = Straight_StartUntilIRDirectionCm(threshold_cm,side,comparison,direction);
    if (rc) return rc;
    Straight_SetIRTravelCacheEnabled(0U);
    lateral_status_t empty = {0}; lc = empty;
    lc.side = side; lc.from_ir = lc.from_edge = 1U; lc.stage = 0U;
    lc.state = LATERAL_RUNNING; lc.straight_state = Straight_State();
    return 0;
}

void Lateral_Step(void)
{
    if (lc.state != LATERAL_RUNNING) return;
    if (lc.stage == 0U)
    {
        lc.straight_state = Straight_State();
        if (lc.from_edge) {
            uint32_t first;
            if (Straight_IRFirstEdgeReading(&first)) {
                lc.initial_ir_cm_x10 = first;
                lc.distance_cm_x10 = first + LATERAL_OBSTACLE_WIDTH_CM * 5U + LATERAL_IR_TO_CENTER_CM * 10U;
            }
        }
        if (lc.straight_state == STRAIGHT_RUNNING) return;
        if (lc.straight_state != STRAIGHT_DONE)
        {
            lc.state = lc.straight_state == STRAIGHT_ABORTED ? LATERAL_ABORTED : LATERAL_STRAIGHT_ERROR;
            return;
        }
        if (lc.from_edge) {
            float sum = CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM;
            if (!lc.initial_ir_cm_x10 || !lc.distance_cm_x10 || lc.distance_cm_x10 > sum * 10.0f + MAX_COMMAND_DISTANCE_CM * 10U) {
                lc.start_rc = lc.initial_ir_cm_x10 ? -2 : -5;
                lc.state = LATERAL_START_ERROR; return;
            }
            uint32_t arc_mm = lc.distance_cm_x10 > sum * 10.0f ? (uint32_t)sum * 10U : lc.distance_cm_x10;
            lc.middle_mm = lc.distance_cm_x10 - arc_mm;
            lc.angle_deg = (int32_t)lroundf(acosf(1.0f - arc_mm / (sum * 10.0f)) *
                (180.0f / 3.14159265358979323846f));
            if (lc.angle_deg < CMD_MIN_ANGLE_DEG || lc.angle_deg > 90) {
                lc.start_rc = -2; lc.state = LATERAL_START_ERROR; return;
            }
        }
        lc.stage = 1U;
        lc.start_rc = StartArc(lc.side);
        lc.turn_state = Turn_State();
        if (lc.start_rc != 0) lc.state = LATERAL_START_ERROR;
        return;
    }
    if (lc.stage == 3U) {
        lc.straight_state = Straight_State();
        if (lc.straight_state == STRAIGHT_RUNNING) return;
        if (lc.straight_state != STRAIGHT_DONE) {
            lc.state = lc.straight_state == STRAIGHT_ABORTED ? LATERAL_ABORTED : LATERAL_STRAIGHT_ERROR;
            return;
        }
        lc.stage = 2U;
        lc.start_rc = StartArc(!lc.side);
        lc.turn_state = Turn_State();
        if (lc.start_rc) lc.state = LATERAL_START_ERROR;
        return;
    }
    lc.turn_state = Turn_State();
    if (lc.turn_state == TURN_RUNNING) return;
    if (lc.turn_state != TURN_DONE)
    {
        lc.state = lc.turn_state == TURN_ABORTED ? LATERAL_ABORTED : LATERAL_TURN_ERROR;
        return;
    }
    if (lc.stage == 2U) { lc.state = LATERAL_DONE; return; }
    if (lc.middle_mm) {
        lc.stage = 3U;
        lc.start_rc = Straight_StartForwardMm(lc.middle_mm);
        lc.straight_state = Straight_State();
        MotionLog_Write("LATERAL,MIDDLE,dir=%s,distance_mm=%lu,start_rc=%d",
            lc.side ? "RIGHT" : "LEFT",(unsigned long)lc.middle_mm,lc.start_rc);
        if (lc.start_rc) lc.state = LATERAL_START_ERROR;
        return;
    }
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
    if (lc.stage == 0U || lc.stage == 3U) Straight_Stop();
    else Turn_Stop();
    lc.straight_state = Straight_State();
    lc.turn_state = Turn_State();
}
lateral_state_t Lateral_State(void) { return lc.state; }
void Lateral_Status(lateral_status_t *out) { if (out) *out = lc; }
