#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H
#include <stdint.h>
#include "control/turn_control.h"
#include "control/straight_control.h"

typedef enum {
    LATERAL_IDLE = 0, LATERAL_RUNNING, LATERAL_DONE,
    LATERAL_ABORTED, LATERAL_TURN_ERROR, LATERAL_START_ERROR, LATERAL_STRAIGHT_ERROR
} lateral_state_t;
typedef struct {
    lateral_state_t state;
    uint8_t side; /* 0=LEFT, 1=RIGHT */
    uint8_t from_ir;
    uint8_t from_edge;
    uint8_t stage; /* 0=IR approach, 1/2=turn arcs; retained on failure */
    uint32_t initial_ir_cm_x10;
    uint32_t distance_cm_x10;
    int32_t angle_deg; /* rounded command angle, equal for both arcs */
    int start_rc;
    turn_state_t turn_state;
    straight_state_t straight_state;
} lateral_status_t;
void Lateral_Init(void);
/* Numeric mode supplies the final displacement in 0.1 cm. IR mode measures
 * the selected sensor at command start, adds obstacle half-width and IR-to-centre offset,
 * then drives until selected IR > 50 cm before starting the two arcs. Returns -5 for
 * unavailable IR, -2 for invalid range, -3 for busy; other errors from TURN. */
int Lateral_Start(uint8_t side, uint8_t from_ir, uint32_t distance_cm_x10);
int Lateral_StartAfterIREdge(int32_t distance_cm, uint8_t side,
                             straight_ir_comparison_t comparison, int32_t threshold_cm);
/* Immediate IR edge detection, without a preceding encoder distance gate. */
int Lateral_StartOnIREdge(uint8_t side, straight_ir_comparison_t comparison,
    int32_t threshold_cm, int8_t direction);
void Lateral_Step(void);
void Lateral_Stop(void);
lateral_state_t Lateral_State(void);
void Lateral_Status(lateral_status_t *out);
#endif
