#ifndef BYPASS_CONTROL_H
#define BYPASS_CONTROL_H
#include <stdint.h>
#include "control/turn_control.h"
typedef enum {
    BYPASS_IDLE = 0, BYPASS_RUNNING, BYPASS_DONE, BYPASS_ABORTED,
    BYPASS_TURN_ERROR, BYPASS_START_ERROR
} bypass_state_t;
typedef struct {
    bypass_state_t state;
    uint8_t side; /* 0=LEFT, 1=RIGHT */
    uint8_t stage; /* 1..3, retained on failure */
    int32_t distance_cm; /* requested maximum lateral displacement */
    int32_t angle_deg; /* outer angle; middle is exactly twice this */
    int start_rc;
    turn_state_t turn_state;
} bypass_status_t;
void Bypass_Init(void);
int Bypass_Start(uint8_t side, int32_t distance_cm);
void Bypass_Step(void);
void Bypass_Stop(void);
bypass_state_t Bypass_State(void);
void Bypass_Status(bypass_status_t *out);
#endif
