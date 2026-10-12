#ifndef SEARCH_LATERAL_CONTROL_H
#define SEARCH_LATERAL_CONTROL_H
#include <stdint.h>
#define SEARCH_LATERAL_DEFAULT_LIMIT_MM 1000U /* 100 cm total search travel */
typedef enum { SEARCH_IDLE, SEARCH_RUNNING, SEARCH_DONE, SEARCH_ABORTED,
               SEARCH_STRAIGHT_ERROR, SEARCH_TURN_ERROR, SEARCH_LATERAL_ERROR,
               SEARCH_START_ERROR } search_lateral_state_t;
typedef struct {
    search_lateral_state_t state;
    uint8_t side, stage, fallback;
    int32_t gate_cm, threshold_cm;
    uint32_t limit_mm, ir_cm_x10;
    int start_rc;
} search_lateral_status_t;
void SearchLateral_Init(void);
int SearchLateral_Start(uint8_t side, int32_t gate_cm, int32_t threshold_cm);
void SearchLateral_Step(void);
void SearchLateral_Stop(void);
search_lateral_state_t SearchLateral_State(void);
void SearchLateral_Status(search_lateral_status_t *out);
#endif
