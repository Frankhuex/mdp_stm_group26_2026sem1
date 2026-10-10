#include "main.h"
#include "assessment/command_interface.h"
#include "control/straight_control.h"
#include "control/straight_tuning.h"   /* STRAIGHT_FORWARD_TRIM_US for calibration */
#include "control/turn_control.h"
#include "control/lateral_control.h"
#include "control/bypass_control.h"
#include "sensors/ir_sensor.h"
#include "control/motion_heading.h"
#include "drivers/robot_hw.h"
#include "drivers/oled.h"
#include "sensors/ultrasonic_sensor.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>

extern UART_HandleTypeDef huart3;

#define CMD_LINE_SIZE          128U
#define CMD_RX_TIMEOUT_MS      10U
#define CMD_ECHO_ENABLE        0U

/* Interrupt-driven RX ring buffer. The old polled HAL_UART_Receive in the main
 * loop dropped bytes whenever an I2C gyro read or a motion step blocked longer
 * than one UART character time (~87us at 115200) - corrupting commands into
 * ERR,UNKNOWN_CMD, typically on the first command after an idle gap. The RXNE
 * interrupt captures every byte into this ring regardless of main-loop timing,
 * and clears overrun; the main loop just drains the ring. */
#define RX_RING_SIZE 512U
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_ring_head = 0U;
static volatile uint16_t rx_ring_tail = 0U;

void CommandInterface_UartIrq(void)
{
    USART_TypeDef *u = huart3.Instance;
    uint32_t sr = u->SR;
    if (sr & USART_SR_RXNE)
    {
        uint8_t b = (uint8_t)(u->DR & 0xFFU);      /* reading DR clears RXNE + ORE */
        uint16_t next = (uint16_t)((rx_ring_head + 1U) % RX_RING_SIZE);
        if (next != rx_ring_tail)                  /* drop on full; never block the ISR */
        {
            rx_ring[rx_ring_head] = b;
            rx_ring_head = next;
        }
    }
    else if (sr & USART_SR_ORE)
    {
        (void)u->DR;                               /* clear a standalone overrun */
    }
}

static int RxRingPop(uint8_t *b)
{
    if (rx_ring_head == rx_ring_tail) return 0;
    *b = rx_ring[rx_ring_tail];
    rx_ring_tail = (uint16_t)((rx_ring_tail + 1U) % RX_RING_SIZE);
    return 1;
}

#define CMD_GYRO_ALPHA         0.35f
#define CMD_GYRO_DEADBAND_DPS  0.20f
#define CMD_LOOP_COMMAND_GAP_MS 300U

/* Retain one sequence without an attached UART receiver. Never overwrite the
 * beginning on overflow: report lost bytes when the user dumps the log.
 * Volatile RAM only; reset/power loss or the next sequence discards it. */
#define SEQUENCE_LOG_CAPACITY (48U * 1024U)
static char sequence_log[SEQUENCE_LOG_CAPACITY + 1U];
static uint32_t sequence_log_used, sequence_log_dropped;
static uint8_t sequence_log_active, sequence_log_capture_only;
static const char *sequence_log_result = "EMPTY";

static char rx_line[CMD_LINE_SIZE];
static uint32_t rx_line_len = 0U;
static uint8_t previous_byte = 0U;
static uint8_t rx_discard = 0U;
static uint8_t gyro_ready = 0U;
static uint8_t command_rejected = 0U;
static uint8_t sequence_mode = 0U;
/* Only the executor's synchronous DispatchLine call may nest a FAST group. */
static uint8_t sequence_row_dispatch = 0U;
static const char *sequence_row_failure = NULL;
static uint8_t sequence_cancelled = 0U;
static uint8_t command_output_suppressed = 0U;
static uint8_t pending_motion_done_suppressed = 0U;
static uint32_t gyro_last_tick = 0U;

static const char *const loop_once_commands[] =
{
    "TURN,RIGHT,R=35,A=90",
    "STRAIGHT,GOCM,40",
    "TURN,LEFT,R=20,A=180",
    "STRAIGHT,UNTILUS,47",
};

/* Fastest-car groups from test_hardcode_command_executor.c.
 * Each route uses the normal parser/controllers, as LOOP,ONCE does. */
static const char *const fast_left1_commands[] =
{
    // "LATERAL,LEFT,25",
    // "STRAIGHT,RIGHTIR,GT,50",
    // "LATERAL,RIGHT,25",

    // "TURN,LEFT,R=35,A=60",
    // "TURN,RIGHT,R=35,A=120",
    // "TURN,LEFT,R=35,A=60",
    // "STRAIGHT,GOCM,-33",

    "BYPASS,LEFT,25",
    "STRAIGHT,BACKWARD,UNTILUS,35",
};
static const char *const fast_left2_commands[] =
{
    "TURN,LEFT,R=35,A=90",
    "STRAIGHT,BACKWARD,RIGHTIR,LT,30",
    "STRAIGHT,RIGHTIR,GT,30",
    "TURN,RIGHT,R=35,A=180",
    "STRAIGHT,RIGHTIR,RISING,50",
    "TURN,RIGHT,R=35,A=90",
	"STRAIGHT,GOCM,30,RIGHTIR,RISING,50,LATERAL",
    "STRAIGHT,UNTILUS,15",
};
static const char *const fast_right1_commands[] =
{
    // "LATERAL,RIGHT,25",
    // "STRAIGHT,LEFTIR,GT,50",
    // "LATERAL,LEFT,25",

    // "TURN,RIGHT,R=35,A=60",
    // "TURN,LEFT,R=35,A=120",
    // "TURN,RIGHT,R=35,A=60",
    // "STRAIGHT,GOCM,-33",

    "BYPASS,RIGHT,28",
    "STRAIGHT,BACKWARD,UNTILUS,35",
};
static const char *const fast_right2_commands[] =
{
    "TURN,RIGHT,R=35,A=90",
    "STRAIGHT,BACKWARD,LEFTIR,LT,30",
    "STRAIGHT,LEFTIR,GT,30",
    "TURN,LEFT,R=35,A=180",
    "STRAIGHT,LEFTIR,RISING,50",
    "TURN,LEFT,R=35,A=90",
	"STRAIGHT,GOCM,30,LEFTIR,RISING,50,LATERAL",
    "STRAIGHT,UNTILUS,15",
};

typedef enum
{
    PENDING_MOTION_NONE = 0,
    PENDING_MOTION_STRAIGHT,
    PENDING_MOTION_UNTILUS,
    PENDING_MOTION_TURN,
    PENDING_MOTION_LATERAL,
    PENDING_MOTION_BYPASS
} pending_motion_t;

static const char *CommandInterface_RunSequenceRows(const char *const commands[],
                                                    uint32_t count,
                                                    uint32_t startup_ms,
                                                    uint32_t gap_ms,
                                                    uint8_t keep_sequence_mode,
                                                    uint8_t emit_sequence_status,
                                                    uint8_t suppress_command_replies,
                                                    uint32_t *failure_index,
                                                    command_motion_observer_t observer);

static pending_motion_t pending_motion = PENDING_MOTION_NONE;
static int32_t pending_straight_cm = 0;
static int32_t pending_turn_radius_cm = 0;
static int32_t pending_turn_angle_deg = 0;
static int8_t pending_turn_dir = 1;

static void Cmd_SendRaw(const uint8_t *data, uint16_t length)
{
    if (data != NULL && length > 0U)
    {
        if (sequence_log_active)
        {
            uint32_t available = SEQUENCE_LOG_CAPACITY - sequence_log_used;
            uint32_t copy = length < available ? length : available;
            memcpy(sequence_log + sequence_log_used, data, copy);
            sequence_log_used += copy;
            sequence_log[sequence_log_used] = '\0';
            sequence_log_dropped += (uint32_t)length - copy;
        }
        if (sequence_log_capture_only || command_output_suppressed) return;
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, length, HAL_MAX_DELAY);
    }
}

static void Cmd_Send(const char *text)
{
    if (text != NULL)
    {
        Cmd_SendRaw((const uint8_t *)text, (uint16_t)strlen(text));
    }
}

static void Cmd_SendLine(const char *text)
{
    Cmd_Send(text);
    Cmd_Send("\r\n");
}

/* Record parser rejection directly, independent of UART transmission. */
static void Cmd_Reject(const char *reason)
{
    command_rejected = 1U;
    Cmd_SendLine(reason);
}

static void Oled_ShowCommand(const char *line)
{
    char buf[17];
    size_t len;

    if (line == NULL) return;

    OLED_Clear();
    OLED_ShowString(0, 0, (const u8 *)"CMD:");

    len = strlen(line);
    for (uint32_t row = 0U; row < 3U; row++)
    {
        size_t start = (size_t)(row * 16U);
        size_t copy_len;

        if (start >= len) break;

        copy_len = len - start;
        if (copy_len > 16U) copy_len = 16U;
        memcpy(buf, line + start, copy_len);
        buf[copy_len] = '\0';
        OLED_ShowString(0, (u8)((row + 1U) * 16U), (const u8 *)buf);
    }

    OLED_Refresh_Gram();
}

static const char *StraightStateText(straight_state_t state)
{
    switch (state)
    {
        case STRAIGHT_IDLE:    return "IDLE";
        case STRAIGHT_RUNNING: return "RUNNING";
        case STRAIGHT_DONE:    return "DONE";
        case STRAIGHT_ABORTED: return "ABORTED";
        case STRAIGHT_TIMEOUT: return "TIMEOUT";
        case STRAIGHT_SENSOR_ERROR: return "SENSOR_ERROR";
        case STRAIGHT_DISTANCE_LIMIT: return "DISTANCE_LIMIT";
        case STRAIGHT_STALL:   return "STALL";
        default:               return "UNKNOWN";
    }
}

static const char *TurnStateText(turn_state_t state)
{
    switch (state)
    {
        case TURN_IDLE:       return "IDLE";
        case TURN_RUNNING:    return "RUNNING";
        case TURN_DONE:       return "DONE";
        case TURN_ABORTED:    return "ABORTED";
        case TURN_TIMEOUT:    return "TIMEOUT";
        case TURN_WRONG_SIGN: return "WRONG_SIGN";
        case TURN_NO_TURN:    return "NO_TURN";
        case TURN_BAD_CONFIG: return "BAD_CONFIG";
        case TURN_NO_IMU:     return "NO_IMU";
        default:              return "UNKNOWN";
    }
}

static const char *TurnDirText(int8_t turn_dir)
{
    return (turn_dir >= 0) ? "RIGHT" : "LEFT";
}

static int ParseIntStrict(const char *text, int32_t *out)
{
    char *endptr = NULL;
    long value;

    if (text == NULL || text[0] == '\0' || out == NULL) return 0;
    errno = 0;
    value = strtol(text, &endptr, 10);
    if (errno == ERANGE || value < INT32_MIN || value > INT32_MAX) return 0;
    if (endptr == text || endptr == NULL || *endptr != '\0') return 0;

    *out = (int32_t)value;
    return 1;
}

static int ParseKeyInt(const char *text, const char *key, int32_t *out)
{
    size_t key_len;

    if (text == NULL || key == NULL || out == NULL) return 0;
    key_len = strlen(key);
    if (strncmp(text, key, key_len) != 0 || text[key_len] != '=') return 0;
    return ParseIntStrict(text + key_len + 1U, out);
}

static void FormatFloat3(char *buf, size_t size, float value)
{
    int32_t x1000 = (int32_t)(value * 1000.0f);
    int32_t abs_v = (x1000 < 0) ? -x1000 : x1000;

    snprintf(buf, size, "%c%ld.%03ld",
             (x1000 < 0) ? '-' : '+',
             (long)(abs_v / 1000),
             (long)(abs_v % 1000));
}

static uint8_t AnyMotionRunning(void)
{
    return (Bypass_State() == BYPASS_RUNNING ||
            Lateral_State() == LATERAL_RUNNING ||
            Straight_State() == STRAIGHT_RUNNING ||
            Turn_State() == TURN_RUNNING);
}

static uint8_t PwmInRange(int32_t pwm, uint32_t maximum)
{
    uint32_t magnitude;

    if (pwm == INT32_MIN) return 0U;
    magnitude = (pwm < 0) ? (uint32_t)(-pwm) : (uint32_t)pwm;
    return (magnitude <= maximum) ? 1U : 0U;
}

static void ClearPendingMotion(void)
{
    pending_motion = PENDING_MOTION_NONE;
    pending_straight_cm = 0;
    pending_turn_radius_cm = 0;
    pending_turn_angle_deg = 0;
    pending_turn_dir = 1;
}

static uint8_t StraightStateIsTerminal(straight_state_t state)
{
    return (state == STRAIGHT_DONE ||
            state == STRAIGHT_ABORTED ||
            state == STRAIGHT_TIMEOUT ||
            state == STRAIGHT_SENSOR_ERROR ||
            state == STRAIGHT_DISTANCE_LIMIT ||
            state == STRAIGHT_STALL);
}

static uint8_t TurnStateIsTerminal(turn_state_t state)
{
    return (state == TURN_DONE ||
            state == TURN_ABORTED ||
            state == TURN_TIMEOUT ||
            state == TURN_WRONG_SIGN ||
            state == TURN_NO_TURN ||
            state == TURN_BAD_CONFIG ||
            state == TURN_NO_IMU);
}

static void FormatUltrasonicMm(char *out, size_t size, uint32_t mm)
{
    if (mm == UINT32_MAX) snprintf(out, size, "INF");
    else snprintf(out, size, "%lu", (unsigned long)mm);
}

static const char *IRComparisonText(straight_ir_comparison_t op)
{
    switch (op) {
        case STRAIGHT_IR_GT: return "GT";
        case STRAIGHT_IR_LT: return "LT";
        case STRAIGHT_IR_RISING: return "RISING";
        case STRAIGHT_IR_FALLING: return "FALLING";
        default: return "UNKNOWN";
    }
}

static int SendIRStatus(uint8_t done)
{
    int32_t threshold;
    uint32_t distance;
    int mode = Straight_IRStatus(&threshold, &distance);
    int32_t target; Straight_Status(NULL, &target, NULL, NULL, NULL, NULL);
    char msg[384], cm[24], gate[64] = "";
    uint32_t raw, age; int range; uint8_t confirm, armed;
    Straight_IRDiagnostics(&raw,&range,&age,&confirm,&armed);
    int32_t gate_cm; uint8_t active;
    if (Straight_IRGateStatus(&gate_cm, &active))
        snprintf(gate,sizeof(gate),",gocm_cm=%ld,phase=%s",(long)gate_cm,active?"IR":"GOCM");
    if (!mode) return 0;
    if (distance == UINT32_MAX) snprintf(cm, sizeof(cm), "FAR");
    else snprintf(cm, sizeof(cm), "%lu.%lu",
                  (unsigned long)(distance / 10U), (unsigned long)(distance % 10U));
    snprintf(msg, sizeof(msg),
             "%sSTRAIGHT,mode=%s,op=%s,state=%s,threshold_cm=%ld,ir_cm=%s,dir=%s%s,ir_raw=%lu,ir_status=%s,ir_age_ms=%lu,confirm=%u,armed=%u\r\n",
             done ? "DONE," : "", mode == 1 ? "LEFTIR" : "RIGHTIR",
             IRComparisonText(Straight_IRComparison()),
             StraightStateText(Straight_State()), (long)threshold, cm,
             target < 0 ? "BACKWARD" : "FORWARD", gate, (unsigned long)raw,
             IRSensor_RangeText((IRSensorRangeStatus)range),(unsigned long)age,confirm,armed);
    Cmd_Send(msg);
    return 1;
}

static const char *LateralStateText(lateral_state_t state)
{
    switch (state) {
        case LATERAL_IDLE: return "IDLE";
        case LATERAL_RUNNING: return "RUNNING";
        case LATERAL_DONE: return "DONE";
        case LATERAL_ABORTED: return "ABORTED";
        case LATERAL_TURN_ERROR: return "TURN_ERROR";
        case LATERAL_START_ERROR: return "START_ERROR";
        case LATERAL_STRAIGHT_ERROR: return "STRAIGHT_ERROR";
        default: return "UNKNOWN";
    }
}
static void SendLateralStatus(uint8_t done)
{
    lateral_status_t st;
    char msg[256];
    Lateral_Status(&st);
    snprintf(msg, sizeof(msg),
        "%sLATERAL,state=%s,dir=%s,source=%s,d_cm=%lu.%lu,a=%ld,stage=%u,ir_cm_x10=%lu,straight_state=%s,turn_state=%s,start_rc=%d\r\n",
        done ? "DONE," : "", LateralStateText(st.state), st.side ? "RIGHT" : "LEFT",
        st.from_edge ? "EDGE_IR" : st.from_ir ? "IR" : "CM", (unsigned long)(st.distance_cm_x10 / 10U),
        (unsigned long)(st.distance_cm_x10 % 10U), (long)st.angle_deg,
        (unsigned)st.stage, (unsigned long)st.initial_ir_cm_x10,
        StraightStateText(st.straight_state), TurnStateText(st.turn_state), st.start_rc);
    Cmd_Send(msg);
}

static const char *BypassStateText(bypass_state_t state)
{
    switch (state) {
        case BYPASS_IDLE: return "IDLE";
        case BYPASS_RUNNING: return "RUNNING";
        case BYPASS_DONE: return "DONE";
        case BYPASS_ABORTED: return "ABORTED";
        case BYPASS_TURN_ERROR: return "TURN_ERROR";
        case BYPASS_START_ERROR: return "START_ERROR";
        default: return "UNKNOWN";
    }
}
static void SendBypassStatus(uint8_t done)
{
    bypass_status_t st; char msg[192]; Bypass_Status(&st);
    snprintf(msg, sizeof(msg),
        "%sBYPASS,state=%s,dir=%s,d_cm=%ld,a=%ld,middle_a=%ld,stage=%u,turn_state=%s,start_rc=%d\r\n",
        done ? "DONE," : "", BypassStateText(st.state), st.side ? "RIGHT" : "LEFT",
        (long)st.distance_cm, (long)st.angle_deg, (long)(2 * st.angle_deg),
        (unsigned)st.stage, TurnStateText(st.turn_state), st.start_rc);
    Cmd_Send(msg);
}

static void NotifyPendingMotionDone(void)
{
    char msg[192];

    if (pending_motion == PENDING_MOTION_BYPASS)
    {
        if (Bypass_State() == BYPASS_RUNNING) return;
        if (!pending_motion_done_suppressed) SendBypassStatus(1U);
        ClearPendingMotion();
    }
    else if (pending_motion == PENDING_MOTION_LATERAL)
    {
        if (Lateral_State() == LATERAL_RUNNING) return;
        if (!pending_motion_done_suppressed) SendLateralStatus(1U);
        ClearPendingMotion();
    }
    else if (pending_motion == PENDING_MOTION_UNTILUS)
    {
        straight_state_t state = Straight_State();
        int32_t threshold;
        uint32_t mm, age;
        char mm_text[12];
        int valid;
        int8_t direction = 0;
        if (!StraightStateIsTerminal(state)) return;
        if (pending_motion_done_suppressed) { ClearPendingMotion(); return; }
        Straight_UltrasonicStatus(&threshold, &mm, &valid, &age);
        uint8_t captured = Straight_UltrasonicStopReading(&mm, &age, &direction);
        if (captured) valid = 0;
        FormatUltrasonicMm(mm_text, sizeof(mm_text), mm);
        snprintf(msg, sizeof(msg),
                 "DONE,STRAIGHT,mode=UNTILUS,state=%s,reason=%s,threshold_cm=%ld,us_mm=%s,us_status=%d,dir=%s,us_age_ms=%lu\r\n",
                 StraightStateText(state),
                 state == STRAIGHT_DONE ? "US_THRESHOLD" : StraightStateText(state),
                 (long)threshold, mm_text, valid,
                 captured ? (direction > 0 ? "FORWARD" : direction < 0 ? "BACKWARD" : "NONE") : "UNKNOWN",
                 (unsigned long)age);
        Cmd_Send(msg);
        ClearPendingMotion();
    }
    else if (pending_motion == PENDING_MOTION_STRAIGHT)
    {
        straight_state_t state = Straight_State();

        if (!StraightStateIsTerminal(state)) return;
        if (pending_motion_done_suppressed) { ClearPendingMotion(); return; }

        if (SendIRStatus(1U)) { ClearPendingMotion(); return; }

        snprintf(msg, sizeof(msg),
                 "DONE,STRAIGHT,state=%s,target_cm=%ld\r\n",
                 StraightStateText(state),
                 (long)pending_straight_cm);
        Cmd_Send(msg);
        ClearPendingMotion();
    }
    else if (pending_motion == PENDING_MOTION_TURN)
    {
        turn_state_t state = Turn_State();

        if (!TurnStateIsTerminal(state)) return;
        if (pending_motion_done_suppressed) { ClearPendingMotion(); return; }

        {
            /* Instrumentation: yaw_meas = gyro angle actually swept (settled,
             * signed: + = LEFT) - compare to a=<commanded> and, for a fwd+rev
             * retrace, between the two turns. peak_dps = peak yaw rate (near
             * +/-250 would mean the gyro saturated). */
            char yaw_s[20];
            char peak_s[20];
            FormatFloat3(yaw_s, sizeof(yaw_s), Turn_SettledYawDeg());
            FormatFloat3(peak_s, sizeof(peak_s), Turn_PeakRateDps());
            snprintf(msg, sizeof(msg),
                     "DONE,TURN,state=%s,dir=%s,r_cm=%ld,a=%ld,yaw_meas=%s,peak_dps=%s\r\n",
                     TurnStateText(state),
                     TurnDirText(pending_turn_dir),
                     (long)pending_turn_radius_cm,
                     (long)pending_turn_angle_deg,
                     yaw_s, peak_s);
        }
        Cmd_Send(msg);
        ClearPendingMotion();
    }
}

static int32_t EncoderLeftRawOf(RobotHW_EncoderSnapshot snapshot)
{
    return (int16_t)snapshot.left;
}

static int32_t EncoderRightRawOf(RobotHW_EncoderSnapshot snapshot)
{
    return (int16_t)snapshot.right;
}

static int32_t EncoderLeftNormalizedOf(RobotHW_EncoderSnapshot snapshot)
{
    return EncoderLeftRawOf(snapshot);
}

static int32_t EncoderRightNormalizedOf(RobotHW_EncoderSnapshot snapshot)
{
    return -EncoderRightRawOf(snapshot);
}

static int EnsurePollingGyro(void)
{
    if (!gyro_ready)
    {
        if (!RobotHW_GyroIsInitialized() && !RobotHW_GyroInit()) return 0;
        if (!MotionHeading_IsTracking()) RobotHW_GyroPrimeFilter(CMD_GYRO_ALPHA);
        gyro_last_tick = HAL_GetTick();
        gyro_ready = 1U;
    }
    return 1;
}

static void GyroPollingStep(void)
{
    if (AnyMotionRunning() || MotionHeading_IsTracking())
    {
        gyro_last_tick = HAL_GetTick();
        return;
    }
    if (!gyro_ready) return;

    uint32_t now = HAL_GetTick();
    uint32_t delta_ms = now - gyro_last_tick;
    if (delta_ms < 10U) return;

    gyro_last_tick = now;
    (void)RobotHW_GyroUpdateYaw((float)delta_ms / 1000.0f,
                                CMD_GYRO_ALPHA,
                                CMD_GYRO_DEADBAND_DPS);
}

static void HandleHeadingStatus(void)
{
    motion_heading_status_t heading;
    char msg[180], target[20], measured[20], error[20];
    MotionHeading_Status(&heading);
    FormatFloat3(target, sizeof(target), heading.target_deg);
    FormatFloat3(measured, sizeof(measured), heading.measured_deg);
    FormatFloat3(error, sizeof(error), heading.error_deg);
    snprintf(msg, sizeof(msg),
        "HEADING,enabled=%u,target=%s,measured=%s,error=%s,tracking=%u,degraded=%u\r\n",
        (unsigned)heading.enabled, target, measured, error,
        (unsigned)heading.tracking, (unsigned)heading.degraded);
    Cmd_Send(msg);
}

static void HandleStraightTrace(void)
{
    char msg[220], local_yaw[20], error[20], rate[20];
    straight_trace_sample_t sample;
    /* Printing is deferred until stopped; never lengthen a live control tick. */
    if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
    uint32_t count = Straight_TraceCount();
    snprintf(msg, sizeof(msg), "TRACE,STRAIGHT,count=%lu,dropped=%lu,fixed_us=%lu\r\n",
             (unsigned long)count, (unsigned long)Straight_TraceDropped(),
             (unsigned long)Straight_FixedSteeringUs());
    Cmd_Send(msg);
    for (uint32_t i = 0U; i < count; ++i)
    {
        if (!Straight_TraceRead(i, &sample)) break;
        FormatFloat3(local_yaw, sizeof(local_yaw), sample.local_yaw_deg);
        FormatFloat3(error, sizeof(error), sample.control_error_deg);
        FormatFloat3(rate, sizeof(rate), sample.rate_dps);
        snprintf(msg, sizeof(msg),
            "TRACE,STRAIGHT,index=%lu,ms=%lu,dir=%s,phase=%c,local_yaw=%s,control_error=%s,rate_dps=%s,servo_us=%lu,gyro_ok=%u\r\n",
            (unsigned long)i, (unsigned long)sample.elapsed_ms,
            sample.direction < 0 ? "REV" : "FWD", (char)sample.phase,
            local_yaw, error, rate, (unsigned long)sample.servo_us,
            (unsigned)sample.gyro_ok);
        Cmd_Send(msg);
        if (!sequence_log_capture_only) MotionHeading_PollIdle();
    }
    Cmd_SendLine("TRACE,STRAIGHT,END");
}

static void HandleStraightStatus(void)
{
    char msg[320];
    char l[20];
    char r[20];
    char d[20];
    char y[20];
    uint32_t elapsed;
    int32_t target;
    float l_cm;
    float r_cm;
    float yaw;
    float diff;

    Straight_Status(&elapsed, &target, &l_cm, &r_cm, &yaw, &diff);
    if (SendIRStatus(0U)) return;
    FormatFloat3(l, sizeof(l), l_cm);
    FormatFloat3(r, sizeof(r), r_cm);
    FormatFloat3(d, sizeof(d), diff);
    FormatFloat3(y, sizeof(y), yaw);

    {
        int32_t threshold;
        uint32_t mm, age;
        char mm_text[12];
        int valid;
        if (Straight_UltrasonicStatus(&threshold, &mm, &valid, &age))
        {
            FormatUltrasonicMm(mm_text, sizeof(mm_text), mm);
            snprintf(msg, sizeof(msg),
                     "STRAIGHT,state=%s,mode=UNTILUS,elapsed=%lu,threshold_cm=%ld,us_mm=%s,us_status=%d,us_age_ms=%lu,l_cm=%s,r_cm=%s,diff_cm=%s,yaw=%s\r\n",
                     StraightStateText(Straight_State()), (unsigned long)elapsed,
                     (long)threshold, mm_text, valid, (unsigned long)age,
                     l, r, d, y);
            Cmd_Send(msg);
            return;
        }
    }

    snprintf(msg, sizeof(msg),
             "STRAIGHT,state=%s,elapsed=%lu,target_cm=%ld,l_cm=%s,r_cm=%s,diff_cm=%s,yaw=%s,route_heading=%u,fixed_us=%lu\r\n",
             StraightStateText(Straight_State()),
             (unsigned long)elapsed,
             (long)target,
             l, r, d, y, (unsigned)MotionHeading_IsEnabled(),
             (unsigned long)Straight_FixedSteeringUs());
    Cmd_Send(msg);
}

static void HandleTurnStatus(void)
{
    char msg[160];
    char y[20];
    uint32_t elapsed;
    int32_t radius_cm;
    int32_t target;
    int8_t turn_dir;
    float yaw;

    Turn_Status(&elapsed, &radius_cm, &target, &yaw, &turn_dir);
    FormatFloat3(y, sizeof(y), yaw);

    snprintf(msg, sizeof(msg),
             "TURN,state=%s,dir=%s,elapsed=%lu,r_cm=%ld,target_deg=%ld,yaw=%s\r\n",
             TurnStateText(Turn_State()),
             TurnDirText(turn_dir),
             (unsigned long)elapsed,
             (long)radius_cm,
             (long)target,
             y);
    Cmd_Send(msg);
}

static void SequenceLog_Begin(void)
{
    sequence_log_used = sequence_log_dropped = 0U;
    sequence_log[0] = '\0';
    sequence_log_capture_only = 0U;
    sequence_log_result = "RUNNING";
    sequence_log_active = 1U;
}

/* Called only between commands, never from the live motor-control tick. The
 * extra diagnostics go straight to RAM, without UART transmission or delays. */
static void SequenceLog_CaptureBefore(uint32_t index, const char *command)
{
    char msg[192];
    sequence_log_capture_only = 1U;
    snprintf(msg, sizeof(msg), "LOG,BEFORE,index=%lu,command=%s\r\n",
             (unsigned long)index, command);
    Cmd_Send(msg);
    HandleHeadingStatus();
    sequence_log_capture_only = 0U;
}

static void SequenceLog_CaptureAfter(uint32_t index, pending_motion_t motion)
{
    char msg[180];
    sequence_log_capture_only = 1U;
    snprintf(msg, sizeof(msg), "LOG,AFTER,index=%lu\r\n", (unsigned long)index);
    Cmd_Send(msg);
    if (motion == PENDING_MOTION_BYPASS)
    {
        SendBypassStatus(0U); HandleTurnStatus();
    }
    else if (motion == PENDING_MOTION_LATERAL)
    {
        SendLateralStatus(0U);
        HandleTurnStatus();
    }
    else if (motion == PENDING_MOTION_TURN)
    {
        char target[20], brake_yaw[20], brake_rate[20];
        float yaw, rate;
        HandleTurnStatus();
        Turn_StopMetrics(&yaw, &rate);
        FormatFloat3(target, sizeof(target), Turn_TargetMagnitudeDeg());
        FormatFloat3(brake_yaw, sizeof(brake_yaw), yaw);
        FormatFloat3(brake_rate, sizeof(brake_rate), rate);
        snprintf(msg, sizeof(msg),
            "LOG,TURN_METRICS,target_mag_deg=%s,brake_yaw=%s,brake_rate_dps=%s,servo_us=%lu\r\n",
            target, brake_yaw, brake_rate, (unsigned long)RobotHW_ServoGetUs());
        Cmd_Send(msg);
    }
    else
    {
        HandleStraightStatus();
        /* Save this trace before the next straight resets its live buffer. */
        HandleStraightTrace();
    }
    HandleHeadingStatus();
    sequence_log_capture_only = 0U;
}

static void HandleSequenceLog(void)
{
    char msg[128];
    if (sequence_log_active || AnyMotionRunning())
    {
        Cmd_Reject("ERR,BUSY");
        return;
    }
    snprintf(msg, sizeof(msg),
        "LOG,SEQ,bytes=%lu,dropped_bytes=%lu,result=%s\r\n",
        (unsigned long)sequence_log_used, (unsigned long)sequence_log_dropped,
        sequence_log_result);
    Cmd_Send(msg);
    for (uint32_t offset = 0U; offset < sequence_log_used; )
    {
        uint32_t length = sequence_log_used - offset;
        if (length > 256U) length = 256U;
        Cmd_SendRaw((const uint8_t *)sequence_log + offset, (uint16_t)length);
        offset += length;
        MotionHeading_PollIdle();
    }
    Cmd_SendLine("LOG,SEQ,END");
}

static void HandleMotorStatus(void)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "MOTOR,l=%ld,r=%ld\r\n",
             (long)RobotHW_MotorGetLeftSigned(),
             (long)RobotHW_MotorGetRightSigned());
    Cmd_Send(msg);
}

static void HandleEncoderStatus(void)
{
    char msg[64];
    RobotHW_EncoderSnapshot enc = RobotHW_EncoderSnapshotRead();
    snprintf(msg, sizeof(msg), "ENC,l=%ld,r=%ld\r\n",
             (long)EncoderLeftNormalizedOf(enc),
             (long)EncoderRightNormalizedOf(enc));
    Cmd_Send(msg);
}

static void HandleEncoderCmStatus(void)
{
    char msg[80];
    char l[20];
    char r[20];
    RobotHW_EncoderSnapshot enc = RobotHW_EncoderSnapshotRead();

    FormatFloat3(l, sizeof(l),
                 (float)EncoderLeftNormalizedOf(enc) / LEFT_COUNTS_PER_CM);
    FormatFloat3(r, sizeof(r),
                 (float)EncoderRightNormalizedOf(enc) / RIGHT_COUNTS_PER_CM);
    snprintf(msg, sizeof(msg), "ENC,l_cm=%s,r_cm=%s\r\n", l, r);
    Cmd_Send(msg);
}

static void HandleEncoderRawStatus(void)
{
    char msg[64];
    RobotHW_EncoderSnapshot enc = RobotHW_EncoderSnapshotRead();
    snprintf(msg, sizeof(msg), "ENC,l=%ld,r=%ld\r\n",
             (long)EncoderLeftRawOf(enc),
             (long)EncoderRightRawOf(enc));
    Cmd_Send(msg);
}

static void HandleGlobalStatus(void)
{
    char msg[192];
    char yaw[20];
    char gz[20];
    RobotHW_EncoderSnapshot enc = RobotHW_EncoderSnapshotRead();

    FormatFloat3(yaw, sizeof(yaw), RobotHW_YawGetDeg());
    FormatFloat3(gz, sizeof(gz), RobotHW_GyroGetLastZDps());
    snprintf(msg, sizeof(msg),
             "STATUS,motor_l=%ld,motor_r=%ld,steer=%lu,enc_l=%ld,enc_r=%ld,yaw=%s,gyro_z=%s,uptime=%lu\r\n",
             (long)RobotHW_MotorGetLeftSigned(),
             (long)RobotHW_MotorGetRightSigned(),
             (unsigned long)RobotHW_ServoGetUs(),
             (long)EncoderLeftNormalizedOf(enc),
             (long)EncoderRightNormalizedOf(enc),
             yaw,
             gz,
             (unsigned long)HAL_GetTick());
    Cmd_Send(msg);
}

/* =========================================================================
 * AUTO SERVO-CENTRE CALIBRATION   (CALIBRATE,CENTER)
 * =========================================================================
 * The "straight" servo pulse is a hand-set value (SERVO_CENTER_US) and drifts
 * with the mechanics. This finds it automatically: drive several short forward
 * runs OPEN-LOOP (no gyro steering, no trim) at different servo pulses, measure
 * how much the gyro says the robot curved on each, then linear-fit the pulse
 * that yields zero yaw drift and apply it live (RobotHW_ServoSetCenterUs).
 *
 * NOTE: needs ~1-1.5 m of clear space ahead (it drives forward and reverses
 * back between trials). If the front steering has mechanical play, the result
 * will be noisy - tighten the linkage first.
 */
#define CAL_RUN_CM        35.0f    /* forward distance measured per trial */
#define CAL_PWM           3000U    /* modest open-loop drive speed */
#define CAL_SWEEP_US      80       /* sweep +/- this far around the current centre */
#define CAL_N_TRIALS      7        /* pulses sampled across the sweep */
#define CAL_PERIOD_MS     20U      /* yaw/encoder sampling period */
#define CAL_GYRO_ALPHA    0.10f
#define CAL_GYRO_DEADBAND 0.08f
#define CAL_MAX_MS        4000U    /* per-run safety timeout */
#define CAL_SETTLE_MS     200U     /* let the servo reach the pulse before driving */
#define CAL_SYNC_K        1.5f     /* drive-wheel balance gain (PWM per count) */
#define CAL_SYNC_MAX      400      /* max balance correction (PWM) */

static float cal_absf(float x) { return (x < 0.0f) ? -x : x; }

/* Drive ~CAL_RUN_CM at a fixed servo `pulse` while integrating yaw. The two
 * DRIVE wheels are balanced by encoder count so motor imbalance doesn't curve
 * the path - leaving the gyro yaw to reflect the SERVO steering alone (this
 * also matches real straight drives, which run the wheel-sync). dir=+1 forward
 * (measured), dir=-1 reverse (repositioning). Returns net gyro yaw (deg). */
static float Cal_Drive(uint32_t pulse, int8_t dir)
{
    RobotHW_ServoSetUsTakeup(pulse);   /* load backlash consistently (incl. settle) */
    RobotHW_YawReset();

    RobotHW_EncoderSnapshot prev = RobotHW_EncoderSnapshotRead();
    float dist = 0.0f;
    int32_t left_total = 0, right_total = 0;
    uint32_t t0 = HAL_GetTick();
    uint32_t prev_t = t0;

    if (dir > 0) RobotHW_MotorSetForward(CAL_PWM, CAL_PWM);
    else         RobotHW_MotorSetBackward(CAL_PWM, CAL_PWM);

    while (dist < CAL_RUN_CM && (HAL_GetTick() - t0) < CAL_MAX_MS)
    {
        uint32_t now = HAL_GetTick();
        if (now - prev_t >= CAL_PERIOD_MS)
        {
            RobotHW_EncoderDelta d = RobotHW_EncoderDeltaRead(&prev);
            left_total  += d.left_delta;
            right_total += d.right_delta;
            dist += (cal_absf((float)d.left_delta) / LEFT_COUNTS_PER_CM +
                     cal_absf((float)d.right_delta) / RIGHT_COUNTS_PER_CM) * 0.5f;
            (void)RobotHW_GyroUpdateYaw((float)(now - prev_t) / 1000.0f,
                                        CAL_GYRO_ALPHA, CAL_GYRO_DEADBAND);

            /* balance drive wheels by travelled distance (counts) */
            int32_t imb = (int32_t)(cal_absf((float)left_total) -
                                    cal_absf((float)right_total));
            int32_t corr = (int32_t)(CAL_SYNC_K * (float)imb);
            if (corr >  CAL_SYNC_MAX) corr =  CAL_SYNC_MAX;
            if (corr < -CAL_SYNC_MAX) corr = -CAL_SYNC_MAX;
            int32_t lp = (int32_t)CAL_PWM - corr;
            int32_t rp = (int32_t)CAL_PWM + corr;
            if (lp < 0) lp = 0;
            if (rp < 0) rp = 0;
            if (dir > 0) RobotHW_MotorSetForward((uint32_t)lp, (uint32_t)rp);
            else         RobotHW_MotorSetBackward((uint32_t)lp, (uint32_t)rp);

            prev_t = now;
        }
        HAL_Delay(1U);
    }

    RobotHW_MotorBrakeBegin();
    HAL_Delay(MOTOR_BRAKE_PULSE_MS);
    RobotHW_MotorBrakeRelease();
    RobotHW_MotorStop();
    return RobotHW_YawGetDeg();
}

static void HandleCalibrateCenter(void)
{
    if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
    if (!RobotHW_GyroInit()) { Cmd_Reject("ERR,NO_DEVICE"); return; }
    Cmd_SendLine("CAL,START,center-calibration (keep ~1.5m clear ahead)");
    RobotHW_GyroCalibrate(GYRO_CALIB_WARMUP_MS, GYRO_CALIB_DELAY_MS, GYRO_CALIB_SAMPLES);

    uint32_t c0 = RobotHW_ServoCenterUs();
    int step = (CAL_N_TRIALS > 1) ? (2 * CAL_SWEEP_US) / (CAL_N_TRIALS - 1) : 0;

    /* least-squares accumulators for yaw = a + b*pulse, plus crossing check */
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    float yaw_min = 1e9f, yaw_max = -1e9f;
    int n = 0;

    for (int i = 0; i < CAL_N_TRIALS; i++)
    {
        /* Sweep candidate CENTRES; drive at centre + the forward trim, i.e. the
         * exact pulse a real STRAIGHT would use - so the fitted zero is a centre
         * value that makes the real drive straight (not a raw pulse offset by
         * the trim, which was the bug). */
        uint32_t center = (uint32_t)((int)c0 - CAL_SWEEP_US + i * step);
        uint32_t drive  = (uint32_t)((int)center + STRAIGHT_FORWARD_TRIM_US);
        float yaw = Cal_Drive(drive, +1);          /* forward, measured */

        char ys[20];
        char msg[88];
        FormatFloat3(ys, sizeof(ys), yaw);
        snprintf(msg, sizeof(msg), "CAL,trial=%d,center=%lu,pulse=%lu,yaw=%s\r\n",
                 i + 1, (unsigned long)center, (unsigned long)drive, ys);
        Cmd_Send(msg);

        float x = (float)center;                   /* fit yaw vs candidate CENTRE */
        sx += x; sy += yaw; sxx += x * x; sxy += x * yaw; n++;
        if (yaw < yaw_min) yaw_min = yaw;
        if (yaw > yaw_max) yaw_max = yaw;

        (void)Cal_Drive(drive, -1);                /* reverse back to ~start */
        HAL_Delay(200U);
    }

    RobotHW_ServoCenter();   /* park at the current centre while we decide */

    /* Only trust a result if the drift actually changed sign across the sweep -
     * otherwise the true centre is outside the swept range (widen it) or the
     * steering isn't responding (backlash). Never extrapolate a guess. */
    if (!(yaw_min < 0.0f && yaw_max > 0.0f))
    {
        Cmd_Reject("ERR,CAL_NO_CROSSING");   /* no straight point seen; see CAL,trial lines */
        return;
    }

    float denom = ((float)n * sxx - sx * sx);
    float b = (cal_absf(denom) > 1e-6f) ? ((float)n * sxy - sx * sy) / denom : 0.0f;
    if (cal_absf(b) < 1e-4f)
    {
        Cmd_Reject("ERR,CAL_FLAT");          /* no usable yaw-vs-pulse slope */
        return;
    }
    float a = (sy - b * sx) / (float)n;
    float best = -a / b;                      /* pulse where fitted yaw == 0 */

    /* with a real crossing this sits inside the sweep; clamp only as a guard */
    float lo = (float)c0 - (float)CAL_SWEEP_US;
    float hi = (float)c0 + (float)CAL_SWEEP_US;
    if (best < lo) best = lo;
    if (best > hi) best = hi;

    uint32_t best_us = (uint32_t)(best + 0.5f);
    RobotHW_ServoSetCenterUs(best_us);        /* apply live for this session */
    RobotHW_ServoCenter();

    char msg[112];
    snprintf(msg, sizeof(msg),
             "CAL,DONE,center=%lu,was=%lu (applied; set SERVO_CENTER_US=%lu to persist)\r\n",
             (unsigned long)best_us, (unsigned long)c0, (unsigned long)best_us);
    Cmd_Send(msg);
}

static void HandleIRGet(uint8_t side)
{
    if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
    IRSensorDistanceReading reading;
    int rc = IRSensor_ReadDistance(&reading, 1U);
    char msg[128], cm[24];
    const char *name = side ? "RIGHTIR" : "LEFTIR";
    if (rc != 0)
    {
        snprintf(msg,sizeof(msg),"%s,cm=INVALID,raw=NA,mv=NA,status=ERROR,rc=%d\r\n",name,rc);
        Cmd_Send(msg); return;
    }
    IRSensorRangeStatus range = side ? reading.right_status : reading.left_status;
    uint32_t distance = side ? reading.right_cm_x10 : reading.left_cm_x10;
    if (range == IR_RANGE_VALID)
        snprintf(cm,sizeof(cm),"%lu.%lu",(unsigned long)(distance/10U),(unsigned long)(distance%10U));
    else snprintf(cm,sizeof(cm),"%s",range == IR_RANGE_NEAR ? "NEAR" : range == IR_RANGE_FAR ? "FAR" : "INVALID");
    snprintf(msg,sizeof(msg),"%s,cm=%s,raw=%u,mv=%u,status=%s,rc=0\r\n",name,cm,
        (unsigned)(side ? reading.adc.right_raw : reading.adc.left_raw),
        (unsigned)(side ? reading.adc.right_mv : reading.adc.left_mv),IRSensor_RangeText(range));
    Cmd_Send(msg);
}

static void DispatchLine(char *line)
{
    char *tok0;
    char *tok1;
    char *tok2;
    char *tok3;
    char *tok4;
    char *tok5;
    char *tok6;
    char *tok7;

    if (line == NULL || line[0] == '\0') return;

    /* strtok collapses empty fields: reject them before tokenization. */
    if (line[0] == ',' || line[strlen(line) - 1U] == ',' || strstr(line, ",,") != NULL)
    {
        Cmd_Reject("ERR,BAD_ARGS");
        return;
    }
    /* Keep display bit-banging out of the running motion/STOP path. */
    if (!AnyMotionRunning()) Oled_ShowCommand(line);

    tok0 = strtok(line, ",");
    tok1 = strtok(NULL, ",");
    tok2 = strtok(NULL, ",");
    tok3 = strtok(NULL, ",");
    tok4 = strtok(NULL, ",");
    tok5 = strtok(NULL, ",");
    tok6 = strtok(NULL, ",");
    tok7 = strtok(NULL, ",");

    if (tok0 == NULL) return;

    if (strcmp(tok0, "PING") == 0 && tok1 == NULL)
    {
        Cmd_SendLine("PONG");
    }
    else if (strcmp(tok0, "HELLO") == 0 && tok1 == NULL)
    {
        Cmd_SendLine("HELLO,mdp21_v8");
    }
    else if (strcmp(tok0, "CALIBRATE") == 0 && tok1 != NULL &&
             strcmp(tok1, "CENTER") == 0 && tok2 == NULL)
    {
        HandleCalibrateCenter();
    }
    else if (strcmp(tok0, "SERVO") == 0 && tok1 != NULL &&
             strcmp(tok1, "CENTER") == 0 && tok2 != NULL && tok3 == NULL)
    {
        /* Manually set the straight-ahead servo centre (us) and apply it live,
         * for bisecting the centre by hand. Persist by setting SERVO_CENTER_US. */
        int32_t us;
        if (!ParseIntStrict(tok2, &us) ||
            us <= (int32_t)CMD_SERVO_LEFT_US || us >= (int32_t)CMD_SERVO_RIGHT_US)
        {
            Cmd_Reject("ERR,BAD_VALUE");
        }
        else
        {
            RobotHW_ServoSetCenterUs((uint32_t)us);
            RobotHW_ServoCenter();
            char msg[48];
            snprintf(msg, sizeof(msg), "OK,SERVO,CENTER,%ld\r\n", (long)us);
            Cmd_Send(msg);
        }
    }
    else if (strcmp(tok0,"LEFTIR") == 0 || strcmp(tok0,"RIGHTIR") == 0)
    {
        if (tok1 && strcmp(tok1,"GET") == 0 && !tok2)
            HandleIRGet(strcmp(tok0,"RIGHTIR") == 0);
        else Cmd_Reject("ERR,BAD_ARGS");
    }
    else if (strcmp(tok0, "US") == 0 && tok1 != NULL &&
             strcmp(tok1, "GET") == 0 && tok2 == NULL)
    {
        /* Static front-ultrasonic read, NO motion. One measurement; replies
         * US,mm=<sensor-to-object mm>,status=<rc>. mm=0 means no echo / out of
         * range (nothing in front) - the Pi uses that to skip the re-approach.
         * mm is sensor->object, also used directly by UNTILUS. */
        uint32_t pulse_us = 0U, dist_mm = 0U;
        int rc;
        if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
        if (UltrasonicSensor_Init() != 0) { Cmd_Reject("ERR,NO_DEVICE"); return; }
        rc = UltrasonicSensor_Read(&pulse_us, &dist_mm);
        char msg[48];
        snprintf(msg, sizeof(msg), "US,mm=%lu,status=%d\r\n",
                 (unsigned long)((rc == 0) ? dist_mm : 0U), rc);
        Cmd_Send(msg);
    }
    else if (strcmp(tok0, "SEQ") == 0)
    {
        if (tok1 != NULL && strcmp(tok1, "LOG") == 0 && tok2 == NULL)
            HandleSequenceLog();
        else Cmd_Reject("ERR,BAD_ARGS");
    }
    else if (strcmp(tok0, "UPTIME") == 0 && tok1 == NULL)
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "UPTIME,ms=%lu\r\n", (unsigned long)HAL_GetTick());
        Cmd_Send(msg);
    }
    else if (strcmp(tok0, "LOOP") == 0)
    {
        if (tok1 != NULL && strcmp(tok1, "ONCE") == 0 && tok2 == NULL)
        {
            char msg[80];
            const char *failure;
            uint32_t failure_index = 0U;
            uint32_t count = (uint32_t)(sizeof(loop_once_commands) / sizeof(loop_once_commands[0]));
            if (AnyMotionRunning() || sequence_mode)
            {
                Cmd_Reject("ERR,BUSY");
                return;
            }
            snprintf(msg, sizeof(msg), "OK,loop=ONCE,count=%lu\r\n",
                     (unsigned long)count);
            Cmd_Send(msg);
            failure = CommandInterface_RunSequenceRows(loop_once_commands,
                count, 0U, CMD_LOOP_COMMAND_GAP_MS, 0U, 0U, 1U, &failure_index, NULL);
            if (failure)
                snprintf(msg, sizeof(msg), "DONE,LOOP,state=%s,index=%lu,count=%lu\r\n",
                         failure, (unsigned long)failure_index, (unsigned long)count);
            else
                snprintf(msg, sizeof(msg), "DONE,LOOP,state=DONE,count=%lu\r\n",
                         (unsigned long)count);
            Cmd_Send(msg);
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "FAST") == 0)
    {
        const char *const *commands = NULL;
        const char *route = NULL;
        uint32_t count = 0U;
        uint32_t failure_index = 0U;
        const char *failure;
        char msg[112];
        if (tok1 == NULL || tok2 != NULL)
        {
            Cmd_Reject("ERR,BAD_ARGS");
            return;
        }
        if (strcmp(tok1, "LEFT1") == 0)
        {
            commands = fast_left1_commands;
            route = "LEFT1";
            count = sizeof(fast_left1_commands) / sizeof(fast_left1_commands[0]);
        }
        else if (strcmp(tok1, "LEFT2") == 0)
        {
            commands = fast_left2_commands;
            route = "LEFT2";
            count = sizeof(fast_left2_commands) / sizeof(fast_left2_commands[0]);
        }
        else if (strcmp(tok1, "RIGHT1") == 0)
        {
            commands = fast_right1_commands;
            route = "RIGHT1";
            count = sizeof(fast_right1_commands) / sizeof(fast_right1_commands[0]);
        }
        else if (strcmp(tok1, "RIGHT2") == 0)
        {
            commands = fast_right2_commands;
            route = "RIGHT2";
            count = sizeof(fast_right2_commands) / sizeof(fast_right2_commands[0]);
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
            return;
        }
        if (AnyMotionRunning() || (sequence_mode && !sequence_row_dispatch))
        {
            Cmd_Reject("ERR,BUSY");
            return;
        }
        snprintf(msg, sizeof(msg), "OK,fast=%s,count=%lu\r\n",
                 route, (unsigned long)count);
        Cmd_Send(msg);
        /* No startup delay or extra inter-command gap for these routes.
         * Normal motion preparation still follows robot_tuning.h. */
        /* Consume internal authorization before polling UART in the group. */
        sequence_row_dispatch = 0U;
        failure = CommandInterface_RunSequenceRows(commands, count,
            0U, 0U, 0U, 0U, 1U, &failure_index, NULL);
        sequence_row_failure = failure;
        if (failure)
            snprintf(msg, sizeof(msg), "DONE,FAST,route=%s,state=%s,index=%lu,count=%lu\r\n",
                     route, failure, (unsigned long)failure_index, (unsigned long)count);
        else
            snprintf(msg, sizeof(msg), "DONE,FAST,route=%s,state=DONE,count=%lu\r\n",
                     route, (unsigned long)count);
        Cmd_Send(msg);
    }
    else if (strcmp(tok0, "HEADING") == 0)
    {
        if (tok1 != NULL && strcmp(tok1, "STATUS") == 0 && tok2 == NULL)
            HandleHeadingStatus();
        else if (tok1 != NULL && tok2 == NULL &&
                 (strcmp(tok1, "RESET") == 0 || strcmp(tok1, "ON") == 0 ||
                  strcmp(tok1, "OFF") == 0))
        {
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            if (strcmp(tok1, "RESET") == 0) MotionHeading_Reset();
            else MotionHeading_SetEnabled(strcmp(tok1, "ON") == 0);
            RobotHW_YawReset();
            gyro_last_tick = HAL_GetTick();
            Cmd_SendLine("OK");
        }
        else Cmd_Reject("ERR,BAD_ARGS");
    }
    else if (strcmp(tok0, "STOP") == 0 && tok1 == NULL)
    {
        Lateral_Stop();
        Bypass_Stop();
        Straight_Stop();
        Turn_Stop();
        MotionHeading_Reset();
        RobotHW_MotorStop();
        RobotHW_ServoCenter();
        Cmd_SendLine("OK");
    }
    else if (strcmp(tok0, "STRAIGHT") == 0)
    {
        uint8_t explicit_direction = tok1 &&
            (strcmp(tok1, "FORWARD") == 0 || strcmp(tok1, "BACKWARD") == 0);
        const char *ir_side = explicit_direction ? tok2 : tok1;
        const char *ir_op = explicit_direction ? tok3 : tok2;
        const char *ir_value = explicit_direction ? tok4 : tok3;
        const char *ir_extra = explicit_direction ? tok5 : tok4;
        int8_t ir_direction = tok1 && strcmp(tok1, "BACKWARD") == 0 ? -1 : 1;
        if (ir_side && (strcmp(ir_side, "LEFTIR") == 0 || strcmp(ir_side, "RIGHTIR") == 0) &&
            ir_op && ir_extra == NULL)
        {
            int32_t cm;
            int rc;
            straight_ir_comparison_t comparison = STRAIGHT_IR_GT;
            if (ir_value != NULL)
            {
                if (strcmp(ir_op, "LT") == 0) comparison = STRAIGHT_IR_LT;
                else if (strcmp(ir_op, "RISING") == 0) comparison = STRAIGHT_IR_RISING;
                else if (strcmp(ir_op, "FALLING") == 0) comparison = STRAIGHT_IR_FALLING;
                else if (strcmp(ir_op, "GT") != 0) { Cmd_Reject("ERR,BAD_ARGS"); return; }
            }
            if (!ParseIntStrict(ir_value != NULL ? ir_value : ir_op, &cm)) { Cmd_Reject("ERR,BAD_ARGS"); return; }
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            rc = Straight_StartUntilIRDirectionCm(cm, strcmp(ir_side, "RIGHTIR") == 0, comparison, ir_direction);
            if (rc == 0)
            {
                char msg[96];
                pending_motion = PENDING_MOTION_STRAIGHT;
                pending_straight_cm = 0;
                snprintf(msg, sizeof(msg), "OK,mode=%s,op=%s,threshold_cm=%ld,dir=%s\r\n", ir_side,
                         IRComparisonText(comparison), (long)cm,
                         ir_direction > 0 ? "FORWARD" : "BACKWARD");
                Cmd_Send(msg);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else if ((tok1 && strcmp(tok1, "UNTILUS") == 0 && tok2 && tok3 == NULL) ||
                 (tok1 && (strcmp(tok1, "FORWARD") == 0 || strcmp(tok1, "BACKWARD") == 0) &&
                  tok2 && strcmp(tok2, "UNTILUS") == 0 && tok3 && tok4 == NULL))
        {
            int32_t cm;
            int rc;
            int8_t direction = strcmp(tok1, "BACKWARD") == 0 ? -1 : 1;
            const char *distance_arg = strcmp(tok1, "UNTILUS") == 0 ? tok2 : tok3;
            if (!ParseIntStrict(distance_arg, &cm))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (AnyMotionRunning())
            {
                Cmd_Reject("ERR,BUSY");
                return;
            }
            /* Compare directly with the sensor distance reported by US,GET,
             * in either direction; no installation offset is added. */
            rc = Straight_StartUntilUltrasonicDirectionCm(cm, direction);
            if (rc == 0)
            {
                char msg[96];
                pending_motion = PENDING_MOTION_UNTILUS;
                snprintf(msg, sizeof(msg), "OK,mode=UNTILUS,threshold_cm=%ld,dir=%s\r\n", (long)cm,
                         direction > 0 ? "FORWARD" : "BACKWARD");
                Cmd_Send(msg);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else if (tok1 != NULL && strcmp(tok1, "CAL") == 0 &&
                 tok2 != NULL && tok3 != NULL && tok4 == NULL)
        {
            int32_t cm, servo_us;
            int rc;
            if (!ParseIntStrict(tok2, &cm) || !ParseIntStrict(tok3, &servo_us))
            { Cmd_Reject("ERR,BAD_ARGS"); return; }
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            rc = Straight_StartCalibrationCm(cm, servo_us);
            if (rc == 0)
            {
                char msg[96];
                pending_motion = PENDING_MOTION_STRAIGHT;
                pending_straight_cm = cm;
                snprintf(msg, sizeof(msg), "OK,mode=CAL,target_cm=%ld,servo_us=%ld,gyro_steering=0\r\n",
                         (long)cm, (long)servo_us);
                Cmd_Send(msg);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else if (tok1 && strcmp(tok1, "GOCM") == 0 && tok2 && tok3 && tok4 && tok5 && !tok7)
        {
            int32_t cm, threshold;
            straight_ir_comparison_t op;
            if (strcmp(tok3,"LEFTIR") && strcmp(tok3,"RIGHTIR")) { Cmd_Reject("ERR,BAD_ARGS"); return; }
            if (!strcmp(tok4,"GT")) op=STRAIGHT_IR_GT;
            else if (!strcmp(tok4,"LT")) op=STRAIGHT_IR_LT;
            else if (!strcmp(tok4,"RISING")) op=STRAIGHT_IR_RISING;
            else if (!strcmp(tok4,"FALLING")) op=STRAIGHT_IR_FALLING;
            else { Cmd_Reject("ERR,BAD_ARGS"); return; }
            if (!ParseIntStrict(tok2,&cm) || !ParseIntStrict(tok5,&threshold)) { Cmd_Reject("ERR,BAD_ARGS"); return; }
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            if (tok6 && (strcmp(tok6,"LATERAL") || (op != STRAIGHT_IR_RISING && op != STRAIGHT_IR_FALLING)))
            { Cmd_Reject("ERR,BAD_ARGS"); return; }
            int rc = tok6 ? Lateral_StartAfterIREdge(cm,!strcmp(tok3,"RIGHTIR"),op,threshold)
                          : Straight_StartCmThenIR(cm,!strcmp(tok3,"RIGHTIR"),op,threshold);
            if (!rc) {
                char msg[160]; pending_motion=tok6 ? PENDING_MOTION_LATERAL : PENDING_MOTION_STRAIGHT; pending_straight_cm=cm;
                snprintf(msg,sizeof(msg),"OK,mode=%s,gocm_cm=%ld,ir=%s,op=%s,threshold_cm=%ld,dir=%s\r\n",
                    tok6 ? "GOCM_IR_LATERAL" : "GOCM_IR",(long)cm,tok3,IRComparisonText(op),(long)threshold,cm<0?"BACKWARD":"FORWARD");
                Cmd_Send(msg);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else if (tok1 != NULL && strcmp(tok1, "GOCM") == 0 && tok2 != NULL && tok3 == NULL)
        {
            int32_t cm;
            int rc;

            if (!ParseIntStrict(tok2, &cm))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (Turn_State() == TURN_RUNNING)
            {
                Cmd_Reject("ERR,BUSY");
                return;
            }

            rc = Straight_StartCm(cm);
            if (rc == 0)
            {
                char msg[64];
                pending_motion = PENDING_MOTION_STRAIGHT;
                pending_straight_cm = cm;
                snprintf(msg, sizeof(msg), "OK,target_cm=%ld\r\n", (long)cm);
                Cmd_Send(msg);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else if (tok1 != NULL && strcmp(tok1, "STOP") == 0 && tok2 == NULL)
        {
            uint8_t aborted = (Straight_State() == STRAIGHT_RUNNING) ? 1U : 0U;
            Lateral_Stop();
            Bypass_Stop();
            Straight_Stop();
            if (!AnyMotionRunning()) MotionHeading_Reset();
            {
                char msg[32];
                snprintf(msg, sizeof(msg), "OK,aborted=%u\r\n", (unsigned)aborted);
                Cmd_Send(msg);
            }
        }
        else if (tok1 != NULL && strcmp(tok1, "STATUS") == 0 && tok2 == NULL)
        {
            HandleStraightStatus();
        }
        else if (tok1 != NULL && strcmp(tok1, "TRACE") == 0 && tok2 == NULL)
        {
            HandleStraightTrace();
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "BYPASS") == 0)
    {
        if (tok1 && strcmp(tok1, "STATUS") == 0 && tok2 == NULL) SendBypassStatus(0U);
        else if (tok1 && strcmp(tok1, "STOP") == 0 && tok2 == NULL)
        {
            Bypass_Stop();
            if (!AnyMotionRunning()) MotionHeading_Reset();
            Cmd_SendLine("OK");
        }
        else if (tok1 && (strcmp(tok1, "LEFT") == 0 || strcmp(tok1, "RIGHT") == 0) &&
                 tok2 && tok3 == NULL)
        {
            int32_t cm;
            if (!ParseIntStrict(tok2, &cm)) { Cmd_Reject("ERR,BAD_ARGS"); return; }
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            int rc = Bypass_Start(strcmp(tok1, "RIGHT") == 0, cm);
            if (rc == 0) {
                pending_motion = PENDING_MOTION_BYPASS;
                Cmd_SendLine("OK,BYPASS"); SendBypassStatus(0U);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else Cmd_Reject("ERR,BAD_ARGS");
    }
    else if (strcmp(tok0, "LATERAL") == 0)
    {
        if (tok1 && strcmp(tok1, "STATUS") == 0 && tok2 == NULL)
            SendLateralStatus(0U);
        else if (tok1 && strcmp(tok1, "STOP") == 0 && tok2 == NULL)
        {
            Lateral_Stop();
            Bypass_Stop();
            if (!AnyMotionRunning()) MotionHeading_Reset();
            Cmd_SendLine("OK");
        }
        else if (tok1 && (strcmp(tok1, "LEFT") == 0 || strcmp(tok1, "RIGHT") == 0) &&
                 tok2 && tok3 == NULL)
        {
            int32_t cm = 0;
            uint8_t from_ir = strcmp(tok2, "IR") == 0;
            if (!from_ir && (!ParseIntStrict(tok2, &cm) || cm <= 0 ||
                cm > CMD_LEFT_SOFT_MIN_RADIUS_CM + CMD_RIGHT_MIN_RADIUS_CM))
            { Cmd_Reject("ERR,BAD_VALUE"); return; }
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            int rc = Lateral_Start(strcmp(tok1, "RIGHT") == 0, from_ir, (uint32_t)cm * 10U);
            if (rc == 0)
            {
                pending_motion = PENDING_MOTION_LATERAL;
                Cmd_SendLine("OK,LATERAL");
                SendLateralStatus(0U);
            }
            else if (rc == -5) Cmd_Reject("ERR,IR_MEASUREMENT_INVALID");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else Cmd_Reject("ERR,BAD_ARGS");
    }
    else if (strcmp(tok0, "TURN") == 0)
    {
        if (tok1 != NULL &&
            (strcmp(tok1, "LEFT") == 0 || strcmp(tok1, "RIGHT") == 0) &&
            tok2 != NULL && tok3 != NULL)
        {
            int32_t radius_cm;
            int32_t angle;
            int8_t turn_dir = (strcmp(tok1, "RIGHT") == 0) ? 1 : -1;
            int rc;

            if (!ParseKeyInt(tok2, "R", &radius_cm) ||
                !ParseKeyInt(tok3, "A", &angle))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (AnyMotionRunning())
            {
                Cmd_Reject("ERR,BUSY");
                return;
            }

            rc = Turn_StartRadiusAngle(radius_cm, angle, turn_dir);
            if (rc == 0)
            {
                char msg[64];
                pending_motion = PENDING_MOTION_TURN;
                pending_turn_radius_cm = radius_cm;
                pending_turn_angle_deg = angle;
                pending_turn_dir = turn_dir;
                snprintf(msg, sizeof(msg), "OK,dir=%s,r_cm=%ld,a=%ld\r\n",
                         TurnDirText(turn_dir), (long)radius_cm, (long)angle);
                Cmd_Send(msg);
            }
            else if (rc == -1) Cmd_Reject("ERR,NO_DEVICE");
            else if (rc == -2) Cmd_Reject("ERR,BAD_VALUE");
            else if (rc == -4) Cmd_Reject("ERR,NOT_STILL");
            else Cmd_Reject("ERR,BUSY");
        }
        else if (tok1 != NULL && strcmp(tok1, "STOP") == 0 && tok2 == NULL)
        {
            uint8_t aborted = (Turn_State() == TURN_RUNNING) ? 1U : 0U;
            Lateral_Stop();
            Bypass_Stop();
            Turn_Stop();
            if (!AnyMotionRunning()) MotionHeading_Reset();
            {
                char msg[32];
                snprintf(msg, sizeof(msg), "OK,aborted=%u\r\n", (unsigned)aborted);
                Cmd_Send(msg);
            }
        }
        else if (tok1 != NULL && strcmp(tok1, "STATUS") == 0 && tok2 == NULL)
        {
            HandleTurnStatus();
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "MOTOR") == 0)
    {
        if (AnyMotionRunning())
        {
            Cmd_Reject("ERR,BUSY");
            return;
        }

        if (tok1 != NULL && strcmp(tok1, "L") == 0 &&
            tok2 != NULL && tok3 == NULL)
        {
            int32_t pwm;
            if (!ParseIntStrict(tok2, &pwm))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (!PwmInRange(pwm, RobotHW_MotorLeftMax()))
            {
                Cmd_Reject("ERR,BAD_VALUE");
                return;
            }
            MotionHeading_Reset();
            RobotHW_MotorSetSigned(pwm, RobotHW_MotorGetRightRequested());
            Cmd_SendLine("OK");
        }
        else if (tok1 != NULL && strcmp(tok1, "R") == 0 &&
                 tok2 != NULL && tok3 == NULL)
        {
            int32_t pwm;
            if (!ParseIntStrict(tok2, &pwm))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (!PwmInRange(pwm, RobotHW_MotorRightMax()))
            {
                Cmd_Reject("ERR,BAD_VALUE");
                return;
            }
            MotionHeading_Reset();
            RobotHW_MotorSetSigned(RobotHW_MotorGetLeftRequested(), pwm);
            Cmd_SendLine("OK");
        }
        else if (tok1 != NULL && strcmp(tok1, "B") == 0 &&
                 tok2 != NULL && tok3 != NULL)
        {
            int32_t left_pwm;
            int32_t right_pwm;
            if (!ParseIntStrict(tok2, &left_pwm) ||
                !ParseIntStrict(tok3, &right_pwm))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (!PwmInRange(left_pwm, RobotHW_MotorLeftMax()) ||
                !PwmInRange(right_pwm, RobotHW_MotorRightMax()))
            {
                Cmd_Reject("ERR,BAD_VALUE");
                return;
            }
            MotionHeading_Reset();
            RobotHW_MotorSetSigned(left_pwm, right_pwm);
            Cmd_SendLine("OK");
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "ENC") == 0)
    {
        if (tok1 != NULL && strcmp(tok1, "RESET") == 0 && tok2 == NULL)
        {
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            MotionHeading_Reset();
            RobotHW_EncoderReset();
            Cmd_SendLine("OK");
        }
        else if (tok1 != NULL && strcmp(tok1, "GET") == 0 && tok2 == NULL)
        {
            HandleEncoderStatus();
        }
        else if (tok1 != NULL && strcmp(tok1, "GET") == 0 &&
                 tok2 != NULL && strcmp(tok2, "CM") == 0 && tok3 == NULL)
        {
            HandleEncoderCmStatus();
        }
        else if (tok1 != NULL && strcmp(tok1, "GET") == 0 &&
                 tok2 != NULL && strcmp(tok2, "RAW") == 0 && tok3 == NULL)
        {
            HandleEncoderRawStatus();
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "STEER") == 0)
    {
        if (tok1 != NULL && strcmp(tok1, "US") == 0 &&
            tok2 != NULL && tok3 == NULL)
        {
            int32_t pulse_us;

            if (!ParseIntStrict(tok2, &pulse_us))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            if (AnyMotionRunning())
            {
                Cmd_Reject("ERR,BUSY");
                return;
            }
            MotionHeading_Reset();
            RobotHW_ServoSetUs((uint32_t)pulse_us);
            Cmd_SendLine("OK");
        }
        else if (tok1 != NULL && strcmp(tok1, "CENTER") == 0 && tok2 == NULL)
        {
            if (AnyMotionRunning())
            {
                Cmd_Reject("ERR,BUSY");
                return;
            }

            MotionHeading_Reset();
            RobotHW_ServoCenter();
            Cmd_SendLine("OK");
        }
        else if (tok1 != NULL && strcmp(tok1, "GET") == 0 && tok2 == NULL)
        {
            char msg[64];
            snprintf(msg, sizeof(msg), "STEER,us=%lu,center=%lu\r\n",
                     (unsigned long)RobotHW_ServoGetUs(),
                     (unsigned long)RobotHW_ServoCenterUs());
            Cmd_Send(msg);
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "GYRO") == 0)
    {
        if (AnyMotionRunning() && (tok1 == NULL || strcmp(tok1, "OFFSET") != 0))
        {
            Cmd_Reject("ERR,BUSY");
            return;
        }
        if (tok1 != NULL && strcmp(tok1, "INIT") == 0 && tok2 == NULL)
        {
            MotionHeading_Reset();
            if (EnsurePollingGyro()) Cmd_SendLine("OK");
            else Cmd_Reject("ERR,NO_DEVICE");
        }
        else if (tok1 != NULL && strcmp(tok1, "CALIB") == 0 && tok2 == NULL)
        {
            char msg[64];
            MotionHeading_Reset();
            if (!EnsurePollingGyro())
            {
                Cmd_Reject("ERR,NO_DEVICE");
                return;
            }
            if (!RobotHW_GyroCalibrate(GYRO_CALIB_WARMUP_MS,
                                       GYRO_CALIB_DELAY_MS,
                                       GYRO_CALIB_SAMPLES))
            {
                Cmd_Reject("ERR,NO_DEVICE");
                return;
            }
            RobotHW_YawReset();
            snprintf(msg, sizeof(msg), "OK,offset=%ld\r\n",
                     (long)RobotHW_GyroGetOffset());
            Cmd_Send(msg);
        }
        else if (tok1 != NULL && strcmp(tok1, "Z") == 0 && tok2 == NULL)
        {
            char msg[80];
            char z[20];
            int16_t raw;

            if (!EnsurePollingGyro())
            {
                Cmd_Reject("ERR,NO_DEVICE");
                return;
            }
            raw = RobotHW_GyroReadZRaw();
            (void)RobotHW_GyroUpdateYaw(0.0f, CMD_GYRO_ALPHA, CMD_GYRO_DEADBAND_DPS);
            FormatFloat3(z, sizeof(z), RobotHW_GyroGetLastZDps());
            snprintf(msg, sizeof(msg), "GYRO,z=%s,raw=%ld\r\n",
                     z, (long)raw);
            Cmd_Send(msg);
        }
        else if (tok1 != NULL && strcmp(tok1, "OFFSET") == 0 && tok2 == NULL)
        {
            char msg[48];
            snprintf(msg, sizeof(msg), "GYRO,offset=%ld\r\n",
                     (long)RobotHW_GyroGetOffset());
            Cmd_Send(msg);
        }
        else if (tok1 != NULL && strcmp(tok1, "SET_OFFSET") == 0 &&
                 tok2 != NULL && tok3 == NULL)
        {
            int32_t offset;
            if (!ParseIntStrict(tok2, &offset))
            {
                Cmd_Reject("ERR,BAD_ARGS");
                return;
            }
            MotionHeading_Reset();
            RobotHW_GyroSetOffset(offset);
            gyro_ready = 1U;
            gyro_last_tick = HAL_GetTick();
            Cmd_SendLine("OK");
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "YAW") == 0)
    {
        if (tok1 != NULL && strcmp(tok1, "RESET") == 0 && tok2 == NULL)
        {
            if (AnyMotionRunning()) { Cmd_Reject("ERR,BUSY"); return; }
            MotionHeading_Reset();
            RobotHW_YawReset();
            Cmd_SendLine("OK");
        }
        else if (tok1 != NULL && strcmp(tok1, "GET") == 0 && tok2 == NULL)
        {
            char msg[64];
            char y[20];
            FormatFloat3(y, sizeof(y), RobotHW_YawGetDeg());
            snprintf(msg, sizeof(msg), "YAW,deg=%s\r\n", y);
            Cmd_Send(msg);
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (strcmp(tok0, "STATUS") == 0)
    {
        if (tok1 == NULL)
        {
            HandleGlobalStatus();
        }
        else if (strcmp(tok1, "MOTOR") == 0 && tok2 == NULL)
        {
            HandleMotorStatus();
        }
        else if (strcmp(tok1, "ENC") == 0 && tok2 == NULL)
        {
            HandleEncoderStatus();
        }
        else
        {
            Cmd_Reject("ERR,BAD_ARGS");
        }
    }
    else if (!AnyMotionRunning())
    {
        Cmd_Reject("ERR,UNKNOWN_CMD");
    }
    else
    {
        Cmd_Reject("ERR,BUSY");
    }
}

static void DispatchReceivedLine(char *line)
{
    if (sequence_mode)
    {
        if (strcmp(line, "STOP") == 0)
            sequence_cancelled = 1U;
        else if (strcmp(line, "PING") != 0 && strcmp(line, "STATUS") != 0 &&
                 strcmp(line, "STRAIGHT,STATUS") != 0 && strcmp(line, "TURN,STATUS") != 0 &&
                 strcmp(line, "LATERAL,STATUS") != 0 &&
                 strcmp(line, "BYPASS,STATUS") != 0 &&
                 strcmp(line, "STRAIGHT,TRACE") != 0 &&
                 strcmp(line, "SEQ,LOG") != 0 &&
                 strcmp(line, "HEADING,STATUS") != 0)
        {
            Cmd_Reject("ERR,BUSY");
            return;
        }
    }
    DispatchLine(line);
}

static void ProcessRxByte(uint8_t b)
{
#if CMD_ECHO_ENABLE
    Cmd_SendRaw(&b, 1U);
#endif

    if (b == '\n' && previous_byte == '\r')
    {
        previous_byte = b;
        return;
    }

    previous_byte = b;

    if (b == '\r' || b == '\n')
    {
        if (rx_discard) { rx_discard = 0U; rx_line_len = 0U; return; }
#if CMD_ECHO_ENABLE
        if (b == '\r')
        {
            uint8_t lf = '\n';
            Cmd_SendRaw(&lf, 1U);
        }
#endif
        if (rx_line_len > 0U)
        {
            rx_line[rx_line_len] = '\0';
            DispatchReceivedLine(rx_line);
            rx_line_len = 0U;
        }
        return;
    }

    if (rx_discard) return;
    if (rx_line_len < (CMD_LINE_SIZE - 1U))
    {
        rx_line[rx_line_len++] = (char)b;
    }
    else
    {
        rx_line_len = 0U;
        rx_discard = 1U;
        Cmd_Reject("ERR,RX_LINE_OVERFLOW");
    }
}

static void CommandInterface_Init(void)
{
    RobotHW_StartMotionPeripherals();
    Straight_Init();
    Turn_Init();
    Lateral_Init();
    Bypass_Init();
    /* Normal UART and hardcoded runs hold each straight's starting direction.
     * Do not carry prior yaw error into the next straight or turn by default.
     * Gyro measurement, local straight PID and gyro-controlled turns remain on. */
    MotionHeading_SetEnabled(0U);
    OLED_Init();
    Oled_ShowCommand("READY");

    rx_line_len = 0U;
    previous_byte = 0U;
    rx_discard = 0U;
    rx_ring_head = 0U;
    rx_ring_tail = 0U;
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);   /* capture RX in the ISR, not polled */
    gyro_ready = 0U;
    gyro_last_tick = HAL_GetTick();
    ClearPendingMotion();
    command_rejected = 0U;
    sequence_mode = 0U;
    sequence_cancelled = 0U;
    command_output_suppressed = 0U;
    pending_motion_done_suppressed = 0U;

    Cmd_SendLine("HELLO,mdp21_v8");

}

static void CommandInterface_Poll(void)
{
    uint8_t rx_byte;
    Straight_Step();
    Turn_Step();
    Lateral_Step();
    Bypass_Step();
    GyroPollingStep();
    NotifyPendingMotionDone();
    while (RxRingPop(&rx_byte)) ProcessRxByte(rx_byte);   /* drain the ISR ring */
}

void CommandInterface_Run(void)
{
    CommandInterface_Init();
    while (1) CommandInterface_Poll();
}

static void Sequence_StopMotors(void)
{
    Lateral_Stop();
    Bypass_Stop();
    Straight_Stop();
    Turn_Stop();
    RobotHW_MotorStop();
    RobotHW_ServoCenter();
    NotifyPendingMotionDone();
}

static int Sequence_Wait(uint32_t duration_ms)
{
    uint32_t start = HAL_GetTick();
    do
    {
        CommandInterface_Poll();
        if (sequence_cancelled) return 0;
    } while (HAL_GetTick() - start < duration_ms);
    return 1;
}

static const char *CommandInterface_RunSequenceRows(const char *const commands[],
                                                    uint32_t count,
                                                    uint32_t startup_ms,
                                                    uint32_t gap_ms,
                                                    uint8_t keep_sequence_mode,
                                                    uint8_t emit_sequence_status,
                                                    uint8_t suppress_command_replies,
                                                    uint32_t *failure_index,
                                                    command_motion_observer_t observer)
{
    char line[CMD_LINE_SIZE];
    char msg[192];
    uint32_t index = 0U;
    const char *failure = NULL;
    uint8_t previous_motion_done_suppressed = pending_motion_done_suppressed;
    uint8_t previous_sequence_mode = sequence_mode;
    uint8_t nested_sequence = sequence_log_active;

    if (failure_index != NULL) *failure_index = 0U;

    sequence_mode = 1U;
    if (!nested_sequence) sequence_cancelled = 0U;
    pending_motion_done_suppressed = suppress_command_replies ? 1U : previous_motion_done_suppressed;
    if (!nested_sequence) SequenceLog_Begin();
    if (emit_sequence_status)
    {
        snprintf(msg, sizeof(msg), "SEQ,START,count=%lu,startup_ms=%lu\r\n",
                 (unsigned long)count, (unsigned long)startup_ms);
        Cmd_Send(msg);
    }
    if (commands == NULL && count != 0U) failure = "BAD_ARRAY";
    if (!failure && !Sequence_Wait(startup_ms)) failure = "USER_STOP";

    for (index = 0U; !failure && index < count; index++)
    {
        const char *command = commands[index];
        pending_motion_t motion;
        size_t length;
        if (!command) { failure = "NULL_COMMAND"; break; }
        length = strlen(command);
        if (length == 0U || length >= sizeof(line) ||
            strchr(command, '\r') || strchr(command, '\n'))
        {
            failure = "BAD_LINE";
            break;
        }
        /* One array element = one complete command; reject ignored extra fields. */
        {
            uint32_t commas = 0U;
            for (size_t i = 0U; i < length; i++)
                if (command[i] == ',') commas++;
            if (commas > 6U) { failure = "BAD_ARGS"; break; }
        }
        if (emit_sequence_status)
        {
            snprintf(msg, sizeof(msg), "SEQ,CMD,index=%lu,text=%s\r\n",
                     (unsigned long)(index + 1U), command);
            Cmd_Send(msg);
        }

        /* WAIT is local to the sequence, not a new UART command. */
        if (strncmp(command, "WAIT,", 5U) == 0)
        {
            int32_t delay_ms;
            if (!ParseIntStrict(command + 5U, &delay_ms) || delay_ms < 0 || delay_ms > 60000)
            {
                failure = "BAD_WAIT";
                break;
            }
            if (!Sequence_Wait((uint32_t)delay_ms)) failure = "USER_STOP";
        }
        else
        {
            uint8_t previous_output_suppressed = command_output_suppressed;
            SequenceLog_CaptureBefore(index + 1U, command);
            memcpy(line, command, length + 1U); /* strtok may mutate this copy. */
            command_rejected = 0U;
            command_output_suppressed = suppress_command_replies ? 1U : previous_output_suppressed;
            sequence_row_failure = NULL;
            sequence_row_dispatch = 1U;
            DispatchLine(line);
            sequence_row_dispatch = 0U;
            command_output_suppressed = previous_output_suppressed;
            if (sequence_row_failure) { failure = sequence_row_failure; break; }
            if (command_rejected) { failure = "COMMAND_REJECTED"; break; }
            motion = pending_motion;
            while (AnyMotionRunning() && !sequence_cancelled)
                CommandInterface_Poll();
            if (sequence_cancelled) failure = "USER_STOP";
            else if ((motion == PENDING_MOTION_STRAIGHT || motion == PENDING_MOTION_UNTILUS) &&
                     Straight_State() != STRAIGHT_DONE)
                failure = StraightStateText(Straight_State());
            else if (motion == PENDING_MOTION_TURN && Turn_State() != TURN_DONE)
                failure = TurnStateText(Turn_State());
            else if (motion == PENDING_MOTION_LATERAL && Lateral_State() != LATERAL_DONE)
                failure = LateralStateText(Lateral_State());
            else if (motion == PENDING_MOTION_BYPASS && Bypass_State() != BYPASS_DONE)
                failure = BypassStateText(Bypass_State());
            /* Includes motions completed immediately in their Start function. */
            NotifyPendingMotionDone();
            if (motion != PENDING_MOTION_NONE)
                SequenceLog_CaptureAfter(index + 1U, motion);
            if (!failure && observer && motion != PENDING_MOTION_NONE)
            {
                if (!Sequence_Wait(300U)) failure = "USER_STOP";
                else observer(index + 1U, command);
            }
        }
        if (failure) break;
        if (emit_sequence_status)
        {
            snprintf(msg, sizeof(msg), "SEQ,STEP_DONE,index=%lu\r\n",
                     (unsigned long)(index + 1U));
            Cmd_Send(msg);
        }
        if ((SEQUENCE_COMMAND_GAP_ENABLE || observer != NULL) &&
            index + 1U < count && !Sequence_Wait(gap_ms))
        {
            failure = "USER_STOP";
            break;
        }
    }
    Sequence_StopMotors();
    if (emit_sequence_status)
    {
        if (failure)
            snprintf(msg, sizeof(msg), "SEQ,STOPPED,index=%lu,reason=%s\r\n",
                     (unsigned long)((index < count) ? index + 1U : 0U), failure);
        else
            snprintf(msg, sizeof(msg), "SEQ,DONE,count=%lu\r\n", (unsigned long)count);
        Cmd_Send(msg);
    }

    if (nested_sequence)
    {
        /* Preserve outer exclusivity, cancellation and the complete RAM log. */
        sequence_mode = previous_sequence_mode;
    }
    else if (!keep_sequence_mode)
    {
        sequence_mode = 0U;
        sequence_cancelled = 0U;
    }

    if (!nested_sequence)
    {
        sequence_log_result = failure ? failure : "DONE";
        sequence_log_active = 0U;
    }
    pending_motion_done_suppressed = previous_motion_done_suppressed;
    if (failure_index != NULL && failure != NULL)
    {
        *failure_index = (index < count) ? index + 1U : 0U;
    }
    return failure;
}

void CommandInterface_RunSequence(const char *const commands[], uint32_t count,
                                  uint32_t startup_ms, uint32_t gap_ms)
{
    CommandInterface_Init();
    (void)CommandInterface_RunSequenceRows(commands, count, startup_ms, gap_ms,
                                           1U, 1U, 0U, NULL, NULL);
    /* Run once. Keep STOP/status available; never restart or resume implicitly. */
    while (1) CommandInterface_Poll();
}

const char *CommandInterface_RunDiagnosticSequence(const char *const commands[],
    uint32_t count, uint32_t startup_ms, uint32_t gap_ms, uint8_t heading_enabled,
    command_motion_observer_t observer, uint32_t *failure_index)
{
    CommandInterface_Init();
    MotionHeading_SetEnabled(heading_enabled);
    char intro[48];
    snprintf(intro, sizeof(intro), "SEQ H%u in %lus", (unsigned)heading_enabled,
        (unsigned long)((startup_ms + 999U) / 1000U));
    Oled_ShowCommand(intro);
    return CommandInterface_RunSequenceRows(commands, count, startup_ms, gap_ms,
        0U, 1U, 0U, failure_index, observer);
}
