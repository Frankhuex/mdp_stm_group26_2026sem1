/* Link the real production straight controller; reuse the IR hardware mocks. */
#define main RunIRRegression
#include "test_ir_straight.c"
#undef main
#include "control/straight_tuning.h"
#include "control/motion_heading.h"
#undef GYRO_IIR_ALPHA
#undef GYRO_RATE_DEADBAND_DPS
#include "../../Core/Src/control/turn_control.c"
#include "../../Core/Src/assessment/command_interface.c"
#include "../../Core/Src/tests/calibration/test_motion_sequence.c"
extern char host_oled_rows[4][17];
extern unsigned long host_uart_tx_bytes;

/* The command sequencer runs unchanged. UART polling advances simulated time;
 * the plant supplies wheel counts and yaw. No physical accuracy is inferred. */
static int near(float a, float b);
static const char *queued_uart;
int HAL_UART_Receive(UART_HandleTypeDef *port, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)port; (void)size;
    tick += timeout;
    if (Turn_State() == TURN_RUNNING && tc_phase == 1U)
        mock_rate = (tc_turn_dir == 1 ? -50.0f : 50.0f) * tc_drive_dir;
    else mock_rate = Straight_State() == STRAIGHT_RUNNING && !mock_braking ? 1.0f : 0.0f;
    encoder_delta = Straight_State() == STRAIGHT_RUNNING && !mock_braking ? 100 : 0;
    if (queued_uart && *queued_uart && AnyMotionRunning())
    { *data = (uint8_t)*queued_uart++; return HAL_OK; }
    return 1;
}
void RobotHW_StartMotionPeripherals(void) {}
int32_t RobotHW_MotorGetLeftSigned(void) { return 0; }
int32_t RobotHW_MotorGetRightSigned(void) { return 0; }
int32_t RobotHW_MotorGetLeftRequested(void) { return 0; }
int32_t RobotHW_MotorGetRightRequested(void) { return 0; }
void RobotHW_MotorSetSigned(int32_t l, int32_t r) { (void)l; (void)r; }
void MotionLog_Write(const char *format, ...) { (void)format; }
void RobotHW_MotorSetSignedUnscaled(int32_t l,int32_t r) { RobotHW_MotorSetSigned(l,r); }
uint8_t RobotHW_GyroIsInitialized(void) { return 1U; }
uint8_t RobotHW_GyroInit(void) { return 1U; }
uint8_t RobotHW_GyroCalibrate(uint32_t a,uint32_t b,uint32_t c)
{ (void)a;(void)b;(void)c; return 1U; }
int32_t RobotHW_GyroGetOffset(void) { return 0; }
int16_t RobotHW_GyroReadZRaw(void) { return 0; }
void RobotHW_GyroSetOffset(int32_t x) { (void)x; }

static motion_heading_status_t command_records[5];
static uint32_t record_ticks[5], command_record_count;
static uint8_t inject_history_gap;
static void observe_command(uint32_t index, const char *command)
{
    assert(!AnyMotionRunning() && index <= 5U);
    assert(command != NULL);
    MotionHeading_Status(&command_records[command_record_count]);
    record_ticks[command_record_count++] = tick;
    if (inject_history_gap) CaptureLeg(index, command);
    if (inject_history_gap && index == 1U) tick += 1000U;
}
static void dispatch(const char *command)
{
    char copy[128];
    memcpy(copy, command, strlen(command) + 1U);
    command_rejected = 0U;
    DispatchLine(copy);
}

static void test_command_sequences(void)
{
    static const char *const route[] = { "STRAIGHT,GOCM,40",
        "TURN,LEFT,R=20,A=90", "STRAIGHT,GOCM,-10" };
    static const char *const mixed[] = { "STRAIGHT,GOCM,30", "TURN,LEFT,R=20,A=90",
        "STRAIGHT,GOCM,20", "TURN,RIGHT,R=35,A=90", "STRAIGHT,GOCM,-10" };
    for (unsigned on = 0; on <= 1U; ++on)
    {
        reset(); Turn_Init(); command_record_count = 0; inject_history_gap = 1U;
        uint32_t failed = 99U;
        assert(CommandInterface_RunDiagnosticSequence(route, 3, 50, 1000,
            (uint8_t)on, observe_command, &failed) == NULL);
        assert(failed == 0U && command_record_count == 3U);
        assert(near(command_records[0].target_deg, 0.0f));
        assert(near(command_records[1].target_deg, 0.0f));
        assert(near(command_records[2].target_deg, 0.0f));
        assert(!command_records[1].enabled && !command_records[1].tracking);
        assert(!command_records[2].enabled && !command_records[2].degraded);
        assert(record_ticks[1] - record_ticks[0] >= 1000U);
        assert(MotionHeading_IsTracking()); /* Completion cleanup preserves route. */
        assert(result_count == 3U && near(results[1].heading.target_deg, 0.0f));
        assert(results[1].is_turn && !results[2].is_turn && results[2].left_cm < 0.0f);
        float saved_measurement = results[0].heading.measured_deg;
        MotionHeading_Observe(RobotHW_YawGetDeg() + 10.0f, 1U);
        ShowResult(0U, 0U);
        assert(near(results[0].heading.measured_deg, saved_measurement));
        assert(strstr(host_oled_rows[0], "F40") && strstr(host_oled_rows[0], "END"));
    }
    reset(); Turn_Init(); command_record_count = 0; inject_history_gap = 0U;
    assert(CommandInterface_RunDiagnosticSequence(mixed, 5, 50, 100,
        1U, observe_command, NULL) == NULL);
    assert(command_record_count == 5U);
    assert(near(command_records[2].target_deg, 0.0f) && !command_records[2].enabled);
    assert(near(command_records[4].target_deg, 0.0f) && !command_records[4].degraded);
    dispatch("HEADING,STATUS"); assert(!command_rejected && MotionHeading_IsTracking());
    dispatch("HEADING,OFF"); assert(!command_rejected && !MotionHeading_IsEnabled());
    dispatch("HEADING,ON"); assert(!command_rejected && MotionHeading_IsEnabled());
    dispatch("STRAIGHT,GOCM,20"); assert(!command_rejected);
    dispatch("HEADING,RESET"); assert(command_rejected && MotionHeading_IsTracking());
    dispatch("TURN,STOP"); assert(!command_rejected && MotionHeading_IsTracking());
    dispatch("STOP"); assert(!command_rejected && !MotionHeading_IsTracking());
    dispatch("TURN,LEFT,R=20,A=90"); assert(!command_rejected);
    dispatch("STRAIGHT,STOP"); assert(!command_rejected && MotionHeading_IsTracking());
    dispatch("STOP");
    dispatch("HEADING,RESET"); assert(!command_rejected && !MotionHeading_IsTracking());
    dispatch("HEADING,ON,EXTRA"); assert(command_rejected);

    reset(); Turn_Init(); queued_uart = "STOP\n"; command_record_count = 0U;
    uint32_t failed = 0;
    const char *reason = CommandInterface_RunDiagnosticSequence(route, 3, 50, 100,
        1U, observe_command, &failed);
    assert(reason && strcmp(reason, "USER_STOP") == 0 && failed == 1U);
    assert(command_record_count == 0U && !AnyMotionRunning());
    queued_uart = NULL;
    puts("PASS: actual command parser/sequencer, mixed routes, A/B, nonfatal history gaps, reset/busy rules, STOP cancellation");
}

static unsigned count_log_text(const char *needle)
{
    unsigned count = 0U;
    const char *cursor = sequence_log;
    while ((cursor = strstr(cursor, needle)) != NULL)
    {
        ++count;
        cursor += strlen(needle);
    }
    return count;
}

static void test_sequence_log_and_default_heading(void)
{
    static const char *const route[] = { "HELLO", "STRAIGHT,GOCM,15",
        "TURN,RIGHT,R=35,A=90", "STRAIGHT,GOCM,15", "STRAIGHT,GOCM,-45",
        "TURN,LEFT,R=20,A=90", "STRAIGHT,GOCM,45" };
    reset(); Turn_Init(); queued_uart = NULL;
    CommandInterface_Init();
    assert(!MotionHeading_IsEnabled());
    (void)MotionHeading_Begin(0.0f);
    MotionHeading_Observe(-3.0f, 1U);
    MotionHeading_End();
    dispatch("TURN,LEFT,R=20,A=90");
    assert(!command_rejected);
    /* Incoming rightward error must not enlarge the first LEFT turn. */
    assert(near(Turn_TargetMagnitudeDeg(), 90.0f * TURN_LEFT_TARGET_SCALE));
    dispatch("STOP");

    CommandInterface_Init();
    assert(CommandInterface_RunSequenceRows(route, 7U, 0U, 0U,
        1U, 1U, 0U, NULL, NULL) == NULL);
    assert(!AnyMotionRunning() && !sequence_log_active && !sequence_log_capture_only);
    assert(sequence_log_used > 0U && sequence_log_dropped == 0U);
    assert(strcmp(sequence_log_result, "DONE") == 0);
    assert(count_log_text("LOG,AFTER,index=") == 6U);
    assert(count_log_text("TRACE,STRAIGHT,END") == 4U);
    assert(count_log_text("LOG,TURN_METRICS,") == 2U);
    assert(strstr(sequence_log, "LOG,BEFORE,index=6,command=TURN,LEFT,R=20,A=90"));
    assert(strstr(sequence_log, "TURN,state=DONE,dir=LEFT"));
    assert(strstr(sequence_log, "SEQ,DONE,count=7"));
    assert(strstr(sequence_log, "HEADING,enabled=1") == NULL);
    assert(!MotionHeading_IsEnabled());
    uint32_t saved_length = sequence_log_used;
    unsigned long tx_before = host_uart_tx_bytes;
    char request[] = "SEQ,LOG";
    command_rejected = 0U;
    DispatchReceivedLine(request); /* Allowed in hardcoded mode after DONE. */
    assert(!command_rejected && host_uart_tx_bytes > tx_before + saved_length);
    dispatch("SEQ,LOG");
    assert(!command_rejected && sequence_log_used == saved_length);
    assert(count_log_text("TRACE,STRAIGHT,END") == 4U);
    dispatch("SEQ,LOG,EXTRA"); assert(command_rejected);

    /* Capture diagnostics without transmitting them or driving anything. */
    SequenceLog_Begin();
    tx_before = host_uart_tx_bytes;
    int saved_drives = drives;
    SequenceLog_CaptureBefore(1U, "TURN,LEFT,R=20,A=90");
    assert(host_uart_tx_bytes == tx_before && drives == saved_drives);
    assert(strstr(sequence_log, "HEADING,enabled=0"));
    dispatch("SEQ,LOG"); assert(command_rejected); /* Busy even in a stopped gap. */
    assert(host_uart_tx_bytes - tx_before == strlen("ERR,BUSY\r\n"));

    /* Saturation retains the start, counts every missing byte and terminates. */
    SequenceLog_Begin();
    sequence_log_capture_only = 1U;
    char chunk[256]; memset(chunk, 'x', sizeof(chunk));
    tx_before = host_uart_tx_bytes;
    for (unsigned i = 0U; i < SEQUENCE_LOG_CAPACITY / sizeof(chunk) + 1U; ++i)
        Cmd_SendRaw((const uint8_t *)chunk, sizeof(chunk));
    assert(sequence_log_used == SEQUENCE_LOG_CAPACITY);
    assert(sequence_log_dropped == sizeof(chunk));
    assert(sequence_log[0] == 'x' && sequence_log[SEQUENCE_LOG_CAPACITY] == '\0');
    assert(host_uart_tx_bytes == tx_before);
    sequence_log_capture_only = sequence_log_active = 0U;

    /* The next sequence replaces the old log and keeps its failure evidence. */
    reset(); queued_uart = "STOP\n";
    const char *reason = CommandInterface_RunDiagnosticSequence(route, 7U,
        0U, 0U, 0U, NULL, NULL);
    assert(reason && strcmp(reason, "USER_STOP") == 0);
    assert(strcmp(sequence_log_result, "USER_STOP") == 0);
    assert(!sequence_log_active && sequence_log_dropped == 0U);
    assert(strstr(sequence_log, "STRAIGHT,state=ABORTED"));
    assert(strstr(sequence_log, "TRACE,STRAIGHT,END"));
    queued_uart = NULL;

    static const char *const bad[] = { "STRAIGHT,GOCM,5" };
    reason = CommandInterface_RunDiagnosticSequence(bad, 1U, 0U, 0U, 0U, NULL, NULL);
    assert(reason && strcmp(reason, "COMMAND_REJECTED") == 0);
    assert(strstr(sequence_log, "ERR,BAD_VALUE"));
    assert(strstr(sequence_log, "LOG,AFTER") == NULL);
    assert(!sequence_log_active);
    puts("PASS: default heading OFF, sequence log retention/replay, no live diagnostic TX, bounds and failures");
}

static int near(float a, float b)
{
    return a - b < 0.001f && b - a < 0.001f;
}

static void heading_step(void)
{
    tick += CONTROL_PERIOD_MS;
    Straight_Step();
}

static float measured_yaw(void)
{
    float yaw;
    Straight_Status(0, 0, 0, 0, &yaw, 0);
    return yaw;
}

static void finish_forward(int cm, float drive_yaw, float brake_yaw)
{
    assert(Straight_StartCm(cm) == 0);
    encoder_delta = 5000;
    mock_rate = drive_yaw * 1000.0f / CONTROL_PERIOD_MS;
    mock_brake_rate = brake_yaw * 1000.0f / MOTOR_BRAKE_PULSE_MS;
    heading_step();
    assert(Straight_State() == STRAIGHT_RUNNING && mock_braking);
    int drive_calls = drives;
    float before_brake_cm, after_brake_cm;
    Straight_Status(0, 0, &before_brake_cm, 0, 0, 0);
    encoder_delta = 10;
    for (unsigned elapsed = CONTROL_PERIOD_MS;
         elapsed <= MOTOR_BRAKE_PULSE_MS; elapsed += CONTROL_PERIOD_MS)
    {
        heading_step();
        assert(drives == drive_calls); /* Brake must not re-enable drive PWM. */
        if (elapsed < MOTOR_BRAKE_PULSE_MS)
            assert(Straight_State() == STRAIGHT_RUNNING && mock_braking);
    }
    assert(Straight_State() == STRAIGHT_DONE && !mock_braking);
    assert(MotionHeading_IsTracking());
    Straight_Status(0, 0, &after_brake_cm, 0, 0, 0);
    assert(after_brake_cm > before_brake_cm);
    assert(near(measured_yaw(), drive_yaw + brake_yaw));
    encoder_delta = 0;
    mock_rate = mock_brake_rate = 0.0f;
}

static motion_heading_status_t heading(void)
{
    motion_heading_status_t s;
    MotionHeading_Status(&s);
    return s;
}

static void run_turn(int side, int angle, float incoming, uint8_t on)
{
    reset(); Turn_Init(); MotionHeading_SetEnabled(on);
    finish_forward(40, incoming, 0.0f);
    float sign = (side == 1 ? -1.0f : 1.0f) * (angle < 0 ? -1.0f : 1.0f);
    float magnitude = (float)(angle < 0 ? -angle : angle);
    assert(Turn_StartRadiusAngle(side == 1 ? 35 : 20, angle, side) == 0);
    assert(near(heading().target_deg, sign * magnitude));
    float expected = (magnitude - (on ? sign * incoming : 0.0f)) *
        (side == 1 ? (angle < 0 ? TURN_RIGHT_REVERSE_TARGET_SCALE : TURN_RIGHT_TARGET_SCALE) :
         angle < 0 ? TURN_LEFT_REVERSE_TARGET_SCALE : TURN_LEFT_TARGET_SCALE);
    assert(near(tc_target_abs_deg, expected));
    assert(near(Turn_TargetMagnitudeDeg(), expected));
    /* Sensor local-zero at motor start cannot erase the inherited error. */
    for (unsigned i = 0; tc_phase == 0U && i < 100U; ++i)
    { tick += 10U; Straight_Step(); Turn_Step(); }
    assert(tc_phase == 1U && mock_direction == (angle < 0 ? -1 : 1));
    assert(near(heading().measured_deg, incoming));
    mock_rate = 50.0f * sign;
    for (unsigned i = 0; tc_phase == 1U && i < 800U; ++i)
    { tick += 10U; Straight_Step(); Turn_Step(); }
    assert(tc_phase == 3U && mock_braking);
    float stop_yaw, stop_rate;
    Turn_StopMetrics(&stop_yaw, &stop_rate);
    assert(near(stop_yaw, RobotHW_YawGetDeg()));
    assert(near(stop_rate, 50.0f));
    mock_rate = 0.0f;
    mock_brake_rate = 2.0f * sign;
    int drives_before_brake = drives;
    for (unsigned i = 0; Turn_State() == TURN_RUNNING && i < 650U; ++i)
    { tick += 10U; Straight_Step(); Turn_Step(); }
    assert(Turn_State() == TURN_DONE && !mock_braking);
    assert(drives == drives_before_brake);
    assert(mock_brake_end_tick - mock_brake_begin_tick == TURN_BRAKE_HOLD_MS);
    float completed_yaw;
    Turn_Status(NULL, NULL, NULL, &completed_yaw, NULL);
    assert(near(completed_yaw - stop_yaw,
        mock_brake_rate * (float)TURN_BRAKE_HOLD_MS / 1000.0f));
    assert(!MotionHeading_IsTracking() && near(heading().target_deg, 0.0f));
    assert(near(heading().measured_deg, 0.0f));
    assert(MotionHeading_IsEnabled() == (side == CMD_TURN_LEFT ? 0U : on));
    float saved_stop_yaw, saved_stop_rate;
    Turn_StopMetrics(&saved_stop_yaw, &saved_stop_rate);
    assert(near(saved_stop_yaw, stop_yaw) && near(saved_stop_rate, stop_rate));
    assert(near(Turn_TargetMagnitudeDeg(), expected));
    mock_brake_rate = 0.0f;
    float prior_error = heading().error_deg;
    assert(Straight_StartCm(-10) == 0);
    assert(near(heading().target_deg, 0.0f));
    assert(near(measured_yaw(), prior_error));
    assert(mock_drive_servo == SERVO_CENTER_US + STRAIGHT_REVERSE_TRIM_US);
}

static void begin_right_return(uint8_t wrap_clock)
{
    reset(); Turn_Init(); ClearPendingMotion();
    if (wrap_clock) tick = UINT32_MAX - 3000U;
    dispatch("TURN,RIGHT,R=35,A=90"); assert(!command_rejected);
    for (unsigned i = 0U; tc_phase != 4U && i < 700U; ++i)
    {
        mock_rate = tc_phase == 1U ? -50.0f : 0.0f;
        tick += 10U; Turn_Step();
    }
    assert(Turn_State() == TURN_RUNNING && tc_phase == 4U);
    assert(mock_servo == 1500U && !mock_braking);
    assert(MotionHeading_IsEnabled() && !MotionHeading_IsTracking());
    assert(near(heading().target_deg, 0.0f) && near(heading().error_deg, 0.0f));
}

static void test_post_turn_actions(void)
{
    /* The real UART completion must wait for both pulse holds. Check clock
     * rollover, motion interlocks, zero drive, and saved turn yaw as well. */
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        begin_right_return((uint8_t)wrap);
        int stopped_drives = drives;
        float final_turn_yaw;
        Turn_Status(NULL, NULL, NULL, &final_turn_yaw, NULL);
        assert(final_turn_yaw < -70.0f);
        dispatch("STRAIGHT,GOCM,-30"); assert(command_rejected);
        dispatch("TURN,LEFT,R=20,A=90"); assert(command_rejected);
        dispatch("STEER,US,1725"); assert(command_rejected);
        /* Preserve turn status even if gyro samples move during the hold. */
        mock_rate = 1.0f;
        for (unsigned elapsed = 10U; elapsed < 500U; elapsed += 10U)
        {
            tick += 10U; Turn_Step(); NotifyPendingMotionDone();
            assert(Turn_State() == TURN_RUNNING && tc_phase == 4U);
            assert(mock_servo == 1500U && drives == stopped_drives);
            assert(pending_motion == PENDING_MOTION_TURN);
        }
        tick += 10U; Turn_Step();
        assert(tc_phase == 5U && mock_servo == 1725U && !mock_braking);
        for (unsigned elapsed = 10U; elapsed < 500U; elapsed += 10U)
        {
            tick += 10U; Turn_Step(); NotifyPendingMotionDone();
            assert(Turn_State() == TURN_RUNNING && tc_phase == 5U);
            assert(mock_servo == 1725U && drives == stopped_drives);
            assert(pending_motion == PENDING_MOTION_TURN);
        }
        tick += 10U; Turn_Step(); NotifyPendingMotionDone();
        assert(Turn_State() == TURN_DONE && pending_motion == PENDING_MOTION_NONE);
        assert(drives == stopped_drives && mock_servo == 1725U);
        float saved_yaw;
        Turn_Status(NULL, NULL, NULL, &saved_yaw, NULL);
        assert(near(saved_yaw, final_turn_yaw));
        assert(MotionHeading_IsEnabled() && !MotionHeading_IsTracking());
        mock_rate = 0.0f;
        dispatch("STRAIGHT,GOCM,-30"); assert(!command_rejected);
        assert(near(heading().error_deg, 0.0f));
        assert(mock_drive_servo == 1733U);
        dispatch("STOP");
    }

    /* STOP must cancel either hold permanently, without a delayed restart. */
    for (unsigned phase = 4U; phase <= 5U; ++phase)
    {
        begin_right_return(0U);
        if (phase == 5U)
            for (unsigned i = 0U; i < 50U; ++i) { tick += 10U; Turn_Step(); }
        assert(tc_phase == phase);
        int stopped_drives = drives;
        dispatch("STOP"); NotifyPendingMotionDone();
        assert(Turn_State() == TURN_ABORTED && pending_motion == PENDING_MOTION_NONE);
        for (unsigned i = 0U; i < 500U; ++i) { tick += 10U; Turn_Step(); }
        assert(Turn_State() == TURN_ABORTED && mock_servo == 1725U);
        assert(drives == stopped_drives && !mock_braking);
    }

    begin_right_return(0U); mock_gyro_ok = 0U;
    for (unsigned i = 0U; i < 5U; ++i) { tick += 10U; Turn_Step(); }
    assert(Turn_State() == TURN_NO_IMU && mock_servo == 1725U && !mock_braking);

    /* A long successful turn gets its full extra one-second return window. */
    begin_right_return(0U);
    tc_start_tick = tick - (TURN_TIMEOUT_MS - 20U);
    for (unsigned i = 0U; i < 100U; ++i) { tick += 10U; Turn_Step(); }
    assert(Turn_State() == TURN_DONE);

    reset(); Turn_Init(); dispatch("TURN,LEFT,R=20,A=90");
    mock_gyro_ok = 0U;
    for (unsigned i = 0U; i < 5U; ++i) { tick += 10U; Turn_Step(); }
    assert(Turn_State() == TURN_NO_IMU && MotionHeading_IsEnabled());
    /* Failed LEFT turns must not execute the successful-turn HEADING,OFF. */
    puts("PASS: post-turn return timing, DONE ordering, resets, interlocks, STOP, faults and rollover");
}

static void test_turn_radius_calibration(void)
{
    /* Regression fixtures for the measured R20 trial and the existing R35
     * right calibration. Exercise production start/status in both gears. */
    for (int angle = -90; angle <= 90; angle += 180)
    {
        int32_t reported_radius = 0, reported_angle = 0;
        int8_t reported_side = 0;
        reset(); Turn_Init();
        assert(Turn_StartRadiusAngle(20, angle, CMD_TURN_LEFT) == 0);
        assert(mock_servo == (angle < 0 ? 1013U : 1024U));
        Turn_Status(NULL, &reported_radius, &reported_angle, NULL, &reported_side);
        assert(reported_radius == 20 && reported_angle == angle && reported_side == CMD_TURN_LEFT);
        assert(near(tc_target_abs_deg, angle < 0 ? 90.72f : 89.10f));
        Turn_Stop();

        reset(); Turn_Init();
        assert(Turn_StartRadiusAngle(35, angle, CMD_TURN_RIGHT) == 0);
        assert(mock_servo == 2569U);
        assert(near(tc_target_abs_deg, angle < 0 ? 91.62f : 88.02f));
        Turn_Stop();
    }

    /* Wider requests must never tighten steering; the calibrated minimum
     * must remain within the configured endpoints, even when saturated. */
    for (int side = CMD_TURN_LEFT; side <= CMD_TURN_RIGHT; side += 2)
    {
        for (int angle = -90; angle <= 90; angle += 180)
        {
            int minimum = side == CMD_TURN_LEFT ? CMD_LEFT_MIN_RADIUS_CM : CMD_RIGHT_MIN_RADIUS_CM;
            uint32_t previous = side == CMD_TURN_LEFT ? CMD_SERVO_LEFT_US : CMD_SERVO_RIGHT_US;
            for (int radius = minimum; radius <= 200; ++radius)
            {
                reset(); Turn_Init();
                assert(Turn_StartRadiusAngle(radius, angle, (int8_t)side) == 0);
                if (side == CMD_TURN_LEFT)
                {
                    assert(mock_servo >= previous && mock_servo <= SERVO_CENTER_US);
                    if (radius == minimum) assert(mock_servo == CMD_SERVO_LEFT_US);
                }
                else assert(mock_servo <= previous && mock_servo >= SERVO_CENTER_US);
                previous = mock_servo;
                Turn_Stop();
            }
            reset(); Turn_Init();
            assert(Turn_StartRadiusAngle(minimum - 1, angle, (int8_t)side) == -2);
            assert(Turn_State() == TURN_IDLE && mock_servo == SERVO_CENTER_US);
            assert(Turn_StartRadiusAngle(2147483647, angle, (int8_t)side) == 0);
            assert(mock_servo == SERVO_CENTER_US);
            Turn_Stop();
        }
    }
    puts("PASS: turn radius calibration, both gears, nominal status/yaw, monotonic mapping and endpoint/range limits");
}

static void test_left_forward_reverse_stop_lead(void)
{
    /* Replay each measured approach rate with a constant-rate mock. Both
     * gears must continue beyond the old ~69 deg brake point. Reverse also
     * carries the preceding forward heading error, as in the floor log.
     * These checks do not model floor coasting or claim physical accuracy. */
    for (int angle = -90; angle <= 90; angle += 180)
    {
        float sign = angle < 0 ? -1.0f : 1.0f;
        float approach_rate = angle < 0 ? 113.64f : 101.80f;
        reset(); Turn_Init();
        if (angle < 0)
        {
            (void)MotionHeading_Begin(90.0f);
            MotionHeading_Observe(91.31f, 1U);
            MotionHeading_End();
        }
        assert(Turn_StartRadiusAngle(20, angle, CMD_TURN_LEFT) == 0);
        if (angle < 0)
        {
            /* Latest return left +4.13 deg of heading: request about 4.1 deg
             * more reverse yaw than its old 87.93 target, not less. */
            assert(Turn_TargetMagnitudeDeg() > 92.04f && Turn_TargetMagnitudeDeg() < 92.05f);
            float added_reverse = Turn_TargetMagnitudeDeg() - 87.93f;
            assert(added_reverse > 4.10f && added_reverse < 4.15f);
        }
        else assert(near(Turn_TargetMagnitudeDeg(), 89.10f));
        for (unsigned i = 0U; tc_phase == 0U && i < 100U; ++i)
        { tick += 10U; Turn_Step(); }
        assert(tc_phase == 1U);
        mock_rate = sign * approach_rate;
        for (unsigned i = 0U; tc_phase == 1U && i < 100U; ++i)
        { tick += 10U; Turn_Step(); }
        assert(tc_phase == 3U && mock_braking);
        float stop_yaw, stop_rate;
        Turn_StopMetrics(&stop_yaw, &stop_rate);
        float progress = sign * stop_yaw;
        assert(near(stop_rate, approach_rate));
        /* Forward's refined endpoint brakes later than the 0.982 trial,
         * while reverse retains its previous timing at the same rate. */
        if (angle > 0) assert(progress > 75.86f && progress < 76.89f);
        else assert(progress > 77.26f && progress < 78.41f);
        Turn_Stop();
    }
    puts("PASS: earlier left forward stop, independent reverse calibration and route heading carry");
}

static void test_right_turn_stop_lead(void)
{
    /* Exercise actual brake decisions in both gears. R35/A90 now has
     * separate gear leads; short-right must still override either lead.
     * Constant simulated rate checks selection/signs, not floor accuracy. */
    for (int angle = 45; angle <= 90; angle += 45)
    {
        for (int gear = -1; gear <= 1; gear += 2)
        {
            reset(); Turn_Init();
            assert(Turn_StartRadiusAngle(35, gear * angle, CMD_TURN_RIGHT) == 0);
            assert(mock_servo == 2569U);
            float expected_target = gear < 0 ? (angle == 90 ? 91.62f : 45.81f)
                                    : angle == 90 ? 88.02f : 44.01f;
            assert(near(Turn_TargetMagnitudeDeg(), expected_target));
            for (unsigned i = 0U; tc_phase == 0U && i < 100U; ++i)
            { tick += 10U; Turn_Step(); }
            assert(tc_phase == 1U && mock_direction == gear);
            mock_rate = -80.0f * (float)gear;
            for (unsigned i = 0U; tc_phase == 1U && i < 200U; ++i)
            { tick += 10U; Turn_Step(); }
            assert(tc_phase == 3U && mock_braking);
            float stop_yaw, stop_rate;
            Turn_StopMetrics(&stop_yaw, &stop_rate);
            float progress = -(float)gear * stop_yaw;
            assert(near(stop_rate, 80.0f));
            if (angle == 90)
            {
                if (gear > 0) assert(progress > 74.81f && progress < 75.63f);
                else assert(progress > 79.21f && progress < 80.03f);
            }
            else
            {
                if (gear > 0) assert(progress > 30.40f && progress < 31.22f);
                else assert(progress > 32.20f && progress < 33.02f);
            }
            Turn_Stop();
        }
    }
    /* A shorter outbound changes the inherited return target. Do not retain
     * the old 93.88 deg reverse target after changing the forward trial. */
    reset(); Turn_Init(); MotionHeading_SetEnabled(1U);
    (void)MotionHeading_Begin(-90.0f);
    MotionHeading_Observe(-87.92f, 1U);
    MotionHeading_End();
    assert(Turn_StartRadiusAngle(35, -90, CMD_TURN_RIGHT) == 0);
    assert(near(Turn_TargetMagnitudeDeg(), 89.50256f));
    Turn_Stop();
    puts("PASS: right gear-specific stopping, short-turn lead override and inherited return target");
}

static void test_forward_response_and_trace(void)
{
    straight_trace_sample_t sample, saved;
    /* A stopped 2-degree route error must produce stronger forward steering,
     * while reverse keeps the previous correction magnitude and sign. */
    for (int dir = -1; dir <= 1; dir += 2)
    {
        for (int error_sign = -1; error_sign <= 1; error_sign += 2)
        {
            reset();
            (void)MotionHeading_Begin(0.0f);
            MotionHeading_Observe(2.0f * error_sign, 1U);
            MotionHeading_End();
            assert(Straight_StartCm(dir * 35) == 0);
            assert(mock_drive_servo == (uint32_t)(dir > 0 ?
                1767 + 60 * error_sign : 1733 - 28 * error_sign));
            assert(Straight_TraceCount() == 1U);
            assert(Straight_TraceRead(0U, &sample));
            assert(sample.phase == 'S' && sample.direction == dir);
            assert(sample.servo_us == mock_drive_servo && sample.gyro_ok);
            assert(near(sample.local_yaw_deg, 0.0f));
            assert(near(sample.control_error_deg, 2.0f * error_sign));
            Straight_Stop();
            assert(Straight_TraceRead(Straight_TraceCount() - 1U, &sample));
            assert(sample.phase == 'E' && sample.servo_us == SERVO_CENTER_US);
        }
    }

    /* OFF still reacts to newly measured local yaw, and trace retains actual
     * drive samples after the servo has been centred at successful completion. */
    reset(); MotionHeading_SetEnabled(0U);
    assert(Straight_StartCm(35) == 0);
    mock_rate = 100.0f; heading_step(); /* +2 deg in one control interval. */
    mock_rate = 0.0f;
    for (unsigned i = 0U; i < 4U; ++i) heading_step();
    assert(Straight_TraceRead(Straight_TraceCount() - 1U, &sample));
    assert(sample.phase == 'D' && sample.elapsed_ms == 100U);
    assert(near(sample.local_yaw_deg, 2.0f) && near(sample.control_error_deg, 2.0f));
    assert(sample.servo_us >= 1827U && sample.servo_us <= 1828U);
    encoder_delta = 5000; heading_step(); encoder_delta = 0;
    for (unsigned i = 0U; i < 4U; ++i) heading_step();
    assert(Straight_State() == STRAIGHT_DONE && Straight_TraceDropped() == 0U);
    assert(Straight_TraceRead(Straight_TraceCount() - 1U, &saved));
    assert(saved.phase == 'E' && saved.servo_us == SERVO_CENTER_US);
    mock_rate = 10.0f; tick += 100U; Straight_Step();
    assert(Straight_TraceRead(Straight_TraceCount() - 1U, &sample));
    assert(near(sample.local_yaw_deg, saved.local_yaw_deg));
    assert(!Straight_TraceRead(Straight_TraceCount(), &sample));
    assert(!Straight_TraceRead(0U, NULL));
    dispatch("STRAIGHT,TRACE"); assert(!command_rejected);
    assert(Straight_StartCm(35) == 0);
    assert(Straight_TraceCount() == 1U);
    dispatch("STRAIGHT,TRACE"); assert(command_rejected && Straight_State() == STRAIGHT_RUNNING);
    dispatch("STOP"); assert(!command_rejected && Straight_State() == STRAIGHT_ABORTED);
    dispatch("STRAIGHT,TRACE"); assert(!command_rejected);

    /* Bounded storage reserves the last entry for END on a long run. */
    reset(); assert(Straight_StartCm(200) == 0);
    for (unsigned i = 0U; i < STRAIGHT_TRACE_CAPACITY + 10U; ++i)
    { tick += STRAIGHT_TRACE_PERIOD_MS; Straight_Step(); }
    assert(Straight_State() == STRAIGHT_RUNNING);
    assert(Straight_TraceCount() == STRAIGHT_TRACE_CAPACITY - 1U);
    assert(Straight_TraceDropped() > 0U);
    Straight_Stop();
    assert(Straight_TraceCount() == STRAIGHT_TRACE_CAPACITY);
    assert(Straight_TraceRead(STRAIGHT_TRACE_CAPACITY - 1U, &sample) && sample.phase == 'E');

    reset(); assert(Straight_StartCm(35) == 0);
    mock_gyro_ok = 0U;
    for (unsigned i = 0U; i < 5U; ++i) heading_step();
    assert(Straight_State() == STRAIGHT_SENSOR_ERROR);
    assert(Straight_TraceRead(Straight_TraceCount() - 1U, &sample));
    assert(sample.phase == 'E' && !sample.gyro_ok && sample.servo_us == SERVO_CENTER_US);
    puts("PASS: independent forward/reverse response, local gyro with OFF, buffered trace, stop/fault handling and trace bounds");
}

static void test_fixed_steering_calibration(void)
{
    straight_trace_sample_t sample;
    reset();
    assert(Straight_StartCalibrationCm(-35, 1770) == -2);
    assert(Straight_StartCalibrationCm(51, 1770) == -2);
    assert(Straight_StartCalibrationCm(35, 1624) == -2);
    assert(Straight_StartCalibrationCm(35, 1826) == -2);
    assert(drives == 0 && Straight_TraceCount() == 0U);
    dispatch("STRAIGHT,CAL,35,1770,EXTRA"); assert(command_rejected);
    dispatch("STRAIGHT,CAL,35,1770x"); assert(command_rejected);
    dispatch("STRAIGHT,CAL,35"); assert(command_rejected);

    /* Inherited heading and new yaw must still be measured, but neither
     * may alter the explicitly requested pulse in this one command. */
    (void)MotionHeading_Begin(0.0f);
    MotionHeading_Observe(2.0f, 1U); MotionHeading_End();
    dispatch("STRAIGHT,CAL,35,1770"); assert(!command_rejected);
    assert(mock_drive_servo == 1770U && Straight_FixedSteeringUs() == 1770U);
    assert(MotionHeading_IsEnabled() && near(measured_yaw(), 2.0f));
    dispatch("STRAIGHT,CAL,35,1740"); assert(command_rejected);
    assert(Straight_FixedSteeringUs() == 1770U && mock_servo == 1770U);
    mock_rate = 20.0f;
    for (unsigned i = 0U; i < 5U; ++i) heading_step();
    assert(Straight_TraceRead(Straight_TraceCount() - 1U, &sample));
    assert(sample.phase == 'D' && sample.servo_us == 1770U && sample.gyro_ok);
    assert(near(sample.local_yaw_deg, 2.0f) && near(sample.control_error_deg, 4.0f));
    mock_rate = -40.0f;
    for (unsigned i = 0U; i < 5U; ++i) heading_step();
    assert(mock_servo == 1770U && near(mock_yaw, -2.0f));
    mock_rate = 0.0f; encoder_delta = 5000; heading_step(); encoder_delta = 0;
    for (unsigned i = 0U; i < 4U; ++i) heading_step();
    assert(Straight_State() == STRAIGHT_DONE && mock_servo == SERVO_CENTER_US);
    assert(Straight_FixedSteeringUs() == 1770U);
    assert(Straight_TraceRead(Straight_TraceCount() - 1U, &sample) && sample.phase == 'E');
    uint32_t retained_count = Straight_TraceCount();
    assert(Straight_StartCalibrationCm(35, 2000) == -2);
    assert(Straight_TraceCount() == retained_count && Straight_FixedSteeringUs() == 1770U);

    /* All normal straight entry points must restore gyro steering. */
    for (unsigned mode = 0U; mode < 3U; ++mode)
    {
        reset(); assert(Straight_StartCalibrationCm(35, 1770) == 0);
        dispatch("STOP");
        assert(Straight_State() == STRAIGHT_ABORTED && mock_servo == SERVO_CENTER_US);
        int rc = mode == 0U ? Straight_StartCm(35) : mode == 1U ?
            Straight_StartUntilIRCm(50, 0U, STRAIGHT_IR_GT) :
            Straight_StartUntilUltrasonicCm(20);
        assert(rc == 0 && Straight_FixedSteeringUs() == 0U);
        assert(mock_servo == SERVO_CENTER_US + STRAIGHT_FORWARD_TRIM_US);
        Straight_Stop();
    }
    reset(); assert(Straight_StartCalibrationCm(35, 1770) == 0);
    mock_gyro_ok = 0U;
    for (unsigned i = 0U; i < 5U; ++i) heading_step();
    assert(Straight_State() == STRAIGHT_SENSOR_ERROR && mock_servo == SERVO_CENTER_US);
    reset(); assert(Straight_StartCalibrationCm(35, 1770) == 0);
    tick = 16000U; Straight_Step();
    assert(Straight_State() == STRAIGHT_TIMEOUT && mock_servo == SERVO_CENTER_US);
    reset(); Turn_Init(); dispatch("TURN,RIGHT,R=35,A=90");
    assert(!command_rejected);
    dispatch("STRAIGHT,CAL,35,1770"); assert(command_rejected);
    dispatch("STOP");
    puts("PASS: fixed steering calibration, measurement, bounds, parser/interlocks, completion, faults and restoration");
}

int main(void)
{
    assert(RunIRRegression() == 0);
    test_sequence_log_and_default_heading();
    test_post_turn_actions();
    test_fixed_steering_calibration();
    test_forward_response_and_trace();
    test_turn_radius_calibration();
    test_left_forward_reverse_stop_lead();
    test_right_turn_stop_lead();
    for (int cm = 20; cm <= 40; cm += 20)
    {
        reset();
        tick = UINT32_MAX - CONTROL_PERIOD_MS;
        finish_forward(cm, 1.5f, 0.5f);
        mock_rate = 5.0f; heading_step(); mock_rate = 0.0f;
        assert(Straight_StartCm(-cm) == 0);
        assert(near(measured_yaw(), 2.1f));
        assert(mock_direction == -1);
        assert(mock_drive_servo < SERVO_CENTER_US + STRAIGHT_REVERSE_TRIM_US);
        assert(MotionHeading_IsTracking());
        mock_rate = -105.0f; heading_step(); mock_rate = 0.0f;
        assert(near(measured_yaw(), 0.0f));
        heading_step();
        assert(mock_servo == SERVO_CENTER_US + STRAIGHT_REVERSE_TRIM_US);
        Straight_Stop();
    }
    reset(); finish_forward(40, -1.5f, 0.0f);
    assert(Straight_StartCm(-40) == 0);
    assert(mock_drive_servo > SERVO_CENTER_US + STRAIGHT_REVERSE_TRIM_US);

    reset(); finish_forward(40, 1.5f, 0.0f);
    assert(Straight_StartCm(40) == 0); /* Forward also carries route error. */
    assert(near(measured_yaw(), 1.5f));
    assert(mock_drive_servo > SERVO_CENTER_US + STRAIGHT_FORWARD_TRIM_US);
    uint32_t held_servo = mock_servo;
    int prior_drives = drives;
    mock_gyro_ok = 0U; heading_step();
    assert(Straight_State() == STRAIGHT_RUNNING && drives > prior_drives);
    assert(mock_servo == held_servo && heading().degraded);
    mock_gyro_ok = 1U; heading_step();
    assert(Straight_State() == STRAIGHT_RUNNING);
    mock_gyro_ok = 0U;
    for (unsigned i = 0; i < 5U; ++i) heading_step();
    assert(Straight_State() == STRAIGHT_SENSOR_ERROR);

    reset(); finish_forward(40, 1.5f, 0.0f);
    tick += 1000U;
    assert(Straight_StartCm(-40) == 0); /* History gap is NOT a route abort. */
    assert(heading().degraded && near(measured_yaw(), 1.5f));
    reset(); finish_forward(40, 1.5f, 0.0f);
    mock_gyro_ok = 0U; heading_step(); mock_gyro_ok = 1U;
    assert(Straight_StartCm(-40) == 0 && heading().degraded);

    reset(); finish_forward(40, 1.5f, 0.0f);
    mock_prepare_rc = -4; prior_drives = drives;
    assert(Straight_StartCm(-40) == -4 && drives == prior_drives);
    tick += 100U; Straight_Step();
    assert(near(heading().measured_deg, 1.5f)); /* Unknown local reset not sampled. */
    mock_prepare_rc = 0;
    assert(Straight_StartCm(-40) == 0 && near(measured_yaw(), 1.5f));

    reset(); finish_forward(40, 1.5f, 0.0f);
    MotionHeading_Reset();
    assert(Straight_StartCm(-40) == 0 && near(measured_yaw(), 0.0f));

    reset(); assert(Straight_StartCm(40) == 0);
    encoder_delta = 5000; mock_gyro_delay_ms = 3U; heading_step();
    assert(mock_brake_begin_tick == 23U);
    encoder_delta = 0; mock_gyro_delay_ms = 0;
    for (tick = 40U; tick <= 100U; tick += 20U) Straight_Step();
    assert(mock_braking);
    tick = 103U; Straight_Step();
    assert(Straight_State() == STRAIGHT_DONE);
    assert(mock_brake_end_tick - mock_brake_begin_tick == MOTOR_BRAKE_PULSE_MS);

    reset(); finish_forward(40, 1.5f, 0.0f);
    assert(Straight_StartUntilIRGreaterCm(20, 0) == 0);
    assert(near(heading().error_deg, 1.5f));
    reset(); finish_forward(40, 1.5f, 0.0f);
    assert(Straight_StartUntilUltrasonicCm(20) == 0);
    assert(near(heading().error_deg, 1.5f));
    mock_rate = 5.0f; heading_step();
    assert(near(heading().error_deg, 1.6f)); /* Stationary ultrasound wait. */
    mock_ultrasonic.distance_mm = 1000U;
    mock_ultrasonic.sequence++; heading_step();
    mock_ultrasonic.sequence++; heading_step();
    assert(near(heading().error_deg, 1.8f)); /* Drive-start zero preserves wait yaw. */
    mock_rate = 0.0f;
    mock_ultrasonic.distance_mm = 100U;
    mock_ultrasonic.sequence++; heading_step();
    mock_ultrasonic.sequence++; heading_step();
    assert(Straight_State() == STRAIGHT_DONE && near(heading().error_deg, 1.8f));
    assert(Straight_StartCm(-10) == 0 && near(measured_yaw(), 1.8f));

    for (int side = -1; side <= 1; side += 2)
        for (int angle = -90; angle <= 90; angle += 180)
            for (unsigned on = 0U; on <= 1U; ++on)
                run_turn(side, angle, 2.0f, (uint8_t)on);

    /* Cumulative angle wrap, correction cap, preserving small turn direction. */
    reset(); (void)MotionHeading_Begin(0.0f);
    MotionHeading_Observe(359.0f, 1U);
    assert(near(MotionHeading_ErrorDeg(), -1.0f));
    assert(near(MotionHeading_Begin(90.0f), 91.0f));
    MotionHeading_Observe(91.0f, 1U);
    assert(near(MotionHeading_ErrorDeg(), 0.0f));
    MotionHeading_Reset(); (void)MotionHeading_Begin(0.0f);
    MotionHeading_Observe(20.0f, 1U);
    assert(near(MotionHeading_Begin(90.0f), 85.0f));
    assert(near(MotionHeading_Begin(-5.0f), -2.5f));

    reset(); Turn_Init(); tick = UINT32_MAX - 100U;
    assert(Turn_StartRadiusAngle(20, 90, -1) == 0);
    for (unsigned i = 0; tc_phase == 0U && i < 100U; ++i)
    { tick += 10U; Turn_Step(); }
    mock_rate = 50.0f;
    for (unsigned i = 0; tc_phase == 1U && i < 800U; ++i)
    { tick += 10U; Turn_Step(); }
    assert(tc_phase == 3U);
    mock_rate = 0.0f;
    for (unsigned i = 0; i < TURN_BRAKE_HOLD_MS / 10U - 1U; ++i)
    { tick += 10U; Turn_Step(); }
    mock_gyro_ok = 0U; tick += 10U; Turn_Step();
    assert(!mock_braking && tc_phase == 2U && Turn_State() == TURN_RUNNING);
    assert(mock_brake_end_tick - mock_brake_begin_tick == TURN_BRAKE_HOLD_MS);
    assert(heading().degraded);
    test_command_sequences();
    puts("PASS: real straight/turn controllers, route carry, forward/reverse signs, A/B mode, braking/pause, wrap, resets, faults and IR regression");
    return 0;
}

