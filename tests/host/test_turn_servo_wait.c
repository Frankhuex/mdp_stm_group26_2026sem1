#define main RunExistingIRTests
#include "test_ir_straight.c"
#undef main
#undef GYRO_IIR_ALPHA
#undef GYRO_RATE_DEADBAND_DPS
#include "../../Core/Src/control/turn_control.c"
static void check_wait(int side, int gear, uint32_t previous) {
 reset(); Turn_Init(); mock_servo = previous; int before = drives;
 assert(Turn_StartRadiusAngle(side < 0 ? 20 : 35, gear * 90, side) == 0);
 uint32_t commanded = mock_servo;
 assert(tc_servo_wait_ms == Command_ServoWaitMs(previous, commanded));
 assert(tc_servo_wait_ms >= TURN_SERVO_WAIT_MIN_MS && tc_servo_wait_ms <= TURN_SERVO_WAIT_FULL_SPAN_MS);
 uint32_t start = tc_phase_tick;
 for (uint32_t elapsed = 10; elapsed < tc_servo_wait_ms; elapsed += 10) {
  tick = start + elapsed; Turn_Step();
  assert(tc_phase == 0 && drives == before);
 }
 tick = start + tc_servo_wait_ms + 10; Turn_Step();
 assert(tc_phase == 1 && drives == before + 1 && mock_direction == gear);
 Turn_Stop(); tick += 1000; Turn_Step(); assert(drives == before + 1);
}
static void check_turn_end_policies(void) {
 for (int side = -1; side <= 1; side += 2) {
  reset(); Turn_Init(); MotionHeading_SetEnabled(1U);
  (void)MotionHeading_Begin(90.0f); MotionHeading_End();
  tc_state = TURN_RUNNING; tc_turn_dir = side; tc_phase = 2U;
  Command_BeginPostTurnActions();
  motion_heading_status_t heading; MotionHeading_Status(&heading);
  assert(heading.enabled == ((side < 0 && TURN_LEFT_HEADING_OFF_ENABLE) ? 0 : 1));
  if (TURN_END_HEADING_RESET_ENABLE || (side < 0 && TURN_LEFT_HEADING_OFF_ENABLE))
   assert(heading.target_deg == 0.0f);
  else assert(heading.target_deg == 90.0f);
  if (side > 0 && TURN_RIGHT_RETURN_ENABLE) {
   assert(tc_phase == 4U && Turn_State() == TURN_RUNNING && mock_servo == TURN_RIGHT_RECENTER_US);
   tick = tc_phase_tick + TURN_RIGHT_RECENTER_HOLD_MS; Turn_Step();
   assert(tc_phase == 5U && mock_servo == SERVO_CENTER_US);
   tick = tc_phase_tick + TURN_RIGHT_CENTER_HOLD_MS; Turn_Step();
  }
  assert(Turn_State() == TURN_DONE);
 }
 reset(); Turn_Init(); tc_state = TURN_RUNNING; tc_turn_dir = -1;
 tc_phase = 3U; tc_phase_tick = tick; tc_last_tick = tick;
 tick += TURN_BRAKE_HOLD_MS; Turn_Step();
 if (TURN_POST_STOP_SETTLE_ENABLE) {
  assert(Turn_State() == TURN_RUNNING && tc_phase == 2U);
  tick = tc_phase_tick + POST_STOP_SETTLE_MS; Turn_Step();
 }
 assert(Turn_State() == TURN_DONE);
}
int main(void) {
 check_turn_end_policies();
 assert(Command_ServoWaitMs(1725,1725) == TURN_SERVO_WAIT_MIN_MS);
 assert(Command_ServoWaitMs(1000,2600) == TURN_SERVO_WAIT_FULL_SPAN_MS);
 assert(Command_ServoWaitMs(2600,1000) == TURN_SERVO_WAIT_FULL_SPAN_MS);
 assert(Command_ServoWaitMs(0,1725) == TURN_SERVO_WAIT_FULL_SPAN_MS);
 assert(Command_ServoWaitMs(1725,1800) < Command_ServoWaitMs(1725,2600));
 for (int side = -1; side <= 1; side += 2)
  for (int gear = -1; gear <= 1; gear += 2) check_wait(side,gear,1725);
 reset(); Turn_Init(); assert(Turn_StartRadiusAngle(20,90,-1) == 0);
 int before = drives; Turn_Stop(); tick += 1000; Turn_Step();
 assert(drives == before && Turn_State() == TURN_ABORTED);
 puts("PASS: variable servo wait, both steering sides/gears, no early drive, STOP during wait");
}
