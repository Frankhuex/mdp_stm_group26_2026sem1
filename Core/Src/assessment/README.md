# Assessment-facing STM32 code

This folder is for integrated robot behaviour, not individual hardware diagnostics.

Current active module:

- `command_interface.c` - USART3 command parser and integrated motion control interface.
  It calls the shared `straight_control`, `turn_control`, and `robot_hw` modules rather than reimplementing them.

Assessment mapping:

- A.1 STM32 <-> RPi USB/Serial movement communication: `command_interface.c`
- A.3 accurate 80-120 cm straight motion: `straight_control.c` (donor long-distance logic integrated; final floor retest/tuning still required)
- A.4 controlled 90-360 degree arc rotation: `turn_control.c`
- A.5 obstacle navigation: not finalized yet; IR/ultrasonic calibration should be completed first.

Do not add duplicate motor, encoder, gyro, straight, or turn implementations here. Assessment code should call the reusable modules.
