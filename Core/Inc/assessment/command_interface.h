#ifndef ASSESSMENT_COMMAND_INTERFACE_H
#define ASSESSMENT_COMMAND_INTERFACE_H

#include <stdint.h>

/* Normal UART/sequence startup defaults route-heading carry OFF. Local gyro
 * steering stays active; explicit HEADING,ON can enable carry for a test. */
void CommandInterface_Run(void);
/* USART3 RXNE/overrun ISR body - call from USART3_IRQHandler. Captures every RX
 * byte into a ring buffer so blocking I2C/motion steps can't drop command bytes. */
void CommandInterface_UartIrq(void);
/* Runs once, then remains stopped while servicing STOP/status and SEQ,LOG.
 * Sequence diagnostics are retained in volatile RAM until reset/power loss
 * or the next sequence. SEQ,LOG is available only after the run stops. */
void CommandInterface_RunSequence(const char *const commands[], uint32_t count,
                                  uint32_t startup_ms, uint32_t gap_ms);
/* Dedicated floor diagnostic; executes the same UART parser/controllers.
 * Observer runs only after a successful motion and a 300 ms sampled settle.
 * Unlike RunSequence, returns after completion/failure for saved OLED pages. */
typedef void (*command_motion_observer_t)(uint32_t index, const char *command);
const char *CommandInterface_RunDiagnosticSequence(const char *const commands[],
    uint32_t count, uint32_t startup_ms, uint32_t gap_ms, uint8_t heading_enabled,
    command_motion_observer_t observer, uint32_t *failure_index);

#endif /* ASSESSMENT_COMMAND_INTERFACE_H */
