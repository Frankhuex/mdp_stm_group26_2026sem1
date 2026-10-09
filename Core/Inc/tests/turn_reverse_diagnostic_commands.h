#ifndef TURN_REVERSE_DIAGNOSTIC_COMMANDS_H
#define TURN_REVERSE_DIAGNOSTIC_COMMANDS_H

/* Included only by the existing executor. Every motion waits for DONE.
 * No STOP or STEER command is inserted between a turn and reverse:
 * both reset route heading. Pulse values use the current shared limits. */
static const char *const commands[] =
{
    "STOP",
#if TURN_REVERSE_DIAGNOSTIC_CASE == 1
    "STEER,US,2600", "WAIT,2000",
    "STEER,CENTER", "STEER,GET", "WAIT,5000",
    "STEER,US,1000", "WAIT,2000",
    "STEER,CENTER", "STEER,GET", "WAIT,5000",
#elif TURN_REVERSE_DIAGNOSTIC_CASE == 2
    "HEADING,OFF", "HEADING,STATUS",
    "STRAIGHT,GOCM,-30", "HEADING,STATUS", "STRAIGHT,STATUS",
#else
    "HEADING,ON",
#if TURN_REVERSE_DIAGNOSTIC_CASE == 3 || TURN_REVERSE_DIAGNOSTIC_CASE == 4
    "TURN,RIGHT,R=35,A=90",
#else
    "TURN,LEFT,R=20,A=90",
#endif
    "TURN,STATUS", "HEADING,STATUS", "STEER,GET",
    "WAIT,5000", "HEADING,STATUS",
#if TURN_REVERSE_DIAGNOSTIC_CASE == 4 || TURN_REVERSE_DIAGNOSTIC_CASE == 6
    "HEADING,OFF",
#endif
    "STRAIGHT,GOCM,-30", "HEADING,STATUS", "STRAIGHT,STATUS",
#endif
    "STOP",
};

#define EXECUTOR_STARTUP_DELAY_MS 10000U
#define EXECUTOR_COMMAND_GAP_MS   0U

#endif /* TURN_REVERSE_DIAGNOSTIC_COMMANDS_H */
