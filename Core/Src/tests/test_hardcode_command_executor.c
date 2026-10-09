#include "tests/test_hardcode_command_executor.h"
#include "assessment/command_interface.h"
#include "test_config.h"

#if TURN_REVERSE_DIAGNOSTIC_CASE < 0 || TURN_REVERSE_DIAGNOSTIC_CASE > 6
#error "TURN_REVERSE_DIAGNOSTIC_CASE must be 0..6"
#endif

#if TURN_REVERSE_DIAGNOSTIC_CASE != 0
#include "tests/turn_reverse_diagnostic_commands.h"
#else

/* ================================================================
 * EDIT THIS ARRAY: one UART-format command per string, WITHOUT CR/LF.
 * Motion commands wait for successful completion before the next row.
 * WAIT,<ms> is executor-only, range 0..60000. The sequence runs ONCE.
 * ================================================================ */
static const char *const commands[] =
{
    "CALIBRATE,CENTER",
    "WAIT,10000",

    "HELLO",
    // --- leg S -> 3 ---
    "STRAIGHT,GOCM,10",
    "TURN,RIGHT,R=35,A=49",
    "STRAIGHT,GOCM,40",
    "TURN,RIGHT,R=35,A=33",
    "STRAIGHT,GOCM,20",
    "TURN,LEFT,R=20,A=-57",
    "TURN,RIGHT,R=35,A=16",
    "TURN,LEFT,R=20,A=-29",
    // RECOGNISE obstacle 3
    // --- leg 3 -> 1 ---
    "STRAIGHT,GOCM,-10",
    "TURN,RIGHT,R=35,A=16",
    "TURN,LEFT,R=20,A=-29",
    "TURN,RIGHT,R=35,A=16",
    "TURN,LEFT,R=20,A=-29",
    // RECOGNISE obstacle 1
    // --- leg 1 -> 2 ---
    "TURN,RIGHT,R=35,A=16",
    "STRAIGHT,GOCM,-20",
    "TURN,RIGHT,R=35,A=65",
    // RECOGNISE obstacle 2
    // --- leg 2 -> 4 ---
    "STRAIGHT,GOCM,-10",
    "TURN,LEFT,R=20,A=-29",
    "TURN,RIGHT,R=35,A=33",
    "TURN,LEFT,R=20,A=-29",
    // RECOGNISE obstacle 4
    // --- leg 4 -> 5 ---
    "TURN,LEFT,R=20,A=57",
    "TURN,RIGHT,R=35,A=65",
    "TURN,LEFT,R=20,A=29",
    "TURN,RIGHT,R=35,A=-16",
    "TURN,LEFT,R=20,A=29",
    "TURN,RIGHT,R=35,A=-16",
    // RECOGNISE obstacle 5
    // --- leg 5 -> 7 ---
    "TURN,RIGHT,R=35,A=16",
    "TURN,LEFT,R=20,A=-57",
    "TURN,RIGHT,R=35,A=16",
    "STRAIGHT,GOCM,10",
    // RECOGNISE obstacle 7
    // --- leg 7 -> 6 ---
    "STRAIGHT,GOCM,-10",
    "TURN,LEFT,R=20,A=57",
    "TURN,RIGHT,R=35,A=16",
    "TURN,LEFT,R=20,A=-57",
    "TURN,RIGHT,R=35,A=33",
    "STRAIGHT,GOCM,10",
    "TURN,RIGHT,R=35,A=33",
    "STRAIGHT,GOCM,-10",
    // RECOGNISE obstacle 6

//		"STRAIGHT,UNTILUS,55",
//		"FAST,RIGHT1",
//		"FAST,LEFT2",

        /*// 2/10/2026 test reverse+turn: closed route, run once.
        // After the initial forward 45 cm, use rear-axle midpoint (0,0).
        // Nose toward +Y; +X is right. The loop returns to this point,
        // ideally 45 cm ahead of the original starting position.
        // Every TURN is a forward 90-degree arc, followed by reverse 35 cm.
        // Ideal pair endpoints (X,Y,heading), in cm:
        // (0,35,E), (20,20,N), (35,40,W), (15,55,S),
        // (0,35,E), (35,35,S), (35,0,W), (0,0,N).
        // RIGHT already includes 1500 -> 1725 us return; LEFT applies
        // HEADING,OFF automatically. No duplicate manual actions here.
        // A physical return to the mark still depends on calibration/slip.
        "STOP",
        "HEADING,OFF",

        "STRAIGHT,GOCM,45",
        // 1-2: right 90, reverse 35. Expected (0,35), facing right.
        "TURN,RIGHT,R=35,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 3-4: left 90, reverse 35. Expected (20,20), facing forward.
        "TURN,LEFT,R=20,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 5-6: left 90, reverse 35. Expected (35,40), facing left.
        "TURN,LEFT,R=20,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 7-8: left 90, reverse 35. Expected (15,55), facing backward.
        "TURN,LEFT,R=20,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 9-10: left 90, reverse 35. Expected (0,35), facing right.
        "TURN,LEFT,R=20,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 11-12: right 90, reverse 35. Expected (35,35), facing backward.
        "TURN,RIGHT,R=35,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 13-14: right 90, reverse 35. Expected (35,0), facing left.
        "TURN,RIGHT,R=35,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",

        // 15-16: right 90, reverse 35. Expected loop origin (0,0), facing forward.
        "TURN,RIGHT,R=35,A=90",
        "TURN,STATUS",
        "HEADING,STATUS",
        "WAIT,2000",
        "STRAIGHT,GOCM,-35",
        "STRAIGHT,STATUS",
        "STRAIGHT,TRACE",
        "WAIT,2000",
        "STOP",*/

		/*
        // ================================================================
        // mock task 1
        // Runs after the existing commands above.
        // RECOGNISE and RAW FORWARD notes are comments only.
        // ================================================================
        "HELLO",
		//``"STRAIGHT,GOCM,35",
        // --- leg S -> 5 ---
        "STRAIGHT,GOCM,15",
        "TURN,RIGHT,R=35,A=90",
        "STRAIGHT,GOCM,15",
        // RECOGNISE obstacle 5

        // --- leg 5 -> 1 ---
        "STRAIGHT,GOCM,-45",
        "TURN,LEFT,R=20,A=90",
        "STRAIGHT,GOCM,45",
        // RECOGNISE obstacle 1

       // --- leg 1 -> 0 ---
        "STRAIGHT,GOCM,-75",
        "TURN,RIGHT,R=35,A=90",
        // RAW FORWARD 5cm
        "TURN,LEFT,R=20,A=90",
        "STRAIGHT,GOCM,40",
        "TURN,RIGHT,R=35,A=90",
        // RECOGNISE obstacle 0

        // --- leg 0 -> 2 ---
		  "STRAIGHT,GOCM,-30",
        "TURN,RIGHT,R=35,A=180",
        // RECOGNISE obstacle 2

        // --- leg 2 -> 4 ---
        "TURN,LEFT,R=20,A=90",
        "STRAIGHT,GOCM,25",
        "TURN,LEFT,R=20,A=-90",
        "STRAIGHT,GOCM,-35",
        "TURN,LEFT,R=20,A=-90",
        // RECOGNISE obstacle 4
		*/

//		// Left 1
//		"STRAIGHT,UNTILUS,42",
//		"TURN,LEFT,R=35,A=60",
//		"TURN,RIGHT,R=35,A=120",
//		"TURN,LEFT,R=35,A=60",
//		"STRAIGHT,GOCM,-33",
//		"STRAIGHT,UNTILUS,35",
//
//		// Left 2
//		"TURN,LEFT,R=35,A=90",
//		"STRAIGHT,RIGHTIR,GT,50",
//		"TURN,RIGHT,R=35,A=180",
//		"STRAIGHT,GOCM,20",
//		"STRAIGHT,RIGHTIR,GT,50",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,RIGHTIR,LT,50",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,RIGHTIR,LT,50",
//		"STRAIGHT,GOCM,-35",
//		"TURN,LEFT,R=35,A=90",
//		"STRAIGHT,UNTILUS,15",
//
//
//		// Right 1
//		"STRAIGHT,UNTILUS,42",
//		"TURN,RIGHT,R=35,A=60",
//		"TURN,LEFT,R=35,A=120",
//		"TURN,RIGHT,R=35,A=60",
//		"STRAIGHT,GOCM,-33",
//		"STRAIGHT,UNTILUS,35",
//
//		// Right 2
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,LEFTIR,GT,50",
//		"TURN,LEFT,R=35,A=180",
//		"STRAIGHT,GOCM,20",
//		"STRAIGHT,LEFTIR,GT,50",
//		"TURN,LEFT,R=35,A=90",
//		"STRAIGHT,LEFTIR,LT,50",
//		"TURN,LEFT,R=35,A=90",
//		"STRAIGHT,LEFTIR,LT,50",
//		"STRAIGHT,GOCM,-35",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,UNTILUS,15",



//		"TURN,RIGHT,R=35,A=-60",
//		"TURN,LEFT,R=35,A=-120",
//		"TURN,RIGHT,R=35,A=-60",

//		"STRAIGHT,GOCM,50",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,UNTILUS,47",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,GOCM,40",
//		"TURN,LEFT,R=20,A=180",
//		"STRAIGHT,UNTILUS,47",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,GOCM,40",
//		"TURN,LEFT,R=20,A=180",
//		"STRAIGHT,UNTILUS,47",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,GOCM,40",
//		"TURN,LEFT,R=20,A=180",
//		"STRAIGHT,UNTILUS,47",
//		"TURN,RIGHT,R=35,A=90",
//		"STRAIGHT,GOCM,40",
//		"TURN,LEFT,R=20,A=180",

    /* Other examples (uncomment/reorder to use):
    "TURN,RIGHT,R=43,A=90",
    "STRAIGHT,UNTILUS,20",
    "STRAIGHT,STATUS",
    */
};

#define EXECUTOR_STARTUP_DELAY_MS 10000U
#define EXECUTOR_COMMAND_GAP_MS   0U
#endif

void Test_HardcodeCommandExecutor(void)
{
    CommandInterface_RunSequence(commands,
        (uint32_t)(sizeof(commands) / sizeof(commands[0])),
        EXECUTOR_STARTUP_DELAY_MS, EXECUTOR_COMMAND_GAP_MS);
}
