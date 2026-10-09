#include "main.h"
#include "tests/component/test_uart.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * ============================================================
 * STEP 16B - FULL USART3 / PuTTY TEST
 * ============================================================
 *
 * PURPOSE
 * -------
 * Dedicated USART3 test only.
 * It does NOT start motors, servo, encoders, IMU, IR, or drive_control.
 *
 * It verifies:
 *   1. STM32 -> PC transmit (TX)
 *   2. PC -> STM32 receive (RX)
 *   3. Bidirectional command/reply
 *   4. CR, LF, and CRLF line endings
 *   5. Text integrity
 *   6. Repeated/burst transmit
 *   7. Receive byte/line counters
 *   8. UART HAL error counters (PE/NE/FE/ORE)
 *
 * PROJECT USART3
 * --------------
 *   USART3 TX : PD8
 *   USART3 RX : PD9
 *   Baud      : 115200
 *   Data      : 8 bits
 *   Parity    : None
 *   Stop      : 1 bit
 *   Flow      : None
 *
 * PuTTY:
 *   Speed        115200
 *   Data bits    8
 *   Stop bits    1
 *   Parity       None
 *   Flow control None
 *
 * NOTE
 * ----
 * UART_TEST_ECHO_ENABLE = 1 is convenient for PuTTY.
 * For Raspberry Pi pyserial testing later, use 0 to avoid per-byte
 * blocking echo at high back-to-back receive rates.
 */

extern UART_HandleTypeDef huart3;

#define UART_TEST_LINE_SIZE          128U
#define UART_TEST_RX_TIMEOUT_MS      10U
#define UART_TEST_ALIVE_PERIOD_MS    1000U
#define UART_TEST_ECHO_ENABLE        1U
#define UART_TEST_MAX_BURST_LINES    100U

/* ============================================================
 * TEST STATE / STATISTICS
 * ============================================================ */

static char     rx_line[UART_TEST_LINE_SIZE];
static uint32_t rx_line_len = 0U;
static uint8_t  previous_byte = 0U;

static uint32_t rx_byte_count = 0U;
static uint32_t rx_line_count = 0U;
static uint32_t tx_message_count = 0U;

static uint32_t uart_hal_error_count = 0U;
static uint32_t uart_pe_count = 0U;
static uint32_t uart_ne_count = 0U;
static uint32_t uart_fe_count = 0U;
static uint32_t uart_ore_count = 0U;

static uint8_t  alive_enabled = 1U;
static uint32_t last_alive_tick = 0U;
static uint32_t alive_seconds = 0U;


/* ============================================================
 * LOW-LEVEL TX
 * ============================================================ */

static void UART_SendRaw(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U)
    {
        return;
    }

    if (HAL_UART_Transmit(&huart3,
                          (uint8_t *)data,
                          length,
                          HAL_MAX_DELAY) == HAL_OK)
    {
        tx_message_count++;
    }
}

static void UART_Send(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    UART_SendRaw((const uint8_t *)text, (uint16_t)strlen(text));
}

static void UART_SendLine(const char *text)
{
    UART_Send(text);
    UART_Send("\r\n");
}


/* ============================================================
 * UART ERROR RECORDING
 * ============================================================ */

static void UART_RecordErrors(void)
{
    uint32_t e = HAL_UART_GetError(&huart3);

    if (e == HAL_UART_ERROR_NONE)
    {
        return;
    }

    uart_hal_error_count++;

#ifdef HAL_UART_ERROR_PE
    if ((e & HAL_UART_ERROR_PE) != 0U) uart_pe_count++;
#endif
#ifdef HAL_UART_ERROR_NE
    if ((e & HAL_UART_ERROR_NE) != 0U) uart_ne_count++;
#endif
#ifdef HAL_UART_ERROR_FE
    if ((e & HAL_UART_ERROR_FE) != 0U) uart_fe_count++;
#endif
#ifdef HAL_UART_ERROR_ORE
    if ((e & HAL_UART_ERROR_ORE) != 0U) uart_ore_count++;
#endif
}


/* ============================================================
 * HELP / REPORTS
 * ============================================================ */

static void UART_PrintBanner(void)
{
    UART_Send("\r\n");
    UART_Send("============================================================\r\n");
    UART_Send(" STEP 16B - STM32 USART3 FULL TEST\r\n");
    UART_Send("============================================================\r\n");
    UART_Send("USART3 TX = PD8\r\n");
    UART_Send("USART3 RX = PD9\r\n");
    UART_Send("115200 baud, 8-N-1, flow control NONE\r\n");
    UART_Send("No motors / servo / IMU are started by this test.\r\n");
    UART_Send("\r\n");
    UART_Send("TX is working if this banner and TX_ALIVE lines appear.\r\n");
    UART_Send("Type HELP then press Enter for all commands.\r\n");
    UART_Send("First RX test: type PING then press Enter.\r\n");
    UART_Send("Expected: PONG\r\n");
    UART_Send("============================================================\r\n");
}

static void UART_PrintHelp(void)
{
    UART_Send("\r\n");
    UART_Send("================ USART3 TEST COMMANDS ================\r\n");
    UART_Send("HELP\r\n");
    UART_Send("  Show this command list.\r\n");
    UART_Send("\r\n");
    UART_Send("PING\r\n");
    UART_Send("  Expected reply: PONG\r\n");
    UART_Send("\r\n");
    UART_Send("HELLO\r\n");
    UART_Send("  Expected reply: HELLO,STM32_USART3_OK\r\n");
    UART_Send("\r\n");
    UART_Send("UPTIME\r\n");
    UART_Send("  Show HAL tick in milliseconds.\r\n");
    UART_Send("\r\n");
    UART_Send("ECHO,<text>\r\n");
    UART_Send("  STM32 returns exactly the text after ECHO,.\r\n");
    UART_Send("  Example: ECHO,ABC123xyz\r\n");
    UART_Send("\r\n");
    UART_Send("CHARTEST\r\n");
    UART_Send("  Send printable ASCII characters 32..126.\r\n");
    UART_Send("\r\n");
    UART_Send("TESTTX\r\n");
    UART_Send("  Send fixed known TX patterns.\r\n");
    UART_Send("\r\n");
    UART_Send("BURST,<n>\r\n");
    UART_Send("  Send n numbered lines; n = 1..100.\r\n");
    UART_Send("  Example: BURST,20\r\n");
    UART_Send("\r\n");
    UART_Send("STATS\r\n");
    UART_Send("  Show RX/TX counters and UART error counters.\r\n");
    UART_Send("\r\n");
    UART_Send("CLEARSTATS\r\n");
    UART_Send("  Reset all counters to zero.\r\n");
    UART_Send("\r\n");
    UART_Send("AUTOTX,ON\r\n");
    UART_Send("AUTOTX,OFF\r\n");
    UART_Send("  Enable/disable TX_ALIVE once every second.\r\n");
    UART_Send("\r\n");
    UART_Send("BANNER\r\n");
    UART_Send("  Print startup banner again.\r\n");
    UART_Send("\r\n");
    UART_Send("NOTE: Commands are case-sensitive; use UPPERCASE.\r\n");
    UART_Send("======================================================\r\n");
}

static void UART_PrintStats(void)
{
    char msg[256];

    snprintf(msg,
             sizeof(msg),
             "STATS,rx_bytes=%lu,rx_lines=%lu,tx_msgs=%lu,"
             "hal_errors=%lu,PE=%lu,NE=%lu,FE=%lu,ORE=%lu,"
             "alive=%s,uptime_ms=%lu\r\n",
             (unsigned long)rx_byte_count,
             (unsigned long)rx_line_count,
             (unsigned long)tx_message_count,
             (unsigned long)uart_hal_error_count,
             (unsigned long)uart_pe_count,
             (unsigned long)uart_ne_count,
             (unsigned long)uart_fe_count,
             (unsigned long)uart_ore_count,
             alive_enabled ? "ON" : "OFF",
             (unsigned long)HAL_GetTick());

    UART_Send(msg);
}

static void UART_ClearStats(void)
{
    rx_byte_count = 0U;
    rx_line_count = 0U;
    tx_message_count = 0U;

    uart_hal_error_count = 0U;
    uart_pe_count = 0U;
    uart_ne_count = 0U;
    uart_fe_count = 0U;
    uart_ore_count = 0U;

    UART_SendLine("OK,STATS_CLEARED");
}


/* ============================================================
 * INDIVIDUAL TESTS
 * ============================================================ */

static void UART_CharacterTest(void)
{
    uint8_t ch;

    UART_SendLine("CHARTEST,BEGIN");
    UART_Send("ASCII_PRINTABLE:");

    for (ch = 32U; ch <= 126U; ch++)
    {
        UART_SendRaw(&ch, 1U);
    }

    UART_Send("\r\n");
    UART_SendLine("CHARTEST,END");
}

static void UART_TestTxPatterns(void)
{
    UART_SendLine("TESTTX,BEGIN");
    UART_SendLine("0123456789");
    UART_SendLine("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    UART_SendLine("abcdefghijklmnopqrstuvwxyz");
    UART_SendLine("!@#$%^&*()_+-=[]{};:',.<>/?");
    UART_SendLine("THE_QUICK_BROWN_FOX_JUMPS_OVER_THE_LAZY_DOG");
    UART_SendLine("TESTTX,END");
}

static void UART_BurstTest(uint32_t count)
{
    char msg[128];
    uint32_t i;

    if (count < 1U || count > UART_TEST_MAX_BURST_LINES)
    {
        UART_SendLine("ERR,BAD_VALUE,BURST_RANGE_1_TO_100");
        return;
    }

    UART_SendLine("BURST,BEGIN");

    for (i = 1U; i <= count; i++)
    {
        snprintf(msg,
                 sizeof(msg),
                 "BURST,%03lu/%03lu,0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n",
                 (unsigned long)i,
                 (unsigned long)count);
        UART_Send(msg);
    }

    UART_SendLine("BURST,END");
}


/* ============================================================
 * COMMAND PARSING
 * ============================================================ */

static void UART_Dispatch(char *line)
{
    if (line == NULL || line[0] == '\0')
    {
        return;
    }

    rx_line_count++;

    if (strcmp(line, "HELP") == 0)
    {
        UART_PrintHelp();
    }
    else if (strcmp(line, "PING") == 0)
    {
        UART_SendLine("PONG");
    }
    else if (strcmp(line, "HELLO") == 0)
    {
        UART_SendLine("HELLO,STM32_USART3_OK");
    }
    else if (strcmp(line, "UPTIME") == 0)
    {
        char msg[64];
        snprintf(msg,
                 sizeof(msg),
                 "UPTIME,ms=%lu\r\n",
                 (unsigned long)HAL_GetTick());
        UART_Send(msg);
    }
    else if (strncmp(line, "ECHO,", 5U) == 0)
    {
        UART_Send("ECHO_REPLY,");
        UART_Send(line + 5U);
        UART_Send("\r\n");
    }
    else if (strcmp(line, "CHARTEST") == 0)
    {
        UART_CharacterTest();
    }
    else if (strcmp(line, "TESTTX") == 0)
    {
        UART_TestTxPatterns();
    }
    else if (strncmp(line, "BURST,", 6U) == 0)
    {
        const char *num = line + 6U;
        char *endptr = NULL;
        long n = strtol(num, &endptr, 10);

        if (num[0] == '\0' || endptr == NULL || *endptr != '\0')
        {
            UART_SendLine("ERR,BAD_VALUE");
        }
        else
        {
            UART_BurstTest((uint32_t)n);
        }
    }
    else if (strcmp(line, "STATS") == 0)
    {
        UART_PrintStats();
    }
    else if (strcmp(line, "CLEARSTATS") == 0)
    {
        UART_ClearStats();
    }
    else if (strcmp(line, "AUTOTX,ON") == 0)
    {
        alive_enabled = 1U;
        last_alive_tick = HAL_GetTick();
        UART_SendLine("OK,AUTOTX=ON");
    }
    else if (strcmp(line, "AUTOTX,OFF") == 0)
    {
        alive_enabled = 0U;
        UART_SendLine("OK,AUTOTX=OFF");
    }
    else if (strcmp(line, "BANNER") == 0)
    {
        UART_PrintBanner();
    }
    else
    {
        UART_Send("RX_OK,text=");
        UART_Send(line);
        UART_Send("\r\n");
    }
}


/* ============================================================
 * RX BYTE / LINE HANDLING
 * ============================================================ */

static void UART_ProcessRxByte(uint8_t b)
{
    rx_byte_count++;

#if UART_TEST_ECHO_ENABLE
    /*
     * Echo the byte to PuTTY so typing is visible even when
     * PuTTY Local Echo is OFF.
     */
    HAL_UART_Transmit(&huart3, &b, 1U, HAL_MAX_DELAY);
#endif

    /*
     * Treat CR, LF, or CRLF as one command terminator.
     * If LF immediately follows CR, ignore the LF.
     */
    if (b == '\n' && previous_byte == '\r')
    {
        previous_byte = b;
        return;
    }

    previous_byte = b;

    if (b == '\r' || b == '\n')
    {
#if UART_TEST_ECHO_ENABLE
        /* If Enter sent only CR, add LF so PuTTY moves to next line. */
        if (b == '\r')
        {
            uint8_t lf = '\n';
            HAL_UART_Transmit(&huart3, &lf, 1U, HAL_MAX_DELAY);
        }
#endif

        if (rx_line_len > 0U)
        {
            rx_line[rx_line_len] = '\0';

            UART_Dispatch(rx_line);

            rx_line_len = 0U;
        }

        return;
    }

    if (rx_line_len < (UART_TEST_LINE_SIZE - 1U))
    {
        rx_line[rx_line_len++] = (char)b;
    }
    else
    {
        rx_line_len = 0U;
        UART_SendLine("ERR,RX_LINE_OVERFLOW");
    }
}


/* ============================================================
 * PERIODIC TX TEST
 * ============================================================ */

static void UART_PeriodicAlive(void)
{
    uint32_t now = HAL_GetTick();

    if (!alive_enabled)
    {
        return;
    }

    if ((now - last_alive_tick) >= UART_TEST_ALIVE_PERIOD_MS)
    {
        char msg[80];

        /*
         * Advance by one exact period rather than now so the counter
         * remains stable if the loop is delayed slightly.
         */
        last_alive_tick += UART_TEST_ALIVE_PERIOD_MS;
        alive_seconds++;

        snprintf(msg,
                 sizeof(msg),
                 "TX_ALIVE,%lu sec\r\n",
                 (unsigned long)alive_seconds);

        UART_Send(msg);
    }
}


/* ============================================================
 * PUBLIC ENTRY POINT
 * ============================================================ */

void Test_UART(void)
{
    uint8_t rx_byte = 0U;

    /*
     * USART3 itself must already have been initialized by main.c:
     *     MX_USART3_UART_Init();
     *
     * This dedicated test intentionally starts no other peripheral.
     */

    rx_line_len = 0U;
    previous_byte = 0U;

    rx_byte_count = 0U;
    rx_line_count = 0U;
    tx_message_count = 0U;

    uart_hal_error_count = 0U;
    uart_pe_count = 0U;
    uart_ne_count = 0U;
    uart_fe_count = 0U;
    uart_ore_count = 0U;

    alive_enabled = 1U;
    last_alive_tick = HAL_GetTick();
    alive_seconds = 0U;

    UART_PrintBanner();

    while (1)
    {
        HAL_StatusTypeDef st;

        st = HAL_UART_Receive(&huart3,
                              &rx_byte,
                              1U,
                              UART_TEST_RX_TIMEOUT_MS);

        if (st == HAL_OK)
        {
            UART_ProcessRxByte(rx_byte);
        }
        else if (st == HAL_ERROR)
        {
            UART_RecordErrors();
        }
        else
        {
            /*
             * HAL_TIMEOUT is expected whenever the user is not typing.
             * It is NOT treated as a USART error.
             */
        }

        UART_PeriodicAlive();
    }
}

