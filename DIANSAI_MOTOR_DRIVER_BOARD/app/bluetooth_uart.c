#include "bluetooth_uart.h"

#include "pid_console.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define BLUETOOTH_UART_RX_BUFFER_SIZE   32U
#define BLUETOOTH_UART_MESSAGE_SIZE     96U
#define BLUETOOTH_UART_IDLE_MS          40U

static uint8_t g_bluetooth_uart_rx_buffer[BLUETOOTH_UART_RX_BUFFER_SIZE];
static char g_bluetooth_uart_message[BLUETOOTH_UART_MESSAGE_SIZE];
static uint16_t g_bluetooth_uart_message_length;
static uint32_t g_bluetooth_uart_last_rx_ms;
static bool g_bluetooth_uart_message_pending;

static void bluetooth_uart_capture(
    const uint8_t *data, uint16_t length, uint32_t now_ms);
static void bluetooth_uart_flush(uint32_t now_ms);

void bluetooth_uart_init(void)
{
    g_bluetooth_uart_message_length = 0U;
    g_bluetooth_uart_last_rx_ms = 0U;
    g_bluetooth_uart_message_pending = false;

    DL_UART_Main_enableFIFOs(UART_3_INST);
    DL_UART_Main_setRXFIFOThreshold(
        UART_3_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    while (!DL_UART_Main_isRXFIFOEmpty(UART_3_INST)) {
        (void) DL_UART_Main_receiveData(UART_3_INST);
    }
}

void bluetooth_uart_task(uint32_t now_ms)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_3_INST)) {
        g_bluetooth_uart_rx_buffer[0] = DL_UART_Main_receiveData(UART_3_INST);
        bluetooth_uart_capture(g_bluetooth_uart_rx_buffer, 1U, now_ms);
    }

    if (g_bluetooth_uart_message_pending &&
        ((uint32_t) (now_ms - g_bluetooth_uart_last_rx_ms) >=
            BLUETOOTH_UART_IDLE_MS)) {
        bluetooth_uart_flush(now_ms);
    }
}

void bluetooth_uart_send_text(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_3_INST, (uint8_t) *text);
        text++;
    }
}

static void bluetooth_uart_capture(
    const uint8_t *data, uint16_t length, uint32_t now_ms)
{
    uint16_t i;

    if (data == NULL) {
        return;
    }

    g_bluetooth_uart_last_rx_ms = now_ms;

    for (i = 0U; i < length; i++) {
        uint8_t ch = data[i];

        if (!g_bluetooth_uart_message_pending) {
            g_bluetooth_uart_message_length = 0U;
            g_bluetooth_uart_message_pending = true;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            bluetooth_uart_flush(now_ms);
            continue;
        }

        if (g_bluetooth_uart_message_length < BLUETOOTH_UART_MESSAGE_SIZE) {
            g_bluetooth_uart_message[g_bluetooth_uart_message_length++] =
                (char) ch;
        }
    }
}

static void bluetooth_uart_flush(uint32_t now_ms)
{
    if (!g_bluetooth_uart_message_pending) {
        return;
    }

    pid_console_process_line(g_bluetooth_uart_message,
        g_bluetooth_uart_message_length, now_ms, PID_CONSOLE_PORT_UART3);
    g_bluetooth_uart_message_length = 0U;
    g_bluetooth_uart_message_pending = false;
}
