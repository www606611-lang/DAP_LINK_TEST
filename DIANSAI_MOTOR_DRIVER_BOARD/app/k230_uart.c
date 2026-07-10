#include "k230_uart.h"

#include "lcd_status.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define K230_UART_PACKET_MAX_LEN   16U
#define K230_UART_CENTER_X         200
#define K230_UART_CENTER_Y         120
#define K230_UART_TIMEOUT_MS       150U

static char g_k230_uart_packet[K230_UART_PACKET_MAX_LEN];
static uint8_t g_k230_uart_length;
static bool g_k230_uart_receiving;
static k230_uart_target_t g_k230_uart_target;
static uint32_t g_k230_uart_last_packet_ms;
static bool g_k230_uart_online;

static void k230_uart_handle_byte(uint8_t data);
static void k230_uart_handle_packet(uint32_t now_ms);
static bool k230_uart_parse_packet(
    const char *packet, uint8_t *valid, uint16_t *cx, uint16_t *cy);
static bool k230_uart_parse_u16(const char **cursor, uint16_t *value);
static void k230_uart_set_target(
    uint8_t valid, uint16_t cx, uint16_t cy, int16_t err_x, int16_t err_y);
static void k230_uart_set_offline(void);

void k230_uart_init(void)
{
    g_k230_uart_length = 0U;
    g_k230_uart_receiving = false;
    g_k230_uart_target.valid = false;
    g_k230_uart_target.cx = 0U;
    g_k230_uart_target.cy = 0U;
    g_k230_uart_target.err_x = 0;
    g_k230_uart_target.err_y = 0;
    g_k230_uart_last_packet_ms = 0U;
    g_k230_uart_online = false;

    DL_UART_Main_enableFIFOs(UART_2_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_2_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
        (void) DL_UART_Main_receiveData(UART_2_INST);
    }

    lcd_status_screen_set_k230(0U, 0U, 0U, 0, 0);
}

void k230_uart_task(uint32_t now_ms)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
        k230_uart_handle_byte(DL_UART_Main_receiveData(UART_2_INST));
        g_k230_uart_last_packet_ms = now_ms;
        g_k230_uart_online = true;
    }

    if (g_k230_uart_online &&
        ((uint32_t) (now_ms - g_k230_uart_last_packet_ms) > K230_UART_TIMEOUT_MS)) {
        k230_uart_set_offline();
    }
}

k230_uart_target_t k230_uart_get_target(void)
{
    return g_k230_uart_target;
}

static void k230_uart_handle_byte(uint8_t data)
{
    if (data == (uint8_t) '@') {
        g_k230_uart_receiving = true;
        g_k230_uart_length = 0U;
        g_k230_uart_packet[g_k230_uart_length++] = (char) data;
        return;
    }

    if (!g_k230_uart_receiving) {
        return;
    }

    if (g_k230_uart_length >= (K230_UART_PACKET_MAX_LEN - 1U)) {
        g_k230_uart_receiving = false;
        g_k230_uart_length = 0U;
        return;
    }

    g_k230_uart_packet[g_k230_uart_length++] = (char) data;

    if (data == (uint8_t) '#') {
        g_k230_uart_packet[g_k230_uart_length] = '\0';
        g_k230_uart_receiving = false;
        k230_uart_handle_packet(g_k230_uart_last_packet_ms);
    }
}

static void k230_uart_handle_packet(uint32_t now_ms)
{
    uint8_t valid;
    uint16_t cx;
    uint16_t cy;

    if (k230_uart_parse_packet(g_k230_uart_packet, &valid, &cx, &cy)) {
        int16_t err_x = (int16_t) cx - K230_UART_CENTER_X;
        int16_t err_y = (int16_t) cy - K230_UART_CENTER_Y;

        g_k230_uart_last_packet_ms = now_ms;
        g_k230_uart_online = true;
        k230_uart_set_target(valid, cx, cy, err_x, err_y);
    }
}

static bool k230_uart_parse_packet(
    const char *packet, uint8_t *valid, uint16_t *cx, uint16_t *cy)
{
    const char *cursor = packet;

    if ((packet == NULL) || (valid == NULL) || (cx == NULL) || (cy == NULL)) {
        return false;
    }

    if (*cursor != '@') {
        return false;
    }
    cursor++;

    if ((*cursor != '0') && (*cursor != '1')) {
        return false;
    }
    *valid = (uint8_t) (*cursor - '0');
    cursor++;

    if (*cursor != ',') {
        return false;
    }
    cursor++;

    if (!k230_uart_parse_u16(&cursor, cx)) {
        return false;
    }

    if (*cursor != ',') {
        return false;
    }
    cursor++;

    if (!k230_uart_parse_u16(&cursor, cy)) {
        return false;
    }

    if (*cursor != '#') {
        return false;
    }
    cursor++;

    return (*cursor == '\0');
}

static bool k230_uart_parse_u16(const char **cursor, uint16_t *value)
{
    uint16_t parsed = 0U;
    uint8_t digits = 0U;
    const char *ptr = *cursor;

    while ((*ptr >= '0') && (*ptr <= '9')) {
        parsed = (uint16_t) (parsed * 10U + (uint16_t) (*ptr - '0'));
        ptr++;
        digits++;
    }

    if (digits == 0U) {
        return false;
    }

    *cursor = ptr;
    *value = parsed;
    return true;
}

static void k230_uart_set_target(
    uint8_t valid, uint16_t cx, uint16_t cy, int16_t err_x, int16_t err_y)
{
    g_k230_uart_target.valid = (valid != 0U);
    g_k230_uart_target.cx = cx;
    g_k230_uart_target.cy = cy;
    g_k230_uart_target.err_x = err_x;
    g_k230_uart_target.err_y = err_y;
    lcd_status_screen_set_k230(valid, cx, cy, err_x, err_y);
}

static void k230_uart_set_offline(void)
{
    g_k230_uart_online = false;
    g_k230_uart_receiving = false;
    g_k230_uart_length = 0U;
    k230_uart_set_target(0U, 0U, 0U, 0, 0);
}
