#include "oled_status.h"

#include "oled.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define OLED_STATUS_TIMER_TEXT_LEN       5U
#define OLED_STATUS_TIMER_X              (OLED_WIDTH - (OLED_STATUS_TIMER_TEXT_LEN * OLED_6X8))
#define OLED_STATUS_TIMER_Y              0U
#define OLED_STATUS_TIMER_WIDTH          (OLED_STATUS_TIMER_TEXT_LEN * OLED_6X8)
#define OLED_STATUS_TIMER_HEIGHT         8U

#define OLED_STATUS_UART_X               0U
#define OLED_STATUS_UART_Y               0U
#define OLED_STATUS_UART_WIDTH           OLED_STATUS_TIMER_X
#define OLED_STATUS_UART_HEIGHT          8U
#define OLED_STATUS_UART_PROMPT          "RX:"
#define OLED_STATUS_UART_PROMPT_LEN      3U
#define OLED_STATUS_UART_COLUMNS         (OLED_STATUS_UART_WIDTH / OLED_6X8)

static uint32_t g_oled_status_last_second;
static char g_oled_status_uart_line[OLED_STATUS_UART_COLUMNS + 1U];
static uint8_t g_oled_status_uart_column;
static bool g_oled_status_uart_dirty;

static void oled_status_draw_timer(uint32_t elapsed_ms);
static void oled_status_clear_uart_lines(void);
static void oled_status_redraw_uart(void);

void oled_status_screen_init(uint32_t elapsed_ms)
{
    g_oled_status_last_second = elapsed_ms / 1000U;

    OLED_Init();
    oled_status_clear_uart_lines();
    oled_status_draw_timer(elapsed_ms);
    oled_status_redraw_uart();
    OLED_Update();
}

void oled_status_screen_task(uint32_t elapsed_ms)
{
    uint32_t current_second = elapsed_ms / 1000U;

    if (current_second != g_oled_status_last_second) {
        g_oled_status_last_second = current_second;
        oled_status_draw_timer(elapsed_ms);
        OLED_UpdateArea(OLED_STATUS_TIMER_X, OLED_STATUS_TIMER_Y,
            OLED_STATUS_TIMER_WIDTH, OLED_STATUS_TIMER_HEIGHT);
    }

    if (g_oled_status_uart_dirty) {
        g_oled_status_uart_dirty = false;
        oled_status_redraw_uart();
        OLED_UpdateArea(OLED_STATUS_UART_X, OLED_STATUS_UART_Y,
            OLED_STATUS_UART_WIDTH, OLED_STATUS_UART_HEIGHT);
    }
}

void oled_status_screen_uart_put(uint8_t data)
{
    if ((data == '\r') || (data == '\n')) {
        return;
    }

    if (g_oled_status_uart_column >= OLED_STATUS_UART_COLUMNS) {
        return;
    }

    if ((data < ' ') || (data > '~')) {
        data = '.';
    }

    g_oled_status_uart_line[g_oled_status_uart_column++] = (char) data;
    g_oled_status_uart_dirty = true;
}

void oled_status_screen_uart_write(const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return;
    }

    oled_status_clear_uart_lines();

    for (uint16_t i = 0U; i < length; i++) {
        oled_status_screen_uart_put(data[i]);
    }
}

static void oled_status_draw_timer(uint32_t elapsed_ms)
{
    char timer_text[OLED_STATUS_TIMER_TEXT_LEN + 1U];
    uint32_t elapsed_seconds = elapsed_ms / 1000U;
    uint32_t minutes = (elapsed_seconds / 60U) % 100U;
    uint32_t seconds = elapsed_seconds % 60U;

    (void) snprintf(timer_text, sizeof(timer_text), "%02lu:%02lu",
        (unsigned long) minutes, (unsigned long) seconds);

    OLED_ClearArea(OLED_STATUS_TIMER_X, OLED_STATUS_TIMER_Y,
        OLED_STATUS_TIMER_WIDTH, OLED_STATUS_TIMER_HEIGHT);
    OLED_ShowString(OLED_STATUS_TIMER_X, OLED_STATUS_TIMER_Y, timer_text,
        OLED_6X8);
}

static void oled_status_clear_uart_lines(void)
{
    (void) memset(g_oled_status_uart_line, ' ', OLED_STATUS_UART_COLUMNS);
    g_oled_status_uart_line[OLED_STATUS_UART_COLUMNS] = '\0';
    (void) memcpy(g_oled_status_uart_line, OLED_STATUS_UART_PROMPT,
        OLED_STATUS_UART_PROMPT_LEN);
    g_oled_status_uart_column = OLED_STATUS_UART_PROMPT_LEN;
    g_oled_status_uart_dirty = true;
}

static void oled_status_redraw_uart(void)
{
    OLED_ClearArea(OLED_STATUS_UART_X, OLED_STATUS_UART_Y,
        OLED_STATUS_UART_WIDTH, OLED_STATUS_UART_HEIGHT);
    OLED_ShowString(OLED_STATUS_UART_X, OLED_STATUS_UART_Y,
        g_oled_status_uart_line, OLED_6X8);
}
