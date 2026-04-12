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
#define OLED_STATUS_UART_LINE_HEIGHT     8U
#define OLED_STATUS_UART_COLUMNS         (OLED_WIDTH / OLED_6X8)
#define OLED_STATUS_UART_ROWS            2U
#define OLED_STATUS_UART_AREA_HEIGHT     (OLED_STATUS_UART_ROWS * OLED_STATUS_UART_LINE_HEIGHT)
#define OLED_STATUS_UART_Y               (OLED_HEIGHT - OLED_STATUS_UART_AREA_HEIGHT)

static uint32_t g_oled_status_last_second;
static char g_oled_status_uart_lines[OLED_STATUS_UART_ROWS]
                                    [OLED_STATUS_UART_COLUMNS + 1U];
static uint8_t g_oled_status_uart_row;
static uint8_t g_oled_status_uart_column;
static bool g_oled_status_uart_clear_before_next_char;
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
        OLED_UpdateArea(OLED_STATUS_UART_X, OLED_STATUS_UART_Y, OLED_WIDTH,
            OLED_STATUS_UART_AREA_HEIGHT);
    }
}

void oled_status_screen_uart_put(uint8_t data)
{
    if (data == '\r') {
        return;
    }

    if (data == '\n') {
        if (g_oled_status_uart_row < (OLED_STATUS_UART_ROWS - 1U)) {
            g_oled_status_uart_row++;
            g_oled_status_uart_column = 0U;
        } else {
            g_oled_status_uart_clear_before_next_char = true;
        }
        return;
    }

    if (g_oled_status_uart_clear_before_next_char) {
        oled_status_clear_uart_lines();
    } else if (g_oled_status_uart_column >= OLED_STATUS_UART_COLUMNS) {
        if (g_oled_status_uart_row < (OLED_STATUS_UART_ROWS - 1U)) {
            g_oled_status_uart_row++;
            g_oled_status_uart_column = 0U;
        } else {
            oled_status_clear_uart_lines();
        }
    }

    if ((data < ' ') || (data > '~')) {
        data = '.';
    }

    g_oled_status_uart_lines[g_oled_status_uart_row]
                            [g_oled_status_uart_column++] = (char) data;
    g_oled_status_uart_dirty = true;
}

void oled_status_screen_uart_write(const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return;
    }

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
    uint8_t row;

    for (row = 0U; row < OLED_STATUS_UART_ROWS; row++) {
        (void) memset(g_oled_status_uart_lines[row], ' ',
            OLED_STATUS_UART_COLUMNS);
        g_oled_status_uart_lines[row][OLED_STATUS_UART_COLUMNS] = '\0';
    }

    g_oled_status_uart_row                    = 0U;
    g_oled_status_uart_column                 = 0U;
    g_oled_status_uart_clear_before_next_char = false;
    g_oled_status_uart_dirty = true;
}

static void oled_status_redraw_uart(void)
{
    uint8_t row;

    OLED_ClearArea(OLED_STATUS_UART_X, OLED_STATUS_UART_Y, OLED_WIDTH,
        OLED_STATUS_UART_AREA_HEIGHT);

    for (row = 0U; row < OLED_STATUS_UART_ROWS; row++) {
        OLED_ShowString(OLED_STATUS_UART_X,
            (uint8_t) (OLED_STATUS_UART_Y +
                       row * OLED_STATUS_UART_LINE_HEIGHT),
            g_oled_status_uart_lines[row], OLED_6X8);
    }
}
