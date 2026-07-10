#include "oled_status.h"

#include "encoder.h"
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

#define OLED_STATUS_ENCODER_TEXT_LEN     8U
#define OLED_STATUS_ENCODER_X            (OLED_WIDTH - (OLED_STATUS_ENCODER_TEXT_LEN * OLED_6X8))
#define OLED_STATUS_ENCODER_LEFT_Y       8U
#define OLED_STATUS_ENCODER_RIGHT_Y      16U
#define OLED_STATUS_ENCODER_WIDTH        (OLED_STATUS_ENCODER_TEXT_LEN * OLED_6X8)
#define OLED_STATUS_ENCODER_HEIGHT       16U
#define OLED_STATUS_ENCODER_DRAW_MS      100U
#define OLED_STATUS_ENCODER_COUNT_MAX    99999L

static uint32_t g_oled_status_last_second;
static uint32_t g_oled_status_last_encoder_draw_ms;
static int32_t g_oled_status_left_count;
static int32_t g_oled_status_right_count;
static char g_oled_status_uart_line[OLED_STATUS_UART_COLUMNS + 1U];
static uint8_t g_oled_status_uart_column;
static bool g_oled_status_uart_dirty;

static void oled_status_draw_timer(uint32_t elapsed_ms);
static void oled_status_draw_encoders(void);
static void oled_status_draw_encoder_line(
    uint8_t y, char label, int32_t count);
static int32_t oled_status_clip_encoder_count(int32_t count);
static void oled_status_clear_uart_lines(void);
static void oled_status_redraw_uart(void);

void oled_status_screen_init(uint32_t elapsed_ms)
{
    g_oled_status_last_second = elapsed_ms / 1000U;
    g_oled_status_last_encoder_draw_ms = elapsed_ms;

    OLED_Init();
    oled_status_clear_uart_lines();
    oled_status_draw_timer(elapsed_ms);
    oled_status_draw_encoders();
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

    if ((uint32_t) (elapsed_ms - g_oled_status_last_encoder_draw_ms) >=
        OLED_STATUS_ENCODER_DRAW_MS) {
        int32_t left_count =
            oled_status_clip_encoder_count(Encoder_GetCount(ENCODER_LEFT));
        int32_t right_count =
            oled_status_clip_encoder_count(Encoder_GetCount(ENCODER_RIGHT));

        g_oled_status_last_encoder_draw_ms = elapsed_ms;

        if ((left_count != g_oled_status_left_count) ||
            (right_count != g_oled_status_right_count)) {
            g_oled_status_left_count = left_count;
            g_oled_status_right_count = right_count;
            oled_status_draw_encoders();
            OLED_UpdateArea(OLED_STATUS_ENCODER_X, OLED_STATUS_ENCODER_LEFT_Y,
                OLED_STATUS_ENCODER_WIDTH, OLED_STATUS_ENCODER_HEIGHT);
        }
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

static void oled_status_draw_encoders(void)
{
    oled_status_draw_encoder_line(
        OLED_STATUS_ENCODER_LEFT_Y, 'L', g_oled_status_left_count);
    oled_status_draw_encoder_line(
        OLED_STATUS_ENCODER_RIGHT_Y, 'R', g_oled_status_right_count);
}

static void oled_status_draw_encoder_line(
    uint8_t y, char label, int32_t count)
{
    char encoder_text[16U];

    (void) snprintf(encoder_text, sizeof(encoder_text), "%c:%+6ld", label,
        (long) oled_status_clip_encoder_count(count));
    encoder_text[OLED_STATUS_ENCODER_TEXT_LEN] = '\0';

    OLED_ClearArea(OLED_STATUS_ENCODER_X, y, OLED_STATUS_ENCODER_WIDTH, 8U);
    OLED_ShowString(OLED_STATUS_ENCODER_X, y, encoder_text, OLED_6X8);
}

static int32_t oled_status_clip_encoder_count(int32_t count)
{
    if (count > OLED_STATUS_ENCODER_COUNT_MAX) {
        return OLED_STATUS_ENCODER_COUNT_MAX;
    }
    if (count < -OLED_STATUS_ENCODER_COUNT_MAX) {
        return -OLED_STATUS_ENCODER_COUNT_MAX;
    }
    return count;
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
