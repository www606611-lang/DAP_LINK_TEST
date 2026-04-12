#include "oled_status.h"

#include "oled.h"

#include <stdio.h>

#define OLED_STATUS_TIMER_TEXT_LEN       5U
#define OLED_STATUS_TIMER_X              (OLED_WIDTH - (OLED_STATUS_TIMER_TEXT_LEN * OLED_6X8))
#define OLED_STATUS_TIMER_Y              0U
#define OLED_STATUS_TIMER_WIDTH          (OLED_STATUS_TIMER_TEXT_LEN * OLED_6X8)
#define OLED_STATUS_TIMER_HEIGHT         8U

static uint32_t g_oled_status_last_second;

static void oled_status_draw_timer(uint32_t elapsed_ms);

void oled_status_screen_init(uint32_t elapsed_ms)
{
    g_oled_status_last_second = elapsed_ms / 1000U;

    OLED_Init();
    oled_status_draw_timer(elapsed_ms);
    OLED_Update();
}

void oled_status_screen_task(uint32_t elapsed_ms)
{
    uint32_t current_second = elapsed_ms / 1000U;

    if (current_second == g_oled_status_last_second) {
        return;
    }

    g_oled_status_last_second = current_second;
    oled_status_draw_timer(elapsed_ms);
    OLED_UpdateArea(OLED_STATUS_TIMER_X, OLED_STATUS_TIMER_Y,
        OLED_STATUS_TIMER_WIDTH, OLED_STATUS_TIMER_HEIGHT);
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
