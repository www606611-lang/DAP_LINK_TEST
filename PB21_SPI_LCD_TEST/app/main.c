#include "board_button.h"
#include "delay.h"
#include "st7789.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

volatile bool g_pb21_pressed;
volatile uint32_t g_pb21_press_count;
volatile uint32_t g_pb21_release_count;
volatile uint32_t g_pb21_last_change_ms;

static void app_display_init(void);
static void app_display_update(uint32_t now_ms);

int main(void)
{
    uint32_t displayed_second = UINT32_MAX;
    bool display_dirty = true;

    SYSCFG_DL_init();
    BoardButton_Init(delay_get_ms());
    g_pb21_pressed = BoardButton_IsPressed();

    ST7789_Init();
    app_display_init();

    while (1) {
        uint32_t now_ms = delay_get_ms();
        uint32_t now_second = now_ms / 1000U;

        BoardButton_Task(now_ms);

        if (BoardButton_GetPressEvent()) {
            g_pb21_pressed = true;
            g_pb21_press_count++;
            g_pb21_last_change_ms = now_ms;
            display_dirty = true;
        }

        if (BoardButton_GetReleaseEvent()) {
            g_pb21_pressed = false;
            g_pb21_release_count++;
            g_pb21_last_change_ms = now_ms;
            display_dirty = true;
        }

        if (now_second != displayed_second) {
            displayed_second = now_second;
            display_dirty = true;
        }

        if (display_dirty) {
            app_display_update(now_ms);
            display_dirty = false;
        }

        __WFI();
    }
}

static void app_display_init(void)
{
    ST7789_Fill(ST7789_COLOR_BLACK);
    ST7789_FillRect(0U, 0U, ST7789_WIDTH, 28U, ST7789_COLOR_BLUE);
    ST7789_ShowString(8U, 6U, "PB21 BUTTON TEST", ST7789_8X16,
        ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
    ST7789_ShowString(8U, 42U, "STATE", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
    ST7789_ShowString(8U, 72U, "PRESS COUNT", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
    ST7789_ShowString(8U, 96U, "RELEASE COUNT", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
    ST7789_ShowString(8U, 120U, "LAST CHANGE", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
    ST7789_ShowString(8U, 144U, "UPTIME", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
}

static void app_display_update(uint32_t now_ms)
{
    uint16_t state_color = g_pb21_pressed ?
        ST7789_COLOR_GREEN : ST7789_COLOR_YELLOW;

    ST7789_Printf(128U, 42U, ST7789_8X16, state_color,
        ST7789_COLOR_BLACK, "%-9s", g_pb21_pressed ? "PRESSED" : "RELEASED");
    ST7789_Printf(184U, 72U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "%8lu", (unsigned long) g_pb21_press_count);
    ST7789_Printf(184U, 96U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "%8lu", (unsigned long) g_pb21_release_count);
    ST7789_Printf(184U, 120U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "%8lu ms", (unsigned long) g_pb21_last_change_ms);
    ST7789_Printf(184U, 144U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "%8lu s", (unsigned long) (now_ms / 1000U));
}
