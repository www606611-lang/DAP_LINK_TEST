#include "board_button.h"

#include "ti_msp_dl_config.h"

#define BOARD_BUTTON_DEBOUNCE_MS    20U

typedef struct {
    bool raw_pressed;
    bool stable_pressed;
    bool press_event;
    bool release_event;
    uint32_t last_raw_change_ms;
} board_button_state_t;

static board_button_state_t g_button;

static bool board_button_read_raw(void)
{
    return DL_GPIO_readPins(USER_BUTTON_PORT, USER_BUTTON_PB21_PIN) == 0U;
}

void BoardButton_Init(uint32_t now_ms)
{
    bool pressed = board_button_read_raw();

    g_button.raw_pressed = pressed;
    g_button.stable_pressed = pressed;
    g_button.press_event = false;
    g_button.release_event = false;
    g_button.last_raw_change_ms = now_ms;
}

void BoardButton_Task(uint32_t now_ms)
{
    bool pressed = board_button_read_raw();

    if (pressed != g_button.raw_pressed) {
        g_button.raw_pressed = pressed;
        g_button.last_raw_change_ms = now_ms;
    }

    if ((pressed != g_button.stable_pressed) &&
        ((uint32_t) (now_ms - g_button.last_raw_change_ms) >=
            BOARD_BUTTON_DEBOUNCE_MS)) {
        g_button.stable_pressed = pressed;
        g_button.press_event = pressed;
        g_button.release_event = !pressed;
    }
}

bool BoardButton_IsPressed(void)
{
    return g_button.stable_pressed;
}

bool BoardButton_GetPressEvent(void)
{
    bool event = g_button.press_event;

    g_button.press_event = false;
    return event;
}

bool BoardButton_GetReleaseEvent(void)
{
    bool event = g_button.release_event;

    g_button.release_event = false;
    return event;
}
