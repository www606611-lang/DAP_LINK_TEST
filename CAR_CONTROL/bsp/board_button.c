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

static board_button_state_t g_buttons[BOARD_BUTTON_ID_COUNT];

static bool board_button_id_is_valid(board_button_id_t button)
{
    return (uint32_t) button < (uint32_t) BOARD_BUTTON_ID_COUNT;
}

static bool board_button_read_raw(board_button_id_t button)
{
    switch (button) {
        case BOARD_BUTTON_ID_PB21:
            return DL_GPIO_readPins(
                USER_BUTTON_PORT, USER_BUTTON_PB21_PIN) == 0U;
        case BOARD_BUTTON_ID_SW2_PB4:
            return DL_GPIO_readPins(
                POSITION_BUTTONS_PORT,
                POSITION_BUTTONS_SW2_PB4_PIN) == 0U;
        case BOARD_BUTTON_ID_SW1_PB5:
            return DL_GPIO_readPins(
                POSITION_BUTTONS_PORT,
                POSITION_BUTTONS_SW1_PB5_PIN) == 0U;
        default:
            return false;
    }
}

void BoardButton_Init(uint32_t now_ms)
{
    uint32_t button;

    for (button = 0U; button < (uint32_t) BOARD_BUTTON_ID_COUNT;
         button++) {
        bool pressed = board_button_read_raw((board_button_id_t) button);

        g_buttons[button].raw_pressed = pressed;
        g_buttons[button].stable_pressed = pressed;
        g_buttons[button].press_event = false;
        g_buttons[button].release_event = false;
        g_buttons[button].last_raw_change_ms = now_ms;
    }
}

void BoardButton_Task(uint32_t now_ms)
{
    uint32_t button;

    for (button = 0U; button < (uint32_t) BOARD_BUTTON_ID_COUNT;
         button++) {
        board_button_state_t *state = &g_buttons[button];
        bool pressed = board_button_read_raw((board_button_id_t) button);

        if (pressed != state->raw_pressed) {
            state->raw_pressed = pressed;
            state->last_raw_change_ms = now_ms;
        }

        if ((pressed != state->stable_pressed) &&
            ((uint32_t) (now_ms - state->last_raw_change_ms) >=
                BOARD_BUTTON_DEBOUNCE_MS)) {
            state->stable_pressed = pressed;
            state->press_event = pressed;
            state->release_event = !pressed;
        }
    }
}

bool BoardButton_IsPressed(void)
{
    return BoardButton_IsPressedId(BOARD_BUTTON_ID_PB21);
}

bool BoardButton_GetPressEvent(void)
{
    return BoardButton_GetPressEventId(BOARD_BUTTON_ID_PB21);
}

bool BoardButton_GetReleaseEvent(void)
{
    return BoardButton_GetReleaseEventId(BOARD_BUTTON_ID_PB21);
}

bool BoardButton_IsPressedId(board_button_id_t button)
{
    if (!board_button_id_is_valid(button)) {
        return false;
    }
    return g_buttons[(uint32_t) button].stable_pressed;
}

bool BoardButton_GetPressEventId(board_button_id_t button)
{
    bool event;

    if (!board_button_id_is_valid(button)) {
        return false;
    }
    event = g_buttons[(uint32_t) button].press_event;
    g_buttons[(uint32_t) button].press_event = false;
    return event;
}

bool BoardButton_GetReleaseEventId(board_button_id_t button)
{
    bool event;

    if (!board_button_id_is_valid(button)) {
        return false;
    }
    event = g_buttons[(uint32_t) button].release_event;
    g_buttons[(uint32_t) button].release_event = false;
    return event;
}
