#include "key.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define KEY_DEBOUNCE_MS    (20U)

typedef struct {
    bool raw_pressed;
    bool stable_pressed;
    bool press_event;
    bool release_event;
    uint32_t last_change_ms;
} key_state_t;

static key_state_t g_key_state[KEY_ID_COUNT];

static bool key_is_valid(key_id_t key);
static bool key_read_raw_pressed(key_id_t key);

void Key_Init(uint32_t now_ms)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t) KEY_ID_COUNT; i++) {
        bool pressed = key_read_raw_pressed((key_id_t) i);

        g_key_state[i].raw_pressed = pressed;
        g_key_state[i].stable_pressed = pressed;
        g_key_state[i].press_event = false;
        g_key_state[i].release_event = false;
        g_key_state[i].last_change_ms = now_ms;
    }
}

void Key_Task(uint32_t now_ms)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t) KEY_ID_COUNT; i++) {
        key_state_t *state = &g_key_state[i];
        bool raw_pressed = key_read_raw_pressed((key_id_t) i);

        if (raw_pressed != state->raw_pressed) {
            state->raw_pressed = raw_pressed;
            state->last_change_ms = now_ms;
        }

        if ((raw_pressed != state->stable_pressed) &&
            ((uint32_t) (now_ms - state->last_change_ms) >= KEY_DEBOUNCE_MS)) {
            state->stable_pressed = raw_pressed;
            if (raw_pressed) {
                state->press_event = true;
            } else {
                state->release_event = true;
            }
        }
    }
}

bool Key_IsPressed(key_id_t key)
{
    if (!key_is_valid(key)) {
        return false;
    }

    return g_key_state[key].stable_pressed;
}

bool Key_GetPressEvent(key_id_t key)
{
    bool pressed;

    if (!key_is_valid(key)) {
        return false;
    }

    pressed = g_key_state[key].press_event;
    g_key_state[key].press_event = false;
    return pressed;
}

bool Key_GetReleaseEvent(key_id_t key)
{
    bool released;

    if (!key_is_valid(key)) {
        return false;
    }

    released = g_key_state[key].release_event;
    g_key_state[key].release_event = false;
    return released;
}

static bool key_is_valid(key_id_t key)
{
    return ((uint32_t) key < (uint32_t) KEY_ID_COUNT);
}

static bool key_read_raw_pressed(key_id_t key)
{
    switch (key) {
        case KEY_ID_B21:
            /* B21/PB21 is wired active-low. */
            return (DL_GPIO_readPins(user_key_PORT, user_key_PIN_21_PIN) == 0U);
        case KEY_ID_DOWN:
            return (DL_GPIO_readPins(key_PORT, key_down_PIN) == 0U);
        default:
            return false;
    }
}
