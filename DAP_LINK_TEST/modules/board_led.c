#include "board_led.h"

#include "ti_msp_dl_config.h"

void board_led_init(void)
{
    /* User_led is initialized by SysConfig in SYSCFG_DL_init(). */
}

void board_led_on(void)
{
    DL_GPIO_setPins(User_led_PORT, User_led_PIN_0_PIN);
}

void board_led_off(void)
{
    DL_GPIO_clearPins(User_led_PORT, User_led_PIN_0_PIN);
}

void board_led_toggle(void)
{
    DL_GPIO_togglePins(User_led_PORT, User_led_PIN_0_PIN);
}

void board_led_set(bool on)
{
    if (on) {
        board_led_on();
    } else {
        board_led_off();
    }
}
