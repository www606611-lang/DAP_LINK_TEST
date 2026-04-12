#ifndef MODULES_BOARD_LED_H
#define MODULES_BOARD_LED_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void board_led_init(void);
void board_led_on(void);
void board_led_off(void);
void board_led_toggle(void);
void board_led_set(bool on);

#ifdef __cplusplus
}
#endif

#endif
