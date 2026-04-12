#ifndef MODULES_OLED_STATUS_H
#define MODULES_OLED_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void oled_status_screen_init(uint32_t elapsed_ms);
void oled_status_screen_task(uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif
