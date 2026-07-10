#ifndef MODULES_UART_DISPLAY_H
#define MODULES_UART_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_display_init(void);
void uart_display_task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
