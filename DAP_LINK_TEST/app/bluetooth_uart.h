#ifndef APP_BLUETOOTH_UART_H
#define APP_BLUETOOTH_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bluetooth_uart_init(void);
void bluetooth_uart_task(uint32_t now_ms);
void bluetooth_uart_send_text(const char *text);

#ifdef __cplusplus
}
#endif

#endif
