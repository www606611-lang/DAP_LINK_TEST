#ifndef APP_K230_UART_H
#define APP_K230_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    uint16_t cx;
    uint16_t cy;
    int16_t err_x;
    int16_t err_y;
} k230_uart_target_t;

void k230_uart_init(void);
void k230_uart_task(uint32_t now_ms);
k230_uart_target_t k230_uart_get_target(void);

#ifdef __cplusplus
}
#endif

#endif
