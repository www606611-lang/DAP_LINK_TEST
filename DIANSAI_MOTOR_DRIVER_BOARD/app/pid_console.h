#ifndef APP_PID_CONSOLE_H
#define APP_PID_CONSOLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_CONSOLE_PORT_UART0 = 0U,
    PID_CONSOLE_PORT_UART3 = 1U
} pid_console_port_t;

void pid_console_process_line(const char *line, uint16_t length,
    uint32_t now_ms, pid_console_port_t port);

#ifdef __cplusplus
}
#endif

#endif
