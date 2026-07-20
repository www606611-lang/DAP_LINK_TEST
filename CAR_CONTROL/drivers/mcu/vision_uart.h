#ifndef DRIVERS_MCU_VISION_UART_H
#define DRIVERS_MCU_VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t received_byte_count;
    uint32_t overflow_count;
} vision_uart_stats_t;

void VisionUart_Init(void);
bool VisionUart_ReadByte(uint8_t *byte);
bool VisionUart_GetStats(vision_uart_stats_t *stats);

#endif
