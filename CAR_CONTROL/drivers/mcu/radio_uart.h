#ifndef DRIVERS_MCU_RADIO_UART_H
#define DRIVERS_MCU_RADIO_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t received_byte_count;
    uint32_t rx_overflow_count;
    uint32_t transmitted_byte_count;
    uint32_t tx_rejected_count;
} radio_uart_stats_t;

void RadioUart_Init(void);
bool RadioUart_ReadByte(uint8_t *byte);
bool RadioUart_Write(const uint8_t *data, uint16_t length);
bool RadioUart_GetStats(radio_uart_stats_t *stats);

#endif
