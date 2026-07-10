#ifndef MODULES_UART0_DMA_H
#define MODULES_UART0_DMA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART0_DMA_OK = 0,
    UART0_DMA_BUSY,
    UART0_DMA_INVALID_ARGUMENT,
    UART0_DMA_NOT_CONFIGURED
} uart0_dma_status_t;

void uart0_dma_init(void);
void uart0_dma_task(void);
void uart0_dma_start_rx_stream(void);
void uart0_dma_stop_rx_stream(void);

bool uart0_dma_is_configured(void);
bool uart0_dma_tx_busy(void);
bool uart0_dma_rx_busy(void);
bool uart0_dma_rx_done(void);
bool uart0_dma_rx_available(void);
bool uart0_dma_rx_overflowed(void);

uart0_dma_status_t uart0_dma_send(const uint8_t *data, uint16_t length);
uart0_dma_status_t uart0_dma_send_text(const char *text);
uart0_dma_status_t uart0_dma_receive(uint8_t *buffer, uint16_t length);
uint16_t uart0_dma_read(uint8_t *buffer, uint16_t max_length);

uint16_t uart0_dma_get_rx_count(void);
void uart0_dma_clear_rx_done(void);
void uart0_dma_clear_rx_overflow(void);

#ifdef __cplusplus
}
#endif

#endif
