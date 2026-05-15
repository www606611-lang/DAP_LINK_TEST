#include "uart_display.h"

#include "lcd_status.h"
#include "uart0_dma.h"

#include <stdbool.h>
#include <stddef.h>

#define UART_DISPLAY_RX_BUFFER_SIZE     32U
#define UART_DISPLAY_MESSAGE_SIZE       64U
/* 串口空闲超过这个时间就认为一帧接收完成，再刷新到 LCD。 */
#define UART_DISPLAY_IDLE_MS            40U

static uint8_t g_uart_display_rx_buffer[UART_DISPLAY_RX_BUFFER_SIZE];
static uint8_t g_uart_display_message[UART_DISPLAY_MESSAGE_SIZE];
static uint16_t g_uart_display_message_length;
static uint32_t g_uart_display_last_rx_ms;
static bool g_uart_display_message_pending;

static void uart_display_capture(const uint8_t *data, uint16_t length,
    uint32_t now_ms);
static void uart_display_flush(void);

void uart_display_init(void)
{
    uart0_dma_init();
    uart0_dma_start_rx_stream();
}

void uart_display_task(uint32_t now_ms)
{
    uint16_t uart_rx_length;

    uart0_dma_task();

    /* DMA 持续收串口，应用层只取出新增数据并拼成一帧。 */
    uart_rx_length = uart0_dma_read(g_uart_display_rx_buffer,
        UART_DISPLAY_RX_BUFFER_SIZE);
    if (uart_rx_length > 0U) {
        uart_display_capture(g_uart_display_rx_buffer, uart_rx_length, now_ms);
    }

    if (g_uart_display_message_pending &&
        ((uint32_t) (now_ms - g_uart_display_last_rx_ms) >=
            UART_DISPLAY_IDLE_MS)) {
        uart_display_flush();
    }
}

static void uart_display_capture(const uint8_t *data, uint16_t length,
    uint32_t now_ms)
{
    uint16_t i;

    if (data == NULL) {
        return;
    }

    g_uart_display_last_rx_ms = now_ms;

    for (i = 0U; i < length; i++) {
        uint8_t ch = data[i];

        if (!g_uart_display_message_pending) {
            g_uart_display_message_length  = 0U;
            g_uart_display_message_pending = true;
        }

        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            uart_display_flush();
            continue;
        }

        if (g_uart_display_message_length < UART_DISPLAY_MESSAGE_SIZE) {
            g_uart_display_message[g_uart_display_message_length++] = ch;
        }
    }
}

static void uart_display_flush(void)
{
    /* LCD 只显示最近一帧串口内容，换行或空闲超时都会触发这里。 */
    lcd_status_screen_uart_write(g_uart_display_message,
        g_uart_display_message_length);
    g_uart_display_message_length  = 0U;
    g_uart_display_message_pending = false;
}
