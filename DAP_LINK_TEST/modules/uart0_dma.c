#include "uart0_dma.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

#ifndef UART0_DMA_TX_BUFFER_SIZE
#define UART0_DMA_TX_BUFFER_SIZE        128U
#endif

#ifndef UART0_DMA_RX_FIFO_SIZE
#define UART0_DMA_RX_FIFO_SIZE          128U
#endif

#define UART0_DMA_RX_CHAN_ID            DMA_CH0_CHAN_ID
#define UART0_DMA_TX_CHAN_ID            DMA_CH1_CHAN_ID

static uint8_t g_uart0_dma_tx_buffer[UART0_DMA_TX_BUFFER_SIZE];
static volatile bool g_uart0_dma_tx_busy;
static volatile bool g_uart0_dma_tx_dma_done;

static volatile bool g_uart0_dma_rx_busy;
static volatile bool g_uart0_dma_rx_done_flag;
static volatile uint16_t g_uart0_dma_rx_length;
static volatile uint16_t g_uart0_dma_rx_count;

static volatile bool g_uart0_dma_rx_stream_enabled;
static volatile bool g_uart0_dma_rx_overflow;
static volatile uint8_t g_uart0_dma_rx_byte;
static volatile uint16_t g_uart0_dma_rx_fifo_head;
static volatile uint16_t g_uart0_dma_rx_fifo_tail;
static uint8_t g_uart0_dma_rx_fifo[UART0_DMA_RX_FIFO_SIZE];

static void uart0_dma_finish_tx(void);
static void uart0_dma_finish_rx(uint16_t count);
static void uart0_dma_rearm_stream_rx(void);
static void uart0_dma_fifo_push(uint8_t data);

void uart0_dma_init(void)
{
    g_uart0_dma_tx_busy = false;
    g_uart0_dma_tx_dma_done = false;
    g_uart0_dma_rx_busy = false;
    g_uart0_dma_rx_done_flag = false;
    g_uart0_dma_rx_length = 0U;
    g_uart0_dma_rx_count = 0U;
    g_uart0_dma_rx_stream_enabled = false;
    g_uart0_dma_rx_overflow = false;
    g_uart0_dma_rx_fifo_head = 0U;
    g_uart0_dma_rx_fifo_tail = 0U;
    DL_DMA_disableChannel(DMA, UART0_DMA_RX_CHAN_ID);
    DL_DMA_disableChannel(DMA, UART0_DMA_TX_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA,
        DL_DMA_INTERRUPT_CHANNEL0 | DL_DMA_INTERRUPT_CHANNEL1);
    DL_UART_Main_clearInterruptStatus(UART_0_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
        DL_UART_MAIN_INTERRUPT_DMA_DONE_TX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void uart0_dma_task(void)
{
    if (g_uart0_dma_tx_busy && g_uart0_dma_tx_dma_done &&
        !DL_UART_Main_isBusy(UART_0_INST)) {
        uart0_dma_finish_tx();
    }
}

void uart0_dma_start_rx_stream(void)
{
    g_uart0_dma_rx_fifo_head       = 0U;
    g_uart0_dma_rx_fifo_tail       = 0U;
    g_uart0_dma_rx_overflow        = false;
    g_uart0_dma_rx_done_flag       = false;
    g_uart0_dma_rx_stream_enabled  = true;

    uart0_dma_rearm_stream_rx();
}

void uart0_dma_stop_rx_stream(void)
{
    g_uart0_dma_rx_stream_enabled = false;
    DL_DMA_disableChannel(DMA, UART0_DMA_RX_CHAN_ID);
    g_uart0_dma_rx_busy = false;
}

bool uart0_dma_is_configured(void)
{
    return true;
}

bool uart0_dma_tx_busy(void)
{
    return g_uart0_dma_tx_busy;
}

bool uart0_dma_rx_busy(void)
{
    return g_uart0_dma_rx_busy;
}

bool uart0_dma_rx_done(void)
{
    return g_uart0_dma_rx_done_flag;
}

bool uart0_dma_rx_available(void)
{
    return g_uart0_dma_rx_fifo_head != g_uart0_dma_rx_fifo_tail;
}

bool uart0_dma_rx_overflowed(void)
{
    return g_uart0_dma_rx_overflow;
}

uart0_dma_status_t uart0_dma_send(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) ||
        (length > UART0_DMA_TX_BUFFER_SIZE)) {
        return UART0_DMA_INVALID_ARGUMENT;
    }

    uart0_dma_task();
    if (g_uart0_dma_tx_busy) {
        return UART0_DMA_BUSY;
    }

    (void) memcpy(g_uart0_dma_tx_buffer, data, length);
    g_uart0_dma_tx_busy     = true;
    g_uart0_dma_tx_dma_done = false;

    DL_DMA_disableChannel(DMA, UART0_DMA_TX_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, UART0_DMA_TX_CHAN_ID,
        (uint32_t) &g_uart0_dma_tx_buffer[0]);
    DL_DMA_setDestAddr(DMA, UART0_DMA_TX_CHAN_ID,
        (uint32_t) &UART_0_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, UART0_DMA_TX_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, UART0_DMA_TX_CHAN_ID);

    return UART0_DMA_OK;
}

uart0_dma_status_t uart0_dma_send_text(const char *text)
{
    size_t length;

    if (text == NULL) {
        return UART0_DMA_INVALID_ARGUMENT;
    }

    length = strlen(text);
    if (length > UINT16_MAX) {
        return UART0_DMA_INVALID_ARGUMENT;
    }

    return uart0_dma_send((const uint8_t *) text, (uint16_t) length);
}

uart0_dma_status_t uart0_dma_receive(uint8_t *buffer, uint16_t length)
{
    if ((buffer == NULL) || (length == 0U)) {
        return UART0_DMA_INVALID_ARGUMENT;
    }
    if (g_uart0_dma_rx_busy || g_uart0_dma_rx_stream_enabled) {
        return UART0_DMA_BUSY;
    }

    g_uart0_dma_rx_busy      = true;
    g_uart0_dma_rx_done_flag = false;
    g_uart0_dma_rx_length    = length;
    g_uart0_dma_rx_count     = 0U;

    DL_DMA_disableChannel(DMA, UART0_DMA_RX_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, UART0_DMA_RX_CHAN_ID,
        (uint32_t) &UART_0_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, UART0_DMA_RX_CHAN_ID, (uint32_t) buffer);
    DL_DMA_setTransferSize(DMA, UART0_DMA_RX_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, UART0_DMA_RX_CHAN_ID);

    return UART0_DMA_OK;
}

uint16_t uart0_dma_read(uint8_t *buffer, uint16_t max_length)
{
    uint16_t count = 0U;

    if ((buffer == NULL) || (max_length == 0U)) {
        return 0U;
    }

    while ((count < max_length) &&
           (g_uart0_dma_rx_fifo_tail != g_uart0_dma_rx_fifo_head)) {
        buffer[count++] = g_uart0_dma_rx_fifo[g_uart0_dma_rx_fifo_tail];
        g_uart0_dma_rx_fifo_tail =
            (uint16_t) ((g_uart0_dma_rx_fifo_tail + 1U) %
                        UART0_DMA_RX_FIFO_SIZE);
    }

    return count;
}

uint16_t uart0_dma_get_rx_count(void)
{
    return g_uart0_dma_rx_count;
}

void uart0_dma_clear_rx_done(void)
{
    g_uart0_dma_rx_done_flag = false;
    g_uart0_dma_rx_count     = 0U;
}

void uart0_dma_clear_rx_overflow(void)
{
    g_uart0_dma_rx_overflow = false;
}

static void uart0_dma_finish_tx(void)
{
    DL_DMA_disableChannel(DMA, UART0_DMA_TX_CHAN_ID);
    g_uart0_dma_tx_busy     = false;
    g_uart0_dma_tx_dma_done = false;
}

static void uart0_dma_finish_rx(uint16_t count)
{
    DL_DMA_disableChannel(DMA, UART0_DMA_RX_CHAN_ID);
    g_uart0_dma_rx_count     = count;
    g_uart0_dma_rx_busy      = false;
    g_uart0_dma_rx_done_flag = true;
}

static void uart0_dma_rearm_stream_rx(void)
{
    g_uart0_dma_rx_busy = true;

    DL_DMA_disableChannel(DMA, UART0_DMA_RX_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, UART0_DMA_RX_CHAN_ID,
        (uint32_t) &UART_0_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, UART0_DMA_RX_CHAN_ID,
        (uint32_t) &g_uart0_dma_rx_byte);
    DL_DMA_setTransferSize(DMA, UART0_DMA_RX_CHAN_ID, 1U);
    DL_DMA_enableChannel(DMA, UART0_DMA_RX_CHAN_ID);
}

static void uart0_dma_fifo_push(uint8_t data)
{
    uint16_t next_head =
        (uint16_t) ((g_uart0_dma_rx_fifo_head + 1U) % UART0_DMA_RX_FIFO_SIZE);

    if (next_head == g_uart0_dma_rx_fifo_tail) {
        g_uart0_dma_rx_overflow = true;
        return;
    }

    g_uart0_dma_rx_fifo[g_uart0_dma_rx_fifo_head] = data;
    g_uart0_dma_rx_fifo_head = next_head;
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_DMA_DONE_TX:
            g_uart0_dma_tx_dma_done = true;
            uart0_dma_task();
            break;
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            if (g_uart0_dma_rx_stream_enabled) {
                uart0_dma_fifo_push(g_uart0_dma_rx_byte);
                uart0_dma_rearm_stream_rx();
            } else {
                uart0_dma_finish_rx(g_uart0_dma_rx_length);
            }
            break;
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            if (!g_uart0_dma_rx_stream_enabled && g_uart0_dma_rx_busy) {
                uint16_t remaining =
                    DL_DMA_getTransferSize(DMA, UART0_DMA_RX_CHAN_ID);

                uart0_dma_finish_rx(
                    (uint16_t) (g_uart0_dma_rx_length - remaining));
            }
            break;
        default:
            break;
    }
}
