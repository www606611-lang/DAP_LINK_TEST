#include "vision_uart.h"

#include "ti_msp_dl_config.h"

#define VISION_UART_RX_CAPACITY 128U

static volatile uint8_t g_rx_buffer[VISION_UART_RX_CAPACITY];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_received_byte_count;
static volatile uint32_t g_overflow_count;

static void vision_uart_receive_pending(void);

void VisionUart_Init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_received_byte_count = 0U;
    g_overflow_count = 0U;

    DL_UART_Main_enableFIFOs(K230_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        K230_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST)) {
        (void) DL_UART_Main_receiveData(K230_UART_INST);
    }
    DL_UART_Main_clearInterruptStatus(
        K230_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(
        K230_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(K230_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_UART_INST_INT_IRQN);
}

void K230_UART_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    do {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(K230_UART_INST);
        if (interrupt_index == DL_UART_MAIN_IIDX_RX) {
            vision_uart_receive_pending();
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

bool VisionUart_ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_rx_tail == g_rx_head)) {
        return false;
    }

    *byte = g_rx_buffer[g_rx_tail];
    g_rx_tail = (uint16_t) ((g_rx_tail + 1U) %
        VISION_UART_RX_CAPACITY);
    return true;
}

bool VisionUart_GetStats(vision_uart_stats_t *stats)
{
    if (stats == NULL) {
        return false;
    }

    stats->received_byte_count = g_received_byte_count;
    stats->overflow_count = g_overflow_count;
    return true;
}

static void vision_uart_receive_pending(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(K230_UART_INST);
        uint16_t next_head = (uint16_t) ((g_rx_head + 1U) %
            VISION_UART_RX_CAPACITY);

        g_received_byte_count++;
        if (next_head == g_rx_tail) {
            g_overflow_count++;
        } else {
            g_rx_buffer[g_rx_head] = byte;
            g_rx_head = next_head;
        }
    }
}
