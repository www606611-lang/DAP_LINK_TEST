#include "radio_uart.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define RADIO_UART_RX_CAPACITY 256U
#define RADIO_UART_TX_CAPACITY 256U

static volatile uint8_t g_rx_buffer[RADIO_UART_RX_CAPACITY];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint8_t g_tx_buffer[RADIO_UART_TX_CAPACITY];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile bool g_tx_active;
static volatile uint32_t g_received_byte_count;
static volatile uint32_t g_rx_overflow_count;
static volatile uint32_t g_transmitted_byte_count;
static volatile uint32_t g_tx_rejected_count;

static void radio_uart_receive_pending(void);
static void radio_uart_transmit_next(void);
static uint16_t radio_uart_tx_free(void);

void RadioUart_Init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_active = false;
    g_received_byte_count = 0U;
    g_rx_overflow_count = 0U;
    g_transmitted_byte_count = 0U;
    g_tx_rejected_count = 0U;

    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_CHASSIS_RADIO_UART_IOMUX_RX,
        GPIO_CHASSIS_RADIO_UART_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_UART_Main_enableFIFOs(CHASSIS_RADIO_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(CHASSIS_RADIO_UART_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    while (!DL_UART_Main_isRXFIFOEmpty(CHASSIS_RADIO_UART_INST)) {
        (void) DL_UART_Main_receiveData(CHASSIS_RADIO_UART_INST);
    }
    DL_UART_Main_disableInterrupt(CHASSIS_RADIO_UART_INST,
        DL_UART_MAIN_INTERRUPT_EOT_DONE);
    DL_UART_Main_clearInterruptStatus(CHASSIS_RADIO_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_EOT_DONE);
    DL_UART_Main_enableInterrupt(CHASSIS_RADIO_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(CHASSIS_RADIO_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CHASSIS_RADIO_UART_INST_INT_IRQN);
}

void CHASSIS_RADIO_UART_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    do {
        interrupt_index = DL_UART_Main_getPendingInterrupt(
            CHASSIS_RADIO_UART_INST);
        if (interrupt_index == DL_UART_MAIN_IIDX_RX) {
            radio_uart_receive_pending();
        } else if (interrupt_index == DL_UART_MAIN_IIDX_EOT_DONE) {
            radio_uart_transmit_next();
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

bool RadioUart_ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_rx_tail == g_rx_head)) {
        return false;
    }

    *byte = g_rx_buffer[g_rx_tail];
    g_rx_tail = (uint16_t) ((g_rx_tail + 1U) %
        RADIO_UART_RX_CAPACITY);
    return true;
}

bool RadioUart_Write(const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint16_t index;

    if ((data == NULL) || (length == 0U)) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (length > radio_uart_tx_free()) {
        g_tx_rejected_count++;
        __set_PRIMASK(primask);
        return false;
    }

    for (index = 0U; index < length; index++) {
        g_tx_buffer[g_tx_head] = data[index];
        g_tx_head = (uint16_t) ((g_tx_head + 1U) %
            RADIO_UART_TX_CAPACITY);
    }
    if (!g_tx_active) {
        g_tx_active = true;
        DL_UART_Main_clearInterruptStatus(CHASSIS_RADIO_UART_INST,
            DL_UART_MAIN_INTERRUPT_EOT_DONE);
        DL_UART_Main_enableInterrupt(CHASSIS_RADIO_UART_INST,
            DL_UART_MAIN_INTERRUPT_EOT_DONE);
        radio_uart_transmit_next();
    }
    __set_PRIMASK(primask);
    return true;
}

bool RadioUart_GetStats(radio_uart_stats_t *stats)
{
    if (stats == NULL) {
        return false;
    }

    stats->received_byte_count = g_received_byte_count;
    stats->rx_overflow_count = g_rx_overflow_count;
    stats->transmitted_byte_count = g_transmitted_byte_count;
    stats->tx_rejected_count = g_tx_rejected_count;
    return true;
}

static void radio_uart_receive_pending(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(CHASSIS_RADIO_UART_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(CHASSIS_RADIO_UART_INST);
        uint16_t next_head = (uint16_t) ((g_rx_head + 1U) %
            RADIO_UART_RX_CAPACITY);

        g_received_byte_count++;
        if (next_head == g_rx_tail) {
            g_rx_overflow_count++;
        } else {
            g_rx_buffer[g_rx_head] = byte;
            g_rx_head = next_head;
        }
    }
}

static void radio_uart_transmit_next(void)
{
    if (g_tx_tail == g_tx_head) {
        g_tx_active = false;
        DL_UART_Main_disableInterrupt(CHASSIS_RADIO_UART_INST,
            DL_UART_MAIN_INTERRUPT_EOT_DONE);
        return;
    }

    DL_UART_Main_transmitData(CHASSIS_RADIO_UART_INST,
        g_tx_buffer[g_tx_tail]);
    g_tx_tail = (uint16_t) ((g_tx_tail + 1U) %
        RADIO_UART_TX_CAPACITY);
    g_transmitted_byte_count++;
}

static uint16_t radio_uart_tx_free(void)
{
    if (g_tx_head >= g_tx_tail) {
        return (uint16_t) (RADIO_UART_TX_CAPACITY -
            (g_tx_head - g_tx_tail) - 1U);
    }
    return (uint16_t) (g_tx_tail - g_tx_head - 1U);
}
