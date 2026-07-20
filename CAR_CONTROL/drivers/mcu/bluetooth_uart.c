#include "bluetooth_uart.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>

#define BLUETOOTH_UART_LINE_SIZE  96U
#define BLUETOOTH_UART_IDLE_MS    40U
#define BLUETOOTH_UART_RX_SIZE   128U
#define BLUETOOTH_UART_TX_SIZE   512U
#define BLUETOOTH_UART_TX_SYNC_BYTES 8U
#define BLUETOOTH_UART_TX_GAP_CYCLES (CPUCLK_FREQ / 50000U)

static char g_line[BLUETOOTH_UART_LINE_SIZE];
static uint16_t g_line_length;
static uint32_t g_last_rx_ms;
static bool g_receiving;
static bool g_line_ready;
static bool g_line_overflow;
static volatile uint8_t g_rx_buffer[BLUETOOTH_UART_RX_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile bool g_rx_overflow;
static volatile uint8_t g_tx_buffer[BLUETOOTH_UART_TX_SIZE];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint8_t g_tx_preamble_remaining;
static uint32_t g_baud_rate;

static void bluetooth_uart_reset_software_state(void);
static void bluetooth_uart_finish_line(void);
static void bluetooth_uart_receive_pending(void);
static void bluetooth_uart_transmit_next(void);
static uint16_t bluetooth_uart_tx_free(void);

void BluetoothUart_Init(void)
{
    bluetooth_uart_reset_software_state();
    g_baud_rate = BLUETOOTH_UART_BAUD_RATE;

    DL_GPIO_initPeripheralOutputFunctionFeatures(
        GPIO_BLUETOOTH_UART_IOMUX_TX,
        GPIO_BLUETOOTH_UART_IOMUX_TX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ_DISABLE);
    DL_UART_Main_enableFIFOs(BLUETOOTH_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        BLUETOOTH_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    while (!DL_UART_Main_isRXFIFOEmpty(BLUETOOTH_UART_INST)) {
        (void) DL_UART_Main_receiveData(BLUETOOTH_UART_INST);
    }
    DL_UART_Main_disableInterrupt(
        BLUETOOTH_UART_INST, DL_UART_MAIN_INTERRUPT_EOT_DONE);
    NVIC_ClearPendingIRQ(BLUETOOTH_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(BLUETOOTH_UART_INST_INT_IRQN);
}

static void bluetooth_uart_reset_software_state(void)
{
    g_line_length = 0U;
    g_last_rx_ms = 0U;
    g_receiving = false;
    g_line_ready = false;
    g_line_overflow = false;
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_overflow = false;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_preamble_remaining = 0U;
}

void BluetoothUart_Task(uint32_t now_ms)
{
    if (g_rx_overflow) {
        g_line_overflow = true;
        g_rx_overflow = false;
    }

    while (!g_line_ready && (g_rx_tail != g_rx_head)) {
        uint8_t ch = g_rx_buffer[g_rx_tail];

        g_rx_tail = (uint16_t) ((g_rx_tail + 1U) %
            BLUETOOTH_UART_RX_SIZE);

        g_last_rx_ms = now_ms;
        g_receiving = true;

        if (ch == (uint8_t) '\r') {
            continue;
        }
        if (ch == (uint8_t) '\n') {
            bluetooth_uart_finish_line();
            continue;
        }
        if (g_line_length < (BLUETOOTH_UART_LINE_SIZE - 1U)) {
            g_line[g_line_length++] = (char) ch;
        } else {
            g_line_overflow = true;
        }
    }

    if (!g_line_ready && g_receiving &&
        ((uint32_t) (now_ms - g_last_rx_ms) >=
            BLUETOOTH_UART_IDLE_MS)) {
        bluetooth_uart_finish_line();
    }

}

void BLUETOOTH_UART_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    do {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(BLUETOOTH_UART_INST);
        if (interrupt_index == DL_UART_MAIN_IIDX_RX) {
            bluetooth_uart_receive_pending();
        } else if (interrupt_index == DL_UART_MAIN_IIDX_EOT_DONE) {
            bluetooth_uart_transmit_next();
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

static void bluetooth_uart_receive_pending(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(BLUETOOTH_UART_INST)) {
        uint8_t ch = DL_UART_Main_receiveData(BLUETOOTH_UART_INST);
        uint16_t next_head = (uint16_t) ((g_rx_head + 1U) %
            BLUETOOTH_UART_RX_SIZE);

        if (next_head == g_rx_tail) {
            g_rx_overflow = true;
        } else {
            g_rx_buffer[g_rx_head] = ch;
            g_rx_head = next_head;
        }
    }
}

bluetooth_uart_line_result_t BluetoothUart_ReadLine(
    char *line, uint16_t capacity)
{
    bluetooth_uart_line_result_t result;
    uint16_t i;

    if (!g_line_ready) {
        return BLUETOOTH_UART_LINE_NONE;
    }

    result = g_line_overflow ?
        BLUETOOTH_UART_LINE_OVERFLOW : BLUETOOTH_UART_LINE_READY;
    if ((line != NULL) && (capacity > 0U)) {
        uint16_t copy_length = g_line_length;
        if (copy_length >= capacity) {
            copy_length = capacity - 1U;
            result = BLUETOOTH_UART_LINE_OVERFLOW;
        }
        for (i = 0U; i < copy_length; i++) {
            line[i] = g_line[i];
        }
        line[copy_length] = '\0';
    }

    g_line_length = 0U;
    g_receiving = false;
    g_line_ready = false;
    g_line_overflow = false;
    return result;
}

void BluetoothUart_WriteText(const char *text)
{
    uint32_t primask;
    uint16_t length = 0U;
    uint16_t i;

    if (text == NULL) {
        return;
    }

    while (text[length] != '\0') {
        if (length >= (BLUETOOTH_UART_TX_SIZE - 1U)) {
            return;
        }
        length++;
    }
    if (length == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (length <= bluetooth_uart_tx_free()) {
        for (i = 0U; i < length; i++) {
            g_tx_buffer[g_tx_head] = (uint8_t) text[i];
            g_tx_head = (uint16_t) ((g_tx_head + 1U) %
                BLUETOOTH_UART_TX_SIZE);
        }
        if (!DL_UART_Main_isBusy(BLUETOOTH_UART_INST)) {
            g_tx_preamble_remaining =
                BLUETOOTH_UART_TX_SYNC_BYTES + 2U;
            DL_UART_Main_clearInterruptStatus(
                BLUETOOTH_UART_INST,
                DL_UART_MAIN_INTERRUPT_EOT_DONE);
            DL_UART_Main_enableInterrupt(
                BLUETOOTH_UART_INST,
                DL_UART_MAIN_INTERRUPT_EOT_DONE);
            bluetooth_uart_transmit_next();
        }
    }
    __set_PRIMASK(primask);
}

bool BluetoothUart_SetBaudRate(uint32_t baud_rate)
{
    if ((baud_rate != 9600U) && (baud_rate != 115200U)) {
        return false;
    }
    if (!BluetoothUart_IsTxIdle()) {
        return false;
    }

    NVIC_DisableIRQ(BLUETOOTH_UART_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(BLUETOOTH_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_EOT_DONE);
    DL_UART_Main_changeConfig(BLUETOOTH_UART_INST);
    DL_UART_Main_configBaudRate(BLUETOOTH_UART_INST,
        BLUETOOTH_UART_INST_FREQUENCY, baud_rate);
    DL_UART_Main_enableFIFOs(BLUETOOTH_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        BLUETOOTH_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    bluetooth_uart_reset_software_state();
    DL_UART_Main_enable(BLUETOOTH_UART_INST);
    DL_UART_Main_clearInterruptStatus(
        BLUETOOTH_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_EOT_DONE);
    DL_UART_Main_enableInterrupt(
        BLUETOOTH_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(BLUETOOTH_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(BLUETOOTH_UART_INST_INT_IRQN);
    g_baud_rate = baud_rate;
    return true;
}

uint32_t BluetoothUart_GetBaudRate(void)
{
    return g_baud_rate;
}

bool BluetoothUart_IsTxIdle(void)
{
    return (g_tx_preamble_remaining == 0U) &&
        (g_tx_tail == g_tx_head) &&
        !DL_UART_Main_isBusy(BLUETOOTH_UART_INST);
}

static void bluetooth_uart_finish_line(void)
{
    if (!g_receiving) {
        return;
    }
    g_line[g_line_length] = '\0';
    g_line_ready = true;
}

static void bluetooth_uart_transmit_next(void)
{
    uint8_t byte;

    if (g_tx_preamble_remaining > 2U) {
        byte = 0x55U;
        g_tx_preamble_remaining--;
    } else if (g_tx_preamble_remaining > 0U) {
        byte = (g_tx_preamble_remaining == 2U) ?
            (uint8_t) '\r' : (uint8_t) '\n';
        g_tx_preamble_remaining--;
    } else if (g_tx_tail != g_tx_head) {
        byte = g_tx_buffer[g_tx_tail];
        g_tx_tail = (uint16_t) ((g_tx_tail + 1U) %
            BLUETOOTH_UART_TX_SIZE);
    } else {
        DL_UART_Main_disableInterrupt(
            BLUETOOTH_UART_INST,
            DL_UART_MAIN_INTERRUPT_EOT_DONE);
        return;
    }

    delay_cycles(BLUETOOTH_UART_TX_GAP_CYCLES);
    DL_UART_Main_transmitData(BLUETOOTH_UART_INST, byte);
}

static uint16_t bluetooth_uart_tx_free(void)
{
    if (g_tx_head >= g_tx_tail) {
        return (uint16_t) (BLUETOOTH_UART_TX_SIZE -
            (g_tx_head - g_tx_tail) - 1U);
    }

    return (uint16_t) (g_tx_tail - g_tx_head - 1U);
}
