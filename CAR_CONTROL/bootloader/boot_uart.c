#include "boot_uart.h"

#include "ti_msp_dl_config.h"

void BootUart_Init(void)
{
    DL_UART_Main_enableFIFOs(BOOT_UART_INST);
    while (!DL_UART_Main_isRXFIFOEmpty(BOOT_UART_INST)) {
        (void) DL_UART_Main_receiveData(BOOT_UART_INST);
    }
}

bool BootUart_TryRead(uint8_t *byte)
{
    if ((byte == 0) || DL_UART_Main_isRXFIFOEmpty(BOOT_UART_INST)) {
        return false;
    }
    *byte = DL_UART_Main_receiveData(BOOT_UART_INST);
    return true;
}

void BootUart_Write(const uint8_t *data, uint32_t length)
{
    if (data == 0) {
        return;
    }
    while (length-- > 0U) {
        DL_UART_Main_transmitDataBlocking(BOOT_UART_INST, *data++);
    }
}

void BootUart_WaitTxIdle(void)
{
    while (DL_UART_Main_isBusy(BOOT_UART_INST)) {
    }
}
