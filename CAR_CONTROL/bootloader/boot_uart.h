#ifndef BOOTLOADER_BOOT_UART_H
#define BOOTLOADER_BOOT_UART_H

#include <stdbool.h>
#include <stdint.h>

void BootUart_Init(void);
bool BootUart_TryRead(uint8_t *byte);
void BootUart_Write(const uint8_t *data, uint32_t length);
void BootUart_WaitTxIdle(void);

#endif
