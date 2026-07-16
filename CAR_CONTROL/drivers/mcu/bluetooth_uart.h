#ifndef DRIVERS_MCU_BLUETOOTH_UART_H
#define DRIVERS_MCU_BLUETOOTH_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BLUETOOTH_UART_LINE_NONE = 0,
    BLUETOOTH_UART_LINE_READY,
    BLUETOOTH_UART_LINE_OVERFLOW
} bluetooth_uart_line_result_t;

void BluetoothUart_Init(void);
void BluetoothUart_Task(uint32_t now_ms);
bluetooth_uart_line_result_t BluetoothUart_ReadLine(
    char *line, uint16_t capacity);
void BluetoothUart_WriteText(const char *text);
bool BluetoothUart_SetBaudRate(uint32_t baud_rate);
uint32_t BluetoothUart_GetBaudRate(void);
bool BluetoothUart_IsTxIdle(void);

#endif
