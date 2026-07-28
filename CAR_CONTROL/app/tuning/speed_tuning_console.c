#include "speed_tuning_console.h"

#include "bluetooth_uart.h"
#include "tuning_command_router.h"
#include "tuning_wave.h"

#include <stdint.h>

#define SPEED_TUNING_LINE_SIZE 96U

static char g_line[SPEED_TUNING_LINE_SIZE];

void SpeedTuningConsole_Init(void)
{
    speed_tuning_wave_init();
    BluetoothUart_WriteText("OK READY v=10\r\n");
}

void SpeedTuningConsole_Task(uint32_t now_ms)
{
    bluetooth_uart_line_result_t result;

    result = BluetoothUart_ReadLine(g_line, sizeof(g_line));
    if (result == BLUETOOTH_UART_LINE_OVERFLOW) {
        BluetoothUart_WriteText("ERR line\r\n");
    } else if (result == BLUETOOTH_UART_LINE_READY) {
        TuningCommandRouter_ProcessLine(g_line, now_ms);
    }

    speed_tuning_send_wave(now_ms);
}
