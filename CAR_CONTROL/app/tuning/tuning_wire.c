#include "tuning_wire.h"

#include "bluetooth_uart.h"

void speed_tuning_write_u32(uint32_t value)
{
    char buffer[12];
    uint16_t index = (uint16_t) (sizeof(buffer) - 1U);

    buffer[index] = '\0';

    do {
        buffer[--index] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    BluetoothUart_WriteText(&buffer[index]);
}

void speed_tuning_write_i32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        BluetoothUart_WriteText("-");
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    speed_tuning_write_u32(magnitude);
}

void speed_tuning_write_float4(float value)
{
    uint32_t scaled;
    uint32_t fraction;
    char digits[5];

    if (value < 0.0f) {
        BluetoothUart_WriteText("-");
        value = -value;
    }
    scaled = (uint32_t) (value * 10000.0f + 0.5f);
    fraction = scaled % 10000U;

    speed_tuning_write_u32(scaled / 10000U);
    BluetoothUart_WriteText(".");
    digits[0] = (char) ('0' + ((fraction / 1000U) % 10U));
    digits[1] = (char) ('0' + ((fraction / 100U) % 10U));
    digits[2] = (char) ('0' + ((fraction / 10U) % 10U));
    digits[3] = (char) ('0' + (fraction % 10U));
    digits[4] = '\0';
    BluetoothUart_WriteText(digits);
}

int32_t speed_tuning_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }
    return (int32_t) (value - 0.5f);
}
