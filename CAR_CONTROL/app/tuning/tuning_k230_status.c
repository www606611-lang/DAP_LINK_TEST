#include "tuning_status.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "chassis_radio_link.h"
#include "tuning_wire.h"

void speed_tuning_send_k230_status(uint32_t now_ms)
{
    chassis_radio_snapshot_t radio;

    if (!ChassisRadioLink_GetSnapshot(&radio)) {
        BluetoothUart_WriteText("ERR k230_status\r\n");
        return;
    }

    BluetoothUart_WriteText("KSTAT online=");
    speed_tuning_write_u32(radio.online ? 1U : 0U);
    BluetoothUart_WriteText(" esp=");
    speed_tuning_write_u32(radio.esp32_online ? 1U : 0U);
    BluetoothUart_WriteText(" k230=");
    speed_tuning_write_u32(radio.k230_online ? 1U : 0U);
    BluetoothUart_WriteText(" role=");
    speed_tuning_write_u32(radio.last_role);
    BluetoothUart_WriteText(" rx=");
    speed_tuning_write_u32(radio.rx_frame_count);
    BluetoothUart_WriteText(" tx=");
    speed_tuning_write_u32(radio.tx_frame_count);
    BluetoothUart_WriteText(" crc=");
    speed_tuning_write_u32(radio.crc_error_count);
    BluetoothUart_WriteText(" len=");
    speed_tuning_write_u32(radio.length_error_count);
    BluetoothUart_WriteText(" ver=");
    speed_tuning_write_u32(radio.version_error_count);
    BluetoothUart_WriteText(" resync=");
    speed_tuning_write_u32(radio.resync_count);
    BluetoothUart_WriteText(" dup=");
    speed_tuning_write_u32(radio.duplicate_count);
    BluetoothUart_WriteText(" old=");
    speed_tuning_write_u32(radio.out_of_order_count);
    BluetoothUart_WriteText(" shadow=");
    speed_tuning_write_u32(radio.shadow_command_count);
    BluetoothUart_WriteText(" bytes=");
    speed_tuning_write_u32(radio.received_byte_count);
    BluetoothUart_WriteText(" rxov=");
    speed_tuning_write_u32(radio.rx_overflow_count);
    BluetoothUart_WriteText(" txdrop=");
    speed_tuning_write_u32(radio.tx_rejected_count);
    BluetoothUart_WriteText(" age=");
    speed_tuning_write_u32(now_ms - radio.last_frame_ms);
    BluetoothUart_WriteText(" timeouts=");
    speed_tuning_write_u32(radio.timeout_count);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText("\r\n");
}
