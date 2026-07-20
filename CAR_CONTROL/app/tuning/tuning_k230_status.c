#include "tuning_status.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "k230_vision_link.h"
#include "tuning_wire.h"

void speed_tuning_send_k230_status(uint32_t now_ms)
{
    k230_vision_snapshot_t vision;

    if (!K230VisionLink_GetSnapshot(&vision)) {
        BluetoothUart_WriteText("ERR k230_status\r\n");
        return;
    }

    BluetoothUart_WriteText("KSTAT online=");
    speed_tuning_write_u32(vision.online ? 1U : 0U);
    BluetoothUart_WriteText(" valid=");
    speed_tuning_write_u32(vision.target_valid ? 1U : 0U);
    BluetoothUart_WriteText(" cx=");
    speed_tuning_write_u32(vision.cx);
    BluetoothUart_WriteText(" cy=");
    speed_tuning_write_u32(vision.cy);
    BluetoothUart_WriteText(" ex=");
    speed_tuning_write_i32(vision.error_x);
    BluetoothUart_WriteText(" ey=");
    speed_tuning_write_i32(vision.error_y);
    BluetoothUart_WriteText(" frames=");
    speed_tuning_write_u32(vision.frame_sequence);
    BluetoothUart_WriteText(" invalid=");
    speed_tuning_write_u32(vision.parse_error_count);
    BluetoothUart_WriteText(" resync=");
    speed_tuning_write_u32(vision.resync_count);
    BluetoothUart_WriteText(" bytes=");
    speed_tuning_write_u32(vision.received_byte_count);
    BluetoothUart_WriteText(" overflow=");
    speed_tuning_write_u32(vision.overflow_count);
    BluetoothUart_WriteText(" age=");
    speed_tuning_write_u32(now_ms - vision.last_frame_ms);
    BluetoothUart_WriteText(" timeouts=");
    speed_tuning_write_u32(vision.timeout_count);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText("\r\n");
}
