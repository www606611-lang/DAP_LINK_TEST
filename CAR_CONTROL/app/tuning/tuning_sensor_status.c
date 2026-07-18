#include "tuning_status.h"

#include "bluetooth_uart.h"
#include "icm20948.h"
#include "tuning_wire.h"

void speed_tuning_send_imu_status(uint32_t now_ms)
{
    icm20948_snapshot_t imu;

    if (!ICM20948_GetSnapshot(&imu)) {
        BluetoothUart_WriteText("ERR imu_status\r\n");
        return;
    }

    BluetoothUart_WriteText("ISTAT ready=");
    speed_tuning_write_u32(imu.ready ? 1U : 0U);
    BluetoothUart_WriteText(" state=");
    speed_tuning_write_u32((uint32_t) imu.state);
    BluetoothUart_WriteText(" addr=");
    speed_tuning_write_u32(imu.address7);
    BluetoothUart_WriteText(" who=");
    speed_tuning_write_u32(imu.who_am_i);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) imu.last_result);
    BluetoothUart_WriteText(" samples=");
    speed_tuning_write_u32(imu.sample_count);
    BluetoothUart_WriteText(" errors=");
    speed_tuning_write_u32(imu.read_error_count);
    BluetoothUart_WriteText(" age=");
    speed_tuning_write_u32(now_ms - imu.last_sample_ms);
    BluetoothUart_WriteText(" ax=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.data.ax_g * 1000.0f));
    BluetoothUart_WriteText(" ay=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.data.ay_g * 1000.0f));
    BluetoothUart_WriteText(" az=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.data.az_g * 1000.0f));
    BluetoothUart_WriteText(" gx=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.data.gx_dps * 1000.0f));
    BluetoothUart_WriteText(" gy=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.data.gy_dps * 1000.0f));
    BluetoothUart_WriteText(" gz=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.data.gz_dps * 1000.0f));
    BluetoothUart_WriteText(" roll=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.roll_deg * 1000.0f));
    BluetoothUart_WriteText(" pitch=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.pitch_deg * 1000.0f));
    BluetoothUart_WriteText(" yaw=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.yaw_deg * 1000.0f));
    BluetoothUart_WriteText(" yr=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.yaw_rate_dps * 1000.0f));
    BluetoothUart_WriteText(" att=");
    speed_tuning_write_u32(imu.attitude_valid ? 1U : 0U);
    BluetoothUart_WriteText(" still=");
    speed_tuning_write_u32(imu.stationary ? 1U : 0U);
    BluetoothUart_WriteText(" an=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.accel_norm_g * 1000.0f));
    BluetoothUart_WriteText(" bias=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.gyro_bias_x_dps * 1000.0f));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.gyro_bias_y_dps * 1000.0f));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.gyro_bias_z_dps * 1000.0f));
    BluetoothUart_WriteText(" q=");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.quaternion_w * 1000000.0f));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.quaternion_x * 1000000.0f));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.quaternion_y * 1000000.0f));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(
        speed_tuning_round_float(imu.quaternion_z * 1000000.0f));
    BluetoothUart_WriteText("\r\n");
}

