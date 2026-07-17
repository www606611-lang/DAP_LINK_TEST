#include "tuning_wave.h"

#include "bluetooth_uart.h"
#include "line_sensor_bringup.h"
#include "tuning_wire.h"
#include "wheel_heading_control.h"
#include "wheel_line_tracking_control.h"
#include "wheel_position_control.h"
#include "wheel_speed_control.h"
#include "wheel_yaw_control.h"

#define SPEED_TUNING_WAVE_INTERVAL_MS 50U

static uint32_t g_last_wave_ms;

void speed_tuning_wave_init(void)
{
    g_last_wave_ms = 0U;
}

void speed_tuning_send_wave(uint32_t now_ms)
{
    wheel_heading_control_snapshot_t heading;
    line_sensor_snapshot_t line_sensor;
    wheel_line_tracking_snapshot_t line_tracking;
    wheel_position_control_snapshot_t position;
    wheel_speed_control_snapshot_t speed;
    wheel_yaw_control_snapshot_t yaw;

    if ((uint32_t) (now_ms - g_last_wave_ms) <
        SPEED_TUNING_WAVE_INTERVAL_MS) {
        return;
    }
    g_last_wave_ms = now_ms;

    if (WheelLineTrackingControl_GetSnapshot(&line_tracking) &&
        line_tracking.running &&
        LineSensorBringup_GetSnapshot(&line_sensor) &&
        WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("linewave:");
        speed_tuning_write_i32(line_tracking.line_error);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            line_tracking.correction_target_pps));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            line_tracking.left_speed_target_pps));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed.left_measured_pps);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            line_tracking.right_speed_target_pps));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed.right_measured_pps);
        BluetoothUart_WriteText(",");
        speed_tuning_write_u32(line_sensor.active_count);
        BluetoothUart_WriteText("\r\n");
        return;
    }

    if (WheelHeadingControl_GetSnapshot(&heading) && heading.running &&
        WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("yawwave:");
        speed_tuning_write_i32(speed_tuning_round_float(
            heading.target_yaw_deg * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            heading.current_yaw_deg * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            heading.error_deg * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            heading.yaw_rate_dps * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            heading.correction_target_pps));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed.left_measured_pps);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed.right_measured_pps);
        BluetoothUart_WriteText("\r\n");
        return;
    }

    if (WheelYawControl_GetSnapshot(&yaw) && yaw.running &&
        WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("yawwave:");
        speed_tuning_write_i32(speed_tuning_round_float(
            yaw.target_yaw_deg * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            yaw.current_yaw_deg * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            yaw.error_deg * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            yaw.yaw_rate_dps * 1000.0f));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            yaw.turn_speed_target_pps));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed.left_measured_pps);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed.right_measured_pps);
        BluetoothUart_WriteText("\r\n");
        return;
    }

    if (!WheelPositionControl_GetSnapshot(&position)) {
        return;
    }

    if (position.running) {
        BluetoothUart_WriteText("poswave:");
        speed_tuning_write_i32(position.left_target_count);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(position.left_count);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(position.right_target_count);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(position.right_count);
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            position.left_speed_target_pps));
        BluetoothUart_WriteText(",");
        speed_tuning_write_i32(speed_tuning_round_float(
            position.right_speed_target_pps));
        BluetoothUart_WriteText("\r\n");
        return;
    }

    if (!WheelSpeedControl_GetSnapshot(&speed)) {
        return;
    }

    BluetoothUart_WriteText("wave:");
    speed_tuning_write_i32(
        speed_tuning_round_float(speed.left_target_pps));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(speed.left_measured_pps);
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(
        speed_tuning_round_float(speed.right_target_pps));
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(speed.right_measured_pps);
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(speed.left_output_permille);
    BluetoothUart_WriteText(",");
    speed_tuning_write_i32(speed.right_output_permille);
    BluetoothUart_WriteText("\r\n");
}
