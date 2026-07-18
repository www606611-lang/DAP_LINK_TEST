#include "tuning_status.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "i2c1_polling.h"
#include "line_follow_mission.h"
#include "line_sensor_bringup.h"
#include "line_tracking_bringup_test.h"
#include "motion_supervisor.h"
#include "runtime_metrics.h"
#include "tuning_wire.h"
#include "wheel_line_tracking_control.h"
#include "wheel_speed_control.h"

void speed_tuning_send_line_config(void)
{
    line_tracking_bringup_config_t config;

    if (!LineTrackingBringupTest_GetConfig(&config)) {
        BluetoothUart_WriteText("ERR config\r\n");
        return;
    }
    BluetoothUart_WriteText("OK LCFG kp=");
    speed_tuning_write_float4(config.control.kp);
    BluetoothUart_WriteText(" ki=");
    speed_tuning_write_float4(config.control.ki);
    BluetoothUart_WriteText(" kd=");
    speed_tuning_write_float4(config.control.kd);
    BluetoothUart_WriteText(" base=");
    speed_tuning_write_float4(config.base_speed_pps);
    BluetoothUart_WriteText(" max=");
    speed_tuning_write_float4(config.control.max_correction_pps);
    BluetoothUart_WriteText(" limit=");
    speed_tuning_write_u32(config.output_limit_permille);
    BluetoothUart_WriteText(" dead=");
    speed_tuning_write_float4(config.control.deadband);
    BluetoothUart_WriteText(" duration=");
    speed_tuning_write_u32(config.duration_ms);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_line_status(uint32_t now_ms)
{
    app_runtime_metrics_snapshot_t runtime;
    line_sensor_snapshot_t sensor;
    i2c1_polling_snapshot_t bus;
    wheel_line_tracking_snapshot_t line_tracking;
    wheel_speed_control_snapshot_t speed;
    uint32_t sample_age_ms;

    if (!AppRuntimeMetrics_GetSnapshot(&runtime) ||
        !LineSensorBringup_GetSnapshot(&sensor) ||
        !I2C1Polling_GetSnapshot(&bus) ||
        !WheelLineTrackingControl_GetSnapshot(&line_tracking) ||
        !WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }
    sample_age_ms = now_ms - sensor.last_sample_ms;
    BluetoothUart_WriteText("LSTAT state=");
    BluetoothUart_WriteText(LineTrackingBringupTest_GetStateText());
    BluetoothUart_WriteText(" sensor=");
    BluetoothUart_WriteText(LineSensorBringup_GetStateText());
    BluetoothUart_WriteText(" raw=");
    speed_tuning_write_u32(sensor.raw);
    BluetoothUart_WriteText(" mask=");
    speed_tuning_write_u32(sensor.active_mask);
    BluetoothUart_WriteText(" count=");
    speed_tuning_write_u32(sensor.active_count);
    BluetoothUart_WriteText(" error=");
    speed_tuning_write_i32(sensor.line_error);
    BluetoothUart_WriteText(" seen=");
    speed_tuning_write_u32(sensor.line_seen ? 1U : 0U);
    BluetoothUart_WriteText(" samples=");
    speed_tuning_write_u32(sensor.sample_count);
    BluetoothUart_WriteText(" errors=");
    speed_tuning_write_u32(sensor.read_error_count);
    BluetoothUart_WriteText(" cal=");
    speed_tuning_write_u32(sensor.calibration_count);
    BluetoothUart_WriteText(" age=");
    speed_tuning_write_u32(sample_age_ms);
    BluetoothUart_WriteText(" busTx=");
    speed_tuning_write_u32(bus.transaction_count);
    BluetoothUart_WriteText(" busRec=");
    speed_tuning_write_u32(bus.recovery_count);
    BluetoothUart_WriteText(" busRes=");
    speed_tuning_write_u32((uint32_t) bus.last_result);
    BluetoothUart_WriteText(" base=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.base_speed_target_pps));
    BluetoothUart_WriteText(" corr=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.correction_target_pps));
    BluetoothUart_WriteText(" yawT=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.target_yaw_rate_dps * 1000.0f));
    BluetoothUart_WriteText(" yawR=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.measured_yaw_rate_dps * 1000.0f));
    BluetoothUart_WriteText(" yawBoost=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.yaw_rate_boost_pps));
    BluetoothUart_WriteText(" imu=");
    speed_tuning_write_u32(
        line_tracking.imu_feedback_valid ? 1U : 0U);
    BluetoothUart_WriteText(" tL=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.left_speed_target_pps));
    BluetoothUart_WriteText(" tR=");
    speed_tuning_write_i32(speed_tuning_round_float(
        line_tracking.right_speed_target_pps));
    BluetoothUart_WriteText(" vL=");
    speed_tuning_write_i32(speed.left_measured_pps);
    BluetoothUart_WriteText(" vR=");
    speed_tuning_write_i32(speed.right_measured_pps);
    BluetoothUart_WriteText(" outL=");
    speed_tuning_write_i32(speed.left_output_permille);
    BluetoothUart_WriteText(" outR=");
    speed_tuning_write_i32(speed.right_output_permille);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) line_tracking.last_result);
    BluetoothUart_WriteText(" sensorRes=");
    speed_tuning_write_u32((uint32_t) sensor.last_result);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText(" lineDt=");
    speed_tuning_write_u32(line_tracking.last_interval_ms);
    BluetoothUart_WriteText(" lineMax=");
    speed_tuning_write_u32(line_tracking.max_interval_ms);
    BluetoothUart_WriteText(" loopMax=");
    speed_tuning_write_u32(runtime.loop_max_interval_ms);
    BluetoothUart_WriteText(" lcdMax=");
    speed_tuning_write_u32(runtime.display_max_duration_ms);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_mission_status(uint32_t now_ms)
{
    line_follow_mission_snapshot_t mission;
    line_sensor_snapshot_t sensor;
    wheel_line_tracking_snapshot_t control;
    uint32_t sample_age_ms;

    if (!LineFollowMission_GetSnapshot(&mission) ||
        !LineSensorBringup_GetSnapshot(&sensor) ||
        !WheelLineTrackingControl_GetSnapshot(&control)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }
    sample_age_ms = now_ms - sensor.last_sample_ms;
    BluetoothUart_WriteText("MSTAT state=");
    BluetoothUart_WriteText(LineFollowMission_GetStateText());
    BluetoothUart_WriteText(" runs=");
    speed_tuning_write_u32(mission.run_count);
    BluetoothUart_WriteText(" base=");
    speed_tuning_write_i32(speed_tuning_round_float(
        mission.base_speed_pps));
    BluetoothUart_WriteText(" limit=");
    speed_tuning_write_u32(mission.output_limit_permille);
    BluetoothUart_WriteText(" elapsed=");
    speed_tuning_write_u32(mission.elapsed_ms);
    BluetoothUart_WriteText(" error=");
    speed_tuning_write_i32(sensor.line_error);
    BluetoothUart_WriteText(" count=");
    speed_tuning_write_u32(sensor.active_count);
    BluetoothUart_WriteText(" seen=");
    speed_tuning_write_u32(sensor.line_seen ? 1U : 0U);
    BluetoothUart_WriteText(" age=");
    speed_tuning_write_u32(sample_age_ms);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) mission.last_result);
    BluetoothUart_WriteText(" control=");
    speed_tuning_write_u32(control.running ? 1U : 0U);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_motion_status(uint32_t now_ms)
{
    motion_supervisor_snapshot_t motion;
    wheel_speed_control_snapshot_t speed;
    (void) now_ms;
    if (!MotionSupervisor_GetSnapshot(&motion) ||
        !WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }
    BluetoothUart_WriteText("OSTAT state=");
    BluetoothUart_WriteText(MotionSupervisor_GetStateText());
    BluetoothUart_WriteText(" runs=");
    speed_tuning_write_u32(motion.run_count);
    BluetoothUart_WriteText(" delta=");
    speed_tuning_write_i32(motion.target_delta_count);
    BluetoothUart_WriteText(" target=");
    speed_tuning_write_i32(motion.target_count);
    BluetoothUart_WriteText(" current=");
    speed_tuning_write_i32(motion.current_count);
    BluetoothUart_WriteText(" error=");
    speed_tuning_write_i32(motion.position_error_count);
    BluetoothUart_WriteText(" yawE=");
    speed_tuning_write_i32(speed_tuning_round_float(
        motion.heading_error_deg * 1000.0f));
    BluetoothUart_WriteText(" base=");
    speed_tuning_write_i32(speed_tuning_round_float(
        motion.base_speed_target_pps));
    BluetoothUart_WriteText(" corr=");
    speed_tuning_write_i32(speed_tuning_round_float(
        motion.heading_correction_pps));
    BluetoothUart_WriteText(" tL=");
    speed_tuning_write_i32(speed_tuning_round_float(
        motion.left_speed_target_pps));
    BluetoothUart_WriteText(" tR=");
    speed_tuning_write_i32(speed_tuning_round_float(
        motion.right_speed_target_pps));
    BluetoothUart_WriteText(" vL=");
    speed_tuning_write_i32(speed.left_measured_pps);
    BluetoothUart_WriteText(" vR=");
    speed_tuning_write_i32(speed.right_measured_pps);
    BluetoothUart_WriteText(" elapsed=");
    speed_tuning_write_u32(motion.elapsed_ms);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) motion.last_result);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText("\r\n");
}

