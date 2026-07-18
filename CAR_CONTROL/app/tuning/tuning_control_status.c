#include "tuning_status.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "encoder_input.h"
#include "heading_bringup_test.h"
#include "icm20948.h"
#include "position_bringup_test.h"
#include "runtime_metrics.h"
#include "speed_bringup_test.h"
#include "tuning_wire.h"
#include "wheel_heading_control.h"
#include "wheel_position_control.h"
#include "wheel_speed_control.h"
#include "wheel_yaw_control.h"
#include "yaw_bringup_test.h"

void speed_tuning_send_config(void)
{
    speed_bringup_config_t config;

    if (!SpeedBringupTest_GetConfig(&config)) {
        BluetoothUart_WriteText("ERR config\r\n");
        return;
    }

    BluetoothUart_WriteText("OK CFG kp=");
    speed_tuning_write_float4(config.pid.kp);
    BluetoothUart_WriteText(" ki=");
    speed_tuning_write_float4(config.pid.ki);
    BluetoothUart_WriteText(" kd=");
    speed_tuning_write_float4(config.pid.kd);
    BluetoothUart_WriteText(" target=");
    speed_tuning_write_float4(config.target_pps);
    BluetoothUart_WriteText(" limit=");
    speed_tuning_write_u32(config.output_limit_permille);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_status(void)
{
    wheel_speed_control_snapshot_t speed;
    encoder_input_snapshot_t left;
    encoder_input_snapshot_t right;

    if (!WheelSpeedControl_GetSnapshot(&speed) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }

    BluetoothUart_WriteText("STAT state=");
    BluetoothUart_WriteText(SpeedBringupTest_GetStateText());
    BluetoothUart_WriteText(" left=");
    speed_tuning_write_i32(left.speed_pps);
    BluetoothUart_WriteText(" right=");
    speed_tuning_write_i32(right.speed_pps);
    BluetoothUart_WriteText(" outL=");
    speed_tuning_write_i32(speed.left_output_permille);
    BluetoothUart_WriteText(" outR=");
    speed_tuning_write_i32(speed.right_output_permille);
    BluetoothUart_WriteText(" invL=");
    speed_tuning_write_u32(left.invalid_transition_count);
    BluetoothUart_WriteText(" invR=");
    speed_tuning_write_u32(right.invalid_transition_count);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) speed.last_result);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_position_config(void)
{
    position_bringup_config_t config;

    if (!PositionBringupTest_GetConfig(&config)) {
        BluetoothUart_WriteText("ERR config\r\n");
        return;
    }

    BluetoothUart_WriteText("OK PCFG kp=");
    speed_tuning_write_float4(config.control.kp);
    BluetoothUart_WriteText(" target=");
    speed_tuning_write_i32(config.target_counts);
    BluetoothUart_WriteText(" max=");
    speed_tuning_write_float4(config.control.max_speed_pps);
    BluetoothUart_WriteText(" limit=");
    speed_tuning_write_u32(config.output_limit_permille);
    BluetoothUart_WriteText(" tol=");
    speed_tuning_write_u32(config.control.tolerance_counts);
    BluetoothUart_WriteText(" syncKp=");
    speed_tuning_write_float4(config.control.sync_kp);
    BluetoothUart_WriteText(" syncMax=");
    speed_tuning_write_float4(
        config.control.sync_max_correction_pps);
    BluetoothUart_WriteText(" settle=");
    speed_tuning_write_u32(config.control.settle_speed_pps);
    BluetoothUart_WriteText(" stime=");
    speed_tuning_write_u32(config.control.settle_time_ms);
    BluetoothUart_WriteText(" timeout=");
    speed_tuning_write_u32(config.timeout_ms);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_position_status(void)
{
    wheel_position_control_snapshot_t position;
    encoder_input_snapshot_t left;
    encoder_input_snapshot_t right;

    if (!WheelPositionControl_GetSnapshot(&position) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }

    BluetoothUart_WriteText("PSTAT state=");
    BluetoothUart_WriteText(PositionBringupTest_GetStateText());
    BluetoothUart_WriteText(" profile=");
    BluetoothUart_WriteText(PositionBringupTest_GetProfileText());
    BluetoothUart_WriteText(" step=");
    speed_tuning_write_u32(PositionBringupTest_GetCurrentStep());
    BluetoothUart_WriteText("/");
    speed_tuning_write_u32(PositionBringupTest_GetStepCount());
    BluetoothUart_WriteText(" done=");
    speed_tuning_write_u32(
        PositionBringupTest_GetCompletedMoveCount());
    BluetoothUart_WriteText(" worst=");
    speed_tuning_write_u32(
        PositionBringupTest_GetWorstFinalErrorCount());
    BluetoothUart_WriteText(" recL=");
    speed_tuning_write_u32(
        PositionBringupTest_GetLeftRecoveryCount());
    BluetoothUart_WriteText(" recR=");
    speed_tuning_write_u32(
        PositionBringupTest_GetRightRecoveryCount());
    BluetoothUart_WriteText(" tL=");
    speed_tuning_write_i32(position.left_target_count);
    BluetoothUart_WriteText(" cL=");
    speed_tuning_write_i32(position.left_count);
    BluetoothUart_WriteText(" eL=");
    speed_tuning_write_i32(position.left_error_count);
    BluetoothUart_WriteText(" tR=");
    speed_tuning_write_i32(position.right_target_count);
    BluetoothUart_WriteText(" cR=");
    speed_tuning_write_i32(position.right_count);
    BluetoothUart_WriteText(" eR=");
    speed_tuning_write_i32(position.right_error_count);
    BluetoothUart_WriteText(" vL=");
    speed_tuning_write_i32(left.speed_pps);
    BluetoothUart_WriteText(" vR=");
    speed_tuning_write_i32(right.speed_pps);
    BluetoothUart_WriteText(" invL=");
    speed_tuning_write_u32(left.invalid_transition_count);
    BluetoothUart_WriteText(" invR=");
    speed_tuning_write_u32(right.invalid_transition_count);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) position.last_result);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_yaw_config(void)
{
    yaw_bringup_config_t config;

    if (!YawBringupTest_GetConfig(&config)) {
        BluetoothUart_WriteText("ERR config\r\n");
        return;
    }
    BluetoothUart_WriteText("OK YCFG kp=");
    speed_tuning_write_float4(config.control.kp);
    BluetoothUart_WriteText(" ki=");
    speed_tuning_write_float4(config.control.ki);
    BluetoothUart_WriteText(" kd=");
    speed_tuning_write_float4(config.control.kd);
    BluetoothUart_WriteText(" target=");
    speed_tuning_write_float4(config.target_yaw_deg);
    BluetoothUart_WriteText(" max=");
    speed_tuning_write_float4(config.control.max_turn_speed_pps);
    BluetoothUart_WriteText(" limit=");
    speed_tuning_write_u32(config.output_limit_permille);
    BluetoothUart_WriteText(" tol=");
    speed_tuning_write_float4(config.control.tolerance_deg);
    BluetoothUart_WriteText(" rate=");
    speed_tuning_write_float4(config.control.settle_yaw_rate_dps);
    BluetoothUart_WriteText(" stime=");
    speed_tuning_write_u32(config.control.settle_time_ms);
    BluetoothUart_WriteText(" timeout=");
    speed_tuning_write_u32(config.timeout_ms);
    BluetoothUart_WriteText(" min=");
    speed_tuning_write_float4(config.control.min_turn_speed_pps);
    BluetoothUart_WriteText(" boost=");
    speed_tuning_write_u32(
        config.control.feedforward_boost_permille);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_yaw_status(void)
{
    app_runtime_metrics_snapshot_t runtime;
    icm20948_snapshot_t imu;
    wheel_yaw_control_snapshot_t yaw;
    wheel_speed_control_snapshot_t speed;

    if (!AppRuntimeMetrics_GetSnapshot(&runtime) ||
        !ICM20948_GetSnapshot(&imu) ||
        !WheelYawControl_GetSnapshot(&yaw) ||
        !WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }
    BluetoothUart_WriteText("YSTAT state=");
    BluetoothUart_WriteText(YawBringupTest_GetStateText());
    BluetoothUart_WriteText(" target=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.target_yaw_deg * 1000.0f));
    BluetoothUart_WriteText(" current=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.current_yaw_deg * 1000.0f));
    BluetoothUart_WriteText(" error=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.error_deg * 1000.0f));
    BluetoothUart_WriteText(" rate=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.yaw_rate_dps * 1000.0f));
    BluetoothUart_WriteText(" turn=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.turn_speed_target_pps));
    BluetoothUart_WriteText(" tL=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.left_speed_target_pps));
    BluetoothUart_WriteText(" tR=");
    speed_tuning_write_i32(
        speed_tuning_round_float(yaw.right_speed_target_pps));
    BluetoothUart_WriteText(" vL=");
    speed_tuning_write_i32(speed.left_measured_pps);
    BluetoothUart_WriteText(" vR=");
    speed_tuning_write_i32(speed.right_measured_pps);
    BluetoothUart_WriteText(" outL=");
    speed_tuning_write_i32(speed.left_output_permille);
    BluetoothUart_WriteText(" outR=");
    speed_tuning_write_i32(speed.right_output_permille);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) yaw.last_result);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText(" loop=");
    speed_tuning_write_u32(runtime.loop_interval_ms);
    BluetoothUart_WriteText(" loopMax=");
    speed_tuning_write_u32(runtime.loop_max_interval_ms);
    BluetoothUart_WriteText(" imuDt=");
    speed_tuning_write_u32(imu.last_interval_ms);
    BluetoothUart_WriteText(" imuMax=");
    speed_tuning_write_u32(imu.max_interval_ms);
    BluetoothUart_WriteText(" yawDt=");
    speed_tuning_write_u32(yaw.last_interval_ms);
    BluetoothUart_WriteText(" yawMax=");
    speed_tuning_write_u32(yaw.max_interval_ms);
    BluetoothUart_WriteText(" lcd=");
    speed_tuning_write_u32(runtime.display_duration_ms);
    BluetoothUart_WriteText(" lcdMax=");
    speed_tuning_write_u32(runtime.display_max_duration_ms);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_heading_config(void)
{
    heading_bringup_config_t config;

    if (!HeadingBringupTest_GetConfig(&config)) {
        BluetoothUart_WriteText("ERR config\r\n");
        return;
    }
    BluetoothUart_WriteText("OK HCFG kp=");
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
    speed_tuning_write_float4(config.control.deadband_deg);
    BluetoothUart_WriteText(" duration=");
    speed_tuning_write_u32(config.duration_ms);
    BluetoothUart_WriteText("\r\n");
}

void speed_tuning_send_heading_status(void)
{
    app_runtime_metrics_snapshot_t runtime;
    icm20948_snapshot_t imu;
    wheel_heading_control_snapshot_t heading;
    wheel_speed_control_snapshot_t speed;

    if (!AppRuntimeMetrics_GetSnapshot(&runtime) ||
        !ICM20948_GetSnapshot(&imu) ||
        !WheelHeadingControl_GetSnapshot(&heading) ||
        !WheelSpeedControl_GetSnapshot(&speed)) {
        BluetoothUart_WriteText("ERR status\r\n");
        return;
    }
    BluetoothUart_WriteText("HSTAT state=");
    BluetoothUart_WriteText(HeadingBringupTest_GetStateText());
    BluetoothUart_WriteText(" target=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.target_yaw_deg * 1000.0f));
    BluetoothUart_WriteText(" current=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.current_yaw_deg * 1000.0f));
    BluetoothUart_WriteText(" error=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.error_deg * 1000.0f));
    BluetoothUart_WriteText(" rate=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.yaw_rate_dps * 1000.0f));
    BluetoothUart_WriteText(" base=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.base_speed_target_pps));
    BluetoothUart_WriteText(" corr=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.correction_target_pps));
    BluetoothUart_WriteText(" tL=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.left_speed_target_pps));
    BluetoothUart_WriteText(" tR=");
    speed_tuning_write_i32(speed_tuning_round_float(
        heading.right_speed_target_pps));
    BluetoothUart_WriteText(" vL=");
    speed_tuning_write_i32(speed.left_measured_pps);
    BluetoothUart_WriteText(" vR=");
    speed_tuning_write_i32(speed.right_measured_pps);
    BluetoothUart_WriteText(" outL=");
    speed_tuning_write_i32(speed.left_output_permille);
    BluetoothUart_WriteText(" outR=");
    speed_tuning_write_i32(speed.right_output_permille);
    BluetoothUart_WriteText(" res=");
    speed_tuning_write_u32((uint32_t) heading.last_result);
    BluetoothUart_WriteText(" hz=");
    speed_tuning_write_u32(
        BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
    BluetoothUart_WriteText(" loopMax=");
    speed_tuning_write_u32(runtime.loop_max_interval_ms);
    BluetoothUart_WriteText(" imuMax=");
    speed_tuning_write_u32(imu.max_interval_ms);
    BluetoothUart_WriteText(" headDt=");
    speed_tuning_write_u32(heading.last_interval_ms);
    BluetoothUart_WriteText(" headMax=");
    speed_tuning_write_u32(heading.max_interval_ms);
    BluetoothUart_WriteText(" lcdMax=");
    speed_tuning_write_u32(runtime.display_max_duration_ms);
    BluetoothUart_WriteText("\r\n");
}
