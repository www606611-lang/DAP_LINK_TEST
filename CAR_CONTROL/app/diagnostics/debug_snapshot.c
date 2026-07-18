#include "debug_snapshot.h"

#include "board_button.h"
#include "board_motor_safe.h"
#include "car_app.h"
#include "control_supervisor.h"
#include "delay.h"
#include "encoder_input.h"
#include "heading_bringup_test.h"
#include "icm20948.h"
#include "jdy31_config.h"
#include "line_sensor_bringup.h"
#include "line_follow_mission.h"
#include "line_tracking_bringup_test.h"
#include "position_bringup_test.h"
#include "reset_diagnostics.h"
#include "speed_bringup_test.h"
#include "wheel_heading_control.h"
#include "wheel_line_tracking_control.h"
#include "wheel_position_control.h"
#include "wheel_speed_control.h"
#include "wheel_yaw_control.h"
#include "yaw_bringup_test.h"

#include <stddef.h>

volatile bool g_car_pb21_pressed;
volatile uint32_t g_car_pb21_press_count;
volatile uint32_t g_car_pb21_interrupt_count;
volatile bool g_car_pb4_pressed;
volatile uint32_t g_car_pb4_press_count;
volatile uint32_t g_car_pb4_interrupt_count;
volatile bool g_car_pb5_pressed;
volatile uint32_t g_car_pb5_press_count;
volatile uint32_t g_car_pb5_interrupt_count;
volatile int32_t g_car_last_button_yaw_mdeg;
volatile uint32_t g_car_last_button_id;
volatile uint32_t g_car_reset_cause;
volatile uint32_t g_car_control_mode;
volatile uint32_t g_car_control_block_reason;
volatile bool g_car_motor_high_impedance;
volatile bool g_car_encoder_shadow_active;
volatile int32_t g_car_encoder_0_count;
volatile int32_t g_car_encoder_0_speed_pps;
volatile uint32_t g_car_encoder_0_edges;
volatile uint32_t g_car_encoder_0_invalid;
volatile int32_t g_car_encoder_1_count;
volatile int32_t g_car_encoder_1_speed_pps;
volatile uint32_t g_car_encoder_1_edges;
volatile uint32_t g_car_encoder_1_invalid;
volatile uint32_t g_car_speed_test_state;
volatile uint32_t g_car_speed_test_run_count;
volatile int32_t g_car_speed_left_target_pps;
volatile int32_t g_car_speed_right_target_pps;
volatile int32_t g_car_speed_left_error_pps;
volatile int32_t g_car_speed_right_error_pps;
volatile int32_t g_car_speed_left_output_permille;
volatile int32_t g_car_speed_right_output_permille;
volatile uint32_t g_car_speed_update_count;
volatile uint32_t g_car_speed_last_result;
volatile uint32_t g_car_position_test_state;
volatile uint32_t g_car_position_test_run_count;
volatile int32_t g_car_position_left_target_count;
volatile int32_t g_car_position_right_target_count;
volatile int32_t g_car_position_left_error_count;
volatile int32_t g_car_position_right_error_count;
volatile int32_t g_car_position_left_speed_target_pps;
volatile int32_t g_car_position_right_speed_target_pps;
volatile int32_t g_car_position_sync_error_count;
volatile int32_t g_car_position_sync_correction_pps;
volatile bool g_car_position_sync_active;
volatile uint32_t g_car_position_update_count;
volatile uint32_t g_car_position_last_result;
volatile bool g_car_position_settled;
volatile bool g_car_imu_ready;
volatile uint32_t g_car_imu_state;
volatile uint32_t g_car_imu_result;
volatile uint32_t g_car_imu_address7;
volatile uint32_t g_car_imu_who_am_i;
volatile uint32_t g_car_imu_sample_count;
volatile uint32_t g_car_imu_read_error_count;
volatile uint32_t g_car_imu_sample_age_ms;
volatile int32_t g_car_imu_ax_mg;
volatile int32_t g_car_imu_ay_mg;
volatile int32_t g_car_imu_az_mg;
volatile int32_t g_car_imu_gx_mdps;
volatile int32_t g_car_imu_gy_mdps;
volatile int32_t g_car_imu_gz_mdps;
volatile int32_t g_car_imu_roll_mdeg;
volatile int32_t g_car_imu_pitch_mdeg;
volatile int32_t g_car_imu_yaw_mdeg;
volatile int32_t g_car_imu_yaw_rate_mdps;
volatile int32_t g_car_imu_accel_norm_mg;
volatile int32_t g_car_imu_bias_x_mdps;
volatile int32_t g_car_imu_bias_y_mdps;
volatile int32_t g_car_imu_bias_z_mdps;
volatile int32_t g_car_imu_quaternion_w_million;
volatile int32_t g_car_imu_quaternion_x_million;
volatile int32_t g_car_imu_quaternion_y_million;
volatile int32_t g_car_imu_quaternion_z_million;
volatile bool g_car_imu_attitude_valid;
volatile bool g_car_imu_stationary;
volatile uint32_t g_car_yaw_test_state;
volatile uint32_t g_car_yaw_test_run_count;
volatile int32_t g_car_yaw_target_mdeg;
volatile int32_t g_car_yaw_current_mdeg;
volatile int32_t g_car_yaw_error_mdeg;
volatile int32_t g_car_yaw_rate_mdps;
volatile int32_t g_car_yaw_turn_target_pps;
volatile int32_t g_car_yaw_left_target_pps;
volatile int32_t g_car_yaw_right_target_pps;
volatile uint32_t g_car_yaw_update_count;
volatile uint32_t g_car_yaw_elapsed_ms;
volatile uint32_t g_car_yaw_last_result;
volatile bool g_car_yaw_settled;
volatile uint32_t g_car_heading_test_state;
volatile uint32_t g_car_heading_test_run_count;
volatile int32_t g_car_heading_target_mdeg;
volatile int32_t g_car_heading_current_mdeg;
volatile int32_t g_car_heading_error_mdeg;
volatile int32_t g_car_heading_rate_mdps;
volatile int32_t g_car_heading_base_target_pps;
volatile int32_t g_car_heading_correction_pps;
volatile int32_t g_car_heading_left_target_pps;
volatile int32_t g_car_heading_right_target_pps;
volatile uint32_t g_car_heading_update_count;
volatile uint32_t g_car_heading_elapsed_ms;
volatile uint32_t g_car_heading_last_result;
volatile uint32_t g_car_line_sensor_state;
volatile uint32_t g_car_line_sensor_raw;
volatile uint32_t g_car_line_sensor_active_mask;
volatile uint32_t g_car_line_sensor_active_count;
volatile int32_t g_car_line_sensor_error;
volatile uint32_t g_car_line_sensor_sample_count;
volatile uint32_t g_car_line_sensor_read_error_count;
volatile uint32_t g_car_line_sensor_calibration_count;
volatile uint32_t g_car_line_sensor_last_result;
volatile bool g_car_line_sensor_seen;
volatile bool g_car_line_sensor_ready;
volatile uint32_t g_car_line_tracking_test_state;
volatile uint32_t g_car_line_tracking_run_count;
volatile uint32_t g_car_line_mission_state;
volatile uint32_t g_car_line_mission_run_count;
volatile int32_t g_car_line_tracking_error;
volatile int32_t g_car_line_tracking_base_target_pps;
volatile int32_t g_car_line_tracking_correction_pps;
volatile int32_t g_car_line_tracking_left_target_pps;
volatile int32_t g_car_line_tracking_right_target_pps;
volatile uint32_t g_car_line_tracking_update_count;
volatile uint32_t g_car_line_tracking_elapsed_ms;
volatile uint32_t g_car_line_tracking_last_result;
volatile int32_t g_car_encoder_count_difference;
volatile int32_t g_car_encoder_speed_difference_pps;
volatile uint32_t g_car_app_state;
volatile uint32_t g_car_app_active_workflow;
volatile uint32_t g_car_app_last_action;
volatile uint32_t g_car_app_transition_count;
volatile uint32_t g_car_jdy31_config_state;
volatile uint32_t g_car_jdy31_uart_baud;
volatile int32_t g_car_jdy31_reported_baud_code;
volatile bool g_car_jdy31_config_success;

static int32_t car_debug_round_float(float value);

void CarDebugSnapshot_Update(void)
{
    encoder_input_snapshot_t encoder_0;
    encoder_input_snapshot_t encoder_1;
    wheel_speed_control_snapshot_t speed;
    wheel_position_control_snapshot_t position;
    wheel_yaw_control_snapshot_t yaw;
    wheel_heading_control_snapshot_t heading;
    wheel_line_tracking_snapshot_t line_tracking;
    line_sensor_snapshot_t line_sensor;
    icm20948_snapshot_t imu;
    jdy31_config_snapshot_t jdy31;
    car_app_snapshot_t car_app;

    g_car_pb21_pressed = BoardButton_IsPressed();
    g_car_pb21_interrupt_count = BoardButton_GetInterruptCountId(
        BOARD_BUTTON_ID_PB21);
    g_car_pb4_pressed = BoardButton_IsPressedId(
        BOARD_BUTTON_ID_SW2_PB4);
    g_car_pb4_interrupt_count = BoardButton_GetInterruptCountId(
        BOARD_BUTTON_ID_SW2_PB4);
    g_car_pb5_pressed = BoardButton_IsPressedId(
        BOARD_BUTTON_ID_SW1_PB5);
    g_car_pb5_interrupt_count = BoardButton_GetInterruptCountId(
        BOARD_BUTTON_ID_SW1_PB5);
    g_car_reset_cause = ResetDiagnostics_GetCause();
    g_car_control_mode = (uint32_t) ControlSupervisor_GetMode();
    g_car_control_block_reason =
        (uint32_t) ControlSupervisor_GetBlockReason();
    g_car_motor_high_impedance = BoardMotorSafe_IsHighImpedance();
    if (CarApp_GetSnapshot(&car_app)) {
        g_car_app_state = (uint32_t) car_app.state;
        g_car_app_active_workflow =
            (uint32_t) car_app.active_workflow;
        g_car_app_last_action = (uint32_t) car_app.action;
        g_car_app_transition_count = car_app.transition_count;
    }
    g_car_speed_test_state = (uint32_t) SpeedBringupTest_GetState();
    g_car_speed_test_run_count = SpeedBringupTest_GetRunCount();
    g_car_position_test_state =
        (uint32_t) PositionBringupTest_GetState();
    g_car_position_test_run_count = PositionBringupTest_GetRunCount();
    g_car_yaw_test_state = (uint32_t) YawBringupTest_GetState();
    g_car_yaw_test_run_count = YawBringupTest_GetRunCount();
    g_car_heading_test_state =
        (uint32_t) HeadingBringupTest_GetState();
    g_car_heading_test_run_count = HeadingBringupTest_GetRunCount();
    if (JDY31_ConfigGetSnapshot(&jdy31)) {
        g_car_jdy31_config_state = (uint32_t) jdy31.state;
        g_car_jdy31_uart_baud = jdy31.uart_baud;
        g_car_jdy31_reported_baud_code =
            jdy31.reported_baud_code;
        g_car_jdy31_config_success = jdy31.success;
    }

    if (WheelSpeedControl_GetSnapshot(&speed)) {
        g_car_speed_left_target_pps =
            car_debug_round_float(speed.left_target_pps);
        g_car_speed_right_target_pps =
            car_debug_round_float(speed.right_target_pps);
        g_car_speed_left_error_pps =
            car_debug_round_float(speed.left_error_pps);
        g_car_speed_right_error_pps =
            car_debug_round_float(speed.right_error_pps);
        g_car_speed_left_output_permille = speed.left_output_permille;
        g_car_speed_right_output_permille = speed.right_output_permille;
        g_car_speed_update_count = speed.update_count;
        g_car_speed_last_result = (uint32_t) speed.last_result;
    }

    if (WheelPositionControl_GetSnapshot(&position)) {
        g_car_position_left_target_count = position.left_target_count;
        g_car_position_right_target_count = position.right_target_count;
        g_car_position_left_error_count = position.left_error_count;
        g_car_position_right_error_count = position.right_error_count;
        g_car_position_left_speed_target_pps =
            car_debug_round_float(position.left_speed_target_pps);
        g_car_position_right_speed_target_pps =
            car_debug_round_float(position.right_speed_target_pps);
        g_car_position_sync_error_count = position.sync_error_count;
        g_car_position_sync_correction_pps =
            car_debug_round_float(position.sync_correction_pps);
        g_car_position_sync_active = position.sync_active;
        g_car_position_update_count = position.update_count;
        g_car_position_last_result = (uint32_t) position.last_result;
        g_car_position_settled = position.settled;
    }

    if (WheelYawControl_GetSnapshot(&yaw)) {
        g_car_yaw_target_mdeg =
            car_debug_round_float(yaw.target_yaw_deg * 1000.0f);
        g_car_yaw_current_mdeg =
            car_debug_round_float(yaw.current_yaw_deg * 1000.0f);
        g_car_yaw_error_mdeg =
            car_debug_round_float(yaw.error_deg * 1000.0f);
        g_car_yaw_rate_mdps =
            car_debug_round_float(yaw.yaw_rate_dps * 1000.0f);
        g_car_yaw_turn_target_pps =
            car_debug_round_float(yaw.turn_speed_target_pps);
        g_car_yaw_left_target_pps =
            car_debug_round_float(yaw.left_speed_target_pps);
        g_car_yaw_right_target_pps =
            car_debug_round_float(yaw.right_speed_target_pps);
        g_car_yaw_update_count = yaw.update_count;
        g_car_yaw_elapsed_ms = yaw.elapsed_ms;
        g_car_yaw_last_result = (uint32_t) yaw.last_result;
        g_car_yaw_settled = yaw.settled;
    }

    if (WheelHeadingControl_GetSnapshot(&heading)) {
        g_car_heading_target_mdeg =
            car_debug_round_float(heading.target_yaw_deg * 1000.0f);
        g_car_heading_current_mdeg =
            car_debug_round_float(heading.current_yaw_deg * 1000.0f);
        g_car_heading_error_mdeg =
            car_debug_round_float(heading.error_deg * 1000.0f);
        g_car_heading_rate_mdps =
            car_debug_round_float(heading.yaw_rate_dps * 1000.0f);
        g_car_heading_base_target_pps =
            car_debug_round_float(heading.base_speed_target_pps);
        g_car_heading_correction_pps =
            car_debug_round_float(heading.correction_target_pps);
        g_car_heading_left_target_pps =
            car_debug_round_float(heading.left_speed_target_pps);
        g_car_heading_right_target_pps =
            car_debug_round_float(heading.right_speed_target_pps);
        g_car_heading_update_count = heading.update_count;
        g_car_heading_elapsed_ms = heading.elapsed_ms;
        g_car_heading_last_result = (uint32_t) heading.last_result;
        if (heading.running) {
            g_car_yaw_target_mdeg = g_car_heading_target_mdeg;
            g_car_yaw_current_mdeg = g_car_heading_current_mdeg;
            g_car_yaw_error_mdeg = g_car_heading_error_mdeg;
            g_car_yaw_rate_mdps = g_car_heading_rate_mdps;
            g_car_yaw_turn_target_pps =
                g_car_heading_correction_pps;
            g_car_yaw_left_target_pps =
                g_car_heading_left_target_pps;
            g_car_yaw_right_target_pps =
                g_car_heading_right_target_pps;
            g_car_yaw_update_count = heading.update_count;
            g_car_yaw_elapsed_ms = heading.elapsed_ms;
            g_car_yaw_last_result = (uint32_t) heading.last_result;
            g_car_yaw_settled = false;
        }
    }

    if (LineSensorBringup_GetSnapshot(&line_sensor)) {
        g_car_line_sensor_state = (uint32_t) line_sensor.state;
        g_car_line_sensor_raw = line_sensor.raw;
        g_car_line_sensor_active_mask = line_sensor.active_mask;
        g_car_line_sensor_active_count = line_sensor.active_count;
        g_car_line_sensor_error = line_sensor.line_error;
        g_car_line_sensor_sample_count = line_sensor.sample_count;
        g_car_line_sensor_read_error_count =
            line_sensor.read_error_count;
        g_car_line_sensor_calibration_count =
            line_sensor.calibration_count;
        g_car_line_sensor_last_result =
            (uint32_t) line_sensor.last_result;
        g_car_line_sensor_seen = line_sensor.line_seen;
        g_car_line_sensor_ready = line_sensor.ready;
    }

    g_car_line_tracking_test_state =
        (uint32_t) LineTrackingBringupTest_GetState();
    g_car_line_tracking_run_count =
        LineTrackingBringupTest_GetRunCount();
    {
        line_follow_mission_snapshot_t mission;

        if (LineFollowMission_GetSnapshot(&mission)) {
            g_car_line_mission_state = (uint32_t) mission.state;
            g_car_line_mission_run_count = mission.run_count;
        }
    }
    if (WheelLineTrackingControl_GetSnapshot(&line_tracking)) {
        g_car_line_tracking_error = line_tracking.line_error;
        g_car_line_tracking_base_target_pps = car_debug_round_float(
            line_tracking.base_speed_target_pps);
        g_car_line_tracking_correction_pps = car_debug_round_float(
            line_tracking.correction_target_pps);
        g_car_line_tracking_left_target_pps = car_debug_round_float(
            line_tracking.left_speed_target_pps);
        g_car_line_tracking_right_target_pps = car_debug_round_float(
            line_tracking.right_speed_target_pps);
        g_car_line_tracking_update_count = line_tracking.update_count;
        g_car_line_tracking_elapsed_ms = line_tracking.elapsed_ms;
        g_car_line_tracking_last_result =
            (uint32_t) line_tracking.last_result;
    }

    if (ICM20948_GetSnapshot(&imu)) {
        uint32_t now_ms = delay_get_ms();

        g_car_imu_ready = imu.ready;
        g_car_imu_state = (uint32_t) imu.state;
        g_car_imu_result = (uint32_t) imu.last_result;
        g_car_imu_address7 = imu.address7;
        g_car_imu_who_am_i = imu.who_am_i;
        g_car_imu_sample_count = imu.sample_count;
        g_car_imu_read_error_count = imu.read_error_count;
        g_car_imu_sample_age_ms = now_ms - imu.last_sample_ms;
        g_car_imu_ax_mg = car_debug_round_float(imu.data.ax_g * 1000.0f);
        g_car_imu_ay_mg = car_debug_round_float(imu.data.ay_g * 1000.0f);
        g_car_imu_az_mg = car_debug_round_float(imu.data.az_g * 1000.0f);
        g_car_imu_gx_mdps =
            car_debug_round_float(imu.data.gx_dps * 1000.0f);
        g_car_imu_gy_mdps =
            car_debug_round_float(imu.data.gy_dps * 1000.0f);
        g_car_imu_gz_mdps =
            car_debug_round_float(imu.data.gz_dps * 1000.0f);
        g_car_imu_roll_mdeg =
            car_debug_round_float(imu.roll_deg * 1000.0f);
        g_car_imu_pitch_mdeg =
            car_debug_round_float(imu.pitch_deg * 1000.0f);
        g_car_imu_yaw_mdeg =
            car_debug_round_float(imu.yaw_deg * 1000.0f);
        g_car_imu_yaw_rate_mdps =
            car_debug_round_float(imu.yaw_rate_dps * 1000.0f);
        g_car_imu_accel_norm_mg =
            car_debug_round_float(imu.accel_norm_g * 1000.0f);
        g_car_imu_bias_x_mdps =
            car_debug_round_float(imu.gyro_bias_x_dps * 1000.0f);
        g_car_imu_bias_y_mdps =
            car_debug_round_float(imu.gyro_bias_y_dps * 1000.0f);
        g_car_imu_bias_z_mdps =
            car_debug_round_float(imu.gyro_bias_z_dps * 1000.0f);
        g_car_imu_quaternion_w_million =
            car_debug_round_float(imu.quaternion_w * 1000000.0f);
        g_car_imu_quaternion_x_million =
            car_debug_round_float(imu.quaternion_x * 1000000.0f);
        g_car_imu_quaternion_y_million =
            car_debug_round_float(imu.quaternion_y * 1000000.0f);
        g_car_imu_quaternion_z_million =
            car_debug_round_float(imu.quaternion_z * 1000000.0f);
        g_car_imu_attitude_valid = imu.attitude_valid;
        g_car_imu_stationary = imu.stationary;
    }

    if (EncoderInput_GetSnapshot(ENCODER_INPUT_0, &encoder_0) &&
        EncoderInput_GetSnapshot(ENCODER_INPUT_1, &encoder_1)) {
        g_car_encoder_shadow_active = true;
        g_car_encoder_0_count = encoder_0.count;
        g_car_encoder_0_speed_pps = encoder_0.speed_pps;
        g_car_encoder_0_edges = encoder_0.edge_count;
        g_car_encoder_0_invalid = encoder_0.invalid_transition_count;
        g_car_encoder_1_count = encoder_1.count;
        g_car_encoder_1_speed_pps = encoder_1.speed_pps;
        g_car_encoder_1_edges = encoder_1.edge_count;
        g_car_encoder_1_invalid = encoder_1.invalid_transition_count;
        g_car_encoder_count_difference =
            encoder_0.count - encoder_1.count;
        g_car_encoder_speed_difference_pps =
            encoder_0.speed_pps - encoder_1.speed_pps;
    } else {
        g_car_encoder_shadow_active = false;
    }
}

void CarDebugSnapshot_RecordButtonPress(car_debug_button_id_t button_id)
{
    g_car_last_button_id = (uint32_t) button_id;
    switch (button_id) {
        case CAR_DEBUG_BUTTON_PB21:
            g_car_pb21_press_count++;
            break;
        case CAR_DEBUG_BUTTON_PB4:
            g_car_pb4_press_count++;
            break;
        case CAR_DEBUG_BUTTON_PB5:
            g_car_pb5_press_count++;
            break;
        default:
            break;
    }
}

void CarDebugSnapshot_SetButtonYawCommand(int32_t yaw_command_mdeg)
{
    g_car_last_button_yaw_mdeg = yaw_command_mdeg;
}

bool CarDebugSnapshot_GetDisplay(car_debug_display_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    snapshot->pb21_pressed = g_car_pb21_pressed;
    snapshot->pb4_pressed = g_car_pb4_pressed;
    snapshot->pb5_pressed = g_car_pb5_pressed;
    snapshot->last_button_yaw_mdeg = g_car_last_button_yaw_mdeg;
    snapshot->last_button_id =
        (car_debug_button_id_t) g_car_last_button_id;
    snapshot->motor_high_impedance = g_car_motor_high_impedance;
    snapshot->imu_ready = g_car_imu_ready;
    snapshot->imu_attitude_valid = g_car_imu_attitude_valid;
    snapshot->imu_yaw_mdeg = g_car_imu_yaw_mdeg;
    snapshot->imu_yaw_rate_mdps = g_car_imu_yaw_rate_mdps;
    snapshot->yaw_target_mdeg = g_car_yaw_target_mdeg;
    snapshot->yaw_error_mdeg = g_car_yaw_error_mdeg;
    snapshot->yaw_elapsed_ms = g_car_yaw_elapsed_ms;
    snapshot->heading_base_target_pps = g_car_heading_base_target_pps;
    snapshot->line_tracking_base_target_pps =
        g_car_line_tracking_base_target_pps;
    snapshot->line_tracking_elapsed_ms = g_car_line_tracking_elapsed_ms;
    snapshot->line_sensor_state = g_car_line_sensor_state;
    snapshot->line_sensor_active_mask = g_car_line_sensor_active_mask;
    snapshot->line_sensor_error = g_car_line_sensor_error;
    snapshot->line_sensor_seen = g_car_line_sensor_seen;
    snapshot->line_sensor_ready = g_car_line_sensor_ready;
    return true;
}

static int32_t car_debug_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }
    return (int32_t) (value - 0.5f);
}
