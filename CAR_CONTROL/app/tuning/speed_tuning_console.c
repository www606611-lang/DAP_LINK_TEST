#include "speed_tuning_console.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "car_app.h"
#include "firmware_update.h"
#include "heading_bringup_test.h"
#include "icm20948.h"
#include "line_follow_mission.h"
#include "line_sensor_bringup.h"
#include "line_tracking_bringup_test.h"
#include "position_bringup_test.h"
#include "speed_bringup_test.h"
#include "system_watchdog.h"
#include "tuning_codec.h"
#include "tuning_status.h"
#include "tuning_wave.h"
#include "tuning_wire.h"
#include "wheel_heading_control.h"
#include "wheel_yaw_control.h"
#include "yaw_bringup_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SPEED_TUNING_LINE_SIZE 96U
#define SPEED_TUNING_TOKEN_MAX 14U

static char g_line[SPEED_TUNING_LINE_SIZE];

static void speed_tuning_process_line(char *line, uint32_t now_ms);

void SpeedTuningConsole_Init(void)
{
    speed_tuning_wave_init();
    BluetoothUart_WriteText("OK READY v=7\r\n");
}

void SpeedTuningConsole_Task(uint32_t now_ms)
{
    bluetooth_uart_line_result_t result;

    result = BluetoothUart_ReadLine(g_line, sizeof(g_line));
    if (result == BLUETOOTH_UART_LINE_OVERFLOW) {
        BluetoothUart_WriteText("ERR line\r\n");
    } else if (result == BLUETOOTH_UART_LINE_READY) {
        speed_tuning_process_line(g_line, now_ms);
    }

    speed_tuning_send_wave(now_ms);
}

static void speed_tuning_process_line(char *line, uint32_t now_ms)
{
    char *tokens[SPEED_TUNING_TOKEN_MAX];
    uint16_t token_count = speed_tuning_tokenize(
        line, tokens, SPEED_TUNING_TOKEN_MAX);

    if (token_count == 0U) {
        return;
    }

    if ((token_count == 2U) &&
        (strcmp(tokens[0], "fw") == 0) &&
        (strcmp(tokens[1], "update") == 0)) {
        if (FirmwareUpdate_RequestBootloader()) {
            BluetoothUart_WriteText("OK FW UPDATE\r\n");
        } else {
            BluetoothUart_WriteText("ERR motor_active\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "app") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        car_app_snapshot_t app;

        if (!CarApp_GetSnapshot(&app)) {
            BluetoothUart_WriteText("ERR app_state\r\n");
            return;
        }
        BluetoothUart_WriteText("ASTAT state=");
        BluetoothUart_WriteText(CarApp_GetStateText());
        BluetoothUart_WriteText(" workflow=");
        speed_tuning_write_u32((uint32_t) app.active_workflow);
        BluetoothUart_WriteText(" action=");
        speed_tuning_write_u32((uint32_t) app.action);
        BluetoothUart_WriteText(" yaw=");
        speed_tuning_write_i32(app.yaw_command_mdeg);
        BluetoothUart_WriteText(" transitions=");
        speed_tuning_write_u32(app.transition_count);
        BluetoothUart_WriteText(" hz=");
        speed_tuning_write_u32(
            BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
        BluetoothUart_WriteText("\r\n");
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "wdt") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        BluetoothUart_WriteText("WSTAT active=");
        speed_tuning_write_u32(
            SystemWatchdog_IsKickEnabled() ? 1U : 0U);
        BluetoothUart_WriteText(" kicks=");
        speed_tuning_write_u32(SystemWatchdog_GetKickCount());
        BluetoothUart_WriteText(" hz=");
        speed_tuning_write_u32(
            BoardMotorSafe_IsHighImpedance() ? 1U : 0U);
        BluetoothUart_WriteText("\r\n");
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "wdt") == 0) &&
        (strcmp(tokens[1], "test") == 0)) {
        if (!BoardMotorSafe_IsHighImpedance()) {
            BluetoothUart_WriteText("ERR motor_active\r\n");
        } else if (!SystemWatchdog_StopKicksForTest()) {
            BluetoothUart_WriteText("ERR wdt_state\r\n");
        } else {
            BluetoothUart_WriteText("OK WDT TEST\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "mission") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_mission_status(now_ms);
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "mission") == 0) &&
        (strcmp(tokens[1], "start") == 0)) {
        if (!LineTrackingBringupTest_IsActive() &&
            LineFollowMission_RequestStart()) {
            BluetoothUart_WriteText("OK MISSION START\r\n");
        } else {
            BluetoothUart_WriteText("ERR run_state\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "mission") == 0) &&
        (strcmp(tokens[1], "stop") == 0)) {
        LineFollowMission_RequestStop();
        BluetoothUart_WriteText("OK MISSION STOP\r\n");
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "line") == 0) &&
        (strcmp(tokens[1], "get") == 0)) {
        speed_tuning_send_line_config();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "line") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_line_status(now_ms);
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "line") == 0) &&
        (strcmp(tokens[1], "cal") == 0)) {
        if (!BoardMotorSafe_IsHighImpedance()) {
            BluetoothUart_WriteText("ERR motor_active\r\n");
        } else if (!LineSensorBringup_RequestCalibration()) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else {
            BluetoothUart_WriteText("OK LINE CAL\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "line") == 0) &&
        (strcmp(tokens[1], "run") == 0)) {
        if (!LineFollowMission_IsActive() &&
            LineTrackingBringupTest_RequestStart()) {
            BluetoothUart_WriteText("OK LINE RUN\r\n");
        } else {
            BluetoothUart_WriteText("ERR run_state\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "line") == 0) &&
        (strcmp(tokens[1], "stop") == 0)) {
        LineTrackingBringupTest_RequestStop();
        BluetoothUart_WriteText("OK LINE STOP\r\n");
        return;
    }
    if ((token_count == 10U) &&
        (strcmp(tokens[0], "line") == 0) &&
        (strcmp(tokens[1], "set") == 0)) {
        line_tracking_bringup_config_t config;
        line_tracking_bringup_config_result_t result;
        uint16_t duration_ms;

        if (LineFollowMission_IsActive()) {
            BluetoothUart_WriteText("ERR busy\r\n");
            return;
        }
        if (!LineTrackingBringupTest_GetConfig(&config) ||
            !speed_tuning_parse_float(tokens[2], &config.control.kp) ||
            !speed_tuning_parse_float(tokens[3], &config.control.ki) ||
            !speed_tuning_parse_float(tokens[4], &config.control.kd) ||
            !speed_tuning_parse_float(tokens[5], &config.base_speed_pps) ||
            !speed_tuning_parse_float(
                tokens[6], &config.control.max_correction_pps) ||
            !speed_tuning_parse_u16(
                tokens[7], &config.output_limit_permille) ||
            !speed_tuning_parse_float(
                tokens[8], &config.control.deadband) ||
            !speed_tuning_parse_u16(tokens[9], &duration_ms)) {
            BluetoothUart_WriteText("ERR number\r\n");
            return;
        }
        config.duration_ms = duration_ms;
        result = LineTrackingBringupTest_SetConfig(&config);
        if (result == LINE_TRACKING_BRINGUP_CONFIG_BUSY) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else if (result != LINE_TRACKING_BRINGUP_CONFIG_OK) {
            BluetoothUart_WriteText("ERR range\r\n");
        } else {
            speed_tuning_send_line_config();
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "spd") == 0) &&
        (strcmp(tokens[1], "get") == 0)) {
        speed_tuning_send_config();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "spd") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_status();
        return;
    }
    if (((token_count == 2U) || (token_count == 3U)) &&
        (strcmp(tokens[0], "spd") == 0) &&
        (strcmp(tokens[1], "run") == 0)) {
        speed_bringup_profile_t profile = SPEED_BRINGUP_PROFILE_RAMP;

        if ((token_count == 3U) &&
            !speed_tuning_parse_profile(tokens[2], &profile)) {
            BluetoothUart_WriteText("ERR profile\r\n");
            return;
        }
        if (!LineFollowMission_IsActive() &&
            SpeedBringupTest_RequestProfile(profile)) {
            BluetoothUart_WriteText("OK RUN ");
            BluetoothUart_WriteText(SpeedBringupTest_GetProfileText());
            BluetoothUart_WriteText("\r\n");
        } else {
            BluetoothUart_WriteText("ERR run_state\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "spd") == 0) &&
        (strcmp(tokens[1], "stop") == 0)) {
        SpeedBringupTest_RequestStop();
        BluetoothUart_WriteText("OK STOP\r\n");
        return;
    }
    if ((token_count == 7U) &&
        (strcmp(tokens[0], "spd") == 0) &&
        (strcmp(tokens[1], "set") == 0)) {
        speed_bringup_config_t config;
        speed_bringup_config_result_t result;

        if (!speed_tuning_parse_float(tokens[2], &config.pid.kp) ||
            !speed_tuning_parse_float(tokens[3], &config.pid.ki) ||
            !speed_tuning_parse_float(tokens[4], &config.pid.kd) ||
            !speed_tuning_parse_float(tokens[5], &config.target_pps) ||
            !speed_tuning_parse_u16(
                tokens[6], &config.output_limit_permille)) {
            BluetoothUart_WriteText("ERR number\r\n");
            return;
        }

        result = SpeedBringupTest_SetConfig(&config);
        if (result == SPEED_BRINGUP_CONFIG_BUSY) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else if (result != SPEED_BRINGUP_CONFIG_OK) {
            BluetoothUart_WriteText("ERR range\r\n");
        } else {
            speed_tuning_send_config();
        }
        return;
    }

    if ((token_count == 2U) &&
        (strcmp(tokens[0], "pos") == 0) &&
        (strcmp(tokens[1], "get") == 0)) {
        speed_tuning_send_position_config();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "imu") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_imu_status(now_ms);
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "get") == 0)) {
        speed_tuning_send_yaw_config();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "heading") == 0) &&
        (strcmp(tokens[1], "get") == 0)) {
        speed_tuning_send_heading_config();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "heading") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_heading_status();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "heading") == 0) &&
        (strcmp(tokens[1], "run") == 0)) {
        if (!LineFollowMission_IsActive() &&
            !YawBringupTest_IsActive() &&
            HeadingBringupTest_RequestStart()) {
            BluetoothUart_WriteText("OK HEADING RUN\r\n");
        } else {
            BluetoothUart_WriteText("ERR run_state\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "heading") == 0) &&
        (strcmp(tokens[1], "stop") == 0)) {
        HeadingBringupTest_RequestStop();
        BluetoothUart_WriteText("OK HEADING STOP\r\n");
        return;
    }
    if ((token_count == 10U) &&
        (strcmp(tokens[0], "heading") == 0) &&
        (strcmp(tokens[1], "set") == 0)) {
        heading_bringup_config_t config;
        heading_bringup_config_result_t result;
        uint16_t duration_ms;

        if (!HeadingBringupTest_GetConfig(&config) ||
            !speed_tuning_parse_float(tokens[2], &config.control.kp) ||
            !speed_tuning_parse_float(tokens[3], &config.control.ki) ||
            !speed_tuning_parse_float(tokens[4], &config.control.kd) ||
            !speed_tuning_parse_float(tokens[5], &config.base_speed_pps) ||
            !speed_tuning_parse_float(
                tokens[6], &config.control.max_correction_pps) ||
            !speed_tuning_parse_u16(
                tokens[7], &config.output_limit_permille) ||
            !speed_tuning_parse_float(
                tokens[8], &config.control.deadband_deg) ||
            !speed_tuning_parse_u16(tokens[9], &duration_ms)) {
            BluetoothUart_WriteText("ERR number\r\n");
            return;
        }
        config.duration_ms = duration_ms;
        result = HeadingBringupTest_SetConfig(&config);
        if (result == HEADING_BRINGUP_CONFIG_BUSY) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else if (result != HEADING_BRINGUP_CONFIG_OK) {
            BluetoothUart_WriteText("ERR range\r\n");
        } else {
            speed_tuning_send_heading_config();
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_yaw_status();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "run") == 0)) {
        if (!LineFollowMission_IsActive() &&
            !HeadingBringupTest_IsActive() &&
            YawBringupTest_RequestStart()) {
            BluetoothUart_WriteText("OK YAW RUN\r\n");
        } else {
            BluetoothUart_WriteText("ERR run_state\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "stop") == 0)) {
        YawBringupTest_RequestStop();
        BluetoothUart_WriteText("OK YAW STOP\r\n");
        return;
    }
    if (((token_count == 12U) || (token_count == 13U) ||
            (token_count == 14U)) &&
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "set") == 0)) {
        yaw_bringup_config_t config;
        yaw_bringup_config_result_t result;
        uint16_t timeout_ms;

        if (!YawBringupTest_GetConfig(&config) ||
            !speed_tuning_parse_float(tokens[2], &config.control.kp) ||
            !speed_tuning_parse_float(tokens[3], &config.control.ki) ||
            !speed_tuning_parse_float(tokens[4], &config.control.kd) ||
            !speed_tuning_parse_float(tokens[5], &config.target_yaw_deg) ||
            !speed_tuning_parse_float(
                tokens[6], &config.control.max_turn_speed_pps) ||
            !speed_tuning_parse_u16(
                tokens[7], &config.output_limit_permille) ||
            !speed_tuning_parse_float(
                tokens[8], &config.control.tolerance_deg) ||
            !speed_tuning_parse_float(
                tokens[9], &config.control.settle_yaw_rate_dps) ||
            !speed_tuning_parse_u16(
                tokens[10], &config.control.settle_time_ms) ||
            !speed_tuning_parse_u16(tokens[11], &timeout_ms) ||
            ((token_count == 13U) &&
                !speed_tuning_parse_float(
                    tokens[12], &config.control.min_turn_speed_pps)) ||
            ((token_count == 14U) &&
                (!speed_tuning_parse_float(
                    tokens[12], &config.control.min_turn_speed_pps) ||
                 !speed_tuning_parse_u16(tokens[13],
                    &config.control.feedforward_boost_permille)))) {
            BluetoothUart_WriteText("ERR number\r\n");
            return;
        }
        config.timeout_ms = timeout_ms;

        result = YawBringupTest_SetConfig(&config);
        if (result == YAW_BRINGUP_CONFIG_BUSY) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else if (result != YAW_BRINGUP_CONFIG_OK) {
            BluetoothUart_WriteText("ERR range\r\n");
        } else {
            speed_tuning_send_yaw_config();
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "imu") == 0) &&
        (strcmp(tokens[1], "zero") == 0)) {
        wheel_yaw_control_snapshot_t yaw;
        wheel_heading_control_snapshot_t heading;

        if (!WheelYawControl_GetSnapshot(&yaw) || yaw.running ||
            !WheelHeadingControl_GetSnapshot(&heading) ||
            heading.running ||
            !BoardMotorSafe_IsHighImpedance()) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else if (!ICM20948_IsReady()) {
            BluetoothUart_WriteText("ERR imu_offline\r\n");
        } else {
            ICM20948_ResetYaw();
            BluetoothUart_WriteText("OK IMU ZERO\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "pos") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_position_status();
        return;
    }
    if (((token_count == 2U) || (token_count == 3U)) &&
        (strcmp(tokens[0], "pos") == 0) &&
        (strcmp(tokens[1], "run") == 0)) {
        position_bringup_profile_t profile =
            POSITION_BRINGUP_PROFILE_SINGLE;

        if ((token_count == 3U) &&
            !speed_tuning_parse_position_profile(
                tokens[2], &profile)) {
            BluetoothUart_WriteText("ERR profile\r\n");
            return;
        }
        if (!LineFollowMission_IsActive() &&
            PositionBringupTest_RequestProfile(profile)) {
            BluetoothUart_WriteText("OK POS RUN ");
            BluetoothUart_WriteText(
                (profile == POSITION_BRINGUP_PROFILE_STRESS) ?
                    "STRESS" : "SINGLE");
            BluetoothUart_WriteText("\r\n");
        } else {
            BluetoothUart_WriteText("ERR run_state\r\n");
        }
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "pos") == 0) &&
        (strcmp(tokens[1], "stop") == 0)) {
        PositionBringupTest_RequestStop();
        BluetoothUart_WriteText("OK POS STOP\r\n");
        return;
    }
    if (((token_count == 7U) || (token_count == 9U)) &&
        (strcmp(tokens[0], "pos") == 0) &&
        (strcmp(tokens[1], "set") == 0)) {
        position_bringup_config_t config;
        position_bringup_config_result_t result;

        if (!PositionBringupTest_GetConfig(&config) ||
            !speed_tuning_parse_float(tokens[2], &config.control.kp) ||
            !speed_tuning_parse_i32(tokens[3], &config.target_counts) ||
            !speed_tuning_parse_float(
                tokens[4], &config.control.max_speed_pps) ||
            !speed_tuning_parse_u16(
                tokens[5], &config.output_limit_permille) ||
            !speed_tuning_parse_u16(
                tokens[6], &config.control.tolerance_counts) ||
            ((token_count == 9U) &&
                (!speed_tuning_parse_float(
                    tokens[7], &config.control.sync_kp) ||
                 !speed_tuning_parse_float(tokens[8],
                    &config.control.sync_max_correction_pps)))) {
            BluetoothUart_WriteText("ERR number\r\n");
            return;
        }

        result = PositionBringupTest_SetConfig(&config);
        if (result == POSITION_BRINGUP_CONFIG_BUSY) {
            BluetoothUart_WriteText("ERR busy\r\n");
        } else if (result != POSITION_BRINGUP_CONFIG_OK) {
            BluetoothUart_WriteText("ERR range\r\n");
        } else {
            speed_tuning_send_position_config();
        }
        return;
    }

    BluetoothUart_WriteText(
        "ERR use spd|pos|yaw|heading|line|mission|imu or fw update\r\n");
}
