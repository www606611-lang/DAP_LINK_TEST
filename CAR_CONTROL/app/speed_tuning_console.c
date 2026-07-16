#include "speed_tuning_console.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "encoder_input.h"
#include "firmware_update.h"
#include "icm20948.h"
#include "position_bringup_test.h"
#include "speed_bringup_test.h"
#include "wheel_position_control.h"
#include "wheel_speed_control.h"
#include "wheel_yaw_control.h"
#include "yaw_bringup_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SPEED_TUNING_LINE_SIZE  96U
#define SPEED_TUNING_TOKEN_MAX  14U
#define SPEED_TUNING_WAVE_INTERVAL_MS 100U

static char g_line[SPEED_TUNING_LINE_SIZE];
static uint32_t g_last_wave_ms;

static void speed_tuning_process_line(char *line, uint32_t now_ms);
static uint16_t speed_tuning_tokenize(
    char *line, char **tokens, uint16_t capacity);
static bool speed_tuning_parse_float(const char *text, float *value);
static bool speed_tuning_parse_i32(const char *text, int32_t *value);
static bool speed_tuning_parse_u16(const char *text, uint16_t *value);
static bool speed_tuning_parse_profile(
    const char *text, speed_bringup_profile_t *profile);
static bool speed_tuning_parse_position_profile(
    const char *text, position_bringup_profile_t *profile);
static void speed_tuning_send_config(void);
static void speed_tuning_send_status(void);
static void speed_tuning_send_position_config(void);
static void speed_tuning_send_position_status(void);
static void speed_tuning_send_yaw_config(void);
static void speed_tuning_send_yaw_status(void);
static void speed_tuning_send_imu_status(uint32_t now_ms);
static void speed_tuning_send_wave(uint32_t now_ms);
static void speed_tuning_write_u32(uint32_t value);
static void speed_tuning_write_i32(int32_t value);
static void speed_tuning_write_float4(float value);
static int32_t speed_tuning_round_float(float value);

void SpeedTuningConsole_Init(void)
{
    g_last_wave_ms = 0U;
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
        if (SpeedBringupTest_RequestProfile(profile)) {
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
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "stat") == 0)) {
        speed_tuning_send_yaw_status();
        return;
    }
    if ((token_count == 2U) &&
        (strcmp(tokens[0], "yaw") == 0) &&
        (strcmp(tokens[1], "run") == 0)) {
        if (YawBringupTest_RequestStart()) {
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

        if (!WheelYawControl_GetSnapshot(&yaw) || yaw.running ||
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
        if (PositionBringupTest_RequestProfile(profile)) {
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
        "ERR use spd|pos|yaw|imu commands or fw update\r\n");
}

static uint16_t speed_tuning_tokenize(
    char *line, char **tokens, uint16_t capacity)
{
    uint16_t count = 0U;
    bool in_token = false;

    while (*line != '\0') {
        if ((*line == ' ') || (*line == '\t')) {
            *line = '\0';
            in_token = false;
        } else if (!in_token) {
            if (count >= capacity) {
                return capacity + 1U;
            }
            tokens[count++] = line;
            in_token = true;
        }
        line++;
    }
    return count;
}

static bool speed_tuning_parse_float(const char *text, float *value)
{
    float parsed = 0.0f;
    float fraction_scale = 0.1f;
    bool negative = false;
    bool have_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }
    if ((*text == '+') || (*text == '-')) {
        negative = (*text == '-');
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        parsed = parsed * 10.0f + (float) (*text - '0');
        have_digit = true;
        text++;
    }
    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            parsed += (float) (*text - '0') * fraction_scale;
            fraction_scale *= 0.1f;
            have_digit = true;
            text++;
        }
    }
    if (!have_digit || (*text != '\0')) {
        return false;
    }

    *value = negative ? -parsed : parsed;
    return true;
}

static bool speed_tuning_parse_u16(const char *text, uint16_t *value)
{
    uint32_t parsed = 0U;
    bool have_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }
    while ((*text >= '0') && (*text <= '9')) {
        parsed = parsed * 10U + (uint32_t) (*text - '0');
        if (parsed > UINT16_MAX) {
            return false;
        }
        have_digit = true;
        text++;
    }
    if (!have_digit || (*text != '\0')) {
        return false;
    }
    *value = (uint16_t) parsed;
    return true;
}

static bool speed_tuning_parse_i32(const char *text, int32_t *value)
{
    uint32_t magnitude = 0U;
    uint32_t limit = (uint32_t) INT32_MAX;
    bool negative = false;
    bool have_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }
    if ((*text == '+') || (*text == '-')) {
        negative = (*text == '-');
        if (negative) {
            limit++;
        }
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t) (*text - '0');

        if ((magnitude > (limit / 10U)) ||
            ((magnitude == (limit / 10U)) &&
                (digit > (limit % 10U)))) {
            return false;
        }
        magnitude = magnitude * 10U + digit;
        have_digit = true;
        text++;
    }
    if (!have_digit || (*text != '\0')) {
        return false;
    }

    if (negative) {
        *value = (magnitude == ((uint32_t) INT32_MAX + 1U)) ?
            INT32_MIN : -(int32_t) magnitude;
    } else {
        *value = (int32_t) magnitude;
    }
    return true;
}

static bool speed_tuning_parse_profile(
    const char *text, speed_bringup_profile_t *profile)
{
    if ((text == NULL) || (profile == NULL)) {
        return false;
    }
    if (strcmp(text, "step") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_STEP;
        return true;
    }
    if (strcmp(text, "reverse") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_REVERSE;
        return true;
    }
    if (strcmp(text, "sweep") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_SWEEP;
        return true;
    }
    if (strcmp(text, "lease") == 0) {
        *profile = SPEED_BRINGUP_PROFILE_LEASE_TEST;
        return true;
    }
    return false;
}

static bool speed_tuning_parse_position_profile(
    const char *text, position_bringup_profile_t *profile)
{
    if ((text == NULL) || (profile == NULL)) {
        return false;
    }
    if (strcmp(text, "stress") == 0) {
        *profile = POSITION_BRINGUP_PROFILE_STRESS;
        return true;
    }
    return false;
}

static void speed_tuning_send_config(void)
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

static void speed_tuning_send_status(void)
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

static void speed_tuning_send_position_config(void)
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

static void speed_tuning_send_position_status(void)
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

static void speed_tuning_send_imu_status(uint32_t now_ms)
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

static void speed_tuning_send_yaw_config(void)
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

static void speed_tuning_send_yaw_status(void)
{
    wheel_yaw_control_snapshot_t yaw;
    wheel_speed_control_snapshot_t speed;

    if (!WheelYawControl_GetSnapshot(&yaw) ||
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
    BluetoothUart_WriteText("\r\n");
}

static void speed_tuning_send_wave(uint32_t now_ms)
{
    wheel_position_control_snapshot_t position;
    wheel_speed_control_snapshot_t speed;
    wheel_yaw_control_snapshot_t yaw;

    if ((uint32_t) (now_ms - g_last_wave_ms) <
        SPEED_TUNING_WAVE_INTERVAL_MS) {
        return;
    }
    g_last_wave_ms = now_ms;

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

static void speed_tuning_write_u32(uint32_t value)
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

static void speed_tuning_write_i32(int32_t value)
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

static void speed_tuning_write_float4(float value)
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

static int32_t speed_tuning_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }
    return (int32_t) (value - 0.5f);
}
