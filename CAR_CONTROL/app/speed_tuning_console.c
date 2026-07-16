#include "speed_tuning_console.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "encoder_input.h"
#include "speed_bringup_test.h"
#include "wheel_speed_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SPEED_TUNING_LINE_SIZE  96U
#define SPEED_TUNING_TOKEN_MAX   8U
#define SPEED_TUNING_WAVE_INTERVAL_MS 100U

static char g_line[SPEED_TUNING_LINE_SIZE];
static uint32_t g_last_wave_ms;

static void speed_tuning_process_line(char *line);
static uint16_t speed_tuning_tokenize(
    char *line, char **tokens, uint16_t capacity);
static bool speed_tuning_parse_float(const char *text, float *value);
static bool speed_tuning_parse_u16(const char *text, uint16_t *value);
static bool speed_tuning_parse_profile(
    const char *text, speed_bringup_profile_t *profile);
static void speed_tuning_send_config(void);
static void speed_tuning_send_status(void);
static void speed_tuning_send_wave(uint32_t now_ms);
static void speed_tuning_write_u32(uint32_t value);
static void speed_tuning_write_i32(int32_t value);
static void speed_tuning_write_float4(float value);
static int32_t speed_tuning_round_float(float value);

void SpeedTuningConsole_Init(void)
{
    g_last_wave_ms = 0U;
    BluetoothUart_WriteText("OK READY v=1\r\n");
}

void SpeedTuningConsole_Task(uint32_t now_ms)
{
    bluetooth_uart_line_result_t result;

    result = BluetoothUart_ReadLine(g_line, sizeof(g_line));
    if (result == BLUETOOTH_UART_LINE_OVERFLOW) {
        BluetoothUart_WriteText("ERR line\r\n");
    } else if (result == BLUETOOTH_UART_LINE_READY) {
        speed_tuning_process_line(g_line);
    }

    speed_tuning_send_wave(now_ms);
}

static void speed_tuning_process_line(char *line)
{
    char *tokens[SPEED_TUNING_TOKEN_MAX];
    uint16_t token_count = speed_tuning_tokenize(
        line, tokens, SPEED_TUNING_TOKEN_MAX);

    if (token_count == 0U) {
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

    BluetoothUart_WriteText(
        "ERR command use: spd get|set|run [step|reverse|sweep]|stop|stat\r\n");
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

static void speed_tuning_send_wave(uint32_t now_ms)
{
    wheel_speed_control_snapshot_t speed;

    if ((uint32_t) (now_ms - g_last_wave_ms) <
        SPEED_TUNING_WAVE_INTERVAL_MS) {
        return;
    }
    g_last_wave_ms = now_ms;

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
    uint32_t scaled = (uint32_t) (value * 10000.0f + 0.5f);
    uint32_t fraction = scaled % 10000U;
    char digits[5];

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
