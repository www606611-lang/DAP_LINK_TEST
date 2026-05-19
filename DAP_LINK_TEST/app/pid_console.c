#include "pid_console.h"

#include "encoder.h"
#include "encoder_position_control.h"
#include "encoder_speed_control.h"
#include "bluetooth_uart.h"
#include "lcd_status.h"
#include "pid_tuning_store.h"
#include "line_tracking_control.h"
#include "uart0_dma.h"
#include "yaw_angle_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PID_CONSOLE_LINE_MAX   192U
#define PID_CONSOLE_TOKEN_MAX  16U
#define PID_CONSOLE_TX_CHUNK   128U

static void pid_console_send_text(const char *text);
static void pid_console_send_help(void);
static void pid_console_reply_ok(void);
static void pid_console_reply_err(void);
static void pid_console_set_lcd_text(const char *text);
static char *pid_console_trim_in_place(char *text);
static void pid_console_lowercase_in_place(char *text);
static uint8_t pid_console_tokenize(char *line, char *tokens[],
    uint8_t max_tokens);
static bool pid_console_parse_float(const char *text, float *value);
static bool pid_console_parse_bool(const char *text, bool *value);
static bool pid_console_token_eq(const char *a, const char *b);
static long pid_console_round_to_long(float value);
static void pid_console_format_float_trimmed(
    float value, char *buffer, size_t buffer_size);
static void pid_console_format_limit_range(
    float min_value, float max_value, char *buffer, size_t buffer_size);
static char pid_console_label_for_id(encoder_id_t id);
static bool pid_console_parse_encoder_id(
    const char *text, encoder_id_t *id);
static void pid_console_update_speed_lcd(encoder_id_t id);
static void pid_console_update_position_lcd(encoder_id_t id, bool outer_loop);
static void pid_console_update_yaw_lcd(void);
static void pid_console_update_line_lcd(void);
static void pid_console_sync_position_speed(void);
static void pid_console_show_speed(encoder_id_t id);
static void pid_console_show_position(encoder_id_t id);
static void pid_console_show_yaw(void);
static void pid_console_show_line(void);
static void pid_console_show_all(void);
static void pid_console_show_store(void);
static void pid_console_autosave_tunings(bool changed);
static bool pid_console_handle_speed(char *const *tokens, uint8_t count,
    uint32_t now_ms);
static bool pid_console_handle_position(char *const *tokens, uint8_t count,
    uint32_t now_ms);
static bool pid_console_handle_yaw(char *const *tokens, uint8_t count,
    uint32_t now_ms);
    static bool pid_console_handle_line(char *const *tokens, uint8_t count,
    uint32_t now_ms);
static bool pid_console_handle_speed_tuning(encoder_id_t id,
    const char *field, float value);
static bool pid_console_handle_position_tuning(encoder_id_t id,
    const char *section, const char *field, float value);
static bool pid_console_handle_yaw_tuning(const char *field, float value);
static bool pid_console_handle_line_tuning(const char *field, float value);

static pid_console_port_t g_pid_console_port = PID_CONSOLE_PORT_UART0;

void pid_console_process_line(const char *line, uint16_t length,
    uint32_t now_ms, pid_console_port_t port)
{
    char local_line[PID_CONSOLE_LINE_MAX];
    char *tokens[PID_CONSOLE_TOKEN_MAX];
    char *text;
    uint8_t count;

    if ((line == NULL) || (length == 0U)) {
        return;
    }

    if (length >= PID_CONSOLE_LINE_MAX) {
        length = (uint16_t) (PID_CONSOLE_LINE_MAX - 1U);
    }

    g_pid_console_port = port;
    (void) memcpy(local_line, line, length);
    local_line[length] = '\0';

    pid_console_lowercase_in_place(local_line);
    text = pid_console_trim_in_place(local_line);
    if ((text == NULL) || (*text == '\0')) {
        return;
    }

    count = pid_console_tokenize(text, tokens, PID_CONSOLE_TOKEN_MAX);
    if (count == 0U) {
        return;
    }

    if (!pid_console_token_eq(tokens[0], "pid")) {
        return;
    }

    if ((count >= 2U) && pid_console_token_eq(tokens[1], "help")) {
        pid_console_send_help();
        return;
    }

    if (count < 2U) {
        pid_console_reply_err();
        return;
    }

    if (pid_console_token_eq(tokens[1], "show")) {
        if (count < 3U) {
            pid_console_show_all();
            pid_console_reply_ok();
            return;
        }

        if (pid_console_token_eq(tokens[2], "all")) {
            pid_console_show_all();
            pid_console_reply_ok();
            return;
        }

        if (pid_console_token_eq(tokens[2], "speed")) {
            if (count >= 4U) {
                encoder_id_t id;

                if (!pid_console_parse_encoder_id(tokens[3], &id)) {
                    pid_console_reply_err();
                    return;
                }
                pid_console_show_speed(id);
            } else {
                pid_console_show_speed(ENCODER_ID_COUNT);
            }
            pid_console_reply_ok();
            return;
        }

        if (pid_console_token_eq(tokens[2], "position")) {
            if (count >= 4U) {
                encoder_id_t id;

                if (!pid_console_parse_encoder_id(tokens[3], &id)) {
                    pid_console_reply_err();
                    return;
                }
                pid_console_show_position(id);
            } else {
                pid_console_show_position(ENCODER_ID_COUNT);
            }
            pid_console_reply_ok();
            return;
        }

        if (pid_console_token_eq(tokens[2], "yaw")) {
            pid_console_show_yaw();
            pid_console_reply_ok();
            return;
        }

        if (pid_console_token_eq(tokens[2], "line")) {
            pid_console_show_line();
            pid_console_reply_ok();
            return;
        }

        if (pid_console_token_eq(tokens[2], "store")) {
            pid_console_show_store();
            pid_console_reply_ok();
            return;
        }

        pid_console_reply_err();
        return;
    }

    if (pid_console_token_eq(tokens[1], "status")) {
        pid_console_show_store();
        pid_console_reply_ok();
        return;
    }

    if (pid_console_token_eq(tokens[1], "save")) {
        pid_tuning_store_status_t status = PidTuningStore_SaveCurrent();
        if (status == PID_TUNING_STORE_OK) {
            pid_console_send_text("pid save ok\r\n");
            pid_console_reply_ok();
        } else {
            pid_console_send_text("pid save err\r\n");
            pid_console_reply_err();
        }
        return;
    }

    if (pid_console_token_eq(tokens[1], "load")) {
        pid_tuning_store_status_t status = PidTuningStore_LoadApply();
        if (status == PID_TUNING_STORE_OK) {
            pid_console_send_text("pid load ok\r\n");
            pid_console_reply_ok();
        } else {
            pid_console_send_text("pid load err\r\n");
            pid_console_reply_err();
        }
        return;
    }

    if (pid_console_token_eq(tokens[1], "clear")) {
        pid_tuning_store_status_t status = PidTuningStore_Clear();
        if (status == PID_TUNING_STORE_OK) {
            pid_console_send_text("pid clear ok\r\n");
            pid_console_reply_ok();
        } else {
            pid_console_send_text("pid clear err\r\n");
            pid_console_reply_err();
        }
        return;
    }

    if (count < 2U) {
        pid_console_reply_err();
        return;
    }

    if (pid_console_token_eq(tokens[1], "speed")) {
        if (pid_console_handle_speed(tokens, count, now_ms)) {
            pid_console_reply_ok();
        } else {
            pid_console_reply_err();
        }
        return;
    }

    if (pid_console_token_eq(tokens[1], "position")) {
        if (pid_console_handle_position(tokens, count, now_ms)) {
            pid_console_reply_ok();
        } else {
            pid_console_reply_err();
        }
        return;
    }

    if (pid_console_token_eq(tokens[1], "yaw")) {
        if (pid_console_handle_yaw(tokens, count, now_ms)) {
            pid_console_reply_ok();
        } else {
            pid_console_reply_err();
        }
        return;
    }

    if (pid_console_token_eq(tokens[1], "line")) {
        if (pid_console_handle_line(tokens, count, now_ms)) {
            pid_console_reply_ok();
        } else {
            pid_console_reply_err();
        }
        return;
    }
}

static void pid_console_send_text(const char *text)
{
    char chunk[PID_CONSOLE_TX_CHUNK];
    size_t length;
    size_t offset = 0U;

    if (text == NULL) {
        return;
    }

    length = strlen(text);
    while (offset < length) {
        size_t chunk_length = length - offset;

        if (chunk_length > (PID_CONSOLE_TX_CHUNK - 1U)) {
            chunk_length = PID_CONSOLE_TX_CHUNK - 1U;
        }

        (void) memcpy(chunk, &text[offset], chunk_length);
        chunk[chunk_length] = '\0';

        if (g_pid_console_port == PID_CONSOLE_PORT_UART3) {
            bluetooth_uart_send_text(chunk);
        } else {
            uart0_dma_task();
            (void) uart0_dma_send_text(chunk);
        }
        offset += chunk_length;
    }
}

static void pid_console_send_help(void)
{
    pid_console_send_text(
        "pid: show/save/load/clear/speed/position/yaw/line. "
        "ex: pid speed right kp 0.06\r\n");
}

static void pid_console_reply_ok(void)
{
    pid_console_send_text("pid:ok\r\n");
}

static void pid_console_reply_err(void)
{
    pid_console_send_text("pid:err\r\n");
}

static char *pid_console_trim_in_place(char *text)
{
    char *end;

    if (text == NULL) {
        return NULL;
    }

    while ((*text == ' ') || (*text == '\t') || (*text == '\r') ||
           (*text == '\n')) {
        text++;
    }

    if (*text == '\0') {
        return text;
    }

    end = text + strlen(text);
    while ((end > text) &&
           ((end[-1] == ' ') || (end[-1] == '\t') ||
            (end[-1] == '\r') || (end[-1] == '\n'))) {
        end--;
    }
    *end = '\0';

    return text;
}

static void pid_console_lowercase_in_place(char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        if ((*text >= 'A') && (*text <= 'Z')) {
            *text = (char) (*text - 'A' + 'a');
        }
        text++;
    }
}

static uint8_t pid_console_tokenize(char *line, char *tokens[],
    uint8_t max_tokens)
{
    char *token;
    uint8_t count = 0U;

    if ((line == NULL) || (tokens == NULL) || (max_tokens == 0U)) {
        return 0U;
    }

    token = strtok(line, " \t,=\r\n");
    while ((token != NULL) && (count < max_tokens)) {
        tokens[count++] = token;
        token = strtok(NULL, " \t,=\r\n");
    }

    return count;
}

static bool pid_console_parse_float(const char *text, float *value)
{
    char *end = NULL;
    float parsed;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }

    parsed = strtof(text, &end);
    if ((end == text) || (*end != '\0')) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool pid_console_parse_bool(const char *text, bool *value)
{
    if ((text == NULL) || (value == NULL)) {
        return false;
    }

    if (pid_console_token_eq(text, "1") || pid_console_token_eq(text, "on") ||
        pid_console_token_eq(text, "true") ||
        pid_console_token_eq(text, "enable") ||
        pid_console_token_eq(text, "enabled")) {
        *value = true;
        return true;
    }

    if (pid_console_token_eq(text, "0") || pid_console_token_eq(text, "off") ||
        pid_console_token_eq(text, "false") ||
        pid_console_token_eq(text, "disable") ||
        pid_console_token_eq(text, "disabled")) {
        *value = false;
        return true;
    }

    return false;
}

static bool pid_console_token_eq(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL)) {
        return false;
    }

    return strcmp(a, b) == 0;
}

static void pid_console_set_lcd_text(const char *text)
{
    lcd_status_screen_set_pid_text(text);
}

static long pid_console_round_to_long(float value)
{
    if (value >= 0.0f) {
        return (long) (value + 0.5f);
    }

    return (long) (value - 0.5f);
}

static void pid_console_format_float_trimmed(
    float value, char *buffer, size_t buffer_size)
{
    long scaled;
    long abs_scaled;
    long integer_part;
    long fraction_part;
    size_t len;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    scaled = pid_console_round_to_long(value * 1000.0f);
    abs_scaled = (scaled < 0L) ? -scaled : scaled;
    integer_part = abs_scaled / 1000L;
    fraction_part = abs_scaled % 1000L;

    if (scaled < 0L) {
        (void) snprintf(buffer, buffer_size, "-%ld.%03ld",
            integer_part, fraction_part);
    } else {
        (void) snprintf(buffer, buffer_size, "%ld.%03ld",
            integer_part, fraction_part);
    }

    len = strlen(buffer);
    while ((len > 0U) && (buffer[len - 1U] == '0')) {
        buffer[--len] = '\0';
    }
    if ((len > 0U) && (buffer[len - 1U] == '.')) {
        buffer[--len] = '\0';
    }
}

static void pid_console_format_limit_range(
    float min_value, float max_value, char *buffer, size_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    (void) snprintf(buffer, buffer_size, "[%ld,%ld]",
        pid_console_round_to_long(min_value),
        pid_console_round_to_long(max_value));
}

static char pid_console_label_for_id(encoder_id_t id)
{
    switch (id) {
        case ENCODER_LEFT:
            return 'L';
        case ENCODER_RIGHT:
            return 'R';
        case ENCODER_ID_COUNT:
            return 'A';
        default:
            return '?';
    }
}

static bool pid_console_parse_encoder_id(
    const char *text, encoder_id_t *id)
{
    if ((text == NULL) || (id == NULL)) {
        return false;
    }

    if (pid_console_token_eq(text, "left") ||
        pid_console_token_eq(text, "l")) {
        *id = ENCODER_LEFT;
        return true;
    }
    if (pid_console_token_eq(text, "right") ||
        pid_console_token_eq(text, "r")) {
        *id = ENCODER_RIGHT;
        return true;
    }
    if (pid_console_token_eq(text, "all") ||
        pid_console_token_eq(text, "a")) {
        *id = ENCODER_ID_COUNT;
        return true;
    }

    return false;
}

static void pid_console_update_speed_lcd(encoder_id_t id)
{
    encoder_speed_control_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char text[96];

    EncoderSpeedControl_GetSpeedConfig(
        (id == ENCODER_ID_COUNT) ? ENCODER_LEFT : id, &config);
    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    (void) snprintf(text, sizeof(text), "SPD %c kp=%s ki=%s kd=%s out=%ld",
        pid_console_label_for_id(id), kp, ki, kd,
        pid_console_round_to_long(config.output_max));
    pid_console_set_lcd_text(text);
}

static void pid_console_sync_position_speed(void)
{
    EncoderPositionControl_SyncSpeedFromCurrent();
}

static void pid_console_update_position_lcd(encoder_id_t id, bool outer_loop)
{
    encoder_position_control_pid_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char text[96];

    if (outer_loop) {
        EncoderPositionControl_GetPositionConfig(
            (id == ENCODER_ID_COUNT) ? ENCODER_LEFT : id, &config);
    } else {
        EncoderPositionControl_GetSpeedConfig(
            (id == ENCODER_ID_COUNT) ? ENCODER_LEFT : id, &config);
    }

    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    (void) snprintf(text, sizeof(text), "%s %c kp=%s ki=%s kd=%s out=%ld",
        outer_loop ? "POSP" : "POSS", pid_console_label_for_id(id),
        kp, ki, kd, pid_console_round_to_long(config.output_max));
    pid_console_set_lcd_text(text);
}

static void pid_console_update_yaw_lcd(void)
{
    yaw_angle_control_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char db[16];
    char text[96];

    YawAngleControl_GetConfig(&config);
    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    pid_console_format_float_trimmed(config.deadband, db, sizeof(db));
    (void) snprintf(text, sizeof(text),
        "YAW kp=%s ki=%s kd=%s out=%ld db=%s",
        kp, ki, kd, pid_console_round_to_long(config.output_max), db);
    pid_console_set_lcd_text(text);
}

static void pid_console_update_line_lcd(void)
{
    line_tracking_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char text[96];

    LineTrackingControl_GetConfig(&config);
    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    (void) snprintf(text, sizeof(text),
        "LINE kp=%s ki=%s kd=%s base=%ld out=%ld",
        kp, ki, kd, pid_console_round_to_long(config.base_speed_pps),
        pid_console_round_to_long(config.right_pwm_limit));
    pid_console_set_lcd_text(text);
}

static void pid_console_show_speed(encoder_id_t id)
{
    if (id == ENCODER_ID_COUNT) {
        pid_console_show_speed(ENCODER_LEFT);
        pid_console_show_speed(ENCODER_RIGHT);
        return;
    }

    encoder_speed_control_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char out[24];
    char ilim[24];
    char db[16];
    char ff[16];
    char fref[16];
    char min_fwd[16];
    char min_rev[16];
    char min_ref[16];
    char line[PID_CONSOLE_LINE_MAX];

    if ((id != ENCODER_LEFT) && (id != ENCODER_RIGHT) &&
        (id != ENCODER_ID_COUNT)) {
        return;
    }

    EncoderSpeedControl_GetSpeedConfig(
        (id == ENCODER_ID_COUNT) ? ENCODER_LEFT : id, &config);
    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    pid_console_format_limit_range(config.output_min, config.output_max,
        out, sizeof(out));
    pid_console_format_limit_range(config.integral_min, config.integral_max,
        ilim, sizeof(ilim));
    pid_console_format_float_trimmed(config.deadband, db, sizeof(db));
    pid_console_format_float_trimmed(config.feedforward_pwm, ff, sizeof(ff));
    pid_console_format_float_trimmed(config.feedforward_reference_pps,
        fref, sizeof(fref));
    pid_console_format_float_trimmed(config.forward_min_drive_pwm,
        min_fwd, sizeof(min_fwd));
    pid_console_format_float_trimmed(config.reverse_min_drive_pwm,
        min_rev, sizeof(min_rev));
    pid_console_format_float_trimmed(config.min_drive_reference_pps,
        min_ref, sizeof(min_ref));

    (void) snprintf(line, sizeof(line),
        "speed %c kp=%s ki=%s kd=%s out=%s ilim=%s db=%s\r\n",
        pid_console_label_for_id(id), kp, ki, kd, out, ilim, db);
    pid_console_send_text(line);
    (void) snprintf(line, sizeof(line),
        "speed %c ff=%s fref=%s min=%s,%s,%s\r\n",
        pid_console_label_for_id(id), ff, fref, min_fwd, min_rev, min_ref);
    pid_console_send_text(line);
    pid_console_update_speed_lcd(id);
}

static void pid_console_show_position(encoder_id_t id)
{
    if (id == ENCODER_ID_COUNT) {
        pid_console_show_position(ENCODER_LEFT);
        pid_console_show_position(ENCODER_RIGHT);
        return;
    }

    encoder_position_control_pid_config_t pos_config;
    encoder_position_control_pid_config_t spd_config;
    char kp[16];
    char ki[16];
    char kd[16];
    char out[24];
    char ilim[24];
    char db[16];
    char line[PID_CONSOLE_LINE_MAX];

    if ((id != ENCODER_LEFT) && (id != ENCODER_RIGHT) &&
        (id != ENCODER_ID_COUNT)) {
        return;
    }

    EncoderPositionControl_GetPositionConfig(
        (id == ENCODER_ID_COUNT) ? ENCODER_LEFT : id, &pos_config);
    EncoderPositionControl_GetSpeedConfig(
        (id == ENCODER_ID_COUNT) ? ENCODER_LEFT : id, &spd_config);

    pid_console_format_float_trimmed(pos_config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(pos_config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(pos_config.kd, kd, sizeof(kd));
    pid_console_format_limit_range(pos_config.output_min, pos_config.output_max,
        out, sizeof(out));
    pid_console_format_limit_range(pos_config.integral_min,
        pos_config.integral_max, ilim, sizeof(ilim));
    pid_console_format_float_trimmed(pos_config.deadband, db, sizeof(db));

    (void) snprintf(line, sizeof(line),
        "position %c pos kp=%s ki=%s kd=%s out=%s ilim=%s db=%s\r\n",
        pid_console_label_for_id(id), kp, ki, kd, out, ilim, db);
    pid_console_send_text(line);

    pid_console_format_float_trimmed(spd_config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(spd_config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(spd_config.kd, kd, sizeof(kd));
    pid_console_format_limit_range(spd_config.output_min, spd_config.output_max,
        out, sizeof(out));
    pid_console_format_limit_range(spd_config.integral_min,
        spd_config.integral_max, ilim, sizeof(ilim));
    pid_console_format_float_trimmed(spd_config.deadband, db, sizeof(db));
    (void) snprintf(line, sizeof(line),
        "position %c spd kp=%s ki=%s kd=%s out=%s ilim=%s db=%s\r\n",
        pid_console_label_for_id(id), kp, ki, kd, out, ilim, db);
    pid_console_send_text(line);

    pid_console_update_position_lcd(id, true);
}

static void pid_console_show_yaw(void)
{
    yaw_angle_control_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char out[24];
    char ilim[24];
    char db[16];
    char line[PID_CONSOLE_LINE_MAX];

    YawAngleControl_GetConfig(&config);
    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    pid_console_format_limit_range(config.output_min, config.output_max,
        out, sizeof(out));
    pid_console_format_limit_range(config.integral_min, config.integral_max,
        ilim, sizeof(ilim));
    pid_console_format_float_trimmed(config.deadband, db, sizeof(db));

    (void) snprintf(line, sizeof(line),
        "yaw kp=%s ki=%s kd=%s out=%s ilim=%s db=%s minturn=%ld maxturn=%ld\r\n",
        kp, ki, kd, out, ilim, db,
        pid_console_round_to_long(config.min_turn_speed_pps),
        pid_console_round_to_long(config.max_turn_speed_pps));
    pid_console_send_text(line);
    pid_console_update_yaw_lcd();
}

static void pid_console_show_line(void)
{
    line_tracking_config_t config;
    char kp[16];
    char ki[16];
    char kd[16];
    char out[24];
    char ilim[24];
    char db[16];
    char base[16];
    char line[PID_CONSOLE_LINE_MAX];

    LineTrackingControl_GetConfig(&config);
    pid_console_format_float_trimmed(config.kp, kp, sizeof(kp));
    pid_console_format_float_trimmed(config.ki, ki, sizeof(ki));
    pid_console_format_float_trimmed(config.kd, kd, sizeof(kd));
    pid_console_format_limit_range(config.output_min, config.output_max,
        out, sizeof(out));
    pid_console_format_limit_range(config.integral_min, config.integral_max,
        ilim, sizeof(ilim));
    pid_console_format_float_trimmed(config.deadband, db, sizeof(db));
    pid_console_format_float_trimmed(config.base_speed_pps, base, sizeof(base));

    (void) snprintf(line, sizeof(line),
        "line kp=%s ki=%s kd=%s out=%s ilim=%s db=%s base=%s\r\n",
        kp, ki, kd, out, ilim, db, base);
    pid_console_send_text(line);
    pid_console_update_line_lcd();
}

static void pid_console_show_all(void)
{
    pid_console_show_speed(ENCODER_LEFT);
    pid_console_show_speed(ENCODER_RIGHT);
    pid_console_show_position(ENCODER_LEFT);
    pid_console_show_position(ENCODER_RIGHT);
    pid_console_show_yaw();
    pid_console_show_line();
    pid_console_set_lcd_text("PID SHOW ALL");
}

static void pid_console_show_store(void)
{
    pid_tuning_store_status_t status = PidTuningStore_GetStatus();
    char line[PID_CONSOLE_LINE_MAX];
    char lcd_line[PID_CONSOLE_LINE_MAX];

    (void) snprintf(line, sizeof(line), "store %s\r\n",
        PidTuningStore_StatusText(status));
    pid_console_send_text(line);
    (void) snprintf(lcd_line, sizeof(lcd_line), "STORE %s",
        PidTuningStore_StatusText(status));
    pid_console_set_lcd_text(lcd_line);
}

static void pid_console_autosave_tunings(bool changed)
{
    (void) changed;
}

static bool pid_console_handle_speed_tuning(encoder_id_t id,
    const char *field, float value)
{
    encoder_speed_control_pid_t left;
    encoder_speed_control_pid_t right;

    EncoderSpeedControl_GetSpeedTunings(&left, &right);

    if (id == ENCODER_LEFT) {
        if (pid_console_token_eq(field, "kp")) {
            left.kp = value;
        } else if (pid_console_token_eq(field, "ki")) {
            left.ki = value;
        } else if (pid_console_token_eq(field, "kd")) {
            left.kd = value;
        }
        EncoderSpeedControl_SetSpeedTunings(id, left.kp, left.ki, left.kd);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        return true;
    }
    if (id == ENCODER_RIGHT) {
        if (pid_console_token_eq(field, "kp")) {
            right.kp = value;
        } else if (pid_console_token_eq(field, "ki")) {
            right.ki = value;
        } else if (pid_console_token_eq(field, "kd")) {
            right.kd = value;
        }
        EncoderSpeedControl_SetSpeedTunings(id, right.kp, right.ki, right.kd);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        return true;
    }

    return false;
}

static bool pid_console_handle_position_tuning(encoder_id_t id,
    const char *section, const char *field, float value)
{
    encoder_position_control_pid_t left;
    encoder_position_control_pid_t right;

    if (pid_console_token_eq(section, "pos")) {
    EncoderPositionControl_GetPositionTunings(&left, &right);
        if (id == ENCODER_LEFT) {
            if (pid_console_token_eq(field, "kp")) {
                left.kp = value;
            } else if (pid_console_token_eq(field, "ki")) {
                left.ki = value;
            } else if (pid_console_token_eq(field, "kd")) {
                left.kd = value;
            }
            EncoderPositionControl_SetPositionTunings(
                id, left.kp, left.ki, left.kd);
            pid_console_update_position_lcd(id, true);
            return true;
        }
        if (id == ENCODER_RIGHT) {
            if (pid_console_token_eq(field, "kp")) {
                right.kp = value;
            } else if (pid_console_token_eq(field, "ki")) {
                right.ki = value;
            } else if (pid_console_token_eq(field, "kd")) {
                right.kd = value;
            }
            EncoderPositionControl_SetPositionTunings(
                id, right.kp, right.ki, right.kd);
            pid_console_update_position_lcd(id, true);
            return true;
        }
        return false;
    }

    EncoderPositionControl_GetSpeedTunings(&left, &right);
    if (id == ENCODER_LEFT) {
        if (pid_console_token_eq(field, "kp")) {
            left.kp = value;
        } else if (pid_console_token_eq(field, "ki")) {
            left.ki = value;
        } else if (pid_console_token_eq(field, "kd")) {
            left.kd = value;
        }
        EncoderPositionControl_SetSpeedTunings(id, left.kp, left.ki, left.kd);
        pid_console_update_position_lcd(id, false);
        return true;
    }
    if (id == ENCODER_RIGHT) {
        if (pid_console_token_eq(field, "kp")) {
            right.kp = value;
        } else if (pid_console_token_eq(field, "ki")) {
            right.ki = value;
        } else if (pid_console_token_eq(field, "kd")) {
            right.kd = value;
        }
        EncoderPositionControl_SetSpeedTunings(
            id, right.kp, right.ki, right.kd);
        pid_console_update_position_lcd(id, false);
        return true;
    }

    return false;
}

static bool pid_console_handle_yaw_tuning(const char *field, float value)
{
    if (pid_console_token_eq(field, "kp")) {
        yaw_angle_control_pid_t pid;

        YawAngleControl_GetTunings(&pid);
        YawAngleControl_SetTunings(value, pid.ki, pid.kd);
        pid_console_update_yaw_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "ki")) {
        yaw_angle_control_pid_t pid;

        YawAngleControl_GetTunings(&pid);
        YawAngleControl_SetTunings(pid.kp, value, pid.kd);
        pid_console_update_yaw_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "kd")) {
        yaw_angle_control_pid_t pid;

        YawAngleControl_GetTunings(&pid);
        YawAngleControl_SetTunings(pid.kp, pid.ki, value);
        pid_console_update_yaw_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "out")) {
        YawAngleControl_SetOutputLimits(-value, value);
        pid_console_update_yaw_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "ilim")) {
        YawAngleControl_SetIntegralLimits(-value, value);
        pid_console_update_yaw_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "db")) {
        YawAngleControl_SetDeadband(value);
        pid_console_update_yaw_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "minturn")) {
        YawAngleControl_SetMinTurnSpeedPps(value);
        pid_console_update_yaw_lcd();
        return true;
    }

    return false;
}

static bool pid_console_handle_line_tuning(const char *field, float value)
{
    if (pid_console_token_eq(field, "kp")) {
        line_tracking_pid_t pid;

        LineTrackingControl_GetTunings(&pid);
        LineTrackingControl_SetTunings(value, pid.ki, pid.kd);
        pid_console_update_line_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "ki")) {
        line_tracking_pid_t pid;

        LineTrackingControl_GetTunings(&pid);
        LineTrackingControl_SetTunings(pid.kp, value, pid.kd);
        pid_console_update_line_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "kd")) {
        line_tracking_pid_t pid;

        LineTrackingControl_GetTunings(&pid);
        LineTrackingControl_SetTunings(pid.kp, pid.ki, value);
        pid_console_update_line_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "out")) {
        LineTrackingControl_SetOutputLimits(-value, value);
        pid_console_update_line_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "ilim")) {
        LineTrackingControl_SetIntegralLimits(-value, value);
        pid_console_update_line_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "db")) {
        LineTrackingControl_SetDeadband(value);
        pid_console_update_line_lcd();
        return true;
    }
    if (pid_console_token_eq(field, "base")) {
        LineTrackingControl_SetBaseSpeedPps(value);
        pid_console_update_line_lcd();
        return true;
    }

    return false;
}


static bool pid_console_handle_speed(char *const *tokens, uint8_t count,
    uint32_t now_ms)
{
    float value;
    float left_value;
    float right_value;
    encoder_id_t id;

    if (count < 3U) {
        return false;
    }

    if (pid_console_token_eq(tokens[2], "target")) {
        if ((count < 5U) ||
            !pid_console_parse_float(tokens[3], &left_value) ||
            !pid_console_parse_float(tokens[4], &right_value)) {
            return false;
        }
        EncoderSpeedControl_SetTargetPps(left_value, right_value);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "stop")) {
        EncoderSpeedControl_Stop();
        (void) now_ms;
        return true;
    }

    if (!pid_console_parse_encoder_id(tokens[2], &id)) {
        return false;
    }

    if ((count < 5U) || !pid_console_parse_float(tokens[4], &value)) {
        return false;
    }

    if (pid_console_token_eq(tokens[3], "kp") ||
        pid_console_token_eq(tokens[3], "ki") ||
        pid_console_token_eq(tokens[3], "kd")) {
        if (id == ENCODER_ID_COUNT) {
            bool changed = pid_console_handle_speed_tuning(
                ENCODER_LEFT, tokens[3], value) &&
                pid_console_handle_speed_tuning(
                    ENCODER_RIGHT, tokens[3], value);
            pid_console_autosave_tunings(changed);
            return changed;
        }
        {
            bool changed = pid_console_handle_speed_tuning(
                id, tokens[3], value);
            pid_console_autosave_tunings(changed);
            return changed;
        }
    }

    if (pid_console_token_eq(tokens[3], "out")) {
        if (id == ENCODER_ID_COUNT) {
            EncoderSpeedControl_SetSpeedOutputLimits(
                ENCODER_LEFT, -value, value);
            EncoderSpeedControl_SetSpeedOutputLimits(
                ENCODER_RIGHT, -value, value);
            pid_console_update_speed_lcd(ENCODER_ID_COUNT);
            pid_console_sync_position_speed();
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderSpeedControl_SetSpeedOutputLimits(id, -value, value);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[3], "ilim")) {
        if (id == ENCODER_ID_COUNT) {
            EncoderSpeedControl_SetSpeedIntegralLimits(
                ENCODER_LEFT, -value, value);
            EncoderSpeedControl_SetSpeedIntegralLimits(
                ENCODER_RIGHT, -value, value);
            pid_console_update_speed_lcd(ENCODER_ID_COUNT);
            pid_console_sync_position_speed();
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderSpeedControl_SetSpeedIntegralLimits(id, -value, value);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[3], "db")) {
        if (id == ENCODER_ID_COUNT) {
            EncoderSpeedControl_SetSpeedDeadband(ENCODER_LEFT, value);
            EncoderSpeedControl_SetSpeedDeadband(ENCODER_RIGHT, value);
            pid_console_update_speed_lcd(ENCODER_ID_COUNT);
            pid_console_sync_position_speed();
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderSpeedControl_SetSpeedDeadband(id, value);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[3], "ff")) {
        if (id == ENCODER_ID_COUNT) {
            EncoderSpeedControl_SetSpeedFeedforwardPwm(ENCODER_LEFT, value);
            EncoderSpeedControl_SetSpeedFeedforwardPwm(ENCODER_RIGHT, value);
            pid_console_update_speed_lcd(ENCODER_ID_COUNT);
            pid_console_sync_position_speed();
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderSpeedControl_SetSpeedFeedforwardPwm(id, value);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[3], "fref")) {
        if (id == ENCODER_ID_COUNT) {
            EncoderSpeedControl_SetSpeedFeedforwardReferencePps(
                ENCODER_LEFT, value);
            EncoderSpeedControl_SetSpeedFeedforwardReferencePps(
                ENCODER_RIGHT, value);
            pid_console_update_speed_lcd(ENCODER_ID_COUNT);
            pid_console_sync_position_speed();
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderSpeedControl_SetSpeedFeedforwardReferencePps(id, value);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[3], "min")) {
        float forward_pwm;
        float reverse_pwm;
        float reference_pps;

        if ((count < 7U) ||
            !pid_console_parse_float(tokens[4], &forward_pwm) ||
            !pid_console_parse_float(tokens[5], &reverse_pwm) ||
            !pid_console_parse_float(tokens[6], &reference_pps)) {
            return false;
        }

        if (id == ENCODER_ID_COUNT) {
            EncoderSpeedControl_SetSpeedMinDriveConfig(
                ENCODER_LEFT, forward_pwm, reverse_pwm, reference_pps);
            EncoderSpeedControl_SetSpeedMinDriveConfig(
                ENCODER_RIGHT, forward_pwm, reverse_pwm, reference_pps);
            pid_console_update_speed_lcd(ENCODER_ID_COUNT);
            pid_console_sync_position_speed();
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderSpeedControl_SetSpeedMinDriveConfig(
            id, forward_pwm, reverse_pwm, reference_pps);
        pid_console_update_speed_lcd(id);
        pid_console_sync_position_speed();
        pid_console_autosave_tunings(true);
        return true;
    }

    return false;
}

static bool pid_console_handle_position(char *const *tokens, uint8_t count,
    uint32_t now_ms)
{
    float value;
    float left_value;
    float right_value;
    encoder_id_t id;

    if (count < 3U) {
        return false;
    }

    if (pid_console_token_eq(tokens[2], "target")) {
        if ((count < 5U) ||
            !pid_console_parse_float(tokens[3], &left_value) ||
            !pid_console_parse_float(tokens[4], &right_value)) {
            return false;
        }
        EncoderPositionControl_SetTargetCount(left_value, right_value);
        (void) now_ms;
        return true;
    }

    if (pid_console_token_eq(tokens[2], "add")) {
        if ((count < 5U) ||
            !pid_console_parse_float(tokens[3], &left_value) ||
            !pid_console_parse_float(tokens[4], &right_value)) {
            return false;
        }
        EncoderPositionControl_AddTargetCount(left_value, right_value);
        (void) now_ms;
        return true;
    }

    if (pid_console_token_eq(tokens[2], "zero")) {
        EncoderPositionControl_ZeroPosition(now_ms);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "stop")) {
        EncoderPositionControl_Stop();
        return true;
    }

    if (!pid_console_parse_encoder_id(tokens[2], &id)) {
        return false;
    }

    if ((count < 6U) ||
        (!pid_console_token_eq(tokens[3], "pos") &&
         !pid_console_token_eq(tokens[3], "spd")) ||
        !pid_console_parse_float(tokens[5], &value)) {
        return false;
    }

    if (pid_console_token_eq(tokens[4], "kp") ||
        pid_console_token_eq(tokens[4], "ki") ||
        pid_console_token_eq(tokens[4], "kd")) {
        if (id == ENCODER_ID_COUNT) {
            bool changed = pid_console_handle_position_tuning(
                ENCODER_LEFT, tokens[3], tokens[4], value) &&
                pid_console_handle_position_tuning(
                    ENCODER_RIGHT, tokens[3], tokens[4], value);
            pid_console_autosave_tunings(changed);
            return changed;
        }
        {
            bool changed = pid_console_handle_position_tuning(
                id, tokens[3], tokens[4], value);
            pid_console_autosave_tunings(changed);
            return changed;
        }
    }

    if (pid_console_token_eq(tokens[4], "out")) {
        if (id == ENCODER_ID_COUNT) {
            if (pid_console_token_eq(tokens[3], "pos")) {
                EncoderPositionControl_SetPositionOutputLimits(
                    ENCODER_LEFT, -value, value);
                EncoderPositionControl_SetPositionOutputLimits(
                    ENCODER_RIGHT, -value, value);
                pid_console_update_position_lcd(ENCODER_ID_COUNT, true);
            } else {
                EncoderPositionControl_SetSpeedOutputLimits(
                    ENCODER_LEFT, -value, value);
                EncoderPositionControl_SetSpeedOutputLimits(
                    ENCODER_RIGHT, -value, value);
                pid_console_update_position_lcd(ENCODER_ID_COUNT, false);
            }
            pid_console_autosave_tunings(true);
            return true;
        }
        if (pid_console_token_eq(tokens[3], "pos")) {
            EncoderPositionControl_SetPositionOutputLimits(
                id, -value, value);
            pid_console_update_position_lcd(id, true);
        } else {
            EncoderPositionControl_SetSpeedOutputLimits(
                id, -value, value);
            pid_console_update_position_lcd(id, false);
        }
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[4], "ilim")) {
        if (id == ENCODER_ID_COUNT) {
            if (pid_console_token_eq(tokens[3], "pos")) {
                EncoderPositionControl_SetPositionIntegralLimits(
                    ENCODER_LEFT, -value, value);
                EncoderPositionControl_SetPositionIntegralLimits(
                    ENCODER_RIGHT, -value, value);
                pid_console_update_position_lcd(ENCODER_ID_COUNT, true);
            } else {
                EncoderPositionControl_SetSpeedIntegralLimits(
                    ENCODER_LEFT, -value, value);
                EncoderPositionControl_SetSpeedIntegralLimits(
                    ENCODER_RIGHT, -value, value);
                pid_console_update_position_lcd(ENCODER_ID_COUNT, false);
            }
            pid_console_autosave_tunings(true);
            return true;
        }
        if (pid_console_token_eq(tokens[3], "pos")) {
            EncoderPositionControl_SetPositionIntegralLimits(
                id, -value, value);
            pid_console_update_position_lcd(id, true);
        } else {
            EncoderPositionControl_SetSpeedIntegralLimits(id, -value, value);
            pid_console_update_position_lcd(id, false);
        }
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[4], "db")) {
        if (id == ENCODER_ID_COUNT) {
            if (pid_console_token_eq(tokens[3], "pos")) {
                EncoderPositionControl_SetPositionDeadband(
                    ENCODER_LEFT, value);
                EncoderPositionControl_SetPositionDeadband(
                    ENCODER_RIGHT, value);
                pid_console_update_position_lcd(ENCODER_ID_COUNT, true);
            } else {
                EncoderPositionControl_SetSpeedDeadband(ENCODER_LEFT, value);
                EncoderPositionControl_SetSpeedDeadband(ENCODER_RIGHT, value);
                pid_console_update_position_lcd(ENCODER_ID_COUNT, false);
            }
            pid_console_autosave_tunings(true);
            return true;
        }
        if (pid_console_token_eq(tokens[3], "pos")) {
            EncoderPositionControl_SetPositionDeadband(id, value);
            pid_console_update_position_lcd(id, true);
        } else {
            EncoderPositionControl_SetSpeedDeadband(id, value);
            pid_console_update_position_lcd(id, false);
        }
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[4], "ff")) {
        if (!pid_console_token_eq(tokens[3], "spd")) {
            return false;
        }
        if (id == ENCODER_ID_COUNT) {
            EncoderPositionControl_SetSpeedFeedforwardPwm(
                ENCODER_LEFT, value);
            EncoderPositionControl_SetSpeedFeedforwardPwm(
                ENCODER_RIGHT, value);
            pid_console_update_position_lcd(ENCODER_ID_COUNT, false);
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderPositionControl_SetSpeedFeedforwardPwm(id, value);
        pid_console_update_position_lcd(id, false);
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[4], "fref")) {
        if (!pid_console_token_eq(tokens[3], "spd")) {
            return false;
        }
        if (id == ENCODER_ID_COUNT) {
            EncoderPositionControl_SetSpeedFeedforwardReferencePps(
                ENCODER_LEFT, value);
            EncoderPositionControl_SetSpeedFeedforwardReferencePps(
                ENCODER_RIGHT, value);
            pid_console_update_position_lcd(ENCODER_ID_COUNT, false);
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderPositionControl_SetSpeedFeedforwardReferencePps(id, value);
        pid_console_update_position_lcd(id, false);
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[4], "min")) {
        float forward_pwm;
        float reverse_pwm;
        float reference_pps;

        if ((count < 8U) ||
            !pid_console_parse_float(tokens[5], &forward_pwm) ||
            !pid_console_parse_float(tokens[6], &reverse_pwm) ||
            !pid_console_parse_float(tokens[7], &reference_pps)) {
            return false;
        }

        if (id == ENCODER_ID_COUNT) {
            EncoderPositionControl_SetSpeedMinDriveConfig(
                ENCODER_LEFT, forward_pwm, reverse_pwm, reference_pps);
            EncoderPositionControl_SetSpeedMinDriveConfig(
                ENCODER_RIGHT, forward_pwm, reverse_pwm, reference_pps);
            pid_console_update_position_lcd(ENCODER_ID_COUNT, false);
            pid_console_autosave_tunings(true);
            return true;
        }
        EncoderPositionControl_SetSpeedMinDriveConfig(
            id, forward_pwm, reverse_pwm, reference_pps);
        pid_console_update_position_lcd(id, false);
        pid_console_autosave_tunings(true);
        return true;
    }

    return false;
}

static bool pid_console_handle_yaw(char *const *tokens, uint8_t count,
    uint32_t now_ms)
{
    float value;
    float left_forward_pwm;
    float left_reverse_pwm;
    float right_forward_pwm;
    float right_reverse_pwm;
    float reference_pps;

    (void) now_ms;

    if (count < 3U) {
        return false;
    }

    if (pid_console_token_eq(tokens[2], "target")) {
        if ((count < 4U) || !pid_console_parse_float(tokens[3], &value)) {
            return false;
        }
        YawAngleControl_SetTargetDeg(value);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "add")) {
        if ((count < 4U) || !pid_console_parse_float(tokens[3], &value)) {
            return false;
        }
        YawAngleControl_AddTargetDeg(value);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "hold")) {
        YawAngleControl_HoldCurrentYaw();
        return true;
    }

    if (pid_console_token_eq(tokens[2], "zero")) {
        YawAngleControl_ZeroYaw(now_ms);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "stop")) {
        YawAngleControl_Stop();
        return true;
    }

    if ((count < 4U) || !pid_console_parse_float(tokens[3], &value)) {
        return false;
    }

    if (pid_console_token_eq(tokens[2], "kp") ||
        pid_console_token_eq(tokens[2], "ki") ||
        pid_console_token_eq(tokens[2], "kd")) {
        bool changed = pid_console_handle_yaw_tuning(tokens[2], value);
        pid_console_autosave_tunings(changed);
        return changed;
    }

    if (pid_console_token_eq(tokens[2], "out")) {
        YawAngleControl_SetOutputLimits(-value, value);
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "ilim")) {
        YawAngleControl_SetIntegralLimits(-value, value);
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "db")) {
        YawAngleControl_SetDeadband(value);
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "minturn")) {
        YawAngleControl_SetMinTurnSpeedPps(value);
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "maxturn")) {
        YawAngleControl_SetMaxTurnSpeedPps(value);
        pid_console_update_yaw_lcd();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "mindrive")) {
        if ((count < 8U) ||
            !pid_console_parse_float(tokens[3], &left_forward_pwm) ||
            !pid_console_parse_float(tokens[4], &left_reverse_pwm) ||
            !pid_console_parse_float(tokens[5], &right_forward_pwm) ||
            !pid_console_parse_float(tokens[6], &right_reverse_pwm) ||
            !pid_console_parse_float(tokens[7], &reference_pps)) {
            return false;
        }
        YawAngleControl_SetDirectionalMinDrivePwm(
            left_forward_pwm, left_reverse_pwm,
            right_forward_pwm, right_reverse_pwm,
            reference_pps);
        pid_console_update_yaw_lcd();
        pid_console_autosave_tunings(true);
        return true;
    }

    return false;
}

static bool pid_console_handle_line(char *const *tokens, uint8_t count,
    uint32_t now_ms)
{
    float value;
    float left_forward_pwm;
    float left_reverse_pwm;
    float right_forward_pwm;
    float right_reverse_pwm;
    float reference_pps;
    bool enabled;

    if (count < 3U) {
        return false;
    }

    if (pid_console_token_eq(tokens[2], "start")) {
        LineTrackingControl_Start();
        return true;
    }

    if (pid_console_token_eq(tokens[2], "stop")) {
        LineTrackingControl_Stop();
        return true;
    }

    if (pid_console_token_eq(tokens[2], "toggle")) {
        LineTrackingControl_Toggle();
        return true;
    }

    if (pid_console_token_eq(tokens[2], "enable")) {
        if ((count < 4U) || !pid_console_parse_bool(tokens[3], &enabled)) {
            return false;
        }
        LineTrackingControl_SetEnabled(enabled);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "motor")) {
        if ((count < 4U) || !pid_console_parse_bool(tokens[3], &enabled)) {
            return false;
        }
        LineTrackingControl_SetMotorOutputEnabled(enabled);
        return true;
    }

    if ((count < 4U) || !pid_console_parse_float(tokens[3], &value)) {
        return false;
    }

    if (pid_console_token_eq(tokens[2], "kp") ||
        pid_console_token_eq(tokens[2], "ki") ||
        pid_console_token_eq(tokens[2], "kd") ||
        pid_console_token_eq(tokens[2], "out") ||
        pid_console_token_eq(tokens[2], "ilim") ||
        pid_console_token_eq(tokens[2], "db") ||
        pid_console_token_eq(tokens[2], "base")) {
        bool changed = pid_console_handle_line_tuning(tokens[2], value);
        pid_console_autosave_tunings(changed);
        return changed;
    }

    if (pid_console_token_eq(tokens[2], "gain")) {
        float turn_gain;

        if ((count < 5U) ||
            !pid_console_parse_float(tokens[3], &value) ||
            !pid_console_parse_float(tokens[4], &turn_gain)) {
            return false;
        }
        LineTrackingControl_SetRightGain(value, turn_gain);
        pid_console_update_line_lcd();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "driveout")) {
        float right_limit;

        if ((count < 5U) ||
            !pid_console_parse_float(tokens[3], &value) ||
            !pid_console_parse_float(tokens[4], &right_limit)) {
            return false;
        }
        LineTrackingControl_SetDriveOutputLimits(value, right_limit);
        pid_console_update_line_lcd();
        pid_console_autosave_tunings(true);
        return true;
    }

    if (pid_console_token_eq(tokens[2], "mindrive")) {
        if ((count < 8U) ||
            !pid_console_parse_float(tokens[3], &left_forward_pwm) ||
            !pid_console_parse_float(tokens[4], &left_reverse_pwm) ||
            !pid_console_parse_float(tokens[5], &right_forward_pwm) ||
            !pid_console_parse_float(tokens[6], &right_reverse_pwm) ||
            !pid_console_parse_float(tokens[7], &reference_pps)) {
            return false;
        }
        LineTrackingControl_SetDirectionalMinDrivePwm(
            left_forward_pwm, left_reverse_pwm,
            right_forward_pwm, right_reverse_pwm,
            reference_pps);
        pid_console_update_line_lcd();
        pid_console_autosave_tunings(true);
        return true;
    }

    (void) now_ms;
    return false;
}
