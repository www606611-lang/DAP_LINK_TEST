#include "line_tracking_app.h"

#include "key.h"
#include "lcd_status.h"
#include "uart0_dma.h"

#include <stddef.h>

#define LINE_TRACKING_APP_VOFA_MS        200U
#define LINE_TRACKING_APP_VOFA_LINE_SIZE 128U

static uint32_t g_line_tracking_app_last_vofa_ms;
static bool g_line_tracking_app_telemetry_enabled;
static bool g_line_tracking_app_last_enabled;

static void line_tracking_app_sync_status_text(void);
static void line_tracking_app_update_lcd_sensor(
    const line_tracking_state_t *state);
static void line_tracking_app_vofa_task(
    uint32_t now_ms, const line_tracking_state_t *state);
static void line_tracking_app_vofa_append_char(
    char *line, uint16_t *length, uint16_t max_length, char value);
static void line_tracking_app_vofa_append_int32(
    char *line, uint16_t *length, uint16_t max_length, int32_t value);
static int32_t line_tracking_app_round_float(float value);

void LineTrackingApp_Init(uint32_t now_ms)
{
    LineTrackingControl_Init(now_ms);
    LineTrackingControl_Stop();
    g_line_tracking_app_last_vofa_ms = now_ms;
    g_line_tracking_app_last_enabled = false;
}

void LineTrackingApp_Task(uint32_t now_ms)
{
    line_tracking_state_t state;

    if (Key_GetPressEvent(KEY_ID_B21)) {
        LineTrackingApp_Toggle();
    }

    if (Key_GetPressEvent(KEY_ID_DOWN)) {
        LineTrackingApp_Stop();
    }

    LineTrackingControl_Task(now_ms);
    LineTrackingControl_GetState(&state);
    if (state.enabled != g_line_tracking_app_last_enabled) {
        line_tracking_app_sync_status_text();
    }
    line_tracking_app_update_lcd_sensor(&state);

    if (g_line_tracking_app_telemetry_enabled) {
        line_tracking_app_vofa_task(now_ms, &state);
    }
}

void LineTrackingApp_Start(void)
{
    LineTrackingControl_Start();
    line_tracking_app_sync_status_text();
}

void LineTrackingApp_Stop(void)
{
    LineTrackingControl_Stop();
    line_tracking_app_sync_status_text();
}

void LineTrackingApp_Toggle(void)
{
    LineTrackingControl_Toggle();
    line_tracking_app_sync_status_text();
}

void LineTrackingApp_SetEnabled(bool enabled)
{
    LineTrackingControl_SetEnabled(enabled);
    line_tracking_app_sync_status_text();
}

bool LineTrackingApp_IsEnabled(void)
{
    return LineTrackingControl_IsEnabled();
}

void LineTrackingApp_SetTelemetryEnabled(bool enabled)
{
    g_line_tracking_app_telemetry_enabled = enabled;
}

void LineTrackingApp_GetState(line_tracking_state_t *state)
{
    LineTrackingControl_GetState(state);
}

static void line_tracking_app_sync_status_text(void)
{
    g_line_tracking_app_last_enabled = LineTrackingControl_IsEnabled();
    if (g_line_tracking_app_last_enabled) {
        lcd_status_screen_set_pid_text("LINE RUN B21");
    } else {
        lcd_status_screen_set_pid_text("LINE READY B21");
    }
}

static void line_tracking_app_update_lcd_sensor(
    const line_tracking_state_t *state)
{
    if (state == NULL) {
        return;
    }

    lcd_status_screen_set_line_sensor(state->raw, state->active_mask,
        state->active_count, state->line_error, state->enabled ? 1U : 0U,
        state->sensor_ok ? 1U : 0U, state->sensor_error);
}

static void line_tracking_app_vofa_task(
    uint32_t now_ms, const line_tracking_state_t *state)
{
    char line[LINE_TRACKING_APP_VOFA_LINE_SIZE];
    uint16_t length = 0U;

    if (state == NULL) {
        return;
    }
    if ((uint32_t) (now_ms - g_line_tracking_app_last_vofa_ms) <
        LINE_TRACKING_APP_VOFA_MS) {
        return;
    }

    if (!state->enabled && !state->line_seen &&
        (state->left_target_pps == 0.0f) &&
        (state->right_target_pps == 0.0f) &&
        (state->left_actual_pps == 0) &&
        (state->right_actual_pps == 0) &&
        (state->turn_correction_pps == 0.0f)) {
        g_line_tracking_app_last_vofa_ms = now_ms;
        return;
    }

    line_tracking_app_vofa_append_char(line, &length, sizeof(line), 'd');
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), ':');
    line_tracking_app_vofa_append_int32(line, &length, sizeof(line),
        line_tracking_app_round_float(state->left_target_pps));
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), ',');
    line_tracking_app_vofa_append_int32(line, &length, sizeof(line),
        line_tracking_app_round_float(state->right_target_pps));
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), ',');
    line_tracking_app_vofa_append_int32(line, &length, sizeof(line),
        state->left_actual_pps);
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), ',');
    line_tracking_app_vofa_append_int32(line, &length, sizeof(line),
        state->right_actual_pps);
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), ',');
    line_tracking_app_vofa_append_int32(line, &length, sizeof(line),
        state->line_error);
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), ',');
    line_tracking_app_vofa_append_int32(line, &length, sizeof(line),
        line_tracking_app_round_float(state->turn_correction_pps));
    line_tracking_app_vofa_append_char(line, &length, sizeof(line), '\n');

    if (length < sizeof(line)) {
        if (uart0_dma_send((const uint8_t *) line, length) == UART0_DMA_OK) {
            g_line_tracking_app_last_vofa_ms = now_ms;
        }
    }
}

static void line_tracking_app_vofa_append_char(
    char *line, uint16_t *length, uint16_t max_length, char value)
{
    if ((line == NULL) || (length == NULL) || (*length >= max_length)) {
        return;
    }

    line[*length] = value;
    (*length)++;
}

static void line_tracking_app_vofa_append_int32(
    char *line, uint16_t *length, uint16_t max_length, int32_t value)
{
    char digits[11];
    uint8_t digit_count = 0U;
    uint32_t magnitude;

    if ((line == NULL) || (length == NULL)) {
        return;
    }

    if (value < 0) {
        line_tracking_app_vofa_append_char(line, length, max_length, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    do {
        digits[digit_count++] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude > 0U) && (digit_count < sizeof(digits)));

    while (digit_count > 0U) {
        line_tracking_app_vofa_append_char(line, length, max_length,
            digits[--digit_count]);
    }
}

static int32_t line_tracking_app_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }

    return (int32_t) (value - 0.5f);
}
