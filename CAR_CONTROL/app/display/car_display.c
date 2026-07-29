#include "car_display.h"

#include "car_app.h"
#include "control_supervisor.h"
#include "debug_snapshot.h"
#include "h_mission.h"
#include "jdy31_config.h"
#include "line_follow_mission.h"
#include "line_sensor.h"
#include "st7789.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CAR_DISPLAY_ROW_CHARS 40U
#define CAR_DISPLAY_ROW_COUNT 8U
#define CAR_DISPLAY_ROW_INVALID CAR_DISPLAY_ROW_COUNT

static char g_display_row_cache[CAR_DISPLAY_ROW_COUNT]
    [CAR_DISPLAY_ROW_CHARS + 1U];
static uint16_t g_display_row_color[CAR_DISPLAY_ROW_COUNT];
static uint16_t g_display_row_background[CAR_DISPLAY_ROW_COUNT];
static bool g_display_row_valid[CAR_DISPLAY_ROW_COUNT];

static void car_display_format_signed_tenths(
    char *buffer, size_t buffer_size, int32_t value_milli);
static void car_display_show_row(uint16_t y, uint16_t color,
    uint16_t background, const char *format, ...);
static void car_display_reset_row_cache(void);
static uint8_t car_display_row_index(uint16_t y);
static const char *car_display_app_state_text(uint32_t state);
static const char *car_display_workflow_text(uint32_t workflow);
static const char *car_display_line_mission_text(uint32_t state);
static const char *car_display_control_mode_text(uint32_t mode);
static const char *car_display_block_reason_text(uint32_t reason);
static const char *car_display_key_text(
    const car_debug_display_snapshot_t *debug);
static int32_t car_display_clamp_i32(
    int32_t value, int32_t minimum, int32_t maximum);
static void car_display_update_h(const car_debug_display_snapshot_t *debug,
    uint32_t now_ms, car_display_phase_t phase, const char *line_bits,
    const char *line_state_text, uint16_t line_color,
    const char *key_text);
static const char *car_display_h_state_text(uint32_t state);
static const char *car_display_h_phase_text(uint32_t phase);
static const char *car_display_h_fault_text(uint32_t fault);

void CarDisplay_Init(void)
{
    car_display_reset_row_cache();
    ST7789_Fill(ST7789_COLOR_BLACK);
    if (JDY31_ConfigIsExclusive()) {
        ST7789_FillRect(0U, 0U, ST7789_WIDTH, 26U,
            ST7789_COLOR_BLUE);
        ST7789_ShowString(8U, 5U, "JDY-31 CONFIG", ST7789_8X16,
            ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
        return;
    }

    ST7789_FillRect(0U, 0U, ST7789_WIDTH, 24U, ST7789_COLOR_BLUE);
    ST7789_DrawLine(0U, 24U, (uint16_t) (ST7789_WIDTH - 1U), 24U,
        ST7789_RGB565(48U, 52U, 60U));
    car_display_show_row(5U, ST7789_COLOR_WHITE, ST7789_COLOR_BLUE,
        "CAR DASHBOARD");
}

void CarDisplay_Update(uint32_t now_ms, car_display_phase_t phase)
{
    car_debug_display_snapshot_t debug;
    char yaw_text[16];
    char yaw_error_text[16];
    char yaw_rate_text[16];
    char target_text[16];
    char heading_error_text[16];
    char line_bits[9];
    const char *line_state_text;
    const char *key_text;
    uint16_t line_color;
    uint16_t health_color;
    uint16_t key_color;
    uint16_t footer_color;
    uint8_t line_index;
    uint32_t uptime_s;

    if (!CarDebugSnapshot_GetDisplay(&debug)) {
        return;
    }

    if (JDY31_ConfigIsExclusive()) {
        jdy31_config_snapshot_t jdy31;

        if (JDY31_ConfigGetSnapshot(&jdy31)) {
            uint16_t config_state_color =
                (jdy31.state == JDY31_CONFIG_SUCCESS) ?
                    ST7789_COLOR_GREEN :
                    ((jdy31.state == JDY31_CONFIG_FAILED) ?
                        ST7789_COLOR_RED : ST7789_COLOR_YELLOW);

            ST7789_PrintfFast(24U, 48U, ST7789_8X16,
                config_state_color, ST7789_COLOR_BLACK,
                "STATE: %-14s", JDY31_ConfigGetStateText());
            ST7789_PrintfFast(24U, 82U, ST7789_8X16,
                ST7789_COLOR_CYAN, ST7789_COLOR_BLACK,
                "UART : %6lu", (unsigned long) jdy31.uart_baud);
            ST7789_PrintfFast(24U, 116U, ST7789_8X16,
                ST7789_COLOR_WHITE, ST7789_COLOR_BLACK,
                "CODE : %6ld", (long) jdy31.reported_baud_code);
            ST7789_PrintfFast(24U, 150U, ST7789_8X16,
                ST7789_COLOR_WHITE, ST7789_COLOR_BLACK,
                "RESP : %-20s", jdy31.last_response);
        }
        return;
    }

    car_display_format_signed_tenths(
        yaw_text, sizeof(yaw_text), debug.imu_yaw_mdeg);
    car_display_format_signed_tenths(
        yaw_error_text, sizeof(yaw_error_text), debug.yaw_error_mdeg);
    car_display_format_signed_tenths(
        yaw_rate_text, sizeof(yaw_rate_text), debug.imu_yaw_rate_mdps);
    car_display_format_signed_tenths(
        target_text, sizeof(target_text), debug.yaw_target_mdeg);
    car_display_format_signed_tenths(
        heading_error_text, sizeof(heading_error_text),
        debug.motion_heading_error_mdeg);

    for (line_index = 0U; line_index < 8U; line_index++) {
        uint8_t bit = (uint8_t) (0x80U >> line_index);

        line_bits[line_index] =
            ((debug.line_sensor_active_mask & bit) != 0U) ? '1' : '0';
    }
    line_bits[8] = '\0';

    if (debug.line_sensor_ready) {
        line_state_text = debug.line_sensor_seen ? "SEEN" : "MISS";
        line_color = debug.line_sensor_seen ?
            ST7789_COLOR_GREEN : ST7789_COLOR_YELLOW;
    } else if ((debug.line_sensor_state ==
            (uint32_t) LINE_SENSOR_STATE_BOOT_WAIT) ||
        (debug.line_sensor_state ==
            (uint32_t) LINE_SENSOR_STATE_CALIBRATION_ON) ||
        (debug.line_sensor_state ==
            (uint32_t) LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT)) {
        line_state_text = "CAL";
        line_color = ST7789_COLOR_YELLOW;
    } else {
        line_state_text = "ERR";
        line_color = ST7789_COLOR_RED;
    }

    key_text = car_display_key_text(&debug);
    key_color = (debug.pb21_pressed || debug.pb4_pressed ||
        debug.pb5_pressed) ? ST7789_COLOR_GREEN : ST7789_COLOR_WHITE;
    if (debug.line_mission_state ==
        (uint32_t) LINE_FOLLOW_MISSION_RUNNING) {
        footer_color = ST7789_COLOR_GREEN;
    } else if (debug.line_mission_state ==
        (uint32_t) LINE_FOLLOW_MISSION_FAULT) {
        footer_color = ST7789_COLOR_RED;
    } else {
        footer_color = key_color;
    }
    health_color = (debug.imu_ready && debug.imu_attitude_valid &&
        debug.line_sensor_ready && debug.radio_online) ?
        ST7789_COLOR_GREEN :
        ST7789_COLOR_YELLOW;
    uptime_s = now_ms / 1000U;
    if (uptime_s > 9999U) {
        uptime_s = 9999U;
    }

    car_display_update_h(&debug, now_ms, phase, line_bits,
        line_state_text, line_color, key_text);
    return;

    switch (phase) {
        case CAR_DISPLAY_PHASE_SPEED:
            car_display_show_row(28U, ST7789_COLOR_CYAN,
                ST7789_COLOR_BLACK,
                "SPD L%+5ld R%+5ld T%+5ld/%+5ld",
                (long) car_display_clamp_i32(
                    debug.encoder_0_speed_pps, -9999, 9999),
                (long) car_display_clamp_i32(
                    debug.encoder_1_speed_pps, -9999, 9999),
                (long) car_display_clamp_i32(
                    debug.speed_left_target_pps, -9999, 9999),
                (long) car_display_clamp_i32(
                    debug.speed_right_target_pps, -9999, 9999));
            break;

        case CAR_DISPLAY_PHASE_ENCODER:
            car_display_show_row(48U, ST7789_COLOR_WHITE,
                ST7789_COLOR_BLACK,
                "ENC L%+7ld R%+7ld D%+7ld",
                (long) car_display_clamp_i32(
                    debug.encoder_0_count, -999999, 999999),
                (long) car_display_clamp_i32(
                    debug.encoder_1_count, -999999, 999999),
                (long) car_display_clamp_i32(
                    debug.encoder_count_difference, -999999, 999999));
            break;

        case CAR_DISPLAY_PHASE_ATTITUDE:
            car_display_show_row(68U,
                (debug.imu_ready && debug.imu_attitude_valid) ?
                    ST7789_COLOR_YELLOW : ST7789_COLOR_RED,
                ST7789_COLOR_BLACK,
                "YAW %s E%s R%s", yaw_text, yaw_error_text,
                yaw_rate_text);
            break;

        case CAR_DISPLAY_PHASE_LINE:
            car_display_show_row(88U, line_color,
                ST7789_COLOR_BLACK,
                "LINE %s E%+3ld N%lu %-4s", line_bits,
                (long) car_display_clamp_i32(
                    debug.line_sensor_error, -99, 99),
                (unsigned long) debug.line_sensor_active_count,
                line_state_text);
            break;

        case CAR_DISPLAY_PHASE_CONTROL:
            switch (debug.active_workflow) {
                case CAR_APP_WORKFLOW_POSITION_TEST:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "POS E%+6ld/%+6ld SY%+5ld",
                        (long) car_display_clamp_i32(
                            debug.position_left_error_count,
                            -99999, 99999),
                        (long) car_display_clamp_i32(
                            debug.position_right_error_count,
                            -99999, 99999),
                        (long) car_display_clamp_i32(
                            debug.position_sync_correction_pps,
                            -9999, 9999));
                    break;

                case CAR_APP_WORKFLOW_LINE_TEST:
                case CAR_APP_WORKFLOW_LINE_MISSION:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "TURN C%+5ld TG%+5ld/%+5ld",
                        (long) car_display_clamp_i32(
                            debug.line_tracking_correction_pps,
                            -9999, 9999),
                        (long) car_display_clamp_i32(
                            debug.line_tracking_left_target_pps,
                            -9999, 9999),
                        (long) car_display_clamp_i32(
                            debug.line_tracking_right_target_pps,
                            -9999, 9999));
                    break;

                case CAR_APP_WORKFLOW_HEADING_TEST:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "HEAD E%s C%+5ld V%+5ld",
                        yaw_error_text,
                        (long) car_display_clamp_i32(
                            debug.heading_correction_pps,
                            -9999, 9999),
                        (long) car_display_clamp_i32(
                            debug.heading_base_target_pps,
                            -9999, 9999));
                    break;

                case CAR_APP_WORKFLOW_MOTION:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "MOT E%+7ld H%s V%+5ld",
                        (long) car_display_clamp_i32(
                            debug.motion_error_count,
                            -999999, 999999),
                        heading_error_text,
                        (long) car_display_clamp_i32(
                            debug.motion_base_target_pps,
                            -9999, 9999));
                    break;

                case CAR_APP_WORKFLOW_YAW_TEST:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "CTRL T%s E%s V%+5ld", target_text,
                        yaw_error_text,
                        (long) car_display_clamp_i32(
                            debug.yaw_turn_target_pps,
                            -9999, 9999));
                    break;

                case CAR_APP_WORKFLOW_SPEED_TEST:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "PWM L%+4ld R%+4ld MODE %-6s",
                        (long) car_display_clamp_i32(
                            debug.speed_left_output_permille,
                            -999, 999),
                        (long) car_display_clamp_i32(
                            debug.speed_right_output_permille,
                            -999, 999),
                        car_display_control_mode_text(
                            debug.control_mode));
                    break;

                default:
                    car_display_show_row(108U, ST7789_COLOR_CYAN,
                        ST7789_COLOR_BLACK,
                        "PWM L%+4ld R%+4ld MODE %-6s",
                        (long) car_display_clamp_i32(
                            debug.speed_left_output_permille,
                            -999, 999),
                        (long) car_display_clamp_i32(
                            debug.speed_right_output_permille,
                            -999, 999),
                        car_display_control_mode_text(
                            debug.control_mode));
                    break;
            }

            break;

        case CAR_DISPLAY_PHASE_HEALTH:
            car_display_show_row(128U, health_color,
                ST7789_COLOR_BLACK,
                "IMU %-3s LINE %-3s ESP %-3s K %-3s A%4lu",
                (debug.imu_ready && debug.imu_attitude_valid) ?
                    "OK" : "BAD",
                debug.line_sensor_ready ? "OK" : "BAD",
                debug.radio_esp32_online ? "ON" : "OFF",
                debug.radio_k230_online ? "ON" : "OFF",
                (unsigned long) debug.radio_frame_age_ms);
            break;

        case CAR_DISPLAY_PHASE_FOOTER:
            car_display_show_row(150U, footer_color,
                ST7789_COLOR_BLACK,
                "KEY %-4s TRK %-5s %-7s P%+4ld/%+4ld",
                key_text,
                car_display_line_mission_text(
                    debug.line_mission_state),
                car_display_block_reason_text(
                    debug.control_block_reason),
                (long) car_display_clamp_i32(
                    debug.speed_left_output_permille, -999, 999),
                (long) car_display_clamp_i32(
                    debug.speed_right_output_permille, -999, 999));
            break;

        case CAR_DISPLAY_PHASE_HEADER:
        default:
            car_display_show_row(5U, ST7789_COLOR_WHITE,
                ST7789_COLOR_BLUE,
                "CAR %-7s %-8s U%4lus",
                car_display_app_state_text(debug.app_state),
                car_display_workflow_text(debug.active_workflow),
                (unsigned long) uptime_s);
            ST7789_ShowAsciiStringFast(264U, 5U,
                debug.motor_high_impedance ? "HIGH-Z" : "ARMED ",
                ST7789_8X16,
                debug.motor_high_impedance ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_RED,
                ST7789_COLOR_BLUE);
            break;
    }
}

static void car_display_update_h(const car_debug_display_snapshot_t *debug,
    uint32_t now_ms, car_display_phase_t phase, const char *line_bits,
    const char *line_state_text, uint16_t line_color,
    const char *key_text)
{
    uint16_t state_color;
    uint32_t elapsed_cs = debug->h_elapsed_ms / 10U;
    uint32_t b_cs = debug->h_b_passage_ms / 10U;
    uint32_t finish_cs = debug->h_finish_ms / 10U;

    (void) now_ms;
    if (debug->h_mission_state == (uint32_t) H_MISSION_STATE_FAULT) {
        state_color = ST7789_COLOR_RED;
    } else if ((debug->h_mission_state ==
            (uint32_t) H_MISSION_STATE_RUNNING) ||
        (debug->h_mission_state ==
            (uint32_t) H_MISSION_STATE_PRECISION_STOP) ||
        (debug->h_mission_state ==
            (uint32_t) H_MISSION_STATE_FINISHED)) {
        state_color = ST7789_COLOR_GREEN;
    } else {
        state_color = ST7789_COLOR_YELLOW;
    }

    switch (phase) {
        case CAR_DISPLAY_PHASE_SPEED:
            car_display_show_row(28U, state_color,
                ST7789_COLOR_BLACK,
                "TASK H%lu SEQ%05lu TGT%+6ld x0.1mm",
                (unsigned long) debug->h_mission_profile,
                (unsigned long) debug->h_mission_sequence,
                (long) car_display_clamp_i32(
                    debug->h_target_x_0p1mm, -9999, 9999));
            break;

        case CAR_DISPLAY_PHASE_ENCODER:
            car_display_show_row(48U, ST7789_COLOR_WHITE,
                ST7789_COLOR_BLACK,
                "TIME %3lu.%02lus B%3lu.%02lus F%3lu.%02lus",
                (unsigned long) (elapsed_cs / 100U),
                (unsigned long) (elapsed_cs % 100U),
                (unsigned long) (b_cs / 100U),
                (unsigned long) (b_cs % 100U),
                (unsigned long) (finish_cs / 100U),
                (unsigned long) (finish_cs % 100U));
            break;

        case CAR_DISPLAY_PHASE_ATTITUDE:
            car_display_show_row(68U,
                (debug->imu_ready && debug->imu_attitude_valid) ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_YELLOW,
                ST7789_COLOR_BLACK,
                "YAW %+7ld RATE %+8ld",
                (long) car_display_clamp_i32(
                    debug->imu_yaw_mdeg, -999999, 999999),
                (long) car_display_clamp_i32(
                    debug->imu_yaw_rate_mdps, -9999999, 9999999));
            break;

        case CAR_DISPLAY_PHASE_LINE:
            car_display_show_row(88U, line_color,
                ST7789_COLOR_BLACK,
                "LINE %s E%+3ld N%lu %-4s", line_bits,
                (long) car_display_clamp_i32(
                    debug->line_sensor_error, -99, 99),
                (unsigned long) debug->line_sensor_active_count,
                line_state_text);
            break;

        case CAR_DISPLAY_PHASE_CONTROL:
            car_display_show_row(108U,
                debug->h_route_calibrated ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_YELLOW,
                ST7789_COLOR_BLACK,
                "ROUTE %-3s D%+7ld A%u L%u B%u F%u",
                debug->h_route_calibrated ? "OK" : "CAL",
                (long) car_display_clamp_i32(
                    debug->h_route_progress_count, -999999, 999999),
                debug->h_route_initial_a_seen ? 1U : 0U,
                debug->h_route_left_a ? 1U : 0U,
                debug->h_route_b_passed ? 1U : 0U,
                debug->h_route_finish_a ? 1U : 0U);
            break;

        case CAR_DISPLAY_PHASE_HEALTH:
            car_display_show_row(128U,
                (debug->imu_ready && debug->imu_attitude_valid &&
                 debug->line_sensor_ready) ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_YELLOW,
                ST7789_COLOR_BLACK,
                "IMU %-3s LINE %-3s ROUTE %-3s MODE %-6s",
                (debug->imu_ready && debug->imu_attitude_valid) ?
                    "OK" : "BAD",
                debug->line_sensor_ready ? "OK" : "BAD",
                debug->h_route_calibrated ? "OK" : "CAL",
                car_display_control_mode_text(debug->control_mode));
            break;

        case CAR_DISPLAY_PHASE_FOOTER:
            car_display_show_row(150U, state_color,
                ST7789_COLOR_BLACK,
                "KEY %-4s PH%-7s FAULT %-7s",
                key_text,
                car_display_h_phase_text(debug->h_mission_phase),
                car_display_h_fault_text(debug->h_mission_fault));
            break;

        case CAR_DISPLAY_PHASE_HEADER:
        default:
            car_display_show_row(5U, ST7789_COLOR_WHITE,
                ST7789_COLOR_BLUE,
                "H%lu %-6s %-7s",
                (unsigned long) debug->h_mission_profile,
                car_display_h_state_text(debug->h_mission_state),
                car_display_h_phase_text(debug->h_mission_phase));
            ST7789_ShowAsciiStringFast(264U, 5U,
                debug->motor_high_impedance ? "HIGH-Z" : "ARMED ",
                ST7789_8X16,
                debug->motor_high_impedance ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_RED,
                ST7789_COLOR_BLUE);
            break;
    }
}

static const char *car_display_h_state_text(uint32_t state)
{
    switch ((h_mission_state_t) state) {
        case H_MISSION_STATE_LOCKED:
            return "LOCK";
        case H_MISSION_STATE_READY:
            return "READY";
        case H_MISSION_STATE_ARMED:
            return "ARMED";
        case H_MISSION_STATE_RUNNING:
            return "RUN";
        case H_MISSION_STATE_PRECISION_STOP:
            return "PSTOP";
        case H_MISSION_STATE_FINISHED:
            return "DONE";
        case H_MISSION_STATE_FAULT:
            return "FAULT";
        default:
            return "UNK";
    }
}

static const char *car_display_h_phase_text(uint32_t phase)
{
    switch ((h_mission_phase_t) phase) {
        case H_MISSION_PHASE_IDLE:
            return "IDLE";
        case H_MISSION_PHASE_BALL_ONLY:
            return "BALL";
        case H_MISSION_PHASE_LEAVE_START_A:
            return "LEAVE-A";
        case H_MISSION_PHASE_RUN_TO_B:
            return "TO-B";
        case H_MISSION_PHASE_RUN_TO_A:
            return "TO-A";
        case H_MISSION_PHASE_PRECISION_STOP:
            return "PSTOP";
        default:
            return "UNK";
    }
}

static const char *car_display_h_fault_text(uint32_t fault)
{
    switch ((h_mission_fault_t) fault) {
        case H_MISSION_FAULT_NONE:
            return "NONE";
        case H_MISSION_FAULT_OPERATOR_STOP:
            return "STOP";
        case H_MISSION_FAULT_CHASSIS:
            return "CHASSIS";
        case H_MISSION_FAULT_LINE:
            return "LINE";
        case H_MISSION_FAULT_K230_LINK:
            return "K230";
        case H_MISSION_FAULT_BALL_CONTROLLER:
            return "BALL";
        default:
            return "UNK";
    }
}

static void car_display_show_row(uint16_t y, uint16_t color,
    uint16_t background, const char *format, ...)
{
    char content[CAR_DISPLAY_ROW_CHARS + 1U];
    char row[CAR_DISPLAY_ROW_CHARS + 1U];
    char changed[CAR_DISPLAY_ROW_CHARS + 1U];
    uint8_t row_index;
    uint8_t start;
    uint8_t end;
    va_list args;

    if (format == NULL) {
        return;
    }
    va_start(args, format);
    (void) vsnprintf(content, sizeof(content), format, args);
    va_end(args);
    (void) snprintf(row, sizeof(row), "%-40.40s", content);
    row_index = car_display_row_index(y);
    if ((row_index == CAR_DISPLAY_ROW_INVALID) ||
        !g_display_row_valid[row_index] ||
        (g_display_row_color[row_index] != color) ||
        (g_display_row_background[row_index] != background)) {
        ST7789_ShowAsciiStringFast(
            0U, y, row, ST7789_8X16, color, background);
    } else {
        start = 0U;
        while (start < CAR_DISPLAY_ROW_CHARS) {
            while ((start < CAR_DISPLAY_ROW_CHARS) &&
                (row[start] == g_display_row_cache[row_index][start])) {
                start++;
            }
            if (start >= CAR_DISPLAY_ROW_CHARS) {
                break;
            }
            end = (uint8_t) (start + 1U);
            while ((end < CAR_DISPLAY_ROW_CHARS) &&
                (row[end] != g_display_row_cache[row_index][end])) {
                end++;
            }
            (void) memcpy(changed, &row[start], end - start);
            changed[end - start] = '\0';
            ST7789_ShowAsciiStringFast((uint16_t) start * 8U, y,
                changed, ST7789_8X16, color, background);
            start = end;
        }
    }

    if (row_index != CAR_DISPLAY_ROW_INVALID) {
        (void) memcpy(g_display_row_cache[row_index], row, sizeof(row));
        g_display_row_color[row_index] = color;
        g_display_row_background[row_index] = background;
        g_display_row_valid[row_index] = true;
    }
}

static void car_display_reset_row_cache(void)
{
    (void) memset(g_display_row_cache, 0, sizeof(g_display_row_cache));
    (void) memset(g_display_row_valid, 0, sizeof(g_display_row_valid));
}

static uint8_t car_display_row_index(uint16_t y)
{
    switch (y) {
        case 5U:
            return 0U;
        case 28U:
            return 1U;
        case 48U:
            return 2U;
        case 68U:
            return 3U;
        case 88U:
            return 4U;
        case 108U:
            return 5U;
        case 128U:
            return 6U;
        case 150U:
            return 7U;
        default:
            return CAR_DISPLAY_ROW_INVALID;
    }
}

static const char *car_display_app_state_text(uint32_t state)
{
    switch ((car_app_state_t) state) {
        case CAR_APP_STATE_LOCKED:
            return "LOCKED";
        case CAR_APP_STATE_READY:
            return "READY";
        case CAR_APP_STATE_SERVICE:
            return "SERVICE";
        case CAR_APP_STATE_MOTION_ACTIVE:
            return "ACTIVE";
        default:
            return "UNKNOWN";
    }
}

static const char *car_display_workflow_text(uint32_t workflow)
{
    switch ((car_app_workflow_t) workflow) {
        case CAR_APP_WORKFLOW_SPEED_TEST:
            return "SPEED";
        case CAR_APP_WORKFLOW_POSITION_TEST:
            return "POSITION";
        case CAR_APP_WORKFLOW_HEADING_TEST:
            return "HEADING";
        case CAR_APP_WORKFLOW_LINE_TEST:
            return "LINE-TST";
        case CAR_APP_WORKFLOW_H_MISSION:
            return "H-MISS";
        case CAR_APP_WORKFLOW_LINE_MISSION:
            return "LINE";
        case CAR_APP_WORKFLOW_MOTION:
            return "MOTION";
        case CAR_APP_WORKFLOW_YAW_TEST:
            return "YAW";
        default:
            return "IDLE";
    }
}

static const char *car_display_line_mission_text(uint32_t state)
{
    switch ((line_follow_mission_state_t) state) {
        case LINE_FOLLOW_MISSION_LOCKED:
            return "LOCK";
        case LINE_FOLLOW_MISSION_READY:
            return "READY";
        case LINE_FOLLOW_MISSION_RUNNING:
            return "RUN";
        case LINE_FOLLOW_MISSION_STOPPED:
            return "STOP";
        case LINE_FOLLOW_MISSION_FAULT:
            return "FAULT";
        default:
            return "UNK";
    }
}

static const char *car_display_control_mode_text(uint32_t mode)
{
    switch ((car_control_mode_t) mode) {
        case CAR_CONTROL_MODE_SAFE_IDLE:
            return "SAFE";
        case CAR_CONTROL_MODE_OPEN_LOOP:
            return "OPEN";
        case CAR_CONTROL_MODE_SPEED:
            return "SPEED";
        case CAR_CONTROL_MODE_POSITION:
            return "POS";
        case CAR_CONTROL_MODE_YAW:
            return "YAW";
        case CAR_CONTROL_MODE_HEADING:
            return "HEAD";
        case CAR_CONTROL_MODE_LINE_TRACKING:
            return "LINE";
        case CAR_CONTROL_MODE_MOTION:
            return "MOTION";
        default:
            return "UNKNOWN";
    }
}

static const char *car_display_block_reason_text(uint32_t reason)
{
    switch ((car_control_block_reason_t) reason) {
        case CAR_CONTROL_BLOCK_NONE:
            return "OK";
        case CAR_CONTROL_BLOCK_STARTUP:
            return "START";
        case CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED:
            return "HW";
        case CAR_CONTROL_BLOCK_SUSPICIOUS_RESET:
            return "RESET";
        case CAR_CONTROL_BLOCK_EMERGENCY_STOP:
            return "ESTOP";
        case CAR_CONTROL_BLOCK_TEST_COMPLETE:
            return "DONE";
        case CAR_CONTROL_BLOCK_OPERATOR_STOP:
            return "STOP";
        case CAR_CONTROL_BLOCK_COMMAND_TIMEOUT:
            return "TIMEOUT";
        default:
            return "UNKNOWN";
    }
}

static const char *car_display_key_text(
    const car_debug_display_snapshot_t *debug)
{
    if (debug == NULL) {
        return "----";
    }
    if (debug->pb21_pressed ||
        (debug->last_button_id == CAR_DEBUG_BUTTON_PB21)) {
        return "PB21";
    }
    if (debug->pb4_pressed ||
        (debug->last_button_id == CAR_DEBUG_BUTTON_PB4)) {
        return "PB4";
    }
    if (debug->pb5_pressed ||
        (debug->last_button_id == CAR_DEBUG_BUTTON_PB5)) {
        return "PB5";
    }
    return "----";
}

static int32_t car_display_clamp_i32(
    int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void car_display_format_signed_tenths(
    char *buffer, size_t buffer_size, int32_t value_milli)
{
    int64_t magnitude = value_milli;
    char sign = '+';

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }
    if (magnitude < 0) {
        sign = '-';
        magnitude = -magnitude;
    }
    magnitude += 50;
    if (magnitude > 999000) {
        magnitude = 999000;
    }
    (void) snprintf(buffer, buffer_size, "%c%03ld.%1ld", sign,
        (long) (magnitude / 1000),
        (long) ((magnitude % 1000) / 100));
}
