#include "car_display.h"

#include "debug_snapshot.h"
#include "heading_bringup_test.h"
#include "jdy31_config.h"
#include "line_sensor.h"
#include "line_tracking_bringup_test.h"
#include "st7789.h"
#include "yaw_bringup_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static void car_display_format_signed_tenths(
    char *buffer, size_t buffer_size, int32_t value_milli);

void CarDisplay_Init(void)
{
    ST7789_Fill(ST7789_COLOR_BLACK);
    if (JDY31_ConfigIsExclusive()) {
        ST7789_FillRect(0U, 0U, ST7789_WIDTH, 26U,
            ST7789_COLOR_BLUE);
        ST7789_ShowString(8U, 5U, "JDY-31 CONFIG", ST7789_8X16,
            ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
        return;
    }
    ST7789_FillRect(0U, 0U, ST7789_WIDTH, 28U, ST7789_COLOR_BLUE);
    ST7789_ShowAsciiStringFast(8U, 6U, "YAW CONTROL",
        ST7789_8X16, ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
    ST7789_DrawLine(16U, 88U, 304U, 88U,
        ST7789_RGB565(48U, 52U, 60U));
    ST7789_ShowAsciiStringFast(16U, 50U, "NOW", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
    ST7789_ShowAsciiStringFast(240U, 62U, "deg", ST7789_8X16,
        ST7789_COLOR_WHITE, ST7789_COLOR_BLACK);
    ST7789_ShowAsciiStringFast(16U, 94U, "TARGET", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
    ST7789_ShowAsciiStringFast(176U, 94U, "ERROR", ST7789_8X16,
        ST7789_COLOR_CYAN, ST7789_COLOR_BLACK);
}

void CarDisplay_Update(uint32_t now_ms, car_display_phase_t phase)
{
    car_debug_display_snapshot_t debug;
    char angle_text[16];
    char state_text[8];
    char command_text[16];
    char target_text[16];
    char error_text[16];
    char rate_text[16];
    char timer_text[12];
    char line_text[24];
    char line_state_text[9];
    char line_bits[9];
    const char *key_text = "KEY ----";
    uint16_t key_color = ST7789_COLOR_WHITE;
    int32_t angle_mdeg;
    int32_t command_mdeg;
    int32_t command_magnitude;
    char command_sign = '+';
    int32_t target_mdeg;
    int32_t error_mdeg;
    uint32_t display_elapsed_ms;
    bool heading_active = HeadingBringupTest_IsActive();
    bool yaw_active = YawBringupTest_IsActive();
    bool line_tracking_active = LineTrackingBringupTest_IsActive();
    bool angle_motion_active = heading_active || yaw_active;
    const char *control_state = line_tracking_active ?
        LineTrackingBringupTest_GetStateText() :
        (heading_active ? HeadingBringupTest_GetStateText() :
            YawBringupTest_GetStateText());
    uint16_t angle_color;
    uint16_t state_color = ST7789_COLOR_WHITE;
    uint16_t line_color = ST7789_COLOR_RED;
    uint8_t line_index;
    int32_t line_error_magnitude;
    char line_error_sign = '+';

    (void) now_ms;

    if (!CarDebugSnapshot_GetDisplay(&debug)) {
        return;
    }
    angle_mdeg = debug.imu_yaw_mdeg;
    command_mdeg = debug.last_button_yaw_mdeg;
    command_magnitude = command_mdeg;
    target_mdeg = debug.yaw_target_mdeg;
    error_mdeg = debug.yaw_error_mdeg;
    display_elapsed_ms = debug.yaw_elapsed_ms;
    angle_color = (debug.imu_ready && debug.imu_attitude_valid) ?
        ST7789_COLOR_YELLOW : ST7789_COLOR_RED;
    line_error_magnitude = debug.line_sensor_error;

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

    if ((YawBringupTest_GetState() == YAW_BRINGUP_TEST_ARMING) &&
        (command_mdeg != 0)) {
        target_mdeg = command_mdeg;
        error_mdeg = command_mdeg;
    }
    car_display_format_signed_tenths(
        angle_text, sizeof(angle_text), angle_mdeg);
    car_display_format_signed_tenths(
        target_text, sizeof(target_text), target_mdeg);
    car_display_format_signed_tenths(
        error_text, sizeof(error_text), error_mdeg);
    car_display_format_signed_tenths(
        rate_text, sizeof(rate_text), debug.imu_yaw_rate_mdps);

    if (command_magnitude < 0) {
        command_sign = '-';
        command_magnitude = -command_magnitude;
    }
    if (heading_active) {
        (void) snprintf(command_text, sizeof(command_text), "V:%4ld",
            (long) debug.heading_base_target_pps);
    } else if (line_tracking_active) {
        (void) snprintf(command_text, sizeof(command_text), "V:%4ld",
            (long) debug.line_tracking_base_target_pps);
    } else if (command_mdeg == 0) {
        (void) snprintf(command_text, sizeof(command_text), "CMD: ---");
    } else {
        (void) snprintf(command_text, sizeof(command_text), "CMD:%c%03ld",
            command_sign, (long) (command_magnitude / 1000));
    }
    (void) snprintf(state_text, sizeof(state_text), "%-6.6s",
        control_state);
    if (line_tracking_active) {
        display_elapsed_ms = debug.line_tracking_elapsed_ms;
    }
    if (display_elapsed_ms > 99990U) {
        display_elapsed_ms = 99990U;
    }
    (void) snprintf(timer_text, sizeof(timer_text), "T%2lu.%02lus",
        (unsigned long) (display_elapsed_ms / 1000U),
        (unsigned long) ((display_elapsed_ms % 1000U) / 10U));

    for (line_index = 0U; line_index < 8U; line_index++) {
        uint8_t bit = (uint8_t) (0x80U >> line_index);

        line_bits[line_index] =
            ((debug.line_sensor_active_mask & bit) != 0U) ? '1' : '0';
    }
    line_bits[8] = '\0';
    if (line_error_magnitude < 0) {
        line_error_sign = '-';
        line_error_magnitude = -line_error_magnitude;
    }
    if (line_error_magnitude > 99) {
        line_error_magnitude = 99;
    }
    (void) snprintf(line_text, sizeof(line_text), "L%s E%c%02ld",
        line_bits, line_error_sign, (long) line_error_magnitude);
    if (debug.line_sensor_ready) {
        (void) snprintf(line_state_text, sizeof(line_state_text), "%-8s",
            debug.line_sensor_seen ? "LINE" : "MISS");
        line_color = debug.line_sensor_seen ?
            ST7789_COLOR_GREEN : ST7789_COLOR_YELLOW;
    } else if ((debug.line_sensor_state ==
            (uint32_t) LINE_SENSOR_STATE_BOOT_WAIT) ||
        (debug.line_sensor_state ==
            (uint32_t) LINE_SENSOR_STATE_CALIBRATION_ON) ||
        (debug.line_sensor_state ==
            (uint32_t) LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT)) {
        (void) snprintf(line_state_text, sizeof(line_state_text), "%-8s",
            "CAL");
        line_color = ST7789_COLOR_YELLOW;
    } else {
        (void) snprintf(line_state_text, sizeof(line_state_text), "%-8s",
            "ERR");
    }

    if (debug.pb21_pressed) {
        key_text = "KEY PB21";
        key_color = ST7789_COLOR_GREEN;
    } else if (debug.pb4_pressed) {
        key_text = "KEY PB4 ";
        key_color = ST7789_COLOR_GREEN;
    } else if (debug.pb5_pressed) {
        key_text = "KEY PB5 ";
        key_color = ST7789_COLOR_GREEN;
    } else {
        switch (debug.last_button_id) {
            case CAR_DEBUG_BUTTON_PB21:
                key_text = "KEY PB21";
                break;
            case CAR_DEBUG_BUTTON_PB4:
                key_text = "KEY PB4 ";
                break;
            case CAR_DEBUG_BUTTON_PB5:
                key_text = "KEY PB5 ";
                break;
            default:
                break;
        }
    }

    if (line_tracking_active) {
        state_color = ST7789_COLOR_CYAN;
    } else if (heading_active) {
        state_color = (HeadingBringupTest_GetState() ==
            HEADING_BRINGUP_TEST_ARMING) ?
            ST7789_COLOR_YELLOW : ST7789_COLOR_CYAN;
    } else {
        switch (YawBringupTest_GetState()) {
            case YAW_BRINGUP_TEST_ARMING:
                state_color = ST7789_COLOR_YELLOW;
                break;
            case YAW_BRINGUP_TEST_RUNNING:
                state_color = ST7789_COLOR_CYAN;
                break;
            case YAW_BRINGUP_TEST_COMPLETE:
                state_color = ST7789_COLOR_GREEN;
                break;
            case YAW_BRINGUP_TEST_ABORTED:
            case YAW_BRINGUP_TEST_LOCKED:
                state_color = ST7789_COLOR_RED;
                break;
            default:
                break;
        }
    }

    switch (phase) {
        case CAR_DISPLAY_PHASE_ANGLE:
            ST7789_ShowAsciiStringScaled(80U, 32U, angle_text, 3U,
                angle_color, ST7789_COLOR_BLACK);
            break;

        case CAR_DISPLAY_PHASE_TARGET:
            ST7789_ShowAsciiStringScaled(16U, 110U, target_text, 2U,
                ST7789_COLOR_WHITE, ST7789_COLOR_BLACK);
            ST7789_ShowAsciiStringScaled(176U, 110U, error_text, 2U,
                ST7789_COLOR_YELLOW, ST7789_COLOR_BLACK);
            break;

        case CAR_DISPLAY_PHASE_FOOTER:
            if (angle_motion_active) {
                ST7789_PrintfFast(8U, 151U, ST7789_8X16,
                    ST7789_COLOR_WHITE, ST7789_COLOR_BLACK,
                    "RATE %s", rate_text);
                ST7789_ShowAsciiStringFast(120U, 151U, key_text,
                    ST7789_8X16, key_color, ST7789_COLOR_BLACK);
                ST7789_ShowAsciiStringFast(224U, 151U, command_text,
                    ST7789_8X16, ST7789_COLOR_WHITE,
                    ST7789_COLOR_BLACK);
            } else {
                ST7789_ShowAsciiStringFast(8U, 151U, line_text,
                    ST7789_8X16, line_color, ST7789_COLOR_BLACK);
                ST7789_ShowAsciiStringFast(120U, 151U, key_text,
                    ST7789_8X16, key_color, ST7789_COLOR_BLACK);
                ST7789_ShowAsciiStringFast(224U, 151U,
                    line_state_text, ST7789_8X16, line_color,
                    ST7789_COLOR_BLACK);
            }
            break;

        case CAR_DISPLAY_PHASE_HEADER:
        default:
            ST7789_ShowAsciiStringFast(112U, 6U, state_text,
                ST7789_8X16, state_color, ST7789_COLOR_BLUE);
            ST7789_ShowAsciiStringFast(176U, 6U, timer_text,
                ST7789_8X16, ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
            ST7789_ShowAsciiStringFast(264U, 6U,
                debug.motor_high_impedance ? "HIGH-Z" : "ARMED ",
                ST7789_8X16,
                debug.motor_high_impedance ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_RED,
                ST7789_COLOR_BLUE);
            break;
    }
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
    if (magnitude > 999000) {
        magnitude = 999000;
    }
    (void) snprintf(buffer, buffer_size, "%c%03ld.%1ld", sign,
        (long) (magnitude / 1000),
        (long) ((magnitude % 1000) / 100));
}
