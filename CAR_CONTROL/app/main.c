#include "board_button.h"
#include "board_gpio_irq.h"
#include "board_motor_safe.h"
#include "board_resources.h"
#include "board_wheel_drive.h"
#include "bluetooth_uart.h"
#include "control_supervisor.h"
#include "delay.h"
#include "encoder_input.h"
#include "firmware_update.h"
#include "heading_bringup_test.h"
#include "icm20948.h"
#include "jdy31_config.h"
#include "position_bringup_test.h"
#include "reset_diagnostics.h"
#include "runtime_metrics.h"
#include "speed_bringup_test.h"
#include "speed_tuning_console.h"
#include "st7789.h"
#include "ti_msp_dl_config.h"
#include "wheel_position_control.h"
#include "wheel_heading_control.h"
#include "wheel_speed_control.h"
#include "wheel_yaw_control.h"
#include "yaw_bringup_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define APP_PB21_YAW_DEG       45.0f
#define APP_SW2_PB4_YAW_DEG   -60.0f
#define APP_SW1_PB5_YAW_DEG    90.0f
#define APP_BUTTON_ID_NONE       0U
#define APP_BUTTON_ID_PB4        4U
#define APP_BUTTON_ID_PB5        5U
#define APP_BUTTON_ID_PB21      21U
#define APP_DISPLAY_SLICE_INTERVAL_MS 50U
#define APP_DISPLAY_PHASE_HEADER       0U
#define APP_DISPLAY_PHASE_ANGLE        1U
#define APP_DISPLAY_PHASE_TARGET       2U
#define APP_DISPLAY_PHASE_FOOTER       3U
#define APP_DISPLAY_PHASE_COUNT        4U

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
volatile int32_t g_car_encoder_count_difference;
volatile int32_t g_car_encoder_speed_difference_pps;
volatile uint32_t g_car_jdy31_config_state;
volatile uint32_t g_car_jdy31_uart_baud;
volatile int32_t g_car_jdy31_reported_baud_code;
volatile bool g_car_jdy31_config_success;

static void app_display_init(void);
static void app_display_update(uint32_t now_ms, uint8_t phase);
static void app_format_signed_tenths(
    char *buffer, size_t buffer_size, int32_t value_milli);
static void app_update_debug_state(void);
static int32_t app_round_float(float value);

int main(void)
{
    uint32_t displayed_period = UINT32_MAX;
    uint8_t display_phase = APP_DISPLAY_PHASE_HEADER;
    bool display_dirty = true;

    FirmwareUpdate_AppInit();
    SYSCFG_DL_init();
    ResetDiagnostics_Init();
    BoardMotorSafe_Init();
    BoardWheelDrive_Init();
    BoardButton_Init(delay_get_ms());
    ControlSupervisor_Init(ResetDiagnostics_IsSuspicious());
    ICM20948_Init(delay_get_ms());
    EncoderInput_Init(delay_get_ms());
    EncoderInput_SetInverted(ENCODER_INPUT_0,
        BOARD_ENCODER_0_FORWARD_INVERTED != 0);
    EncoderInput_SetInverted(ENCODER_INPUT_1,
        BOARD_ENCODER_1_FORWARD_INVERTED != 0);
    BoardGpioIrq_Init();
    WheelSpeedControl_Init(delay_get_ms());
    WheelPositionControl_Init(delay_get_ms());
    WheelYawControl_Init(delay_get_ms());
    WheelHeadingControl_Init(delay_get_ms());
    SpeedBringupTest_Init(ResetDiagnostics_IsSuspicious());
    PositionBringupTest_Init(ResetDiagnostics_IsSuspicious());
    YawBringupTest_Init(ResetDiagnostics_IsSuspicious());
    HeadingBringupTest_Init(ResetDiagnostics_IsSuspicious());
    BluetoothUart_Init();
    JDY31_ConfigInit(delay_get_ms(),
        CAR_JDY31_CONFIGURE_ON_BOOT != 0);
    if (!JDY31_ConfigIsExclusive()) {
        SpeedTuningConsole_Init();
    }

    ST7789_Init();
    app_display_init();
    app_update_debug_state();
    AppRuntimeMetrics_Init(delay_get_ms());

    while (1) {
        uint32_t now_ms = delay_get_ms();
        uint32_t display_interval_ms = JDY31_ConfigIsExclusive() ?
            200U : APP_DISPLAY_SLICE_INTERVAL_MS;
        uint32_t now_display_period = now_ms / display_interval_ms;
        bool pb21_press_event;
        bool pb4_press_event;
        bool pb5_press_event;
        bool pb21_release_event;
        bool pb4_release_event;
        bool pb5_release_event;
        bool any_press_event;
        bool any_release_event;

        AppRuntimeMetrics_RecordLoop(now_ms);
        BoardButton_Task(now_ms);
        pb21_press_event = BoardButton_GetPressEventId(
            BOARD_BUTTON_ID_PB21);
        pb4_press_event = BoardButton_GetPressEventId(
            BOARD_BUTTON_ID_SW2_PB4);
        pb5_press_event = BoardButton_GetPressEventId(
            BOARD_BUTTON_ID_SW1_PB5);
        pb21_release_event = BoardButton_GetReleaseEventId(
            BOARD_BUTTON_ID_PB21);
        pb4_release_event = BoardButton_GetReleaseEventId(
            BOARD_BUTTON_ID_SW2_PB4);
        pb5_release_event = BoardButton_GetReleaseEventId(
            BOARD_BUTTON_ID_SW1_PB5);
        any_press_event = pb21_press_event || pb4_press_event ||
            pb5_press_event;
        any_release_event = pb21_release_event || pb4_release_event ||
            pb5_release_event;
        EncoderInput_Task(now_ms);
        ICM20948_Task(now_ms);
        BluetoothUart_Task(now_ms);
        JDY31_ConfigTask(now_ms);
        if (!JDY31_ConfigIsExclusive()) {
            SpeedTuningConsole_Task(now_ms);
        }
        FirmwareUpdate_Task();
        ControlSupervisor_Task(now_ms);
        if (any_press_event && !JDY31_ConfigIsExclusive() &&
            !FirmwareUpdate_IsPending()) {
            if (SpeedBringupTest_GetState() ==
                    SPEED_BRINGUP_TEST_RUNNING) {
                SpeedBringupTest_RequestStop();
                g_car_last_button_yaw_mdeg = 0;
            } else if (PositionBringupTest_GetState() ==
                    POSITION_BRINGUP_TEST_RUNNING) {
                PositionBringupTest_RequestStop();
                g_car_last_button_yaw_mdeg = 0;
            } else if (HeadingBringupTest_IsActive()) {
                HeadingBringupTest_RequestStop();
                g_car_last_button_yaw_mdeg = 0;
            } else if (YawBringupTest_IsActive()) {
                YawBringupTest_RequestStop();
                g_car_last_button_yaw_mdeg = 0;
            } else if (pb21_press_event) {
                if (YawBringupTest_RequestTurn(APP_PB21_YAW_DEG)) {
                    g_car_last_button_yaw_mdeg = 45000;
                }
            } else if (pb4_press_event) {
                if (YawBringupTest_RequestTurn(
                        APP_SW2_PB4_YAW_DEG)) {
                    g_car_last_button_yaw_mdeg = -60000;
                }
            } else if (pb5_press_event &&
                YawBringupTest_RequestTurn(APP_SW1_PB5_YAW_DEG)) {
                g_car_last_button_yaw_mdeg = 90000;
            }
        }
        SpeedBringupTest_Task(now_ms, false);
        PositionBringupTest_Task(now_ms, false);
        YawBringupTest_Task(now_ms);
        HeadingBringupTest_Task(now_ms);
        WheelPositionControl_Task(now_ms);
        WheelYawControl_Task(now_ms);
        WheelHeadingControl_Task(now_ms);
        WheelSpeedControl_Task(now_ms);

        if (pb21_press_event) {
            g_car_last_button_id = APP_BUTTON_ID_PB21;
            g_car_pb21_press_count++;
            display_phase = APP_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (pb4_press_event) {
            g_car_last_button_id = APP_BUTTON_ID_PB4;
            g_car_pb4_press_count++;
            display_phase = APP_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (pb5_press_event) {
            g_car_last_button_id = APP_BUTTON_ID_PB5;
            g_car_pb5_press_count++;
            display_phase = APP_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (any_release_event) {
            display_phase = APP_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (now_display_period != displayed_period) {
            displayed_period = now_display_period;
            display_dirty = true;
        }

        app_update_debug_state();

        if (display_dirty) {
            uint32_t display_started_ms = delay_get_ms();

            app_display_update(now_ms, display_phase);
            AppRuntimeMetrics_RecordDisplay(
                delay_get_ms() - display_started_ms);
            display_phase = (uint8_t) ((display_phase + 1U) %
                APP_DISPLAY_PHASE_COUNT);
            display_dirty = false;
        }

        __WFI();
    }
}

static void app_display_init(void)
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

static void app_display_update(uint32_t now_ms, uint8_t phase)
{
    char angle_text[16];
    char state_text[8];
    char command_text[16];
    char target_text[16];
    char error_text[16];
    char rate_text[16];
    char timer_text[12];
    const char *key_text = "KEY ----";
    uint16_t key_color = ST7789_COLOR_WHITE;
    int32_t angle_mdeg = g_car_imu_yaw_mdeg;
    int32_t command_mdeg = g_car_last_button_yaw_mdeg;
    int32_t command_magnitude = command_mdeg;
    char command_sign = '+';
    int32_t target_mdeg = g_car_yaw_target_mdeg;
    int32_t error_mdeg = g_car_yaw_error_mdeg;
    uint32_t display_elapsed_ms = g_car_yaw_elapsed_ms;
    bool heading_active = HeadingBringupTest_IsActive();
    const char *control_state = heading_active ?
        HeadingBringupTest_GetStateText() :
        YawBringupTest_GetStateText();
    uint16_t angle_color = (g_car_imu_ready &&
        g_car_imu_attitude_valid) ? ST7789_COLOR_YELLOW : ST7789_COLOR_RED;
    uint16_t state_color = ST7789_COLOR_WHITE;

    (void) now_ms;

    if (JDY31_ConfigIsExclusive()) {
        jdy31_config_snapshot_t jdy31;

        if (JDY31_ConfigGetSnapshot(&jdy31)) {
            uint16_t state_color = (jdy31.state == JDY31_CONFIG_SUCCESS) ?
                ST7789_COLOR_GREEN :
                ((jdy31.state == JDY31_CONFIG_FAILED) ?
                    ST7789_COLOR_RED : ST7789_COLOR_YELLOW);

            ST7789_PrintfFast(24U, 48U, ST7789_8X16, state_color,
                ST7789_COLOR_BLACK, "STATE: %-14s",
                JDY31_ConfigGetStateText());
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
    app_format_signed_tenths(angle_text, sizeof(angle_text), angle_mdeg);
    app_format_signed_tenths(target_text, sizeof(target_text), target_mdeg);
    app_format_signed_tenths(error_text, sizeof(error_text), error_mdeg);
    app_format_signed_tenths(
        rate_text, sizeof(rate_text), g_car_imu_yaw_rate_mdps);

    if (command_magnitude < 0) {
        command_sign = '-';
        command_magnitude = -command_magnitude;
    }
    if (heading_active) {
        (void) snprintf(command_text, sizeof(command_text), "V:%4ld",
            (long) g_car_heading_base_target_pps);
    } else if (command_mdeg == 0) {
        (void) snprintf(command_text, sizeof(command_text), "CMD: ---");
    } else {
        (void) snprintf(command_text, sizeof(command_text), "CMD:%c%03ld",
            command_sign, (long) (command_magnitude / 1000));
    }
    (void) snprintf(state_text, sizeof(state_text), "%-6.6s",
        control_state);
    if (display_elapsed_ms > 99990U) {
        display_elapsed_ms = 99990U;
    }
    (void) snprintf(timer_text, sizeof(timer_text), "T%2lu.%02lus",
        (unsigned long) (display_elapsed_ms / 1000U),
        (unsigned long) ((display_elapsed_ms % 1000U) / 10U));

    if (g_car_pb21_pressed) {
        key_text = "KEY PB21";
        key_color = ST7789_COLOR_GREEN;
    } else if (g_car_pb4_pressed) {
        key_text = "KEY PB4 ";
        key_color = ST7789_COLOR_GREEN;
    } else if (g_car_pb5_pressed) {
        key_text = "KEY PB5 ";
        key_color = ST7789_COLOR_GREEN;
    } else {
        switch (g_car_last_button_id) {
            case APP_BUTTON_ID_PB21:
                key_text = "KEY PB21";
                break;
            case APP_BUTTON_ID_PB4:
                key_text = "KEY PB4 ";
                break;
            case APP_BUTTON_ID_PB5:
                key_text = "KEY PB5 ";
                break;
            default:
                break;
        }
    }

    if (heading_active) {
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
        case APP_DISPLAY_PHASE_ANGLE:
            ST7789_ShowAsciiStringScaled(80U, 32U, angle_text, 3U,
                angle_color, ST7789_COLOR_BLACK);
            break;

        case APP_DISPLAY_PHASE_TARGET:
            ST7789_ShowAsciiStringScaled(16U, 110U, target_text, 2U,
                ST7789_COLOR_WHITE, ST7789_COLOR_BLACK);
            ST7789_ShowAsciiStringScaled(176U, 110U, error_text, 2U,
                ST7789_COLOR_YELLOW, ST7789_COLOR_BLACK);
            break;

        case APP_DISPLAY_PHASE_FOOTER:
            ST7789_PrintfFast(8U, 151U, ST7789_8X16,
                ST7789_COLOR_WHITE, ST7789_COLOR_BLACK,
                "RATE %s", rate_text);
            ST7789_ShowAsciiStringFast(120U, 151U, key_text,
                ST7789_8X16, key_color, ST7789_COLOR_BLACK);
            ST7789_ShowAsciiStringFast(224U, 151U, command_text,
                ST7789_8X16, ST7789_COLOR_WHITE, ST7789_COLOR_BLACK);
            break;

        case APP_DISPLAY_PHASE_HEADER:
        default:
            ST7789_ShowAsciiStringFast(112U, 6U, state_text,
                ST7789_8X16, state_color, ST7789_COLOR_BLUE);
            ST7789_ShowAsciiStringFast(176U, 6U, timer_text,
                ST7789_8X16, ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
            ST7789_ShowAsciiStringFast(264U, 6U,
                g_car_motor_high_impedance ? "HIGH-Z" : "ARMED ",
                ST7789_8X16,
                g_car_motor_high_impedance ?
                    ST7789_COLOR_GREEN : ST7789_COLOR_RED,
                ST7789_COLOR_BLUE);
            break;
    }
}

static void app_format_signed_tenths(
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

static void app_update_debug_state(void)
{
    encoder_input_snapshot_t encoder_0;
    encoder_input_snapshot_t encoder_1;
    wheel_speed_control_snapshot_t speed;
    wheel_position_control_snapshot_t position;
    wheel_yaw_control_snapshot_t yaw;
    wheel_heading_control_snapshot_t heading;
    icm20948_snapshot_t imu;
    jdy31_config_snapshot_t jdy31;

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
            app_round_float(speed.left_target_pps);
        g_car_speed_right_target_pps =
            app_round_float(speed.right_target_pps);
        g_car_speed_left_error_pps =
            app_round_float(speed.left_error_pps);
        g_car_speed_right_error_pps =
            app_round_float(speed.right_error_pps);
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
            app_round_float(position.left_speed_target_pps);
        g_car_position_right_speed_target_pps =
            app_round_float(position.right_speed_target_pps);
        g_car_position_sync_error_count = position.sync_error_count;
        g_car_position_sync_correction_pps =
            app_round_float(position.sync_correction_pps);
        g_car_position_sync_active = position.sync_active;
        g_car_position_update_count = position.update_count;
        g_car_position_last_result = (uint32_t) position.last_result;
        g_car_position_settled = position.settled;
    }

    if (WheelYawControl_GetSnapshot(&yaw)) {
        g_car_yaw_target_mdeg =
            app_round_float(yaw.target_yaw_deg * 1000.0f);
        g_car_yaw_current_mdeg =
            app_round_float(yaw.current_yaw_deg * 1000.0f);
        g_car_yaw_error_mdeg =
            app_round_float(yaw.error_deg * 1000.0f);
        g_car_yaw_rate_mdps =
            app_round_float(yaw.yaw_rate_dps * 1000.0f);
        g_car_yaw_turn_target_pps =
            app_round_float(yaw.turn_speed_target_pps);
        g_car_yaw_left_target_pps =
            app_round_float(yaw.left_speed_target_pps);
        g_car_yaw_right_target_pps =
            app_round_float(yaw.right_speed_target_pps);
        g_car_yaw_update_count = yaw.update_count;
        g_car_yaw_elapsed_ms = yaw.elapsed_ms;
        g_car_yaw_last_result = (uint32_t) yaw.last_result;
        g_car_yaw_settled = yaw.settled;
    }

    if (WheelHeadingControl_GetSnapshot(&heading)) {
        g_car_heading_target_mdeg =
            app_round_float(heading.target_yaw_deg * 1000.0f);
        g_car_heading_current_mdeg =
            app_round_float(heading.current_yaw_deg * 1000.0f);
        g_car_heading_error_mdeg =
            app_round_float(heading.error_deg * 1000.0f);
        g_car_heading_rate_mdps =
            app_round_float(heading.yaw_rate_dps * 1000.0f);
        g_car_heading_base_target_pps =
            app_round_float(heading.base_speed_target_pps);
        g_car_heading_correction_pps =
            app_round_float(heading.correction_target_pps);
        g_car_heading_left_target_pps =
            app_round_float(heading.left_speed_target_pps);
        g_car_heading_right_target_pps =
            app_round_float(heading.right_speed_target_pps);
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
        g_car_imu_ax_mg = app_round_float(imu.data.ax_g * 1000.0f);
        g_car_imu_ay_mg = app_round_float(imu.data.ay_g * 1000.0f);
        g_car_imu_az_mg = app_round_float(imu.data.az_g * 1000.0f);
        g_car_imu_gx_mdps = app_round_float(imu.data.gx_dps * 1000.0f);
        g_car_imu_gy_mdps = app_round_float(imu.data.gy_dps * 1000.0f);
        g_car_imu_gz_mdps = app_round_float(imu.data.gz_dps * 1000.0f);
        g_car_imu_roll_mdeg = app_round_float(imu.roll_deg * 1000.0f);
        g_car_imu_pitch_mdeg = app_round_float(imu.pitch_deg * 1000.0f);
        g_car_imu_yaw_mdeg = app_round_float(imu.yaw_deg * 1000.0f);
        g_car_imu_yaw_rate_mdps =
            app_round_float(imu.yaw_rate_dps * 1000.0f);
        g_car_imu_accel_norm_mg =
            app_round_float(imu.accel_norm_g * 1000.0f);
        g_car_imu_bias_x_mdps =
            app_round_float(imu.gyro_bias_x_dps * 1000.0f);
        g_car_imu_bias_y_mdps =
            app_round_float(imu.gyro_bias_y_dps * 1000.0f);
        g_car_imu_bias_z_mdps =
            app_round_float(imu.gyro_bias_z_dps * 1000.0f);
        g_car_imu_quaternion_w_million =
            app_round_float(imu.quaternion_w * 1000000.0f);
        g_car_imu_quaternion_x_million =
            app_round_float(imu.quaternion_x * 1000000.0f);
        g_car_imu_quaternion_y_million =
            app_round_float(imu.quaternion_y * 1000000.0f);
        g_car_imu_quaternion_z_million =
            app_round_float(imu.quaternion_z * 1000000.0f);
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

static int32_t app_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }
    return (int32_t) (value - 0.5f);
}
