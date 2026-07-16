#include "board_button.h"
#include "board_motor_safe.h"
#include "board_resources.h"
#include "board_wheel_drive.h"
#include "bluetooth_uart.h"
#include "control_supervisor.h"
#include "delay.h"
#include "encoder_input.h"
#include "icm20948.h"
#include "position_bringup_test.h"
#include "reset_diagnostics.h"
#include "speed_bringup_test.h"
#include "speed_tuning_console.h"
#include "st7789.h"
#include "ti_msp_dl_config.h"
#include "wheel_position_control.h"
#include "wheel_speed_control.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_PB21_MOVE_COUNTS     4000
#define APP_SW2_PB4_MOVE_COUNTS -2000
#define APP_SW1_PB5_MOVE_COUNTS  2000

volatile bool g_car_pb21_pressed;
volatile uint32_t g_car_pb21_press_count;
volatile bool g_car_pb4_pressed;
volatile uint32_t g_car_pb4_press_count;
volatile bool g_car_pb5_pressed;
volatile uint32_t g_car_pb5_press_count;
volatile int32_t g_car_last_button_move_counts;
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
volatile int32_t g_car_encoder_count_difference;
volatile int32_t g_car_encoder_speed_difference_pps;

static void app_display_init(void);
static void app_display_update(uint32_t now_ms);
static void app_update_debug_state(void);
static int32_t app_round_float(float value);

int main(void)
{
    uint32_t displayed_second = UINT32_MAX;
    uint32_t displayed_encoder_period = UINT32_MAX;
    bool display_dirty = true;

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
    WheelSpeedControl_Init(delay_get_ms());
    WheelPositionControl_Init(delay_get_ms());
    SpeedBringupTest_Init(ResetDiagnostics_IsSuspicious());
    PositionBringupTest_Init(ResetDiagnostics_IsSuspicious());
    BluetoothUart_Init();
    SpeedTuningConsole_Init();

    ST7789_Init();
    app_display_init();
    app_update_debug_state();

    while (1) {
        uint32_t now_ms = delay_get_ms();
        uint32_t now_second = now_ms / 1000U;
        uint32_t now_encoder_period = now_ms / 200U;
        bool pb21_press_event;
        bool pb4_press_event;
        bool pb5_press_event;
        bool pb21_release_event;
        bool pb4_release_event;
        bool pb5_release_event;
        bool any_press_event;
        bool any_release_event;

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
        SpeedTuningConsole_Task(now_ms);
        ControlSupervisor_Task(now_ms);
        if (any_press_event) {
            if (SpeedBringupTest_GetState() ==
                    SPEED_BRINGUP_TEST_RUNNING) {
                SpeedBringupTest_RequestStop();
                g_car_last_button_move_counts = 0;
            } else if (PositionBringupTest_GetState() ==
                    POSITION_BRINGUP_TEST_RUNNING) {
                PositionBringupTest_RequestStop();
                g_car_last_button_move_counts = 0;
            } else if (pb21_press_event) {
                if (PositionBringupTest_RequestMove(
                        APP_PB21_MOVE_COUNTS)) {
                    g_car_last_button_move_counts =
                        APP_PB21_MOVE_COUNTS;
                }
            } else if (pb4_press_event) {
                if (PositionBringupTest_RequestMove(
                        APP_SW2_PB4_MOVE_COUNTS)) {
                    g_car_last_button_move_counts =
                        APP_SW2_PB4_MOVE_COUNTS;
                }
            } else if (pb5_press_event &&
                PositionBringupTest_RequestMove(
                    APP_SW1_PB5_MOVE_COUNTS)) {
                g_car_last_button_move_counts =
                    APP_SW1_PB5_MOVE_COUNTS;
            }
        }
        SpeedBringupTest_Task(now_ms, false);
        PositionBringupTest_Task(now_ms, false);
        WheelPositionControl_Task(now_ms);
        WheelSpeedControl_Task(now_ms);

        if (pb21_press_event) {
            g_car_pb21_press_count++;
            display_dirty = true;
        }
        if (pb4_press_event) {
            g_car_pb4_press_count++;
            display_dirty = true;
        }
        if (pb5_press_event) {
            g_car_pb5_press_count++;
            display_dirty = true;
        }
        if (any_release_event) {
            display_dirty = true;
        }
        if (now_second != displayed_second) {
            displayed_second = now_second;
            display_dirty = true;
        }
        if (now_encoder_period != displayed_encoder_period) {
            displayed_encoder_period = now_encoder_period;
            display_dirty = true;
        }

        app_update_debug_state();

        if (display_dirty) {
            app_display_update(now_ms);
            display_dirty = false;
        }

        __WFI();
    }
}

static void app_display_init(void)
{
    ST7789_Fill(ST7789_COLOR_BLACK);
    ST7789_FillRect(0U, 0U, ST7789_WIDTH, 26U, ST7789_COLOR_BLUE);
    ST7789_ShowString(8U, 5U, "CAR POSITION + IMU", ST7789_8X16,
        ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
}

static void app_display_update(uint32_t now_ms)
{
    uint16_t reset_color = ResetDiagnostics_IsSuspicious() ?
        ST7789_COLOR_RED : ST7789_COLOR_GREEN;
    uint16_t button_color = (g_car_pb21_pressed || g_car_pb4_pressed ||
        g_car_pb5_pressed) ?
        ST7789_COLOR_GREEN : ST7789_COLOR_WHITE;

    ST7789_Printf(8U, 30U, ST7789_8X16, reset_color,
        ST7789_COLOR_BLACK, "RESET:%-25s",
        ResetDiagnostics_GetCauseText());
    ST7789_Printf(8U, 48U, ST7789_8X16, button_color,
        ST7789_COLOR_BLACK, "K 21:%c 4:%c 5:%c CMD:%+5ld",
        g_car_pb21_pressed ? 'P' : '-',
        g_car_pb4_pressed ? 'P' : '-',
        g_car_pb5_pressed ? 'P' : '-',
        (long) g_car_last_button_move_counts);
    ST7789_Printf(8U, 66U, ST7789_8X16, ST7789_COLOR_YELLOW,
        ST7789_COLOR_BLACK, "P:%-6s %-6s O:%4ld/%4ld",
        PositionBringupTest_GetStateText(),
        g_car_motor_high_impedance ? "HIGH-Z" : "ARMED",
        (long) g_car_speed_left_output_permille,
        (long) g_car_speed_right_output_permille);
    ST7789_Printf(8U, 84U, ST7789_8X16, ST7789_COLOR_CYAN,
        ST7789_COLOR_BLACK, "L T:%7ld C:%7ld E:%6ld",
        (long) g_car_position_left_target_count,
        (long) g_car_encoder_0_count,
        (long) g_car_position_left_error_count);
    ST7789_Printf(8U, 102U, ST7789_8X16, ST7789_COLOR_CYAN,
        ST7789_COLOR_BLACK, "R T:%7ld C:%7ld E:%6ld",
        (long) g_car_position_right_target_count,
        (long) g_car_encoder_1_count,
        (long) g_car_position_right_error_count);
    ST7789_Printf(8U, 120U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "VT:%5ld/%-5ld V:%5ld/%-5ld",
        (long) g_car_position_left_speed_target_pps,
        (long) g_car_position_right_speed_target_pps,
        (long) g_car_encoder_0_speed_pps,
        (long) g_car_encoder_1_speed_pps);
    ST7789_FillRect(0U, 138U, ST7789_WIDTH, 18U,
        ST7789_COLOR_BLACK);
    ST7789_Printf(8U, 138U, ST7789_6X8,
        g_car_imu_ready ? ST7789_COLOR_GREEN : ST7789_COLOR_RED,
        ST7789_COLOR_BLACK,
        "IMU:%s ID:%02lX Y:%7ld GZ:%7ld N:%lu",
        g_car_imu_ready ? "OK" : "ERR",
        (unsigned long) g_car_imu_who_am_i,
        (long) g_car_imu_yaw_mdeg,
        (long) g_car_imu_gz_mdps,
        (unsigned long) g_car_imu_sample_count);
    ST7789_Printf(8U, 147U, ST7789_6X8, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "SYNC:%5ld C:%4ld INV:%lu/%lu",
        (long) g_car_position_sync_error_count,
        (long) g_car_position_sync_correction_pps,
        (unsigned long) g_car_encoder_0_invalid,
        (unsigned long) g_car_encoder_1_invalid);
    ST7789_Printf(8U, 157U, ST7789_6X8, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "M:%s B:%s R:%lu UP:%08lu",
        ControlSupervisor_GetModeText(),
        ControlSupervisor_GetBlockReasonText(),
        (unsigned long) g_car_position_test_run_count,
        (unsigned long) (now_ms / 1000U));
}

static void app_update_debug_state(void)
{
    encoder_input_snapshot_t encoder_0;
    encoder_input_snapshot_t encoder_1;
    wheel_speed_control_snapshot_t speed;
    wheel_position_control_snapshot_t position;
    icm20948_snapshot_t imu;

    g_car_pb21_pressed = BoardButton_IsPressed();
    g_car_pb4_pressed = BoardButton_IsPressedId(
        BOARD_BUTTON_ID_SW2_PB4);
    g_car_pb5_pressed = BoardButton_IsPressedId(
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
