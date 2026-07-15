#include "board_button.h"
#include "board_motor_safe.h"
#include "board_resources.h"
#include "control_supervisor.h"
#include "delay.h"
#include "encoder_input.h"
#include "motor_bringup_test.h"
#include "reset_diagnostics.h"
#include "st7789.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

volatile bool g_car_pb21_pressed;
volatile uint32_t g_car_pb21_press_count;
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
volatile uint32_t g_car_motor_test_state;
volatile int32_t g_car_motor_test_command;
volatile int32_t g_car_motor_test_command_a;
volatile int32_t g_car_motor_test_command_b;
volatile uint32_t g_car_motor_test_run_count;
volatile uint32_t g_car_motor_test_channel;
volatile int32_t g_car_encoder_count_difference;
volatile int32_t g_car_encoder_speed_difference_pps;

static void app_display_init(void);
static void app_display_update(uint32_t now_ms);
static void app_update_debug_state(void);

int main(void)
{
    uint32_t displayed_second = UINT32_MAX;
    uint32_t displayed_encoder_period = UINT32_MAX;
    bool display_dirty = true;

    SYSCFG_DL_init();
    ResetDiagnostics_Init();
    BoardMotorSafe_Init();
    BoardButton_Init(delay_get_ms());
    ControlSupervisor_Init(ResetDiagnostics_IsSuspicious());
    EncoderInput_Init(delay_get_ms());
    EncoderInput_SetInverted(ENCODER_INPUT_0,
        BOARD_ENCODER_0_FORWARD_INVERTED != 0);
    EncoderInput_SetInverted(ENCODER_INPUT_1,
        BOARD_ENCODER_1_FORWARD_INVERTED != 0);
    MotorBringupTest_Init(ResetDiagnostics_IsSuspicious());

    ST7789_Init();
    app_display_init();
    app_update_debug_state();

    while (1) {
        uint32_t now_ms = delay_get_ms();
        uint32_t now_second = now_ms / 1000U;
        uint32_t now_encoder_period = now_ms / 200U;
        bool press_event;
        bool release_event;

        BoardButton_Task(now_ms);
        press_event = BoardButton_GetPressEvent();
        release_event = BoardButton_GetReleaseEvent();
        EncoderInput_Task(now_ms);
        ControlSupervisor_Task(now_ms);
        MotorBringupTest_Task(now_ms, press_event);

        if (press_event) {
            g_car_pb21_press_count++;
            display_dirty = true;
        }
        if (release_event) {
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
    ST7789_ShowString(8U, 5U, "CAR DUAL MOTOR TEST", ST7789_8X16,
        ST7789_COLOR_WHITE, ST7789_COLOR_BLUE);
}

static void app_display_update(uint32_t now_ms)
{
    uint16_t reset_color = ResetDiagnostics_IsSuspicious() ?
        ST7789_COLOR_RED : ST7789_COLOR_GREEN;
    uint16_t button_color = g_car_pb21_pressed ?
        ST7789_COLOR_GREEN : ST7789_COLOR_WHITE;

    ST7789_Printf(8U, 30U, ST7789_8X16, reset_color,
        ST7789_COLOR_BLACK, "RESET:%-25s",
        ResetDiagnostics_GetCauseText());
    ST7789_Printf(8U, 48U, ST7789_8X16, button_color,
        ST7789_COLOR_BLACK, "PB21:%-8s COUNT:%05lu",
        g_car_pb21_pressed ? "PRESSED" : "RELEASED",
        (unsigned long) g_car_pb21_press_count);
    ST7789_Printf(8U, 66U, ST7789_8X16, ST7789_COLOR_YELLOW,
        ST7789_COLOR_BLACK, "M:%s %-6s T:%-6s A:%4ld B:%4ld",
        MotorBringupTest_GetMotorText(),
        g_car_motor_high_impedance ? "HIGH-Z" : "ARMED",
        MotorBringupTest_GetStateText(),
        (long) g_car_motor_test_command_a,
        (long) g_car_motor_test_command_b);
    ST7789_Printf(8U, 84U, ST7789_8X16, ST7789_COLOR_CYAN,
        ST7789_COLOR_BLACK, "E0 %s/%s C:%8ld V:%6ld",
        BOARD_ENCODER_0_A_PIN_NAME, BOARD_ENCODER_0_B_PIN_NAME,
        (long) g_car_encoder_0_count,
        (long) g_car_encoder_0_speed_pps);
    ST7789_Printf(8U, 102U, ST7789_8X16, ST7789_COLOR_CYAN,
        ST7789_COLOR_BLACK, "E1 %s/%s C:%8ld V:%6ld",
        BOARD_ENCODER_1_A_PIN_NAME, BOARD_ENCODER_1_B_PIN_NAME,
        (long) g_car_encoder_1_count,
        (long) g_car_encoder_1_speed_pps);
    ST7789_Printf(8U, 120U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "DIFF C:%8ld V:%7ld",
        (long) g_car_encoder_count_difference,
        (long) g_car_encoder_speed_difference_pps);
    ST7789_Printf(8U, 138U, ST7789_8X16, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "INV:%5lu/%-5lu ED:%5lu/%-5lu",
        (unsigned long) g_car_encoder_0_invalid,
        (unsigned long) g_car_encoder_1_invalid,
        (unsigned long) g_car_encoder_0_edges,
        (unsigned long) g_car_encoder_1_edges);
    ST7789_Printf(8U, 157U, ST7789_6X8, ST7789_COLOR_WHITE,
        ST7789_COLOR_BLACK, "M:%s B:%s R:%lu UP:%08lu",
        ControlSupervisor_GetModeText(),
        ControlSupervisor_GetBlockReasonText(),
        (unsigned long) g_car_motor_test_run_count,
        (unsigned long) (now_ms / 1000U));
}

static void app_update_debug_state(void)
{
    encoder_input_snapshot_t encoder_0;
    encoder_input_snapshot_t encoder_1;

    g_car_pb21_pressed = BoardButton_IsPressed();
    g_car_reset_cause = ResetDiagnostics_GetCause();
    g_car_control_mode = (uint32_t) ControlSupervisor_GetMode();
    g_car_control_block_reason =
        (uint32_t) ControlSupervisor_GetBlockReason();
    g_car_motor_high_impedance = BoardMotorSafe_IsHighImpedance();
    g_car_motor_test_state = (uint32_t) MotorBringupTest_GetState();
    g_car_motor_test_command = MotorBringupTest_GetCommand();
    g_car_motor_test_command_a = MotorBringupTest_GetCommandA();
    g_car_motor_test_command_b = MotorBringupTest_GetCommandB();
    g_car_motor_test_run_count = MotorBringupTest_GetRunCount();
    g_car_motor_test_channel = MotorBringupTest_GetMotorChannel();

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
