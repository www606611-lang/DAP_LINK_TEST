#include "ti_msp_dl_config.h"
#include "timer.h"
#include "encoder.h"
#include "encoder_position_control.h"
#include "encoder_speed_control.h"
#include "fault_capture.h"
#include "icm20948.h"
#include "key.h"
#include "lcd_status.h"
#include "bluetooth_uart.h"
#include "pid_tuning_store.h"
#include "motor.h"
#include "uart0_dma.h"
#include "uart_display.h"

#include <stdio.h>

#define APP_POSITION_STEP_COUNT          300.0f
#define APP_POSITION_STEP_MAX_COUNT      60000.0f
#define APP_POSITION_STEP_MIN_COUNT      (-APP_POSITION_STEP_MAX_COUNT)
#define APP_POSITION_VOFA_MS             250U
#define APP_POSITION_VOFA_LINE_SIZE      128U
#define APP_POSITION_LEFT_PWM_LIMIT      1000.0f
#define APP_POSITION_RIGHT_PWM_LIMIT     1000.0f
#define APP_POSITION_RIGHT_MIN_DRIVE_PWM 0.0f
#define APP_POSITION_MIN_DRIVE_REF_PPS   3000.0f

#define APP_STAGE_INIT_BEGIN             1U
#define APP_STAGE_INIT_IMU               2U
#define APP_STAGE_INIT_MOTOR             3U
#define APP_STAGE_INIT_ENCODER           4U
#define APP_STAGE_INIT_KEY               5U
#define APP_STAGE_INIT_SPEED             6U
#define APP_STAGE_INIT_LCD               7U
#define APP_STAGE_INIT_UART              8U
#define APP_STAGE_TASK_ENCODER           11U
#define APP_STAGE_TASK_IMU               12U
#define APP_STAGE_TASK_KEY               13U
#define APP_STAGE_TASK_SPEED             14U
#define APP_STAGE_TASK_UI                15U
#define APP_STAGE_TASK_UART              16U
#define APP_STAGE_TASK_LCD               17U

static void app_init(void);
static void app_task(void);
static void app_position_step_task(uint32_t now_ms);
static void app_position_step_echo(const char *key_name);
static void app_position_step_apply_targets(void);
static void app_report_reset_cause(void);
static const char *app_reset_cause_to_string(DL_SYSCTL_RESET_CAUSE cause);
static const char *app_fault_reason_to_string(uint32_t reason);
static const char *app_nmi_source_to_string(uint32_t nmi_iidx);
static void app_position_step_vofa_task(uint32_t now_ms);
static void app_vofa_append_char(
    char *line, uint16_t *length, uint16_t max_length, char value);
static void app_vofa_append_int32(
    char *line, uint16_t *length, uint16_t max_length, int32_t value);
static void app_vofa_append_hex32(
    char *line, uint16_t *length, uint16_t max_length, uint32_t value);
static int32_t app_vofa_round_float(float value);

static uint32_t g_app_position_last_vofa_ms;
static float g_app_position_left_target_count;
static float g_app_position_right_target_count;

int main(void)
{
    app_init();

    while (1) {
        app_task();
    }
}

static void app_init(void)
{
    uint32_t now_ms;

    FaultCapture_SetStage(APP_STAGE_INIT_BEGIN);
    SYSCFG_DL_init();
    timer_common_init();

    now_ms = timer_common_get_ms();
    FaultCapture_SetStage(APP_STAGE_INIT_IMU);
    ICM20948_TaskInit(now_ms);
    FaultCapture_SetStage(APP_STAGE_INIT_MOTOR);
    Motor_Init();
    FaultCapture_SetStage(APP_STAGE_INIT_ENCODER);
    Encoder_Init(now_ms);
    FaultCapture_SetStage(APP_STAGE_INIT_KEY);
    Key_Init(now_ms);
    app_report_reset_cause();
    FaultCapture_SetStage(APP_STAGE_INIT_SPEED);
    EncoderSpeedControl_Init(now_ms);
    EncoderSpeedControl_SetOutputLimits(
        APP_POSITION_LEFT_PWM_LIMIT, APP_POSITION_RIGHT_PWM_LIMIT);
    EncoderSpeedControl_SetDirectionalMinDrivePwm(0.0f, 0.0f,
        APP_POSITION_RIGHT_MIN_DRIVE_PWM, APP_POSITION_RIGHT_MIN_DRIVE_PWM,
        APP_POSITION_MIN_DRIVE_REF_PPS);
    FaultCapture_SetStage(APP_STAGE_INIT_LCD);
    lcd_status_screen_init(now_ms);
    EncoderPositionControl_Init(now_ms);
    EncoderPositionControl_SyncSpeedFromCurrent();
    (void) PidTuningStore_LoadApply();
    g_app_position_left_target_count = 0.0f;
    g_app_position_right_target_count = 0.0f;
    FaultCapture_SetStage(APP_STAGE_INIT_LCD);
    EncoderPositionControl_SetTargetCount(0.0f, 0.0f);
    FaultCapture_SetStage(APP_STAGE_INIT_UART);
    uart_display_init();
    bluetooth_uart_init();
    g_app_position_last_vofa_ms = now_ms;
    FaultCapture_ClearStage();
}

static void app_task(void)
{
    uint32_t now_ms = timer_common_get_ms();

    FaultCapture_SetStage(APP_STAGE_TASK_ENCODER);
    Encoder_Task(now_ms);
    FaultCapture_SetStage(APP_STAGE_TASK_IMU);
    ICM20948_Task(now_ms);
    FaultCapture_SetStage(APP_STAGE_TASK_KEY);
    Key_Task(now_ms);
    FaultCapture_SetStage(APP_STAGE_TASK_SPEED);
    app_position_step_task(now_ms);
    FaultCapture_SetStage(APP_STAGE_TASK_UI);
    app_position_step_vofa_task(now_ms);
    FaultCapture_SetStage(APP_STAGE_TASK_UART);
    uart_display_task(now_ms);
    bluetooth_uart_task(now_ms);
    FaultCapture_SetStage(APP_STAGE_TASK_LCD);
    lcd_status_screen_task(now_ms);
    FaultCapture_ClearStage();
}

static void app_position_step_task(uint32_t now_ms)
{
    if (Key_GetPressEvent(KEY_ID_B21)) {
        g_app_position_left_target_count += APP_POSITION_STEP_COUNT;
        g_app_position_right_target_count += APP_POSITION_STEP_COUNT;
        if (g_app_position_left_target_count > APP_POSITION_STEP_MAX_COUNT) {
            g_app_position_left_target_count = APP_POSITION_STEP_MAX_COUNT;
        }
        if (g_app_position_right_target_count > APP_POSITION_STEP_MAX_COUNT) {
            g_app_position_right_target_count = APP_POSITION_STEP_MAX_COUNT;
        }
        app_position_step_apply_targets();
        app_position_step_echo("B21+");
    }

    if (Key_GetPressEvent(KEY_ID_DOWN)) {
        g_app_position_left_target_count -= APP_POSITION_STEP_COUNT;
        g_app_position_right_target_count -= APP_POSITION_STEP_COUNT;
        if (g_app_position_left_target_count < APP_POSITION_STEP_MIN_COUNT) {
            g_app_position_left_target_count = APP_POSITION_STEP_MIN_COUNT;
        }
        if (g_app_position_right_target_count < APP_POSITION_STEP_MIN_COUNT) {
            g_app_position_right_target_count = APP_POSITION_STEP_MIN_COUNT;
        }
        app_position_step_apply_targets();
        app_position_step_echo("PB6-");
    }

    EncoderPositionControl_Task(now_ms);
}

static void app_position_step_echo(const char *key_name)
{
    char line[64];

    if (key_name == NULL) {
        return;
    }

    (void) snprintf(line, sizeof(line),
        "key:%s left=%.0f right=%.0f\r\n", key_name,
        g_app_position_left_target_count, g_app_position_right_target_count);
    bluetooth_uart_send_text(line);
}

static void app_position_step_apply_targets(void)
{
    EncoderPositionControl_SetTargetCount(
        g_app_position_left_target_count, g_app_position_right_target_count);
}

static void app_report_reset_cause(void)
{
    char line[128];
    uint16_t length = 0U;
    const char prefix[] = "RST:";
    fault_capture_record_t fault;
    bool has_fault;
    DL_SYSCTL_RESET_CAUSE cause = DL_SYSCTL_getResetCause();
    const char *text = app_reset_cause_to_string(cause);
    uint32_t stage = FaultCapture_GetStage();
    uint16_t i;

    for (i = 0U; (i < (uint16_t) (sizeof(prefix) - 1U)) &&
        (length < sizeof(line)); i++) {
        line[length++] = prefix[i];
    }

    while ((*text != '\0') && (length < sizeof(line))) {
        line[length++] = *text++;
    }

    has_fault = FaultCapture_Read(&fault);
    if (has_fault) {
        const char *reason = app_fault_reason_to_string(fault.reason);

        app_vofa_append_char(line, &length, sizeof(line), ' ');
        app_vofa_append_char(line, &length, sizeof(line), 'F');
        app_vofa_append_char(line, &length, sizeof(line), ':');
        while ((*reason != '\0') && (length < sizeof(line))) {
            line[length++] = *reason++;
        }
        if (fault.reason == FAULT_CAPTURE_REASON_NMI) {
            const char *nmi = app_nmi_source_to_string(fault.nmi_iidx);

            app_vofa_append_char(line, &length, sizeof(line), ' ');
            app_vofa_append_char(line, &length, sizeof(line), 'N');
            app_vofa_append_char(line, &length, sizeof(line), ':');
            while ((*nmi != '\0') && (length < sizeof(line))) {
                line[length++] = *nmi++;
            }
        }
        app_vofa_append_char(line, &length, sizeof(line), ' ');
        app_vofa_append_char(line, &length, sizeof(line), 'V');
        app_vofa_append_char(line, &length, sizeof(line), ':');
        app_vofa_append_int32(line, &length, sizeof(line),
            (int32_t) fault.active_vector);
        app_vofa_append_char(line, &length, sizeof(line), ' ');
        app_vofa_append_char(line, &length, sizeof(line), 'P');
        app_vofa_append_char(line, &length, sizeof(line), 'C');
        app_vofa_append_char(line, &length, sizeof(line), ':');
        app_vofa_append_hex32(line, &length, sizeof(line),
            fault.stacked_pc);
        app_vofa_append_char(line, &length, sizeof(line), ' ');
        app_vofa_append_char(line, &length, sizeof(line), 'L');
        app_vofa_append_char(line, &length, sizeof(line), 'R');
        app_vofa_append_char(line, &length, sizeof(line), ':');
        app_vofa_append_hex32(line, &length, sizeof(line),
            fault.stacked_lr);
        app_vofa_append_char(line, &length, sizeof(line), ' ');
        app_vofa_append_char(line, &length, sizeof(line), 'S');
        app_vofa_append_char(line, &length, sizeof(line), 'P');
        app_vofa_append_char(line, &length, sizeof(line), ':');
        app_vofa_append_hex32(line, &length, sizeof(line),
            fault.fault_sp);
        FaultCapture_Clear();
    }
    if (stage != 0U) {
        app_vofa_append_char(line, &length, sizeof(line), ' ');
        app_vofa_append_char(line, &length, sizeof(line), 'S');
        app_vofa_append_char(line, &length, sizeof(line), ':');
        app_vofa_append_int32(line, &length, sizeof(line), (int32_t) stage);
    }
    FaultCapture_ClearStage();

    if ((length + 2U) <= sizeof(line)) {
        line[length++] = '\r';
        line[length++] = '\n';
    }

    if (length > 0U) {
        (void) uart0_dma_send((const uint8_t *) line, length);
    }
}

static const char *app_fault_reason_to_string(uint32_t reason)
{
    switch (reason) {
        case FAULT_CAPTURE_REASON_HARDFAULT:
            return "HF";
        case FAULT_CAPTURE_REASON_NMI:
            return "NMI";
        default:
            return "UNK";
    }
}

static const char *app_nmi_source_to_string(uint32_t nmi_iidx)
{
    switch (nmi_iidx) {
        case DL_SYSCTL_NMI_IIDX_SRAM_DED:
            return "SRAM_DED";
        case DL_SYSCTL_NMI_IIDX_FLASH_DED:
            return "FLASH_DED";
        case DL_SYSCTL_NMI_IIDX_LFCLK_FAIL:
            return "LFCLK_FAIL";
        case DL_SYSCTL_NMI_IIDX_WWDT1_FAULT:
            return "WWDT1";
        case DL_SYSCTL_NMI_IIDX_WWDT0_FAULT:
            return "WWDT0";
        case DL_SYSCTL_NMI_IIDX_BORLVL:
            return "BORLVL";
        case DL_SYSCTL_NMI_IIDX_NO_INT:
            return "NO_INT";
        default:
            return "UNK";
    }
}

static const char *app_reset_cause_to_string(DL_SYSCTL_RESET_CAUSE cause)
{
    switch (cause) {
        case DL_SYSCTL_RESET_CAUSE_NO_RESET:
            return "NO_RESET";
        case DL_SYSCTL_RESET_CAUSE_POR_HW_FAILURE:
            return "POR_HW_FAILURE";
        case DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST:
            return "POR_EXTERNAL_NRST";
        case DL_SYSCTL_RESET_CAUSE_POR_SW_TRIGGERED:
            return "POR_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
            return "BOR_SUPPLY_FAILURE";
        case DL_SYSCTL_RESET_CAUSE_BOR_WAKE_FROM_SHUTDOWN:
            return "BOR_WAKE_FROM_SHUTDOWN";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_NON_PMU_PARITY_FAULT:
            return "BOOTRST_NON_PMU_PARITY_FAULT";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_CLOCK_FAULT:
            return "BOOTRST_CLOCK_FAULT";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_SW_TRIGGERED:
            return "BOOTRST_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST:
            return "BOOTRST_EXTERNAL_NRST";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_BSL_EXIT:
            return "SYSRST_BSL_EXIT";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_BSL_ENTRY:
            return "SYSRST_BSL_ENTRY";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION:
            return "SYSRST_WWDT0_VIOLATION";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT1_VIOLATION:
            return "SYSRST_WWDT1_VIOLATION";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_FLASH_ECC_ERROR:
            return "SYSRST_FLASH_ECC_ERROR";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
            return "SYSRST_CPU_LOCKUP_VIOLATION";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_DEBUG_TRIGGERED:
            return "SYSRST_DEBUG_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_SW_TRIGGERED:
            return "SYSRST_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_CPURST_DEBUG_TRIGGERED:
            return "CPURST_DEBUG_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_CPURST_SW_TRIGGERED:
            return "CPURST_SW_TRIGGERED";
        default:
            return "UNKNOWN";
    }
}

static void app_position_step_vofa_task(uint32_t now_ms)
{
    char line[APP_POSITION_VOFA_LINE_SIZE];
    uint16_t length = 0U;
    float target_left_count;
    float target_right_count;
    encoder_position_control_state_t left;
    encoder_position_control_state_t right;
    int32_t error_left_count;
    int32_t error_right_count;

    if ((uint32_t) (now_ms - g_app_position_last_vofa_ms) <
        APP_POSITION_VOFA_MS) {
        return;
    }

    app_vofa_append_char(line, &length, sizeof(line), 'd');
    app_vofa_append_char(line, &length, sizeof(line), ':');
    EncoderPositionControl_GetTargetCount(&target_left_count,
        &target_right_count);
    EncoderPositionControl_GetState(&left, &right);
    if ((target_left_count == 0.0f) && (target_right_count == 0.0f) &&
        (left.count == 0) && (right.count == 0) &&
        (left.cascade_speed_target_pps == 0.0f) &&
        (right.cascade_speed_target_pps == 0.0f) &&
        (left.pwm_command == 0.0f) && (right.pwm_command == 0.0f)) {
        g_app_position_last_vofa_ms = now_ms;
        return;
    }
    error_left_count = app_vofa_round_float(target_left_count -
        (float) left.count);
    error_right_count = app_vofa_round_float(target_right_count -
        (float) right.count);
    app_vofa_append_int32(line, &length, sizeof(line),
        app_vofa_round_float(target_left_count));
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        app_vofa_round_float(target_right_count));
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        left.count);
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        right.count);
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line), error_left_count);
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line), error_right_count);
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        app_vofa_round_float(left.cascade_speed_target_pps));
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        app_vofa_round_float(right.cascade_speed_target_pps));
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        app_vofa_round_float(left.pwm_command));
    app_vofa_append_char(line, &length, sizeof(line), ',');
    app_vofa_append_int32(line, &length, sizeof(line),
        app_vofa_round_float(right.pwm_command));
    app_vofa_append_char(line, &length, sizeof(line), '\n');

    if (length < sizeof(line)) {
        if (uart0_dma_send((const uint8_t *) line, length) == UART0_DMA_OK) {
            g_app_position_last_vofa_ms = now_ms;
        }
    }
}

static void app_vofa_append_char(
    char *line, uint16_t *length, uint16_t max_length, char value)
{
    if ((line == 0) || (length == 0) || (*length >= max_length)) {
        return;
    }

    line[*length] = value;
    (*length)++;
}

static void app_vofa_append_int32(
    char *line, uint16_t *length, uint16_t max_length, int32_t value)
{
    char digits[11];
    uint8_t digit_count = 0U;
    uint32_t magnitude;

    if ((line == 0) || (length == 0)) {
        return;
    }

    if (value < 0) {
        app_vofa_append_char(line, length, max_length, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    do {
        digits[digit_count++] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude > 0U) && (digit_count < sizeof(digits)));

    while (digit_count > 0U) {
        app_vofa_append_char(line, length, max_length,
            digits[--digit_count]);
    }
}

static void app_vofa_append_hex32(
    char *line, uint16_t *length, uint16_t max_length, uint32_t value)
{
    int8_t shift;

    app_vofa_append_char(line, length, max_length, '0');
    app_vofa_append_char(line, length, max_length, 'x');

    for (shift = 28; shift >= 0; shift -= 4) {
        uint8_t digit = (uint8_t) ((value >> shift) & 0x0FU);
        app_vofa_append_char(line, length, max_length,
            (char) ((digit < 10U) ? ('0' + digit) :
                                  ('A' + (digit - 10U))));
    }
}

static int32_t app_vofa_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }

    return (int32_t) (value - 0.5f);
}
