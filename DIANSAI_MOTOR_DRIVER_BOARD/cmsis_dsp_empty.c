#include "ti_msp_dl_config.h"
#include "app_diagnostics.h"
#include "timer.h"
#include "encoder_position_control.h"
#include "encoder_speed_control.h"
#include "icm20948.h"
#include "key.h"
#include "lcd_status.h"
#include "bluetooth_uart.h"
#include "line_tracking_control.h"
#include "pid_tuning_store.h"
#include "motor.h"
#include "uart_display.h"
#include "yaw_angle_control.h"

static void app_init(void);
static void app_task(void);

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

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_BEGIN);
    SYSCFG_DL_init();
    timer_common_init();

    now_ms = timer_common_get_ms();
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_IMU);
    ICM20948_TaskInit(now_ms);
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_MOTOR);
    Motor_Init();
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_ENCODER);
    Encoder_Init(now_ms);
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_KEY);
    Key_Init(now_ms);

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_SPEED);
    EncoderSpeedControl_Init(now_ms);

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_LINE);
    LineTrackingControl_Init(now_ms);

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_YAW);
    YawAngleControl_Init(now_ms);

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_POSITION);
    EncoderPositionControl_Init(now_ms);
    EncoderPositionControl_SyncSpeedFromCurrent();

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_LCD);
    lcd_status_screen_init(now_ms);
    (void) PidTuningStore_LoadApply();
    YawAngleControl_Stop();
    lcd_status_screen_set_pid_text("IDLE BT");

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_INIT_UART);
    uart_display_init();
    AppDiagnostics_ReportResetCause();
    bluetooth_uart_init();
    AppDiagnostics_ClearStage();
}

static void app_task(void)
{
    uint32_t now_ms = timer_common_get_ms();

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_TASK_ENCODER);
    Encoder_Task(now_ms);
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_TASK_IMU);
    ICM20948_Task(now_ms);
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_TASK_KEY);
    Key_Task(now_ms);

    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_TASK_UART);
    uart_display_task(now_ms);
    bluetooth_uart_task(now_ms);
    AppDiagnostics_SetStage(APP_DIAGNOSTICS_STAGE_TASK_LCD);
    lcd_status_screen_task(now_ms);
    AppDiagnostics_ClearStage();
}
