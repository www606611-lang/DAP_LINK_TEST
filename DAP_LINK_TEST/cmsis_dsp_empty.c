#include "ti_msp_dl_config.h"
#include "timer.h"
#include "encoder.h"
#include "icm20948.h"
#include "key.h"
#include "lcd_status.h"
#include "line_tracking_control.h"
#include "motor.h"
#include "uart_display.h"

static void app_init(void);
static void app_task(void);
static void app_line_tracking_ui_task(uint32_t now_ms);

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

    SYSCFG_DL_init();
    timer_common_init();

    now_ms = timer_common_get_ms();
    ICM20948_TaskInit(now_ms);
    Motor_Init();
    Encoder_Init(now_ms);
    Key_Init(now_ms);
    LineTrackingControl_Init(now_ms);
    lcd_status_screen_init(now_ms);
    uart_display_init();
}

static void app_task(void)
{
    uint32_t now_ms = timer_common_get_ms();

    Encoder_Task(now_ms);
    ICM20948_Task(now_ms);
    Key_Task(now_ms);
    LineTrackingControl_Task(now_ms);
    app_line_tracking_ui_task(now_ms);
    uart_display_task(now_ms);
    lcd_status_screen_task(now_ms);
}

static void app_line_tracking_ui_task(uint32_t now_ms)
{
    line_tracking_state_t state;

    if (Key_GetPressEvent(KEY_ID_B21)) {
        LineTrackingControl_Toggle();
    }

    LineTrackingControl_GetState(&state);
    lcd_status_screen_set_line_sensor(state.raw, state.active_mask,
        state.active_count, state.line_error, state.enabled ? 1U : 0U,
        state.sensor_ok ? 1U : 0U, state.sensor_error);
}
