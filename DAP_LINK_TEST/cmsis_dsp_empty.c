#include "ti_msp_dl_config.h"
#include "timer.h"
#include "encoder_speed_test.h"
#include "encoder.h"
#include "icm20948.h"
#include "lcd_status.h"
#include "motor.h"
#include "uart_display.h"

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

    SYSCFG_DL_init();
    timer_common_init();

    now_ms = timer_common_get_ms();
    ICM20948_TaskInit(now_ms);
    Motor_Init();
    Encoder_Init(now_ms);
    encoder_speed_test_init(now_ms);
    lcd_status_screen_init(now_ms);
    uart_display_init();
}

static void app_task(void)
{
    uint32_t now_ms = timer_common_get_ms();

    Encoder_Task(now_ms);
    ICM20948_Task(now_ms);
    uart_display_task(now_ms);
    lcd_status_screen_task(now_ms);
    encoder_speed_test_task(now_ms);
}
