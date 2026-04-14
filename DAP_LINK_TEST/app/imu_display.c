#include "imu_display.h"

#include "icm20948.h"
#include "oled.h"

#include <stdbool.h>

#define IMU_DISPLAY_UPDATE_MS          10U
#define IMU_DISPLAY_DRAW_MS            50U
#define IMU_DISPLAY_RETRY_MS           1000U

#define IMU_DISPLAY_ROLL_Y             24U
#define IMU_DISPLAY_PITCH_Y            32U
#define IMU_DISPLAY_YAW_Y              40U
#define IMU_DISPLAY_LINE_HEIGHT        8U
#define IMU_DISPLAY_TEXT_LEN           8U
#define IMU_DISPLAY_X                  (OLED_WIDTH - (IMU_DISPLAY_TEXT_LEN * OLED_6X8))
#define IMU_DISPLAY_WIDTH              (IMU_DISPLAY_TEXT_LEN * OLED_6X8)
#define IMU_DISPLAY_VALUE_X            (IMU_DISPLAY_X + 12U)
#define IMU_DISPLAY_FONT               OLED_6X8

static bool g_imu_display_ready;
static bool g_imu_display_skip_first_update;
static uint8_t g_imu_display_init_error;
static uint32_t g_imu_display_last_update_ms;
static uint32_t g_imu_display_last_draw_ms;
static uint32_t g_imu_display_next_retry_ms;

static void imu_display_try_init(uint32_t now_ms);
static void imu_display_draw_angles(void);
static void imu_display_draw_error(uint8_t error_code);
static void imu_display_clear_rows(void);

void imu_display_init(uint32_t now_ms)
{
    g_imu_display_ready = false;
    g_imu_display_skip_first_update = false;
    g_imu_display_init_error = 0U;
    g_imu_display_last_update_ms = now_ms;
    g_imu_display_last_draw_ms = now_ms;
    g_imu_display_next_retry_ms = now_ms;

    imu_display_try_init(now_ms);
}

void imu_display_task(uint32_t now_ms)
{
    if (!g_imu_display_ready) {
        if ((uint32_t) (now_ms - g_imu_display_next_retry_ms) <
            IMU_DISPLAY_RETRY_MS) {
            return;
        }
        imu_display_try_init(now_ms);
        return;
    }

    if ((uint32_t) (now_ms - g_imu_display_last_update_ms) >=
        IMU_DISPLAY_UPDATE_MS) {
        if (g_imu_display_skip_first_update) {
            g_imu_display_skip_first_update = false;
            g_imu_display_last_update_ms = now_ms;
        } else {
            float dt =
                (float) (now_ms - g_imu_display_last_update_ms) / 1000.0f;

            g_imu_display_last_update_ms = now_ms;
            ICM20948_UpdateAngle(dt);
        }
    }

    if ((uint32_t) (now_ms - g_imu_display_last_draw_ms) >=
        IMU_DISPLAY_DRAW_MS) {
        g_imu_display_last_draw_ms = now_ms;
        imu_display_draw_angles();
    }
}

static void imu_display_try_init(uint32_t now_ms)
{
    g_imu_display_init_error = ICM20948_Init();
    g_imu_display_next_retry_ms = now_ms;

    if (g_imu_display_init_error == 0U) {
        g_imu_display_ready = true;
        g_imu_display_skip_first_update = true;
        g_imu_display_last_update_ms = now_ms;
        g_imu_display_last_draw_ms = now_ms;
        imu_display_draw_angles();
    } else {
        g_imu_display_ready = false;
        imu_display_draw_error(g_imu_display_init_error);
    }
}

static void imu_display_draw_angles(void)
{
    ICM20948_Angle_t angle = ICM20948_GetAngle();

    imu_display_clear_rows();

    OLED_ShowString(IMU_DISPLAY_X, IMU_DISPLAY_ROLL_Y, "R:",
        IMU_DISPLAY_FONT);
    OLED_ShowFloatNum(IMU_DISPLAY_VALUE_X, IMU_DISPLAY_ROLL_Y, angle.roll, 3U,
        1U, IMU_DISPLAY_FONT);

    OLED_ShowString(IMU_DISPLAY_X, IMU_DISPLAY_PITCH_Y, "P:",
        IMU_DISPLAY_FONT);
    OLED_ShowFloatNum(IMU_DISPLAY_VALUE_X, IMU_DISPLAY_PITCH_Y, angle.pitch,
        3U, 1U, IMU_DISPLAY_FONT);

    OLED_ShowString(IMU_DISPLAY_X, IMU_DISPLAY_YAW_Y, "Y:",
        IMU_DISPLAY_FONT);
    OLED_ShowFloatNum(IMU_DISPLAY_VALUE_X, IMU_DISPLAY_YAW_Y, angle.yaw, 3U,
        1U, IMU_DISPLAY_FONT);

    OLED_UpdateArea(IMU_DISPLAY_X, IMU_DISPLAY_ROLL_Y, IMU_DISPLAY_WIDTH,
        (uint8_t) (IMU_DISPLAY_LINE_HEIGHT * 3U));
}

static void imu_display_draw_error(uint8_t error_code)
{
    imu_display_clear_rows();

    OLED_ShowString(IMU_DISPLAY_X, IMU_DISPLAY_ROLL_Y, "ICM:",
        IMU_DISPLAY_FONT);
    OLED_ShowNum((uint8_t) (IMU_DISPLAY_X + 24U), IMU_DISPLAY_ROLL_Y,
        error_code, 2U, IMU_DISPLAY_FONT);
    OLED_ShowString(IMU_DISPLAY_X, IMU_DISPLAY_PITCH_Y, "ADDR68",
        IMU_DISPLAY_FONT);
    OLED_ShowString(IMU_DISPLAY_X, IMU_DISPLAY_YAW_Y, "CHK I2C",
        IMU_DISPLAY_FONT);

    OLED_UpdateArea(IMU_DISPLAY_X, IMU_DISPLAY_ROLL_Y, IMU_DISPLAY_WIDTH,
        (uint8_t) (IMU_DISPLAY_LINE_HEIGHT * 3U));
}

static void imu_display_clear_rows(void)
{
    OLED_ClearArea(IMU_DISPLAY_X, IMU_DISPLAY_ROLL_Y, IMU_DISPLAY_WIDTH,
        (uint8_t) (IMU_DISPLAY_LINE_HEIGHT * 3U));
}
