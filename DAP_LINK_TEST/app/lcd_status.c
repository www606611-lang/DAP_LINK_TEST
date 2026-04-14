#include "lcd_status.h"

#include "encoder.h"
#include "icm20948.h"
#include "st7789.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define LCD_STATUS_BG                  ST7789_COLOR_BLACK
#define LCD_STATUS_PANEL               ST7789_RGB565(8U, 24U, 32U)
#define LCD_STATUS_HEADER              ST7789_RGB565(0U, 84U, 120U)
#define LCD_STATUS_GRID                ST7789_RGB565(24U, 64U, 72U)
#define LCD_STATUS_LABEL               ST7789_COLOR_CYAN
#define LCD_STATUS_TEXT                ST7789_COLOR_WHITE
#define LCD_STATUS_VALUE               ST7789_COLOR_YELLOW
#define LCD_STATUS_WARN                ST7789_COLOR_RED

#define LCD_STATUS_FONT                ST7789_8X16
#define LCD_STATUS_SMALL_FONT          ST7789_6X8
#define LCD_STATUS_CHAR_W              8U
#define LCD_STATUS_CHAR_H              16U

#define LCD_STATUS_TIMER_TEXT_LEN      5U
#define LCD_STATUS_TIMER_X             (ST7789_WIDTH - 8U - (LCD_STATUS_TIMER_TEXT_LEN * LCD_STATUS_CHAR_W))
#define LCD_STATUS_TIMER_Y             4U

#define LCD_STATUS_UART_X              8U
#define LCD_STATUS_UART_Y              28U
#define LCD_STATUS_UART_TEXT_X         40U
#define LCD_STATUS_UART_WIDTH          (ST7789_WIDTH - LCD_STATUS_UART_TEXT_X - 8U)
#define LCD_STATUS_UART_COLUMNS        (LCD_STATUS_UART_WIDTH / LCD_STATUS_CHAR_W)
#define LCD_STATUS_UART_PROMPT         "RX:"

#define LCD_STATUS_ENCODER_HEADER_Y    52U
#define LCD_STATUS_ENCODER_Y           62U
#define LCD_STATUS_ENCODER_LINE_H      20U
#define LCD_STATUS_ENCODER_DRAW_MS     100U
#define LCD_STATUS_ENCODER_COUNT_MAX   99999L
#define LCD_STATUS_ENCODER_SPEED_MAX   99999L

#define LCD_STATUS_IMU_DRAW_MS         50U
#define LCD_STATUS_IMU_X               8U
#define LCD_STATUS_IMU_Y               104U
#define LCD_STATUS_IMU_VALUE_X         32U
#define LCD_STATUS_IMU_LINE_H          20U

/* LCD 只在数值变化时局部刷新，避免全屏清屏导致画面闪烁。 */
static uint32_t g_lcd_status_last_second;
static uint32_t g_lcd_status_last_encoder_draw_ms;
static int32_t g_lcd_status_left_count;
static int32_t g_lcd_status_right_count;
static int32_t g_lcd_status_left_speed;
static int32_t g_lcd_status_right_speed;
static char g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS + 1U];
static uint8_t g_lcd_status_uart_column;
static bool g_lcd_status_uart_dirty;

static bool g_lcd_status_imu_ready;
static uint8_t g_lcd_status_imu_error;
static uint32_t g_lcd_status_imu_last_draw_ms;

static void lcd_status_draw_static(void);
static void lcd_status_draw_timer(uint32_t elapsed_ms);
static void lcd_status_draw_encoders(void);
static void lcd_status_draw_encoder_line(uint16_t y, char label,
    int32_t count, int32_t speed);
static int32_t lcd_status_clip_count(int32_t count);
static int32_t lcd_status_clip_speed(int32_t speed);
static void lcd_status_clear_uart_line(void);
static void lcd_status_redraw_uart(void);
static void lcd_status_imu_task(uint32_t now_ms);
static void lcd_status_imu_draw_current(void);
static void lcd_status_imu_draw_angles(void);
static void lcd_status_imu_draw_error(uint8_t error_code);
static void lcd_status_clear_imu_area(void);

void lcd_status_screen_init(uint32_t now_ms)
{
    g_lcd_status_last_second = now_ms / 1000U;
    g_lcd_status_last_encoder_draw_ms = now_ms;
    g_lcd_status_left_count = lcd_status_clip_count(Encoder_GetCount(ENCODER_LEFT));
    g_lcd_status_right_count = lcd_status_clip_count(Encoder_GetCount(ENCODER_RIGHT));
    g_lcd_status_left_speed = lcd_status_clip_speed(Encoder_GetSpeedPps(ENCODER_LEFT));
    g_lcd_status_right_speed = lcd_status_clip_speed(Encoder_GetSpeedPps(ENCODER_RIGHT));

    g_lcd_status_imu_ready = false;
    g_lcd_status_imu_error = 0U;
    g_lcd_status_imu_last_draw_ms = now_ms;

    ST7789_Init();
    lcd_status_draw_static();
    lcd_status_clear_uart_line();
    lcd_status_draw_timer(now_ms);
    lcd_status_draw_encoders();
    lcd_status_redraw_uart();
    g_lcd_status_imu_ready = ICM20948_IsReady();
    g_lcd_status_imu_error = ICM20948_GetLastError();
    lcd_status_imu_draw_current();
}

void lcd_status_screen_task(uint32_t now_ms)
{
    uint32_t current_second = now_ms / 1000U;

    /* 秒计时区域固定宽度，直接覆写旧字符即可。 */
    if (current_second != g_lcd_status_last_second) {
        g_lcd_status_last_second = current_second;
        lcd_status_draw_timer(now_ms);
    }

    /* 编码器数据变化较快，限频后再比较，没变就不写屏。 */
    if ((uint32_t) (now_ms - g_lcd_status_last_encoder_draw_ms) >=
        LCD_STATUS_ENCODER_DRAW_MS) {
        int32_t left_count =
            lcd_status_clip_count(Encoder_GetCount(ENCODER_LEFT));
        int32_t right_count =
            lcd_status_clip_count(Encoder_GetCount(ENCODER_RIGHT));
        int32_t left_speed =
            lcd_status_clip_speed(Encoder_GetSpeedPps(ENCODER_LEFT));
        int32_t right_speed =
            lcd_status_clip_speed(Encoder_GetSpeedPps(ENCODER_RIGHT));

        g_lcd_status_last_encoder_draw_ms = now_ms;

        if ((left_count != g_lcd_status_left_count) ||
            (right_count != g_lcd_status_right_count) ||
            (left_speed != g_lcd_status_left_speed) ||
            (right_speed != g_lcd_status_right_speed)) {
            g_lcd_status_left_count = left_count;
            g_lcd_status_right_count = right_count;
            g_lcd_status_left_speed = left_speed;
            g_lcd_status_right_speed = right_speed;
            lcd_status_draw_encoders();
        }
    }

    /* 串口收满一帧或空闲超时后才刷新显示。 */
    if (g_lcd_status_uart_dirty) {
        g_lcd_status_uart_dirty = false;
        lcd_status_redraw_uart();
    }

    lcd_status_imu_task(now_ms);
}

void lcd_status_screen_uart_put(uint8_t data)
{
    /* 换行只作为一帧结束标记，不直接显示到状态栏。 */
    if ((data == '\r') || (data == '\n')) {
        return;
    }

    if (g_lcd_status_uart_column >= LCD_STATUS_UART_COLUMNS) {
        return;
    }

    if ((data < ' ') || (data > '~')) {
        data = '.';
    }

    g_lcd_status_uart_line[g_lcd_status_uart_column++] = (char) data;
    g_lcd_status_uart_dirty = true;
}

void lcd_status_screen_uart_write(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == NULL) {
        return;
    }

    lcd_status_clear_uart_line();

    for (i = 0U; i < length; i++) {
        lcd_status_screen_uart_put(data[i]);
    }
}

static void lcd_status_draw_static(void)
{
    /* 背景和分隔线只画一次，后续任务只更新动态字段。 */
    ST7789_Clear(LCD_STATUS_BG);
    ST7789_FillRect(0U, 0U, ST7789_WIDTH, 24U, LCD_STATUS_HEADER);
    ST7789_FillRect(0U, 24U, ST7789_WIDTH, 22U, LCD_STATUS_PANEL);
    ST7789_FillRect(0U, 50U, ST7789_WIDTH, 48U, LCD_STATUS_PANEL);
    ST7789_FillRect(0U, 100U, ST7789_WIDTH, 70U, LCD_STATUS_PANEL);

    ST7789_ShowString(8U, 4U, "TFT STATUS", LCD_STATUS_FONT,
        LCD_STATUS_TEXT, LCD_STATUS_HEADER);
    ST7789_ShowString(LCD_STATUS_UART_X, LCD_STATUS_UART_Y,
        LCD_STATUS_UART_PROMPT, LCD_STATUS_FONT, LCD_STATUS_LABEL,
        LCD_STATUS_PANEL);

    ST7789_ShowString(40U, LCD_STATUS_ENCODER_HEADER_Y, "CNT",
        LCD_STATUS_SMALL_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(136U, LCD_STATUS_ENCODER_HEADER_Y, "SPD",
        LCD_STATUS_SMALL_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_DrawLine(0U, 48U, (uint16_t) (ST7789_WIDTH - 1U), 48U,
        LCD_STATUS_GRID);
    ST7789_DrawLine(0U, 98U, (uint16_t) (ST7789_WIDTH - 1U), 98U,
        LCD_STATUS_GRID);
}

static void lcd_status_draw_timer(uint32_t elapsed_ms)
{
    char timer_text[LCD_STATUS_TIMER_TEXT_LEN + 1U];
    uint32_t elapsed_seconds = elapsed_ms / 1000U;
    uint32_t minutes = (elapsed_seconds / 60U) % 100U;
    uint32_t seconds = elapsed_seconds % 60U;

    (void) snprintf(timer_text, sizeof(timer_text), "%02lu:%02lu",
        (unsigned long) minutes, (unsigned long) seconds);

    ST7789_ShowString(LCD_STATUS_TIMER_X, LCD_STATUS_TIMER_Y, timer_text,
        LCD_STATUS_FONT, LCD_STATUS_TEXT, LCD_STATUS_HEADER);
}

static void lcd_status_draw_encoders(void)
{
    lcd_status_draw_encoder_line(LCD_STATUS_ENCODER_Y, 'L',
        g_lcd_status_left_count, g_lcd_status_left_speed);
    lcd_status_draw_encoder_line(
        (uint16_t) (LCD_STATUS_ENCODER_Y + LCD_STATUS_ENCODER_LINE_H), 'R',
        g_lcd_status_right_count, g_lcd_status_right_speed);
}

static void lcd_status_draw_encoder_line(uint16_t y, char label, int32_t count,
    int32_t speed)
{
    ST7789_ShowChar(8U, y, label, LCD_STATUS_FONT, LCD_STATUS_LABEL,
        LCD_STATUS_PANEL);
    ST7789_ShowString(24U, y, ":", LCD_STATUS_FONT, LCD_STATUS_LABEL,
        LCD_STATUS_PANEL);
    ST7789_ShowSignedNum(40U, y, count, 5U, LCD_STATUS_FONT,
        LCD_STATUS_VALUE, LCD_STATUS_PANEL);
    ST7789_ShowString(104U, y, "pps", LCD_STATUS_FONT, LCD_STATUS_LABEL,
        LCD_STATUS_PANEL);
    ST7789_ShowSignedNum(136U, y, speed, 5U, LCD_STATUS_FONT,
        LCD_STATUS_TEXT, LCD_STATUS_PANEL);
}

static int32_t lcd_status_clip_count(int32_t count)
{
    if (count > LCD_STATUS_ENCODER_COUNT_MAX) {
        return LCD_STATUS_ENCODER_COUNT_MAX;
    }
    if (count < -LCD_STATUS_ENCODER_COUNT_MAX) {
        return -LCD_STATUS_ENCODER_COUNT_MAX;
    }
    return count;
}

static int32_t lcd_status_clip_speed(int32_t speed)
{
    if (speed > LCD_STATUS_ENCODER_SPEED_MAX) {
        return LCD_STATUS_ENCODER_SPEED_MAX;
    }
    if (speed < -LCD_STATUS_ENCODER_SPEED_MAX) {
        return -LCD_STATUS_ENCODER_SPEED_MAX;
    }
    return speed;
}

static void lcd_status_clear_uart_line(void)
{
    /* 用空格填满整行，下一次 ShowString 可以完整覆盖旧内容。 */
    (void) memset(g_lcd_status_uart_line, ' ', LCD_STATUS_UART_COLUMNS);
    g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS] = '\0';
    g_lcd_status_uart_column = 0U;
    g_lcd_status_uart_dirty = true;
}

static void lcd_status_redraw_uart(void)
{
    ST7789_ShowString(LCD_STATUS_UART_TEXT_X, LCD_STATUS_UART_Y,
        g_lcd_status_uart_line, LCD_STATUS_FONT, LCD_STATUS_TEXT,
        LCD_STATUS_PANEL);
}

static void lcd_status_imu_task(uint32_t now_ms)
{
    bool ready = ICM20948_IsReady();
    uint8_t error_code = ICM20948_GetLastError();

    /* 姿态解算在 ICM20948_Task() 中完成，这里只负责把状态画到 LCD。 */
    if ((ready != g_lcd_status_imu_ready) ||
        ((!ready) && (error_code != g_lcd_status_imu_error))) {
        g_lcd_status_imu_ready = ready;
        g_lcd_status_imu_error = error_code;
        g_lcd_status_imu_last_draw_ms = now_ms;
        lcd_status_imu_draw_current();
        return;
    }

    if (!g_lcd_status_imu_ready) {
        return;
    }

    if ((uint32_t) (now_ms - g_lcd_status_imu_last_draw_ms) >=
        LCD_STATUS_IMU_DRAW_MS) {
        g_lcd_status_imu_last_draw_ms = now_ms;
        lcd_status_imu_draw_angles();
    }
}

static void lcd_status_imu_draw_current(void)
{
    if (g_lcd_status_imu_ready) {
        lcd_status_clear_imu_area();
        lcd_status_imu_draw_angles();
    } else {
        lcd_status_imu_draw_error(g_lcd_status_imu_error);
    }
}

static void lcd_status_imu_draw_angles(void)
{
    ICM20948_Angle_t angle = ICM20948_GetAngle();

    ST7789_ShowString(LCD_STATUS_IMU_X, LCD_STATUS_IMU_Y, "R:",
        LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowFloatNum(LCD_STATUS_IMU_VALUE_X, LCD_STATUS_IMU_Y, angle.roll,
        3U, 1U, LCD_STATUS_FONT, LCD_STATUS_VALUE, LCD_STATUS_PANEL);

    ST7789_ShowString(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H), "P:",
        LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowFloatNum(LCD_STATUS_IMU_VALUE_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H), angle.pitch,
        3U, 1U, LCD_STATUS_FONT, LCD_STATUS_VALUE, LCD_STATUS_PANEL);

    ST7789_ShowString(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H * 2U), "Y:",
        LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowFloatNum(LCD_STATUS_IMU_VALUE_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H * 2U), angle.yaw,
        3U, 1U, LCD_STATUS_FONT, LCD_STATUS_VALUE, LCD_STATUS_PANEL);
}

static void lcd_status_imu_draw_error(uint8_t error_code)
{
    lcd_status_clear_imu_area();

    ST7789_ShowString(LCD_STATUS_IMU_X, LCD_STATUS_IMU_Y, "ICM:",
        LCD_STATUS_FONT, LCD_STATUS_WARN, LCD_STATUS_PANEL);
    ST7789_ShowNum((uint16_t) (LCD_STATUS_IMU_X + 32U), LCD_STATUS_IMU_Y,
        error_code, 2U, LCD_STATUS_FONT, LCD_STATUS_WARN, LCD_STATUS_PANEL);
    ST7789_ShowString(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H), "ADDR68",
        LCD_STATUS_FONT, LCD_STATUS_WARN, LCD_STATUS_PANEL);
    ST7789_ShowString(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H * 2U), "CHK I2C",
        LCD_STATUS_FONT, LCD_STATUS_WARN, LCD_STATUS_PANEL);
}

static void lcd_status_clear_imu_area(void)
{
    ST7789_FillRect(0U, 100U, ST7789_WIDTH, 70U, LCD_STATUS_PANEL);
}
