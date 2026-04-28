#include "lcd_status.h"

#include "encoder.h"
#include "icm20948.h"
#include "st7789.h"
#include "zdt_stepper.h"

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
#define LCD_STATUS_CHAR_W              8U

#define LCD_STATUS_TIMER_TEXT_LEN      5U
#define LCD_STATUS_TIMER_X             (ST7789_WIDTH - 8U - (LCD_STATUS_TIMER_TEXT_LEN * LCD_STATUS_CHAR_W))
#define LCD_STATUS_TIMER_Y             4U

#define LCD_STATUS_UART_X              8U
#define LCD_STATUS_UART_Y              28U
#define LCD_STATUS_UART_TEXT_X         40U
#define LCD_STATUS_UART_WIDTH          (ST7789_WIDTH - LCD_STATUS_UART_TEXT_X - 8U)
#define LCD_STATUS_UART_COLUMNS        (LCD_STATUS_UART_WIDTH / LCD_STATUS_CHAR_W)
#define LCD_STATUS_UART_PROMPT         "RX:"

#define LCD_STATUS_TOP_Y               50U
#define LCD_STATUS_TOP_HEIGHT          48U
#define LCD_STATUS_TOP_SPLIT_X         160U

#define LCD_STATUS_ENCODER_HEADER_Y    50U
#define LCD_STATUS_ENCODER_Y           66U
#define LCD_STATUS_ENCODER_LINE_H      16U
#define LCD_STATUS_ENCODER_DRAW_MS     200U
#define LCD_STATUS_ENCODER_COUNT_MAX   99999L
#define LCD_STATUS_ENCODER_SPEED_MAX   9999L

#define LCD_STATUS_IMU_DRAW_MS         200U
#define LCD_STATUS_IMU_X               168U
#define LCD_STATUS_IMU_Y               50U
#define LCD_STATUS_IMU_VALUE_X         184U
#define LCD_STATUS_IMU_LINE_H          16U

#define LCD_STATUS_BOTTOM_Y            100U
#define LCD_STATUS_BOTTOM_HEIGHT       70U
#define LCD_STATUS_BOTTOM_SPLIT_X      224U

#define LCD_STATUS_K230_HEADER_X       8U
#define LCD_STATUS_K230_HEADER_Y       104U
#define LCD_STATUS_K230_LINE1_Y        122U
#define LCD_STATUS_K230_LINE2_Y        138U
#define LCD_STATUS_K230_LINE3_Y        154U
#define LCD_STATUS_K230_X_MAX          999U
#define LCD_STATUS_K230_Y_MAX          999U
#define LCD_STATUS_K230_ERR_MAX        999
#define LCD_STATUS_K230_DRAW_MS        60U

#define LCD_STATUS_STEPPER_DRAW_MS     200U
#define LCD_STATUS_STEPPER_HEADER_X    232U
#define LCD_STATUS_STEPPER_HEADER_Y    104U
#define LCD_STATUS_STEPPER_Y           122U
#define LCD_STATUS_STEPPER_LINE_H      16U

static uint32_t g_lcd_status_last_second;
static uint32_t g_lcd_status_last_encoder_draw_ms;
static int32_t g_lcd_status_left_count;
static int32_t g_lcd_status_right_count;
static int32_t g_lcd_status_left_speed;
static int32_t g_lcd_status_right_speed;

static uint32_t g_lcd_status_last_stepper_draw_ms;
static int16_t g_lcd_status_stepper_1_speed;
static int16_t g_lcd_status_stepper_2_speed;

static bool g_lcd_status_k230_valid;
static uint16_t g_lcd_status_k230_cx;
static uint16_t g_lcd_status_k230_cy;
static int16_t g_lcd_status_k230_err_x;
static int16_t g_lcd_status_k230_err_y;
static bool g_lcd_status_k230_dirty;
static uint32_t g_lcd_status_last_k230_draw_ms;

static bool g_lcd_status_imu_ready;
static uint8_t g_lcd_status_imu_error;
static uint32_t g_lcd_status_imu_last_draw_ms;

static char g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS + 1U];
static uint8_t g_lcd_status_uart_column;
static bool g_lcd_status_uart_dirty;

static void lcd_status_draw_static(void);
static void lcd_status_draw_timer(uint32_t elapsed_ms);
static void lcd_status_draw_encoders(void);
static void lcd_status_draw_encoder_line(uint16_t y, char label,
    int32_t count, int32_t speed);
static void lcd_status_draw_steppers(void);
static void lcd_status_draw_stepper_line(uint16_t y, char label, int16_t speed);
static void lcd_status_draw_k230(void);
static int32_t lcd_status_clip_count(int32_t count);
static int32_t lcd_status_clip_speed(int32_t speed);
static int16_t lcd_status_clip_stepper_speed(int16_t speed);
static uint16_t lcd_status_clip_k230_coord(uint16_t value, uint16_t max_value);
static int16_t lcd_status_clip_k230_error(int16_t value);
static void lcd_status_clear_uart_line(void);
static void lcd_status_redraw_uart(void);
static void lcd_status_imu_task(uint32_t now_ms);
static void lcd_status_imu_draw_current(void);
static void lcd_status_imu_draw_angles(void);
static void lcd_status_imu_draw_error(uint8_t error_code);
static void lcd_status_clear_imu_area(void);
static void lcd_status_draw_ascii(uint16_t x, uint16_t y, const char *text,
    uint16_t color, uint16_t bg_color);

void lcd_status_screen_init(uint32_t now_ms)
{
    g_lcd_status_last_second = now_ms / 1000U;
    g_lcd_status_last_encoder_draw_ms = now_ms;
    g_lcd_status_left_count = lcd_status_clip_count(Encoder_GetCount(ENCODER_LEFT));
    g_lcd_status_right_count = lcd_status_clip_count(Encoder_GetCount(ENCODER_RIGHT));
    g_lcd_status_left_speed = lcd_status_clip_speed(Encoder_GetSpeedPps(ENCODER_LEFT));
    g_lcd_status_right_speed = lcd_status_clip_speed(Encoder_GetSpeedPps(ENCODER_RIGHT));

    g_lcd_status_last_stepper_draw_ms = now_ms;
    g_lcd_status_stepper_1_speed =
        lcd_status_clip_stepper_speed(ZdtStepper_GetTargetSpeedRpm(ZDT_STEPPER_1));
    g_lcd_status_stepper_2_speed =
        lcd_status_clip_stepper_speed(ZdtStepper_GetTargetSpeedRpm(ZDT_STEPPER_2));

    g_lcd_status_k230_valid = false;
    g_lcd_status_k230_cx = 0U;
    g_lcd_status_k230_cy = 0U;
    g_lcd_status_k230_err_x = 0;
    g_lcd_status_k230_err_y = 0;
    g_lcd_status_k230_dirty = false;
    g_lcd_status_last_k230_draw_ms = now_ms;

    g_lcd_status_imu_ready = false;
    g_lcd_status_imu_error = 0U;
    g_lcd_status_imu_last_draw_ms = now_ms;

    ST7789_Init();
    lcd_status_draw_static();
    lcd_status_clear_uart_line();
    lcd_status_draw_timer(now_ms);
    lcd_status_draw_encoders();
    lcd_status_draw_steppers();
    lcd_status_draw_k230();
    lcd_status_redraw_uart();
    g_lcd_status_imu_ready = ICM20948_IsReady();
    g_lcd_status_imu_error = ICM20948_GetLastError();
    lcd_status_imu_draw_current();
}

void lcd_status_screen_task(uint32_t now_ms)
{
    uint32_t current_second = now_ms / 1000U;

    if (current_second != g_lcd_status_last_second) {
        g_lcd_status_last_second = current_second;
        lcd_status_draw_timer(now_ms);
    }

    if (g_lcd_status_k230_dirty &&
        ((uint32_t) (now_ms - g_lcd_status_last_k230_draw_ms) >=
            LCD_STATUS_K230_DRAW_MS)) {
        g_lcd_status_k230_dirty = false;
        g_lcd_status_last_k230_draw_ms = now_ms;
        lcd_status_draw_k230();
    }

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

    if ((uint32_t) (now_ms - g_lcd_status_last_stepper_draw_ms) >=
        LCD_STATUS_STEPPER_DRAW_MS) {
        int16_t stepper_1_speed = lcd_status_clip_stepper_speed(
            ZdtStepper_GetTargetSpeedRpm(ZDT_STEPPER_1));
        int16_t stepper_2_speed = lcd_status_clip_stepper_speed(
            ZdtStepper_GetTargetSpeedRpm(ZDT_STEPPER_2));

        g_lcd_status_last_stepper_draw_ms = now_ms;

        if ((stepper_1_speed != g_lcd_status_stepper_1_speed) ||
            (stepper_2_speed != g_lcd_status_stepper_2_speed)) {
            g_lcd_status_stepper_1_speed = stepper_1_speed;
            g_lcd_status_stepper_2_speed = stepper_2_speed;
            lcd_status_draw_steppers();
        }
    }

    if (g_lcd_status_uart_dirty) {
        g_lcd_status_uart_dirty = false;
        lcd_status_redraw_uart();
    }

    lcd_status_imu_task(now_ms);
}

void lcd_status_screen_uart_put(uint8_t data)
{
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

void lcd_status_screen_set_k230(
    uint8_t valid, uint16_t cx, uint16_t cy, int16_t err_x, int16_t err_y)
{
    bool k230_valid = (valid != 0U);
    uint16_t k230_cx = lcd_status_clip_k230_coord(cx, LCD_STATUS_K230_X_MAX);
    uint16_t k230_cy = lcd_status_clip_k230_coord(cy, LCD_STATUS_K230_Y_MAX);
    int16_t k230_err_x = lcd_status_clip_k230_error(err_x);
    int16_t k230_err_y = lcd_status_clip_k230_error(err_y);

    if ((k230_valid == g_lcd_status_k230_valid) &&
        (k230_cx == g_lcd_status_k230_cx) &&
        (k230_cy == g_lcd_status_k230_cy) &&
        (k230_err_x == g_lcd_status_k230_err_x) &&
        (k230_err_y == g_lcd_status_k230_err_y)) {
        return;
    }

    g_lcd_status_k230_valid = k230_valid;
    g_lcd_status_k230_cx = k230_cx;
    g_lcd_status_k230_cy = k230_cy;
    g_lcd_status_k230_err_x = k230_err_x;
    g_lcd_status_k230_err_y = k230_err_y;
    g_lcd_status_k230_dirty = true;
}

static void lcd_status_draw_static(void)
{
    ST7789_Clear(LCD_STATUS_BG);
    ST7789_FillRect(0U, 0U, ST7789_WIDTH, 24U, LCD_STATUS_HEADER);
    ST7789_FillRect(0U, 24U, ST7789_WIDTH, 22U, LCD_STATUS_PANEL);
    ST7789_FillRect(0U, LCD_STATUS_TOP_Y, ST7789_WIDTH, LCD_STATUS_TOP_HEIGHT,
        LCD_STATUS_PANEL);
    ST7789_FillRect(0U, LCD_STATUS_BOTTOM_Y, ST7789_WIDTH,
        LCD_STATUS_BOTTOM_HEIGHT, LCD_STATUS_PANEL);

    ST7789_ShowString(8U, 4U, "TFT STATUS", LCD_STATUS_FONT,
        LCD_STATUS_TEXT, LCD_STATUS_HEADER);
    ST7789_ShowString(LCD_STATUS_UART_X, LCD_STATUS_UART_Y,
        LCD_STATUS_UART_PROMPT, LCD_STATUS_FONT, LCD_STATUS_LABEL,
        LCD_STATUS_PANEL);

    ST7789_ShowString(8U, LCD_STATUS_ENCODER_HEADER_Y, "ENC",
        LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(40U, LCD_STATUS_ENCODER_HEADER_Y, "CNT",
        LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(104U, LCD_STATUS_ENCODER_HEADER_Y, "SPD",
        LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);

    ST7789_ShowString(LCD_STATUS_K230_HEADER_X, LCD_STATUS_K230_HEADER_Y,
        "K230", LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(LCD_STATUS_STEPPER_HEADER_X, LCD_STATUS_STEPPER_HEADER_Y,
        "RPM", LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(8U, LCD_STATUS_K230_LINE1_Y, "V:", LCD_STATUS_FONT,
        LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(8U, LCD_STATUS_K230_LINE2_Y, "X:", LCD_STATUS_FONT,
        LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_ShowString(80U, LCD_STATUS_K230_LINE2_Y, "Y:", LCD_STATUS_FONT,
        LCD_STATUS_LABEL, LCD_STATUS_PANEL);
    ST7789_DrawLine(0U, 48U, (uint16_t) (ST7789_WIDTH - 1U), 48U,
        LCD_STATUS_GRID);
    ST7789_DrawLine(0U, 98U, (uint16_t) (ST7789_WIDTH - 1U), 98U,
        LCD_STATUS_GRID);
    ST7789_DrawLine(LCD_STATUS_TOP_SPLIT_X, LCD_STATUS_TOP_Y,
        LCD_STATUS_TOP_SPLIT_X,
        (uint16_t) (LCD_STATUS_TOP_Y + LCD_STATUS_TOP_HEIGHT - 1U),
        LCD_STATUS_GRID);
    ST7789_DrawLine(LCD_STATUS_BOTTOM_SPLIT_X, LCD_STATUS_BOTTOM_Y,
        LCD_STATUS_BOTTOM_SPLIT_X,
        (uint16_t) (LCD_STATUS_BOTTOM_Y + LCD_STATUS_BOTTOM_HEIGHT - 1U),
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

    lcd_status_draw_ascii(LCD_STATUS_TIMER_X, LCD_STATUS_TIMER_Y, timer_text,
        LCD_STATUS_TEXT, LCD_STATUS_HEADER);
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
    char line[20];

    (void) snprintf(line, sizeof(line), "%c:%+06ld %+05ld", label,
        (long) count, (long) speed);
    lcd_status_draw_ascii(8U, y, line, LCD_STATUS_VALUE, LCD_STATUS_PANEL);
}

static void lcd_status_draw_steppers(void)
{
    lcd_status_draw_stepper_line(LCD_STATUS_STEPPER_Y, '1',
        g_lcd_status_stepper_1_speed);
    lcd_status_draw_stepper_line(
        (uint16_t) (LCD_STATUS_STEPPER_Y + LCD_STATUS_STEPPER_LINE_H), '2',
        g_lcd_status_stepper_2_speed);
}

static void lcd_status_draw_stepper_line(uint16_t y, char label, int16_t speed)
{
    char line[12];

    (void) snprintf(line, sizeof(line), "%c:%+04d", label, (int) speed);
    lcd_status_draw_ascii(LCD_STATUS_STEPPER_HEADER_X, y, line,
        LCD_STATUS_VALUE, LCD_STATUS_PANEL);
}

static void lcd_status_draw_k230(void)
{
    char line1[10];
    char line2[16];
    uint16_t value_color = g_lcd_status_k230_valid ? LCD_STATUS_VALUE :
                                                     LCD_STATUS_WARN;

    (void) snprintf(line1, sizeof(line1), "V:%1u %-4s",
        g_lcd_status_k230_valid ? 1U : 0U,
        g_lcd_status_k230_valid ? "LOCK" : "LOST");
    (void) snprintf(line2, sizeof(line2), "X:%03u Y:%03u",
        g_lcd_status_k230_cx, g_lcd_status_k230_cy);

    lcd_status_draw_ascii(
        8U, LCD_STATUS_K230_LINE1_Y, line1, value_color, LCD_STATUS_PANEL);
    lcd_status_draw_ascii(8U, LCD_STATUS_K230_LINE2_Y, line2,
        LCD_STATUS_VALUE, LCD_STATUS_PANEL);
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

static int16_t lcd_status_clip_stepper_speed(int16_t speed)
{
    if (speed > (int16_t) ZDT_STEPPER_MAX_RPM) {
        return (int16_t) ZDT_STEPPER_MAX_RPM;
    }
    if (speed < -(int16_t) ZDT_STEPPER_MAX_RPM) {
        return -(int16_t) ZDT_STEPPER_MAX_RPM;
    }
    return speed;
}

static uint16_t lcd_status_clip_k230_coord(uint16_t value, uint16_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t lcd_status_clip_k230_error(int16_t value)
{
    if (value > LCD_STATUS_K230_ERR_MAX) {
        return LCD_STATUS_K230_ERR_MAX;
    }
    if (value < -LCD_STATUS_K230_ERR_MAX) {
        return -LCD_STATUS_K230_ERR_MAX;
    }
    return value;
}

static void lcd_status_clear_uart_line(void)
{
    (void) memset(g_lcd_status_uart_line, ' ', LCD_STATUS_UART_COLUMNS);
    g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS] = '\0';
    g_lcd_status_uart_column = 0U;
    g_lcd_status_uart_dirty = true;
}

static void lcd_status_redraw_uart(void)
{
    lcd_status_draw_ascii(LCD_STATUS_UART_TEXT_X, LCD_STATUS_UART_Y,
        g_lcd_status_uart_line, LCD_STATUS_TEXT, LCD_STATUS_PANEL);
}

static void lcd_status_imu_task(uint32_t now_ms)
{
    bool ready = ICM20948_IsReady();
    uint8_t error_code = ICM20948_GetLastError();

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
    char line[12];
    ICM20948_Angle_t angle = ICM20948_GetAngle();

    (void) snprintf(line, sizeof(line), "R:%+06.1f", angle.roll);
    lcd_status_draw_ascii(
        LCD_STATUS_IMU_X, LCD_STATUS_IMU_Y, line, LCD_STATUS_VALUE, LCD_STATUS_PANEL);

    (void) snprintf(line, sizeof(line), "P:%+06.1f", angle.pitch);
    lcd_status_draw_ascii(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H), line,
        LCD_STATUS_VALUE, LCD_STATUS_PANEL);

    (void) snprintf(line, sizeof(line), "Y:%+06.1f", angle.yaw);
    lcd_status_draw_ascii(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H * 2U), line,
        LCD_STATUS_VALUE, LCD_STATUS_PANEL);
}

static void lcd_status_imu_draw_error(uint8_t error_code)
{
    char line[12];

    lcd_status_clear_imu_area();

    lcd_status_draw_ascii(LCD_STATUS_IMU_X, LCD_STATUS_IMU_Y, "IMU ERR",
        LCD_STATUS_WARN, LCD_STATUS_PANEL);
    (void) snprintf(line, sizeof(line), "ICM:%02u", error_code);
    lcd_status_draw_ascii(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H), line,
        LCD_STATUS_WARN, LCD_STATUS_PANEL);
    lcd_status_draw_ascii(LCD_STATUS_IMU_X,
        (uint16_t) (LCD_STATUS_IMU_Y + LCD_STATUS_IMU_LINE_H * 2U), "CHK I2C",
        LCD_STATUS_WARN, LCD_STATUS_PANEL);
}

static void lcd_status_clear_imu_area(void)
{
    ST7789_FillRect(LCD_STATUS_TOP_SPLIT_X, LCD_STATUS_TOP_Y,
        (uint16_t) (ST7789_WIDTH - LCD_STATUS_TOP_SPLIT_X),
        LCD_STATUS_TOP_HEIGHT, LCD_STATUS_PANEL);
}

static void lcd_status_draw_ascii(uint16_t x, uint16_t y, const char *text,
    uint16_t color, uint16_t bg_color)
{
    ST7789_ShowAsciiStringFast(
        x, y, text, LCD_STATUS_FONT, color, bg_color);
}
