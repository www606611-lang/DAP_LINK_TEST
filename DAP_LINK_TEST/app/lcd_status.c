#include "lcd_status.h"

#include "icm20948.h"
#include "st7789.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
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

#define LCD_STATUS_K230_TITLE_X        8U
#define LCD_STATUS_K230_LINE1_X        8U
#define LCD_STATUS_K230_LINE1_Y        66U
#define LCD_STATUS_K230_LINE2_X        8U
#define LCD_STATUS_K230_LINE2_Y        82U
#define LCD_STATUS_K230_TEXT_LEN       16U
#define LCD_STATUS_K230_X_MAX          999U
#define LCD_STATUS_K230_Y_MAX          999U
#define LCD_STATUS_K230_ERR_MAX        999

#define LCD_STATUS_IMU_LINE_X          168U
#define LCD_STATUS_IMU_LINE1_Y         50U
#define LCD_STATUS_IMU_LINE2_Y         66U
#define LCD_STATUS_IMU_LINE3_Y         82U
#define LCD_STATUS_IMU_DRAW_MS         100U
#define LCD_STATUS_IMU_TEXT_LEN        11U

#define LCD_STATUS_BOTTOM_Y            100U
#define LCD_STATUS_BOTTOM_HEIGHT       70U
#define LCD_STATUS_STEPPER_TITLE_X     8U
#define LCD_STATUS_STEPPER_TEXT_X      8U
#define LCD_STATUS_STEPPER_TEXT_Y      122U
#define LCD_STATUS_STEPPER_DRAW_MS     120U
#define LCD_STATUS_STEPPER_TEXT_LEN    22U

#define LCD_STATUS_TIMER_DRAW_MS       1000U

typedef enum {
    LCD_STATUS_ITEM_TIMER = 0,
    LCD_STATUS_ITEM_UART,
    LCD_STATUS_ITEM_K230_LINE1,
    LCD_STATUS_ITEM_K230_LINE2,
    LCD_STATUS_ITEM_IMU_LINE1,
    LCD_STATUS_ITEM_IMU_LINE2,
    LCD_STATUS_ITEM_IMU_LINE3,
    LCD_STATUS_ITEM_STEPPER,
    LCD_STATUS_ITEM_COUNT
} lcd_status_item_t;

static uint32_t g_lcd_status_last_second;
static uint32_t g_lcd_status_last_imu_sample_ms;
static uint32_t g_lcd_status_last_stepper_sample_ms;

static bool g_lcd_status_k230_valid;
static uint16_t g_lcd_status_k230_cx;
static uint16_t g_lcd_status_k230_cy;
static int16_t g_lcd_status_k230_err_x;
static int16_t g_lcd_status_k230_err_y;

static bool g_lcd_status_imu_ready;
static uint8_t g_lcd_status_imu_error;

static int16_t g_lcd_status_stepper_1_speed;
static int16_t g_lcd_status_stepper_2_speed;

static char g_lcd_status_timer_text[LCD_STATUS_TIMER_TEXT_LEN + 1U];
static char g_lcd_status_timer_drawn[LCD_STATUS_TIMER_TEXT_LEN + 1U];

static char g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS + 1U];
static char g_lcd_status_uart_drawn[LCD_STATUS_UART_COLUMNS + 1U];
static uint8_t g_lcd_status_uart_column;

static char g_lcd_status_k230_line1[LCD_STATUS_K230_TEXT_LEN + 1U];
static char g_lcd_status_k230_line2[LCD_STATUS_K230_TEXT_LEN + 1U];
static char g_lcd_status_k230_drawn_line1[LCD_STATUS_K230_TEXT_LEN + 1U];
static char g_lcd_status_k230_drawn_line2[LCD_STATUS_K230_TEXT_LEN + 1U];

static char g_lcd_status_imu_line1[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_line2[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_line3[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_drawn_line1[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_drawn_line2[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_drawn_line3[LCD_STATUS_IMU_TEXT_LEN + 1U];

static char g_lcd_status_stepper_line[LCD_STATUS_STEPPER_TEXT_LEN + 1U];
static char g_lcd_status_stepper_drawn_line[LCD_STATUS_STEPPER_TEXT_LEN + 1U];

static uint16_t g_lcd_status_dirty_mask;
static uint8_t g_lcd_status_next_item;

static void lcd_status_draw_static(void);
static void lcd_status_update_timer_text(uint32_t elapsed_ms);
static void lcd_status_update_k230_lines(void);
static void lcd_status_update_imu_lines(uint32_t now_ms);
static void lcd_status_update_stepper_line(uint32_t now_ms);
static void lcd_status_draw_next_dirty(void);
static void lcd_status_draw_item(lcd_status_item_t item);
static void lcd_status_mark_dirty(lcd_status_item_t item);
static bool lcd_status_is_dirty(lcd_status_item_t item);
static void lcd_status_clear_dirty(lcd_status_item_t item);
static void lcd_status_format_padded(
    char *buffer, size_t buffer_size, const char *format, ...);
static void lcd_status_format_angle(char axis, float value, char *buffer,
    size_t buffer_size);
static int16_t lcd_status_clip_stepper_speed(int16_t speed);
static void lcd_status_clear_uart_line(void);
static void lcd_status_draw_ascii(uint16_t x, uint16_t y, const char *text,
    uint16_t color, uint16_t bg_color);

void lcd_status_screen_init(uint32_t now_ms)
{
    g_lcd_status_last_second = now_ms / 1000U;
    g_lcd_status_last_imu_sample_ms = now_ms - LCD_STATUS_IMU_DRAW_MS;
    g_lcd_status_last_stepper_sample_ms = now_ms - LCD_STATUS_STEPPER_DRAW_MS;

    g_lcd_status_k230_valid = false;
    g_lcd_status_k230_cx = 0U;
    g_lcd_status_k230_cy = 0U;
    g_lcd_status_k230_err_x = 0;
    g_lcd_status_k230_err_y = 0;

    g_lcd_status_imu_ready = false;
    g_lcd_status_imu_error = 0U;

    g_lcd_status_stepper_1_speed = (int16_t) 0x7FFF;
    g_lcd_status_stepper_2_speed = (int16_t) 0x7FFF;

    (void) memset(g_lcd_status_timer_text, ' ', LCD_STATUS_TIMER_TEXT_LEN);
    g_lcd_status_timer_text[LCD_STATUS_TIMER_TEXT_LEN] = '\0';
    (void) memset(g_lcd_status_timer_drawn, 0, sizeof(g_lcd_status_timer_drawn));

    lcd_status_clear_uart_line();
    (void) memset(g_lcd_status_uart_drawn, 0, sizeof(g_lcd_status_uart_drawn));

    (void) memset(g_lcd_status_k230_drawn_line1, 0, sizeof(g_lcd_status_k230_drawn_line1));
    (void) memset(g_lcd_status_k230_drawn_line2, 0, sizeof(g_lcd_status_k230_drawn_line2));
    (void) memset(g_lcd_status_imu_drawn_line1, 0, sizeof(g_lcd_status_imu_drawn_line1));
    (void) memset(g_lcd_status_imu_drawn_line2, 0, sizeof(g_lcd_status_imu_drawn_line2));
    (void) memset(g_lcd_status_imu_drawn_line3, 0, sizeof(g_lcd_status_imu_drawn_line3));
    (void) memset(g_lcd_status_stepper_drawn_line, 0, sizeof(g_lcd_status_stepper_drawn_line));

    g_lcd_status_dirty_mask = 0U;
    g_lcd_status_next_item = 0U;

    ST7789_Init();
    lcd_status_draw_static();

    lcd_status_update_timer_text(now_ms);
    lcd_status_update_k230_lines();
    lcd_status_update_imu_lines(now_ms);
    lcd_status_update_stepper_line(now_ms);

    lcd_status_draw_item(LCD_STATUS_ITEM_TIMER);
    lcd_status_draw_item(LCD_STATUS_ITEM_UART);
    lcd_status_draw_item(LCD_STATUS_ITEM_K230_LINE1);
    lcd_status_draw_item(LCD_STATUS_ITEM_K230_LINE2);
    lcd_status_draw_item(LCD_STATUS_ITEM_IMU_LINE1);
    lcd_status_draw_item(LCD_STATUS_ITEM_IMU_LINE2);
    lcd_status_draw_item(LCD_STATUS_ITEM_IMU_LINE3);
    lcd_status_draw_item(LCD_STATUS_ITEM_STEPPER);
    g_lcd_status_dirty_mask = 0U;
}

void lcd_status_screen_task(uint32_t now_ms)
{
    uint32_t current_second = now_ms / 1000U;

    if (current_second != g_lcd_status_last_second) {
        g_lcd_status_last_second = current_second;
        lcd_status_update_timer_text(now_ms);
    }

    lcd_status_update_imu_lines(now_ms);
    lcd_status_update_stepper_line(now_ms);
    lcd_status_draw_next_dirty();
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
    lcd_status_mark_dirty(LCD_STATUS_ITEM_UART);
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
    (void) valid;
    (void) cx;
    (void) cy;
    (void) err_x;
    (void) err_y;
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

    ST7789_ShowString(8U, 4U, "TRACK STATUS", LCD_STATUS_FONT,
        LCD_STATUS_TEXT, LCD_STATUS_HEADER);
    ST7789_ShowString(LCD_STATUS_UART_X, LCD_STATUS_UART_Y,
        LCD_STATUS_UART_PROMPT, LCD_STATUS_FONT, LCD_STATUS_LABEL,
        LCD_STATUS_PANEL);
    ST7789_ShowString(LCD_STATUS_STEPPER_TITLE_X, LCD_STATUS_BOTTOM_Y + 4U,
        "STEP RPM", LCD_STATUS_FONT, LCD_STATUS_LABEL, LCD_STATUS_PANEL);

    ST7789_DrawLine(0U, 48U, (uint16_t) (ST7789_WIDTH - 1U), 48U,
        LCD_STATUS_GRID);
    ST7789_DrawLine(0U, 98U, (uint16_t) (ST7789_WIDTH - 1U), 98U,
        LCD_STATUS_GRID);
    ST7789_DrawLine(LCD_STATUS_TOP_SPLIT_X, LCD_STATUS_TOP_Y,
        LCD_STATUS_TOP_SPLIT_X,
        (uint16_t) (LCD_STATUS_TOP_Y + LCD_STATUS_TOP_HEIGHT - 1U),
        LCD_STATUS_GRID);
}

static void lcd_status_update_timer_text(uint32_t elapsed_ms)
{
    uint32_t elapsed_seconds = elapsed_ms / 1000U;
    uint32_t minutes = (elapsed_seconds / 60U) % 100U;
    uint32_t seconds = elapsed_seconds % 60U;

    lcd_status_format_padded(g_lcd_status_timer_text,
        sizeof(g_lcd_status_timer_text), "%02lu:%02lu",
        (unsigned long) minutes, (unsigned long) seconds);

    if (strcmp(g_lcd_status_timer_text, g_lcd_status_timer_drawn) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_TIMER);
    }
}

static void lcd_status_update_k230_lines(void)
{
    lcd_status_format_padded(g_lcd_status_k230_line1,
        sizeof(g_lcd_status_k230_line1), "");
    lcd_status_format_padded(g_lcd_status_k230_line2,
        sizeof(g_lcd_status_k230_line2), "");

    if (strcmp(g_lcd_status_k230_line1, g_lcd_status_k230_drawn_line1) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_K230_LINE1);
    }
    if (strcmp(g_lcd_status_k230_line2, g_lcd_status_k230_drawn_line2) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_K230_LINE2);
    }
}

static void lcd_status_update_imu_lines(uint32_t now_ms)
{
    bool ready;
    uint8_t error_code;

    if ((uint32_t) (now_ms - g_lcd_status_last_imu_sample_ms) <
        LCD_STATUS_IMU_DRAW_MS) {
        return;
    }

    g_lcd_status_last_imu_sample_ms = now_ms;
    ready = ICM20948_IsReady();
    error_code = ICM20948_GetLastError();

    if (ready) {
        ICM20948_Angle_t angle = ICM20948_GetAngle();

        lcd_status_format_angle(
            'R', angle.roll, g_lcd_status_imu_line1, sizeof(g_lcd_status_imu_line1));
        lcd_status_format_angle(
            'P', angle.pitch, g_lcd_status_imu_line2, sizeof(g_lcd_status_imu_line2));
        lcd_status_format_angle(
            'Y', angle.yaw, g_lcd_status_imu_line3, sizeof(g_lcd_status_imu_line3));
    } else {
        lcd_status_format_padded(g_lcd_status_imu_line1,
            sizeof(g_lcd_status_imu_line1), "IMU ERR");
        lcd_status_format_padded(g_lcd_status_imu_line2,
            sizeof(g_lcd_status_imu_line2), "ICM:%02u", error_code);
        lcd_status_format_padded(g_lcd_status_imu_line3,
            sizeof(g_lcd_status_imu_line3), "CHK I2C");
    }

    g_lcd_status_imu_ready = ready;
    g_lcd_status_imu_error = error_code;

    if (strcmp(g_lcd_status_imu_line1, g_lcd_status_imu_drawn_line1) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_IMU_LINE1);
    }
    if (strcmp(g_lcd_status_imu_line2, g_lcd_status_imu_drawn_line2) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_IMU_LINE2);
    }
    if (strcmp(g_lcd_status_imu_line3, g_lcd_status_imu_drawn_line3) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_IMU_LINE3);
    }
}

static void lcd_status_update_stepper_line(uint32_t now_ms)
{
    int16_t stepper_1_speed;
    int16_t stepper_2_speed;

    if ((uint32_t) (now_ms - g_lcd_status_last_stepper_sample_ms) <
        LCD_STATUS_STEPPER_DRAW_MS) {
        return;
    }

    g_lcd_status_last_stepper_sample_ms = now_ms;
    stepper_1_speed = lcd_status_clip_stepper_speed(
        ZdtStepper_GetTargetSpeedRpm(ZDT_STEPPER_1));
    stepper_2_speed = lcd_status_clip_stepper_speed(
        ZdtStepper_GetTargetSpeedRpm(ZDT_STEPPER_2));

    if ((stepper_1_speed == g_lcd_status_stepper_1_speed) &&
        (stepper_2_speed == g_lcd_status_stepper_2_speed)) {
        return;
    }

    g_lcd_status_stepper_1_speed = stepper_1_speed;
    g_lcd_status_stepper_2_speed = stepper_2_speed;

    lcd_status_format_padded(g_lcd_status_stepper_line,
        sizeof(g_lcd_status_stepper_line), "1:%+04d  2:%+04d",
        (int) g_lcd_status_stepper_1_speed, (int) g_lcd_status_stepper_2_speed);

    if (strcmp(g_lcd_status_stepper_line, g_lcd_status_stepper_drawn_line) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_STEPPER);
    }
}

static void lcd_status_draw_next_dirty(void)
{
    uint8_t offset;

    for (offset = 0U; offset < (uint8_t) LCD_STATUS_ITEM_COUNT; offset++) {
        lcd_status_item_t item = (lcd_status_item_t) (
            (g_lcd_status_next_item + offset) % (uint8_t) LCD_STATUS_ITEM_COUNT);

        if (lcd_status_is_dirty(item)) {
            lcd_status_draw_item(item);
            lcd_status_clear_dirty(item);
            g_lcd_status_next_item =
                (uint8_t) ((uint8_t) item + 1U) % (uint8_t) LCD_STATUS_ITEM_COUNT;
            return;
        }
    }
}

static void lcd_status_draw_item(lcd_status_item_t item)
{
    switch (item) {
        case LCD_STATUS_ITEM_TIMER:
            lcd_status_draw_ascii(LCD_STATUS_TIMER_X, LCD_STATUS_TIMER_Y,
                g_lcd_status_timer_text, LCD_STATUS_TEXT, LCD_STATUS_HEADER);
            (void) memcpy(g_lcd_status_timer_drawn, g_lcd_status_timer_text,
                sizeof(g_lcd_status_timer_drawn));
            break;
        case LCD_STATUS_ITEM_UART:
            lcd_status_draw_ascii(LCD_STATUS_UART_TEXT_X, LCD_STATUS_UART_Y,
                g_lcd_status_uart_line, LCD_STATUS_TEXT, LCD_STATUS_PANEL);
            (void) memcpy(g_lcd_status_uart_drawn, g_lcd_status_uart_line,
                sizeof(g_lcd_status_uart_drawn));
            break;
        case LCD_STATUS_ITEM_K230_LINE1:
            lcd_status_draw_ascii(LCD_STATUS_K230_LINE1_X, LCD_STATUS_K230_LINE1_Y,
                g_lcd_status_k230_line1,
                g_lcd_status_k230_valid ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL);
            (void) memcpy(g_lcd_status_k230_drawn_line1, g_lcd_status_k230_line1,
                sizeof(g_lcd_status_k230_drawn_line1));
            break;
        case LCD_STATUS_ITEM_K230_LINE2:
            lcd_status_draw_ascii(LCD_STATUS_K230_LINE2_X, LCD_STATUS_K230_LINE2_Y,
                g_lcd_status_k230_line2, LCD_STATUS_VALUE, LCD_STATUS_PANEL);
            (void) memcpy(g_lcd_status_k230_drawn_line2, g_lcd_status_k230_line2,
                sizeof(g_lcd_status_k230_drawn_line2));
            break;
        case LCD_STATUS_ITEM_IMU_LINE1:
            lcd_status_draw_ascii(LCD_STATUS_IMU_LINE_X, LCD_STATUS_IMU_LINE1_Y,
                g_lcd_status_imu_line1,
                g_lcd_status_imu_ready ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL);
            (void) memcpy(g_lcd_status_imu_drawn_line1, g_lcd_status_imu_line1,
                sizeof(g_lcd_status_imu_drawn_line1));
            break;
        case LCD_STATUS_ITEM_IMU_LINE2:
            lcd_status_draw_ascii(LCD_STATUS_IMU_LINE_X, LCD_STATUS_IMU_LINE2_Y,
                g_lcd_status_imu_line2,
                g_lcd_status_imu_ready ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL);
            (void) memcpy(g_lcd_status_imu_drawn_line2, g_lcd_status_imu_line2,
                sizeof(g_lcd_status_imu_drawn_line2));
            break;
        case LCD_STATUS_ITEM_IMU_LINE3:
            lcd_status_draw_ascii(LCD_STATUS_IMU_LINE_X, LCD_STATUS_IMU_LINE3_Y,
                g_lcd_status_imu_line3,
                g_lcd_status_imu_ready ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL);
            (void) memcpy(g_lcd_status_imu_drawn_line3, g_lcd_status_imu_line3,
                sizeof(g_lcd_status_imu_drawn_line3));
            break;
        case LCD_STATUS_ITEM_STEPPER:
            lcd_status_draw_ascii(LCD_STATUS_STEPPER_TEXT_X, LCD_STATUS_STEPPER_TEXT_Y,
                g_lcd_status_stepper_line, LCD_STATUS_VALUE, LCD_STATUS_PANEL);
            (void) memcpy(
                g_lcd_status_stepper_drawn_line, g_lcd_status_stepper_line,
                sizeof(g_lcd_status_stepper_drawn_line));
            break;
        default:
            break;
    }
}

static void lcd_status_mark_dirty(lcd_status_item_t item)
{
    g_lcd_status_dirty_mask =
        (uint16_t) (g_lcd_status_dirty_mask | (uint16_t) (1U << (uint8_t) item));
}

static bool lcd_status_is_dirty(lcd_status_item_t item)
{
    return ((g_lcd_status_dirty_mask & (uint16_t) (1U << (uint8_t) item)) != 0U);
}

static void lcd_status_clear_dirty(lcd_status_item_t item)
{
    g_lcd_status_dirty_mask =
        (uint16_t) (g_lcd_status_dirty_mask & (uint16_t) ~(1U << (uint8_t) item));
}

static void lcd_status_format_padded(
    char *buffer, size_t buffer_size, const char *format, ...)
{
    va_list args;
    size_t len;

    if ((buffer == NULL) || (buffer_size == 0U) || (format == NULL)) {
        return;
    }

    va_start(args, format);
    (void) vsnprintf(buffer, buffer_size, format, args);
    va_end(args);

    len = strlen(buffer);
    while (len < (buffer_size - 1U)) {
        buffer[len++] = ' ';
    }
    buffer[buffer_size - 1U] = '\0';
}

static void lcd_status_format_angle(char axis, float value, char *buffer,
    size_t buffer_size)
{
    int32_t scaled;
    int32_t abs_scaled;
    char sign;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    if (value >= 0.0f) {
        scaled = (int32_t) (value * 10.0f + 0.5f);
    } else {
        scaled = (int32_t) (value * 10.0f - 0.5f);
    }

    sign = (scaled < 0) ? '-' : '+';
    abs_scaled = (scaled < 0) ? -scaled : scaled;

    lcd_status_format_padded(buffer, buffer_size, "%c:%c%03ld.%01ld", axis,
        sign, (long) (abs_scaled / 10), (long) (abs_scaled % 10));
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

static void lcd_status_clear_uart_line(void)
{
    (void) memset(g_lcd_status_uart_line, ' ', LCD_STATUS_UART_COLUMNS);
    g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS] = '\0';
    g_lcd_status_uart_column = 0U;
    lcd_status_mark_dirty(LCD_STATUS_ITEM_UART);
}

static void lcd_status_draw_ascii(uint16_t x, uint16_t y, const char *text,
    uint16_t color, uint16_t bg_color)
{
    ST7789_ShowAsciiStringFast(
        x, y, text, LCD_STATUS_FONT, color, bg_color);
}
