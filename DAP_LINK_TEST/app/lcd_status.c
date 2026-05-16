#include "lcd_status.h"

#include "encoder.h"
#include "icm20948.h"
#include "st7789.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LCD_STATUS_BG                  ST7789_COLOR_BLACK
#define LCD_STATUS_PANEL_TOP           ST7789_RGB565(10U, 20U, 28U)
#define LCD_STATUS_PANEL_MID           ST7789_RGB565(12U, 18U, 30U)
#define LCD_STATUS_PANEL_BOTTOM        ST7789_RGB565(18U, 20U, 26U)
#define LCD_STATUS_GRID                ST7789_RGB565(28U, 48U, 60U)
#define LCD_STATUS_LABEL               ST7789_COLOR_CYAN
#define LCD_STATUS_TEXT                ST7789_COLOR_WHITE
#define LCD_STATUS_VALUE               ST7789_COLOR_YELLOW
#define LCD_STATUS_WARN                ST7789_COLOR_RED

#define LCD_STATUS_FONT                ST7789_8X16
#define LCD_STATUS_CHAR_W              8U

#define LCD_STATUS_TIMER_TEXT_LEN      5U
#define LCD_STATUS_TIMER_X             (ST7789_WIDTH - 8U - (LCD_STATUS_TIMER_TEXT_LEN * LCD_STATUS_CHAR_W))
#define LCD_STATUS_TIMER_Y             4U

#define LCD_STATUS_UART_Y              0U
#define LCD_STATUS_UART_H              22U
#define LCD_STATUS_UART_X              6U
#define LCD_STATUS_UART_TEXT_X         6U
#define LCD_STATUS_UART_WIDTH          (LCD_STATUS_TIMER_X - LCD_STATUS_UART_TEXT_X - 6U)
#define LCD_STATUS_UART_COLUMNS        (LCD_STATUS_UART_WIDTH / LCD_STATUS_CHAR_W)

#define LCD_STATUS_IMU_Y               26U
#define LCD_STATUS_IMU_H               56U
#define LCD_STATUS_IMU_X               0U
#define LCD_STATUS_IMU_TEXT_X          6U
#define LCD_STATUS_IMU_LINE1_Y         32U
#define LCD_STATUS_IMU_LINE2_Y         48U
#define LCD_STATUS_IMU_LINE3_Y         64U
#define LCD_STATUS_IMU_TEXT_LEN        16U
#define LCD_STATUS_IMU_DRAW_MS         100U

#define LCD_STATUS_ENCODER_Y           90U
#define LCD_STATUS_ENCODER_H           28U
#define LCD_STATUS_ENCODER_X           0U
#define LCD_STATUS_ENCODER_TEXT_X      6U
#define LCD_STATUS_ENCODER_LINE_Y      96U
#define LCD_STATUS_ENCODER_TEXT_LEN    20U
#define LCD_STATUS_ENCODER_DRAW_MS     120U
#define LCD_STATUS_ENCODER_SPEED_MAX   9999L
#define LCD_STATUS_ENCODER_SPEED_SCALE 1L

#define LCD_STATUS_LINE_Y              118U
#define LCD_STATUS_LINE_H              44U
#define LCD_STATUS_LINE_TEXT_X         6U
#define LCD_STATUS_LINE_TEXT_LEN1      16U
#define LCD_STATUS_LINE_TEXT_LEN2      24U
#define LCD_STATUS_LINE_LINE1_Y        124U
#define LCD_STATUS_LINE_LINE2_Y        140U
#define LCD_STATUS_LINE_PANEL          ST7789_RGB565(12U, 22U, 18U)

typedef enum {
    LCD_STATUS_ITEM_TIMER = 0,
    LCD_STATUS_ITEM_UART,
    LCD_STATUS_ITEM_IMU_LINE1,
    LCD_STATUS_ITEM_IMU_LINE2,
    LCD_STATUS_ITEM_IMU_LINE3,
    LCD_STATUS_ITEM_ENCODER,
    LCD_STATUS_ITEM_LINE_SENSOR,
    LCD_STATUS_ITEM_COUNT
} lcd_status_item_t;

static uint32_t g_lcd_status_last_second;
static uint32_t g_lcd_status_last_imu_sample_ms;
static uint32_t g_lcd_status_last_encoder_sample_ms;
static bool g_lcd_status_imu_ready;
static uint8_t g_lcd_status_imu_error;

static char g_lcd_status_timer_text[LCD_STATUS_TIMER_TEXT_LEN + 1U];
static char g_lcd_status_timer_drawn[LCD_STATUS_TIMER_TEXT_LEN + 1U];
static char g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS + 1U];
static char g_lcd_status_uart_drawn[LCD_STATUS_UART_COLUMNS + 1U];
static uint8_t g_lcd_status_uart_column;
static char g_lcd_status_imu_line1[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_line2[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_line3[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_drawn_line1[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_drawn_line2[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_imu_drawn_line3[LCD_STATUS_IMU_TEXT_LEN + 1U];
static char g_lcd_status_encoder_line[LCD_STATUS_ENCODER_TEXT_LEN + 1U];
static char g_lcd_status_encoder_drawn[LCD_STATUS_ENCODER_TEXT_LEN + 1U];
static char g_lcd_status_line_line1[LCD_STATUS_LINE_TEXT_LEN1 + 1U];
static char g_lcd_status_line_line2[LCD_STATUS_LINE_TEXT_LEN2 + 1U];
static char g_lcd_status_line_drawn1[LCD_STATUS_LINE_TEXT_LEN1 + 1U];
static char g_lcd_status_line_drawn2[LCD_STATUS_LINE_TEXT_LEN2 + 1U];
static uint8_t g_lcd_status_line_raw;
static uint8_t g_lcd_status_line_active_mask;
static uint8_t g_lcd_status_line_active_count;
static int16_t g_lcd_status_line_error;
static uint8_t g_lcd_status_line_enabled;
static uint8_t g_lcd_status_line_sensor_ok;
static uint8_t g_lcd_status_line_sensor_error;
static uint16_t g_lcd_status_dirty_mask;
static uint8_t g_lcd_status_next_item;

static void lcd_status_draw_static(void);
static void lcd_status_update_timer_text(uint32_t elapsed_ms);
static void lcd_status_update_imu_lines(uint32_t now_ms);
static void lcd_status_update_encoder_line(uint32_t now_ms);
static void lcd_status_draw_next_dirty(void);
static void lcd_status_draw_item(lcd_status_item_t item);
static void lcd_status_mark_dirty(lcd_status_item_t item);
static bool lcd_status_is_dirty(lcd_status_item_t item);
static void lcd_status_clear_dirty(lcd_status_item_t item);
static void lcd_status_format_padded(
    char *buffer, size_t buffer_size, const char *format, ...);
static void lcd_status_format_angle(
    char axis, float value, char *buffer, size_t buffer_size);
static int32_t lcd_status_clip_encoder_speed(int32_t speed_pps);
static int32_t lcd_status_scale_encoder_speed(int32_t speed_pps);
static void lcd_status_clear_uart_line(void);
static void lcd_status_format_line_sensor(
    char *line1, size_t line1_size, char *line2, size_t line2_size);
static void lcd_status_draw_ascii(uint16_t x, uint16_t y, const char *text,
    uint16_t color, uint16_t bg_color);

void lcd_status_screen_init(uint32_t now_ms)
{
    g_lcd_status_last_second = now_ms / 1000U;
    g_lcd_status_last_imu_sample_ms = now_ms - LCD_STATUS_IMU_DRAW_MS;
    g_lcd_status_last_encoder_sample_ms = now_ms - LCD_STATUS_ENCODER_DRAW_MS;
    g_lcd_status_imu_ready = false;
    g_lcd_status_imu_error = 0U;

    (void) memset(g_lcd_status_timer_text, ' ', LCD_STATUS_TIMER_TEXT_LEN);
    g_lcd_status_timer_text[LCD_STATUS_TIMER_TEXT_LEN] = '\0';
    (void) memset(g_lcd_status_timer_drawn, 0, sizeof(g_lcd_status_timer_drawn));
    lcd_status_clear_uart_line();
    (void) memset(g_lcd_status_uart_drawn, 0, sizeof(g_lcd_status_uart_drawn));
    (void) memset(g_lcd_status_imu_drawn_line1, 0, sizeof(g_lcd_status_imu_drawn_line1));
    (void) memset(g_lcd_status_imu_drawn_line2, 0, sizeof(g_lcd_status_imu_drawn_line2));
    (void) memset(g_lcd_status_imu_drawn_line3, 0, sizeof(g_lcd_status_imu_drawn_line3));
    (void) memset(g_lcd_status_encoder_drawn, 0, sizeof(g_lcd_status_encoder_drawn));
    (void) memset(g_lcd_status_line_drawn1, 0, sizeof(g_lcd_status_line_drawn1));
    (void) memset(g_lcd_status_line_drawn2, 0, sizeof(g_lcd_status_line_drawn2));

    g_lcd_status_dirty_mask = 0U;
    g_lcd_status_next_item = 0U;

    ST7789_Init();
    lcd_status_draw_static();

    lcd_status_update_timer_text(now_ms);
    lcd_status_update_imu_lines(now_ms);
    lcd_status_update_encoder_line(now_ms);
    lcd_status_format_line_sensor(g_lcd_status_line_line1,
        sizeof(g_lcd_status_line_line1), g_lcd_status_line_line2,
        sizeof(g_lcd_status_line_line2));

    lcd_status_draw_item(LCD_STATUS_ITEM_TIMER);
    lcd_status_draw_item(LCD_STATUS_ITEM_UART);
    lcd_status_draw_item(LCD_STATUS_ITEM_IMU_LINE1);
    lcd_status_draw_item(LCD_STATUS_ITEM_IMU_LINE2);
    lcd_status_draw_item(LCD_STATUS_ITEM_IMU_LINE3);
    lcd_status_draw_item(LCD_STATUS_ITEM_ENCODER);
    lcd_status_draw_item(LCD_STATUS_ITEM_LINE_SENSOR);
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
    lcd_status_update_encoder_line(now_ms);
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

void lcd_status_screen_set_line_sensor(uint8_t raw, uint8_t active_mask,
    uint8_t active_count, int16_t line_error, uint8_t enabled,
    uint8_t sensor_ok, uint8_t sensor_error)
{
    g_lcd_status_line_raw = raw;
    g_lcd_status_line_active_mask = active_mask;
    g_lcd_status_line_active_count = active_count;
    g_lcd_status_line_error = line_error;
    g_lcd_status_line_enabled = enabled ? 1U : 0U;
    g_lcd_status_line_sensor_ok = sensor_ok ? 1U : 0U;
    g_lcd_status_line_sensor_error = sensor_error;

    lcd_status_format_line_sensor(g_lcd_status_line_line1,
        sizeof(g_lcd_status_line_line1), g_lcd_status_line_line2,
        sizeof(g_lcd_status_line_line2));
    if ((strcmp(g_lcd_status_line_line1, g_lcd_status_line_drawn1) != 0) ||
        (strcmp(g_lcd_status_line_line2, g_lcd_status_line_drawn2) != 0)) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_LINE_SENSOR);
    }
}

static void lcd_status_draw_static(void)
{
    ST7789_Clear(LCD_STATUS_BG);
    ST7789_FillRect(0U, LCD_STATUS_UART_Y, ST7789_WIDTH,
        LCD_STATUS_UART_H, LCD_STATUS_PANEL_TOP);
    ST7789_FillRect(LCD_STATUS_IMU_X, LCD_STATUS_IMU_Y, ST7789_WIDTH,
        LCD_STATUS_IMU_H, LCD_STATUS_PANEL_MID);
    ST7789_FillRect(LCD_STATUS_ENCODER_X, LCD_STATUS_ENCODER_Y, ST7789_WIDTH,
        LCD_STATUS_ENCODER_H, LCD_STATUS_PANEL_BOTTOM);

    ST7789_DrawLine(0U, (uint16_t) (LCD_STATUS_IMU_Y - 4U),
        (uint16_t) (ST7789_WIDTH - 1U), (uint16_t) (LCD_STATUS_IMU_Y - 4U),
        LCD_STATUS_GRID);
    ST7789_DrawLine(0U, (uint16_t) (LCD_STATUS_ENCODER_Y - 4U),
        (uint16_t) (ST7789_WIDTH - 1U), (uint16_t) (LCD_STATUS_ENCODER_Y - 4U),
        LCD_STATUS_GRID);
    ST7789_FillRect(0U, LCD_STATUS_LINE_Y, ST7789_WIDTH,
        LCD_STATUS_LINE_H, LCD_STATUS_LINE_PANEL);
    ST7789_DrawLine(0U, (uint16_t) (LCD_STATUS_LINE_Y - 4U),
        (uint16_t) (ST7789_WIDTH - 1U), (uint16_t) (LCD_STATUS_LINE_Y - 4U),
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

        lcd_status_format_angle('R', angle.roll, g_lcd_status_imu_line1,
            sizeof(g_lcd_status_imu_line1));
        lcd_status_format_angle('P', angle.pitch, g_lcd_status_imu_line2,
            sizeof(g_lcd_status_imu_line2));
        lcd_status_format_angle('Y', angle.yaw, g_lcd_status_imu_line3,
            sizeof(g_lcd_status_imu_line3));
    } else {
        lcd_status_format_padded(g_lcd_status_imu_line1,
            sizeof(g_lcd_status_imu_line1), "IMU ERR");
        lcd_status_format_padded(g_lcd_status_imu_line2,
            sizeof(g_lcd_status_imu_line2), "ICM:%02u",
            (unsigned int) error_code);
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

static void lcd_status_update_encoder_line(uint32_t now_ms)
{
    int32_t left_speed;
    int32_t right_speed;

    if ((uint32_t) (now_ms - g_lcd_status_last_encoder_sample_ms) <
        LCD_STATUS_ENCODER_DRAW_MS) {
        return;
    }

    g_lcd_status_last_encoder_sample_ms = now_ms;
    left_speed = lcd_status_scale_encoder_speed(
        Encoder_GetSpeedPps(ENCODER_LEFT));
    right_speed = lcd_status_scale_encoder_speed(
        Encoder_GetSpeedPps(ENCODER_RIGHT));

    lcd_status_format_padded(g_lcd_status_encoder_line,
        sizeof(g_lcd_status_encoder_line), "L:%+05ld R:%+05ld",
        (long) left_speed, (long) right_speed);

    if (strcmp(g_lcd_status_encoder_line, g_lcd_status_encoder_drawn) != 0) {
        lcd_status_mark_dirty(LCD_STATUS_ITEM_ENCODER);
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
                g_lcd_status_timer_text, LCD_STATUS_TEXT, LCD_STATUS_PANEL_TOP);
            (void) memcpy(g_lcd_status_timer_drawn, g_lcd_status_timer_text,
                sizeof(g_lcd_status_timer_drawn));
            break;
        case LCD_STATUS_ITEM_UART:
            lcd_status_draw_ascii(LCD_STATUS_UART_TEXT_X, LCD_STATUS_TIMER_Y,
                g_lcd_status_uart_line, LCD_STATUS_TEXT, LCD_STATUS_PANEL_TOP);
            (void) memcpy(g_lcd_status_uart_drawn, g_lcd_status_uart_line,
                sizeof(g_lcd_status_uart_drawn));
            break;
        case LCD_STATUS_ITEM_IMU_LINE1:
            lcd_status_draw_ascii(LCD_STATUS_IMU_TEXT_X, LCD_STATUS_IMU_LINE1_Y,
                g_lcd_status_imu_line1,
                g_lcd_status_imu_ready ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL_MID);
            (void) memcpy(g_lcd_status_imu_drawn_line1, g_lcd_status_imu_line1,
                sizeof(g_lcd_status_imu_drawn_line1));
            break;
        case LCD_STATUS_ITEM_IMU_LINE2:
            lcd_status_draw_ascii(LCD_STATUS_IMU_TEXT_X, LCD_STATUS_IMU_LINE2_Y,
                g_lcd_status_imu_line2,
                g_lcd_status_imu_ready ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL_MID);
            (void) memcpy(g_lcd_status_imu_drawn_line2, g_lcd_status_imu_line2,
                sizeof(g_lcd_status_imu_drawn_line2));
            break;
        case LCD_STATUS_ITEM_IMU_LINE3:
            lcd_status_draw_ascii(LCD_STATUS_IMU_TEXT_X, LCD_STATUS_IMU_LINE3_Y,
                g_lcd_status_imu_line3,
                g_lcd_status_imu_ready ? LCD_STATUS_VALUE : LCD_STATUS_WARN,
                LCD_STATUS_PANEL_MID);
            (void) memcpy(g_lcd_status_imu_drawn_line3, g_lcd_status_imu_line3,
                sizeof(g_lcd_status_imu_drawn_line3));
            break;
        case LCD_STATUS_ITEM_ENCODER:
            lcd_status_draw_ascii(LCD_STATUS_ENCODER_TEXT_X,
                LCD_STATUS_ENCODER_LINE_Y, g_lcd_status_encoder_line,
                LCD_STATUS_VALUE, LCD_STATUS_PANEL_BOTTOM);
            (void) memcpy(g_lcd_status_encoder_drawn, g_lcd_status_encoder_line,
                sizeof(g_lcd_status_encoder_drawn));
            break;
        case LCD_STATUS_ITEM_LINE_SENSOR:
            lcd_status_draw_ascii(LCD_STATUS_LINE_TEXT_X, LCD_STATUS_LINE_LINE1_Y,
                g_lcd_status_line_line1, LCD_STATUS_TEXT,
                LCD_STATUS_LINE_PANEL);
            lcd_status_draw_ascii(LCD_STATUS_LINE_TEXT_X, LCD_STATUS_LINE_LINE2_Y,
                g_lcd_status_line_line2, LCD_STATUS_VALUE,
                LCD_STATUS_LINE_PANEL);
            (void) memcpy(g_lcd_status_line_drawn1, g_lcd_status_line_line1,
                sizeof(g_lcd_status_line_drawn1));
            (void) memcpy(g_lcd_status_line_drawn2, g_lcd_status_line_line2,
                sizeof(g_lcd_status_line_drawn2));
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

static void lcd_status_format_angle(
    char axis, float value, char *buffer, size_t buffer_size)
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

static int32_t lcd_status_clip_encoder_speed(int32_t speed_pps)
{
    if (speed_pps > LCD_STATUS_ENCODER_SPEED_MAX) {
        return LCD_STATUS_ENCODER_SPEED_MAX;
    }
    if (speed_pps < -LCD_STATUS_ENCODER_SPEED_MAX) {
        return -LCD_STATUS_ENCODER_SPEED_MAX;
    }
    return speed_pps;
}

static int32_t lcd_status_scale_encoder_speed(int32_t speed_pps)
{
    if (speed_pps >= 0) {
        speed_pps = (speed_pps + (LCD_STATUS_ENCODER_SPEED_SCALE / 2L)) /
            LCD_STATUS_ENCODER_SPEED_SCALE;
    } else {
        speed_pps = (speed_pps - (LCD_STATUS_ENCODER_SPEED_SCALE / 2L)) /
            LCD_STATUS_ENCODER_SPEED_SCALE;
    }

    return lcd_status_clip_encoder_speed(speed_pps);
}

static void lcd_status_clear_uart_line(void)
{
    (void) memset(g_lcd_status_uart_line, ' ', LCD_STATUS_UART_COLUMNS);
    g_lcd_status_uart_line[LCD_STATUS_UART_COLUMNS] = '\0';
    g_lcd_status_uart_column = 0U;
    lcd_status_mark_dirty(LCD_STATUS_ITEM_UART);
}

static void lcd_status_format_line_sensor(
    char *line1, size_t line1_size, char *line2, size_t line2_size)
{
    char bits[9];
    uint8_t i;

    if ((line1 == NULL) || (line2 == NULL) ||
        (line1_size == 0U) || (line2_size == 0U)) {
        return;
    }

    for (i = 0U; i < 8U; i++) {
        uint8_t bit = (uint8_t) (0x80U >> i);
        bits[i] = ((g_lcd_status_line_raw & bit) != 0U) ? '1' : '0';
    }
    bits[8] = '\0';

    lcd_status_format_padded(line1, line1_size, "IR:%s", bits);
    lcd_status_format_padded(line2, line2_size, "C:%u E:%+d %s",
        (unsigned int) g_lcd_status_line_active_count,
        (int) g_lcd_status_line_error,
        g_lcd_status_line_enabled ? "ON" : "OFF");
}

static void lcd_status_draw_ascii(uint16_t x, uint16_t y, const char *text,
    uint16_t color, uint16_t bg_color)
{
    ST7789_ShowAsciiStringFast(x, y, text, LCD_STATUS_FONT, color, bg_color);
}
