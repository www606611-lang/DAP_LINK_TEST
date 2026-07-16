#ifndef MODULES_ST7789_H
#define MODULES_ST7789_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ST7789_WIDTH                 320U
#define ST7789_HEIGHT                170U

/* 常用 RGB565 颜色。 */
#define ST7789_COLOR_BLACK           0x0000U
#define ST7789_COLOR_BLUE            0x001FU
#define ST7789_COLOR_GREEN           0x07E0U
#define ST7789_COLOR_CYAN            0x07FFU
#define ST7789_COLOR_RED             0xF800U
#define ST7789_COLOR_MAGENTA         0xF81FU
#define ST7789_COLOR_YELLOW          0xFFE0U
#define ST7789_COLOR_WHITE           0xFFFFU

#define ST7789_8X16                  8U
#define ST7789_6X8                   6U

#define ST7789_UNFILLED              0U
#define ST7789_FILLED                1U

#define ST7789_RGB565(r, g, b) \
    ((uint16_t) (((uint16_t) ((r) & 0xF8U) << 8) | \
                 ((uint16_t) ((g) & 0xFCU) << 3) | \
                 ((uint16_t) ((b) & 0xF8U) >> 3)))

void ST7789_Init(void);
void ST7789_SetBacklight(uint8_t enabled);
void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/* 基础绘图接口。 */
void ST7789_Fill(uint16_t color);
void ST7789_FillRect(
    uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void ST7789_Clear(uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_DrawLine(
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ST7789_DrawRectangle(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height, uint8_t is_filled, uint16_t color);
void ST7789_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2, uint8_t is_filled, uint16_t color);
void ST7789_DrawCircle(
    uint16_t x, uint16_t y, uint16_t radius, uint8_t is_filled, uint16_t color);
void ST7789_DrawEllipse(uint16_t x, uint16_t y, uint16_t a, uint16_t b,
    uint8_t is_filled, uint16_t color);
void ST7789_DrawArc(uint16_t x, uint16_t y, uint16_t radius,
    int16_t start_angle, int16_t end_angle, uint8_t is_filled,
    uint16_t color);

/* 文字、数字和字模显示接口，字体数据来自 OLED_Data.c。 */
void ST7789_ShowImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
    const uint8_t *image, uint16_t color, uint16_t bg_color);
void ST7789_ShowAsciiStringFast(uint16_t x, uint16_t y, const char *str,
    uint8_t font_size, uint16_t color, uint16_t bg_color);
void ST7789_ShowAsciiStringScaled(uint16_t x, uint16_t y, const char *str,
    uint8_t scale, uint16_t color, uint16_t bg_color);
void ST7789_ShowChar(uint16_t x, uint16_t y, char ch, uint8_t font_size,
    uint16_t color, uint16_t bg_color);
void ST7789_ShowString(uint16_t x, uint16_t y, const char *str,
    uint8_t font_size, uint16_t color, uint16_t bg_color);
void ST7789_ShowNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length,
    uint8_t font_size, uint16_t color, uint16_t bg_color);
void ST7789_ShowSignedNum(uint16_t x, uint16_t y, int32_t number,
    uint8_t length, uint8_t font_size, uint16_t color, uint16_t bg_color);
void ST7789_ShowHexNum(uint16_t x, uint16_t y, uint32_t number,
    uint8_t length, uint8_t font_size, uint16_t color, uint16_t bg_color);
void ST7789_ShowBinNum(uint16_t x, uint16_t y, uint32_t number,
    uint8_t length, uint8_t font_size, uint16_t color, uint16_t bg_color);
void ST7789_ShowFloatNum(uint16_t x, uint16_t y, double number,
    uint8_t int_length, uint8_t fra_length, uint8_t font_size, uint16_t color,
    uint16_t bg_color);
void ST7789_ShowChinese(uint16_t x, uint16_t y, const char *chinese,
    uint16_t color, uint16_t bg_color);
void ST7789_Printf(uint16_t x, uint16_t y, uint8_t font_size, uint16_t color,
    uint16_t bg_color, const char *format, ...);
void ST7789_PrintfFast(uint16_t x, uint16_t y, uint8_t font_size,
    uint16_t color, uint16_t bg_color, const char *format, ...);

void ST7789_DrawTestPattern(void);

#ifdef __cplusplus
}
#endif

#endif
