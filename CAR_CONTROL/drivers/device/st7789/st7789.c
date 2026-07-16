#include "st7789.h"

#include "delay.h"
/* ST7789 复用原 OLED 的 6x8/8x16 字库数据，不再依赖 OLED 驱动。 */
#include "OLED_Data.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ST7789_X_OFFSET              0U
#define ST7789_Y_OFFSET              35U
#define ST7789_MADCTL_VALUE          0x70U
#define ST7789_COLMOD_RGB565         0x05U
#define ST7789_PRINTF_BUFFER_SIZE    64U
#define ST7789_SPI_SCR_DIVIDER       1U
#define ST7789_DMA_CHAN_ID           DMA_CH2_CHAN_ID
#define ST7789_DMA_MIN_BYTES         64U
#define ST7789_DMA_BUFFER_SIZE       1024U
#define ST7789_WAIT_SPINS            2000000U

/* 1.9 寸 170x320 模块的可视区域在 ST7789 GRAM 中有 35 行偏移。 */

#define ST7789_SWRESET               0x01U
#define ST7789_SLPOUT                0x11U
#define ST7789_INVON                 0x21U
#define ST7789_CASET                 0x2AU
#define ST7789_RASET                 0x2BU
#define ST7789_RAMWR                 0x2CU
#define ST7789_MADCTL                0x36U
#define ST7789_COLMOD                0x3AU
#define ST7789_PORCTRL               0xB2U
#define ST7789_GCTRL                 0xB7U
#define ST7789_VCOMS                 0xBBU
#define ST7789_LCMCTRL               0xC0U
#define ST7789_VDVVRHEN              0xC2U
#define ST7789_VRHS                  0xC3U
#define ST7789_VDVS                  0xC4U
#define ST7789_FRCTRL2               0xC6U
#define ST7789_PWCTRL1               0xD0U
#define ST7789_PVGAMCTRL             0xE0U
#define ST7789_NVGAMCTRL             0xE1U
#define ST7789_DISPON                0x29U

static void st7789_select(void);
static void st7789_deselect(void);
static void st7789_wait_idle(void);
static void st7789_write_bytes_polling(const uint8_t *data, uint32_t length);
static void st7789_write_bytes_dma(const uint8_t *data, uint32_t length);
static void st7789_write_bytes(const uint8_t *data, uint32_t length);
static void st7789_write_command(uint8_t command);
static void st7789_write_command_data(
    uint8_t command, const uint8_t *data, uint32_t length);
static void st7789_write_pixel_color(uint16_t color);
static void st7789_write_color(uint16_t color, uint32_t count);
static void st7789_flush_pixel_buffer(
    uint8_t *buffer, uint32_t *buffered_bytes);
static void st7789_reset_dma_state(void);
static void st7789_hardware_reset(void);
static uint32_t st7789_pow_u32(uint32_t base, uint32_t exp);
static uint8_t st7789_ascii_index(char ch);
static uint8_t st7789_font_height(uint8_t font_size);
static const uint8_t *st7789_find_chinese(const char *utf8_char);
static void st7789_draw_pixel_i32(int32_t x, int32_t y, uint16_t color);
static void st7789_draw_vline_i32(
    int32_t x, int32_t y0, int32_t y1, uint16_t color);
static void st7789_draw_circle_points(
    int32_t x, int32_t y, int32_t px, int32_t py, uint16_t color);
static uint8_t st7789_point_in_polygon(uint8_t nvert, const int32_t *vertx,
    const int32_t *verty, int32_t testx, int32_t testy);
static uint8_t st7789_is_in_angle(
    int32_t x, int32_t y, int16_t start_angle, int16_t end_angle);

static volatile bool g_st7789_dma_busy;
static uint8_t g_st7789_dma_buffer[ST7789_DMA_BUFFER_SIZE];

static void st7789_select(void)
{
    DL_GPIO_clearPins(LCD_CTRL_PORT, LCD_CTRL_LCD_CS_PIN);
}

static void st7789_deselect(void)
{
    st7789_wait_idle();
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_CS_PIN);
}

static void st7789_wait_idle(void)
{
    uint32_t spins = ST7789_WAIT_SPINS;

    while ((g_st7789_dma_busy || DL_SPI_isBusy(SPI_LCD_INST)) &&
        (spins > 0U)) {
        spins--;
    }

    if ((spins == 0U) &&
        (g_st7789_dma_busy || DL_SPI_isBusy(SPI_LCD_INST))) {
        st7789_reset_dma_state();
    }
}

static void st7789_write_bytes_polling(const uint8_t *data, uint32_t length)
{
    while (length > 0U) {
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST)) {
        }
        DL_SPI_transmitData8(SPI_LCD_INST, *data);
        data++;
        length--;
    }

    st7789_wait_idle();
}

static void st7789_write_bytes_dma(const uint8_t *data, uint32_t length)
{
    uint32_t spins = ST7789_WAIT_SPINS;

    if ((data == NULL) || (length == 0U)) {
        return;
    }

    st7789_wait_idle();
    g_st7789_dma_busy = true;

    DL_DMA_disableChannel(DMA, ST7789_DMA_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_setSrcAddr(DMA, ST7789_DMA_CHAN_ID, (uint32_t) data);
    DL_DMA_setDestAddr(
        DMA, ST7789_DMA_CHAN_ID, (uint32_t) &SPI_LCD_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, ST7789_DMA_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, ST7789_DMA_CHAN_ID);

    while (g_st7789_dma_busy && (spins > 0U)) {
        spins--;
    }

    if (g_st7789_dma_busy) {
        st7789_reset_dma_state();
    }

    st7789_wait_idle();
}

static void st7789_write_bytes(const uint8_t *data, uint32_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return;
    }

    if (length >= ST7789_DMA_MIN_BYTES) {
        st7789_write_bytes_dma(data, length);
    } else {
        st7789_write_bytes_polling(data, length);
    }
}

static void st7789_write_command(uint8_t command)
{
    st7789_select();
    DL_GPIO_clearPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);
    st7789_write_bytes(&command, 1U);
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);
    st7789_deselect();
}

static void st7789_write_command_data(
    uint8_t command, const uint8_t *data, uint32_t length)
{
    st7789_select();
    /* DC=0 写命令，DC=1 写后续参数或像素数据。 */
    DL_GPIO_clearPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);
    st7789_write_bytes(&command, 1U);

    if ((data != NULL) && (length > 0U)) {
        DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);
        st7789_write_bytes(data, length);
    }

    st7789_deselect();
}

static void st7789_write_pixel_color(uint16_t color)
{
    uint8_t high = (uint8_t) (color >> 8);
    uint8_t low = (uint8_t) color;

    while (DL_SPI_isTXFIFOFull(SPI_LCD_INST)) {
    }
    DL_SPI_transmitData8(SPI_LCD_INST, high);

    while (DL_SPI_isTXFIFOFull(SPI_LCD_INST)) {
    }
    DL_SPI_transmitData8(SPI_LCD_INST, low);
}

static void st7789_write_color(uint16_t color, uint32_t count)
{
    uint8_t high = (uint8_t) (color >> 8);
    uint8_t low = (uint8_t) color;
    uint32_t buffer_bytes = ST7789_DMA_BUFFER_SIZE;
    uint32_t i;

    if ((count * 2U) < ST7789_DMA_MIN_BYTES) {
        while (count-- > 0U) {
            st7789_write_pixel_color(color);
        }

        st7789_wait_idle();
        return;
    }

    if ((buffer_bytes & 0x1U) != 0U) {
        buffer_bytes--;
    }

    for (i = 0U; i < buffer_bytes; i += 2U) {
        g_st7789_dma_buffer[i] = high;
        g_st7789_dma_buffer[i + 1U] = low;
    }

    while (count > 0U) {
        uint32_t burst_pixels = buffer_bytes / 2U;

        if (burst_pixels > count) {
            burst_pixels = count;
        }

        st7789_write_bytes(g_st7789_dma_buffer, burst_pixels * 2U);
        count -= burst_pixels;
    }
}

static void st7789_flush_pixel_buffer(
    uint8_t *buffer, uint32_t *buffered_bytes)
{
    if ((buffer == NULL) || (buffered_bytes == NULL) ||
        (*buffered_bytes == 0U)) {
        return;
    }

    st7789_write_bytes(buffer, *buffered_bytes);
    *buffered_bytes = 0U;
}

static void st7789_reset_dma_state(void)
{
    g_st7789_dma_busy = false;
    DL_DMA_disableChannel(DMA, ST7789_DMA_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
}

static void st7789_hardware_reset(void)
{
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_CS_PIN |
                                   LCD_CTRL_LCD_DC_PIN |
                                   LCD_CTRL_LCD_BLK_PIN);

    DL_GPIO_clearPins(LCD_CTRL_PORT, LCD_CTRL_LCD_RES_PIN);
    delay_us(30000U);
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_RES_PIN);
    delay_us(120000U);
}

static uint32_t st7789_pow_u32(uint32_t base, uint32_t exp)
{
    uint32_t result = 1U;

    while (exp-- > 0U) {
        result *= base;
    }

    return result;
}

static uint8_t st7789_ascii_index(char ch)
{
    if ((ch < ' ') || (ch > '~')) {
        return (uint8_t) ('?' - ' ');
    }

    return (uint8_t) (ch - ' ');
}

static uint8_t st7789_font_height(uint8_t font_size)
{
    if (font_size == ST7789_8X16) {
        return 16U;
    }
    if (font_size == ST7789_6X8) {
        return 8U;
    }

    return 0U;
}

static const uint8_t *st7789_find_chinese(const char *utf8_char)
{
    uint32_t i;

    for (i = 0U; i < OLED_CF16x16_COUNT; i++) {
        if (strncmp(OLED_CF16x16[i].Index, utf8_char, OLED_CHN_CHAR_WIDTH) ==
            0) {
            return OLED_CF16x16[i].Data;
        }
    }

    return Diode;
}

static void st7789_draw_pixel_i32(int32_t x, int32_t y, uint16_t color)
{
    if ((x < 0) || (y < 0) || (x >= (int32_t) ST7789_WIDTH) ||
        (y >= (int32_t) ST7789_HEIGHT)) {
        return;
    }

    ST7789_DrawPixel((uint16_t) x, (uint16_t) y, color);
}

static void st7789_draw_vline_i32(
    int32_t x, int32_t y0, int32_t y1, uint16_t color)
{
    int32_t temp;

    if ((x < 0) || (x >= (int32_t) ST7789_WIDTH)) {
        return;
    }

    if (y0 > y1) {
        temp = y0;
        y0 = y1;
        y1 = temp;
    }

    if ((y1 < 0) || (y0 >= (int32_t) ST7789_HEIGHT)) {
        return;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (y1 >= (int32_t) ST7789_HEIGHT) {
        y1 = (int32_t) ST7789_HEIGHT - 1;
    }

    ST7789_FillRect((uint16_t) x, (uint16_t) y0, 1U,
        (uint16_t) (y1 - y0 + 1), color);
}

static void st7789_draw_circle_points(
    int32_t x, int32_t y, int32_t px, int32_t py, uint16_t color)
{
    st7789_draw_pixel_i32(x + px, y + py, color);
    st7789_draw_pixel_i32(x + py, y + px, color);
    st7789_draw_pixel_i32(x - px, y - py, color);
    st7789_draw_pixel_i32(x - py, y - px, color);
    st7789_draw_pixel_i32(x + px, y - py, color);
    st7789_draw_pixel_i32(x + py, y - px, color);
    st7789_draw_pixel_i32(x - px, y + py, color);
    st7789_draw_pixel_i32(x - py, y + px, color);
}

static uint8_t st7789_point_in_polygon(uint8_t nvert, const int32_t *vertx,
    const int32_t *verty, int32_t testx, int32_t testy)
{
    int32_t i;
    int32_t j;
    uint8_t inside = 0U;

    for (i = 0, j = (int32_t) (nvert - 1U); i < (int32_t) nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) /
                            (verty[j] - verty[i]) +
                        vertx[i])) {
            inside = (uint8_t) !inside;
        }
    }

    return inside;
}

static uint8_t st7789_is_in_angle(
    int32_t x, int32_t y, int16_t start_angle, int16_t end_angle)
{
    int16_t point_angle;

    point_angle = (int16_t) (atan2((double) y, (double) x) * 180.0 /
                             3.14159265358979323846);
    if (start_angle < end_angle) {
        return (uint8_t) ((point_angle >= start_angle) &&
                          (point_angle <= end_angle));
    }

    return (uint8_t) ((point_angle >= start_angle) ||
                      (point_angle <= end_angle));
}

void ST7789_Init(void)
{
    static const uint8_t porch[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    static const uint8_t power[] = {0xA4U, 0xA1U};
    static const uint8_t positive_gamma[] = {
        0xF0U, 0x00U, 0x04U, 0x04U, 0x04U, 0x05U, 0x29U,
        0x33U, 0x3EU, 0x38U, 0x12U, 0x12U, 0x28U, 0x30U
    };
    static const uint8_t negative_gamma[] = {
        0xF0U, 0x07U, 0x0AU, 0x0DU, 0x0BU, 0x07U, 0x28U,
        0x33U, 0x3EU, 0x36U, 0x14U, 0x14U, 0x29U, 0x32U
    };
    uint8_t data;

    /* SysConfig 默认把 LCD SPI 设在 8 MHz，这里提升到 20 MHz 以减轻字符刷屏卡顿。 */
    DL_SPI_setBitRateSerialClockDivider(
        SPI_LCD_INST, ST7789_SPI_SCR_DIVIDER);
    st7789_reset_dma_state();
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);

    st7789_hardware_reset();

    st7789_write_command(ST7789_SWRESET);
    delay_us(120000U);

    st7789_write_command(ST7789_SLPOUT);
    delay_us(120000U);

    data = ST7789_MADCTL_VALUE;
    st7789_write_command_data(ST7789_MADCTL, &data, 1U);

    data = ST7789_COLMOD_RGB565;
    st7789_write_command_data(ST7789_COLMOD, &data, 1U);

    st7789_write_command_data(ST7789_PORCTRL, porch, sizeof(porch));

    data = 0x35U;
    st7789_write_command_data(ST7789_GCTRL, &data, 1U);

    data = 0x1AU;
    st7789_write_command_data(ST7789_VCOMS, &data, 1U);

    data = 0x2CU;
    st7789_write_command_data(ST7789_LCMCTRL, &data, 1U);

    data = 0x01U;
    st7789_write_command_data(ST7789_VDVVRHEN, &data, 1U);

    data = 0x0BU;
    st7789_write_command_data(ST7789_VRHS, &data, 1U);

    data = 0x20U;
    st7789_write_command_data(ST7789_VDVS, &data, 1U);

    data = 0x0FU;
    st7789_write_command_data(ST7789_FRCTRL2, &data, 1U);

    st7789_write_command_data(ST7789_PWCTRL1, power, sizeof(power));
    st7789_write_command(ST7789_INVON);
    st7789_write_command_data(
        ST7789_PVGAMCTRL, positive_gamma, sizeof(positive_gamma));
    st7789_write_command_data(
        ST7789_NVGAMCTRL, negative_gamma, sizeof(negative_gamma));

    delay_us(10000U);
    st7789_write_command(ST7789_DISPON);
    delay_us(20000U);

    ST7789_SetBacklight(1U);
    ST7789_Fill(ST7789_COLOR_BLACK);
}

void ST7789_SetBacklight(uint8_t enabled)
{
    if (enabled != 0U) {
        DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_BLK_PIN);
    } else {
        DL_GPIO_clearPins(LCD_CTRL_PORT, LCD_CTRL_LCD_BLK_PIN);
    }
}

void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    /* 对外坐标从屏幕左上角算起，发给控制器前补上面板偏移。 */
    x0 = (uint16_t) (x0 + ST7789_X_OFFSET);
    x1 = (uint16_t) (x1 + ST7789_X_OFFSET);
    y0 = (uint16_t) (y0 + ST7789_Y_OFFSET);
    y1 = (uint16_t) (y1 + ST7789_Y_OFFSET);

    data[0] = (uint8_t) (x0 >> 8);
    data[1] = (uint8_t) x0;
    data[2] = (uint8_t) (x1 >> 8);
    data[3] = (uint8_t) x1;
    st7789_write_command_data(ST7789_CASET, data, sizeof(data));

    data[0] = (uint8_t) (y0 >> 8);
    data[1] = (uint8_t) y0;
    data[2] = (uint8_t) (y1 >> 8);
    data[3] = (uint8_t) y1;
    st7789_write_command_data(ST7789_RASET, data, sizeof(data));

    st7789_write_command(ST7789_RAMWR);
}

void ST7789_Fill(uint16_t color)
{
    ST7789_FillRect(0U, 0U, ST7789_WIDTH, ST7789_HEIGHT, color);
}

void ST7789_FillRect(
    uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint32_t x_end;
    uint32_t y_end;

    if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT) ||
        (width == 0U) || (height == 0U)) {
        return;
    }

    x_end = (uint32_t) x + width;
    y_end = (uint32_t) y + height;
    if (x_end > ST7789_WIDTH) {
        x_end = ST7789_WIDTH;
    }
    if (y_end > ST7789_HEIGHT) {
        y_end = ST7789_HEIGHT;
    }

    ST7789_SetWindow(x, y, (uint16_t) (x_end - 1U), (uint16_t) (y_end - 1U));

    st7789_select();
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);
    st7789_write_color(color, (x_end - x) * (y_end - y));
    st7789_deselect();
}

void ST7789_Clear(uint16_t color)
{
    ST7789_Fill(color);
}

void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    ST7789_FillRect(x, y, 1U, 1U, color);
}

void ST7789_DrawLine(
    uint16_t x0_in, uint16_t y0_in, uint16_t x1_in, uint16_t y1_in,
    uint16_t color)
{
    int32_t x0 = x0_in;
    int32_t y0 = y0_in;
    int32_t x1 = x1_in;
    int32_t y1 = y1_in;
    int32_t dx;
    int32_t sx;
    int32_t dy;
    int32_t sy;
    int32_t err;

    if (y0 == y1) {
        uint16_t x = (x0 < x1) ? (uint16_t) x0 : (uint16_t) x1;
        uint16_t width = (uint16_t) (abs((int) (x1 - x0)) + 1);
        ST7789_FillRect(x, (uint16_t) y0, width, 1U, color);
        return;
    }
    if (x0 == x1) {
        uint16_t y = (y0 < y1) ? (uint16_t) y0 : (uint16_t) y1;
        uint16_t height = (uint16_t) (abs((int) (y1 - y0)) + 1);
        ST7789_FillRect((uint16_t) x0, y, 1U, height, color);
        return;
    }

    dx = abs((int) (x1 - x0));
    sx = (x0 < x1) ? 1 : -1;
    dy = -abs((int) (y1 - y0));
    sy = (y0 < y1) ? 1 : -1;
    err = dx + dy;

    while (1) {
        int32_t e2;

        st7789_draw_pixel_i32(x0, y0, color);
        if ((x0 == x1) && (y0 == y1)) {
            break;
        }

        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ST7789_DrawRectangle(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height, uint8_t is_filled, uint16_t color)
{
    uint32_t x_end;
    uint32_t y_end;

    if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT) ||
        (width == 0U) || (height == 0U)) {
        return;
    }

    x_end = (uint32_t) x + width;
    y_end = (uint32_t) y + height;
    if (x_end > ST7789_WIDTH) {
        x_end = ST7789_WIDTH;
    }
    if (y_end > ST7789_HEIGHT) {
        y_end = ST7789_HEIGHT;
    }
    width = (uint16_t) (x_end - x);
    height = (uint16_t) (y_end - y);

    if (is_filled != ST7789_UNFILLED) {
        ST7789_FillRect(x, y, width, height, color);
        return;
    }

    ST7789_FillRect(x, y, width, 1U, color);
    if (height > 1U) {
        ST7789_FillRect(x, (uint16_t) (y_end - 1U), width, 1U, color);
    }
    if (height > 2U) {
        ST7789_FillRect(x, (uint16_t) (y + 1U), 1U,
            (uint16_t) (height - 2U), color);
        if (width > 1U) {
            ST7789_FillRect((uint16_t) (x_end - 1U), (uint16_t) (y + 1U),
                1U, (uint16_t) (height - 2U), color);
        }
    }
}

void ST7789_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2, uint8_t is_filled, uint16_t color)
{
    int32_t vx[3];
    int32_t vy[3];
    int32_t minx = x0;
    int32_t miny = y0;
    int32_t maxx = x0;
    int32_t maxy = y0;
    int32_t xi;
    int32_t yi;

    if (is_filled == ST7789_UNFILLED) {
        ST7789_DrawLine(x0, y0, x1, y1, color);
        ST7789_DrawLine(x0, y0, x2, y2, color);
        ST7789_DrawLine(x1, y1, x2, y2, color);
        return;
    }

    if ((int32_t) x1 < minx) minx = x1;
    if ((int32_t) x2 < minx) minx = x2;
    if ((int32_t) y1 < miny) miny = y1;
    if ((int32_t) y2 < miny) miny = y2;
    if ((int32_t) x1 > maxx) maxx = x1;
    if ((int32_t) x2 > maxx) maxx = x2;
    if ((int32_t) y1 > maxy) maxy = y1;
    if ((int32_t) y2 > maxy) maxy = y2;

    if ((minx >= (int32_t) ST7789_WIDTH) ||
        (miny >= (int32_t) ST7789_HEIGHT)) {
        return;
    }
    if (maxx >= (int32_t) ST7789_WIDTH) {
        maxx = (int32_t) ST7789_WIDTH - 1;
    }
    if (maxy >= (int32_t) ST7789_HEIGHT) {
        maxy = (int32_t) ST7789_HEIGHT - 1;
    }

    vx[0] = x0;
    vx[1] = x1;
    vx[2] = x2;
    vy[0] = y0;
    vy[1] = y1;
    vy[2] = y2;

    for (xi = minx; xi <= maxx; xi++) {
        for (yi = miny; yi <= maxy; yi++) {
            if (st7789_point_in_polygon(3U, vx, vy, xi, yi) != 0U) {
                st7789_draw_pixel_i32(xi, yi, color);
            }
        }
    }
}

void ST7789_DrawCircle(
    uint16_t x_in, uint16_t y_in, uint16_t radius, uint8_t is_filled,
    uint16_t color)
{
    int32_t x = x_in;
    int32_t y = y_in;
    int32_t px = 0;
    int32_t py = radius;
    int32_t d = 1 - (int32_t) radius;

    if (is_filled != ST7789_UNFILLED) {
        st7789_draw_vline_i32(x, y - py, y + py, color);
    } else {
        st7789_draw_circle_points(x, y, px, py, color);
    }

    while (px < py) {
        px++;
        if (d < 0) {
            d += 2 * px + 1;
        } else {
            py--;
            d += 2 * (px - py) + 1;
        }

        if (is_filled != ST7789_UNFILLED) {
            st7789_draw_vline_i32(x + px, y - py, y + py, color);
            st7789_draw_vline_i32(x - px, y - py, y + py, color);
            st7789_draw_vline_i32(x + py, y - px, y + px, color);
            st7789_draw_vline_i32(x - py, y - px, y + px, color);
        } else {
            st7789_draw_circle_points(x, y, px, py, color);
        }
    }
}

void ST7789_DrawEllipse(uint16_t x_in, uint16_t y_in, uint16_t a_in,
    uint16_t b_in, uint8_t is_filled, uint16_t color)
{
    int32_t x = x_in;
    int32_t y = y_in;
    int32_t px = 0;
    int32_t py = b_in;
    int32_t a = a_in;
    int32_t b = b_in;
    double d1 = (double) b * b + (double) a * a * (-(double) b + 0.5);
    double d2;

    if ((a == 0) || (b == 0)) {
        return;
    }

    if (is_filled != ST7789_UNFILLED) {
        st7789_draw_vline_i32(x, y - py, y + py, color);
    } else {
        st7789_draw_pixel_i32(x + px, y + py, color);
        st7789_draw_pixel_i32(x - px, y - py, color);
        st7789_draw_pixel_i32(x - px, y + py, color);
        st7789_draw_pixel_i32(x + px, y - py, color);
    }

    while ((double) b * b * (px + 1) < (double) a * a * (py - 0.5)) {
        if (d1 <= 0.0) {
            d1 += (double) b * b * (2 * px + 3);
        } else {
            d1 += (double) b * b * (2 * px + 3) +
                  (double) a * a * (-2 * py + 2);
            py--;
        }
        px++;

        if (is_filled != ST7789_UNFILLED) {
            st7789_draw_vline_i32(x + px, y - py, y + py, color);
            st7789_draw_vline_i32(x - px, y - py, y + py, color);
        } else {
            st7789_draw_pixel_i32(x + px, y + py, color);
            st7789_draw_pixel_i32(x - px, y - py, color);
            st7789_draw_pixel_i32(x - px, y + py, color);
            st7789_draw_pixel_i32(x + px, y - py, color);
        }
    }

    d2 = (double) b * b * (px + 0.5) * (px + 0.5) +
         (double) a * a * (py - 1) * (py - 1) - (double) a * a * b * b;

    while (py > 0) {
        if (d2 <= 0.0) {
            d2 += (double) b * b * (2 * px + 2) +
                  (double) a * a * (-2 * py + 3);
            px++;
        } else {
            d2 += (double) a * a * (-2 * py + 3);
        }
        py--;

        if (is_filled != ST7789_UNFILLED) {
            st7789_draw_vline_i32(x + px, y - py, y + py, color);
            st7789_draw_vline_i32(x - px, y - py, y + py, color);
        } else {
            st7789_draw_pixel_i32(x + px, y + py, color);
            st7789_draw_pixel_i32(x - px, y - py, color);
            st7789_draw_pixel_i32(x - px, y + py, color);
            st7789_draw_pixel_i32(x + px, y - py, color);
        }
    }
}

void ST7789_DrawArc(uint16_t x_in, uint16_t y_in, uint16_t radius,
    int16_t start_angle, int16_t end_angle, uint8_t is_filled,
    uint16_t color)
{
    int32_t x = x_in;
    int32_t y = y_in;
    int32_t px = 0;
    int32_t py = radius;
    int32_t d = 1 - (int32_t) radius;
    int32_t j;

    while (px <= py) {
        int32_t points[8][2] = {
            {px, py}, {py, px}, {-px, -py}, {-py, -px},
            {px, -py}, {py, -px}, {-px, py}, {-py, px}
        };
        uint8_t i;

        for (i = 0U; i < 8U; i++) {
            if (st7789_is_in_angle(points[i][0], points[i][1], start_angle,
                    end_angle) != 0U) {
                st7789_draw_pixel_i32(x + points[i][0], y + points[i][1],
                    color);
            }
        }

        if (is_filled != ST7789_UNFILLED) {
            for (j = -py; j <= py; j++) {
                if (st7789_is_in_angle(px, j, start_angle, end_angle) != 0U) {
                    st7789_draw_pixel_i32(x + px, y + j, color);
                }
                if (st7789_is_in_angle(-px, j, start_angle, end_angle) != 0U) {
                    st7789_draw_pixel_i32(x - px, y + j, color);
                }
            }
            for (j = -px; j <= px; j++) {
                if (st7789_is_in_angle(py, j, start_angle, end_angle) != 0U) {
                    st7789_draw_pixel_i32(x + py, y + j, color);
                }
                if (st7789_is_in_angle(-py, j, start_angle, end_angle) != 0U) {
                    st7789_draw_pixel_i32(x - py, y + j, color);
                }
            }
        }

        px++;
        if (d < 0) {
            d += 2 * px + 1;
        } else {
            py--;
            d += 2 * (px - py) + 1;
        }
    }
}

void ST7789_ShowImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
    const uint8_t *image, uint16_t color, uint16_t bg_color)
{
    uint16_t draw_width;
    uint16_t draw_height;
    uint16_t xi;
    uint16_t yi;
    uint8_t color_high = (uint8_t) (color >> 8);
    uint8_t color_low = (uint8_t) color;
    uint8_t bg_high = (uint8_t) (bg_color >> 8);
    uint8_t bg_low = (uint8_t) bg_color;
    uint32_t buffered_bytes = 0U;

    if ((image == NULL) || (x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT) ||
        (width == 0U) || (height == 0U)) {
        return;
    }

    draw_width = width;
    draw_height = height;
    if ((uint32_t) x + draw_width > ST7789_WIDTH) {
        draw_width = (uint16_t) (ST7789_WIDTH - x);
    }
    if ((uint32_t) y + draw_height > ST7789_HEIGHT) {
        draw_height = (uint16_t) (ST7789_HEIGHT - y);
    }

    ST7789_SetWindow(x, y, (uint16_t) (x + draw_width - 1U),
        (uint16_t) (y + draw_height - 1U));

    st7789_select();
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);
    for (yi = 0U; yi < draw_height; yi++) {
        for (xi = 0U; xi < draw_width; xi++) {
            /* 字库按列存储，每 8 个纵向像素共用一个字节。 */
            uint8_t src = image[(yi / 8U) * width + xi];
            bool pixel_on =
                ((src & (uint8_t) (1U << (yi % 8U))) != 0U);

            g_st7789_dma_buffer[buffered_bytes++] =
                pixel_on ? color_high : bg_high;
            g_st7789_dma_buffer[buffered_bytes++] =
                pixel_on ? color_low : bg_low;

            if (buffered_bytes >= ST7789_DMA_BUFFER_SIZE) {
                st7789_flush_pixel_buffer(
                    g_st7789_dma_buffer, &buffered_bytes);
            }
        }
    }
    st7789_flush_pixel_buffer(g_st7789_dma_buffer, &buffered_bytes);
    st7789_deselect();
}

void ST7789_ShowAsciiStringFast(uint16_t x, uint16_t y, const char *str,
    uint8_t font_size, uint16_t color, uint16_t bg_color)
{
    uint8_t char_height = st7789_font_height(font_size);
    uint16_t char_width = font_size;
    uint16_t visible_chars = 0U;
    uint16_t draw_height;
    uint16_t row;
    uint16_t col;
    uint16_t ch_index;
    uint8_t color_high = (uint8_t) (color >> 8);
    uint8_t color_low = (uint8_t) color;
    uint8_t bg_high = (uint8_t) (bg_color >> 8);
    uint8_t bg_low = (uint8_t) bg_color;
    uint32_t buffered_bytes = 0U;

    if ((str == NULL) || (char_height == 0U) || (x >= ST7789_WIDTH) ||
        (y >= ST7789_HEIGHT)) {
        return;
    }

    while ((str[visible_chars] != '\0') && (str[visible_chars] != '\n') &&
           ((uint32_t) x + (uint32_t) ((visible_chars + 1U) * char_width) <=
               ST7789_WIDTH)) {
        visible_chars++;
    }

    if (visible_chars == 0U) {
        return;
    }

    draw_height = char_height;
    if ((uint32_t) y + draw_height > ST7789_HEIGHT) {
        draw_height = (uint16_t) (ST7789_HEIGHT - y);
    }

    ST7789_SetWindow(x, y,
        (uint16_t) (x + visible_chars * char_width - 1U),
        (uint16_t) (y + draw_height - 1U));

    st7789_select();
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);

    for (row = 0U; row < draw_height; row++) {
        uint8_t glyph_row = (uint8_t) (row / 8U);
        uint8_t glyph_mask = (uint8_t) (1U << (row % 8U));

        for (ch_index = 0U; ch_index < visible_chars; ch_index++) {
            const uint8_t *glyph;

            if (font_size == ST7789_8X16) {
                glyph = OLED_F8x16[st7789_ascii_index(str[ch_index])];
            } else {
                glyph = OLED_F6x8[st7789_ascii_index(str[ch_index])];
            }

            for (col = 0U; col < char_width; col++) {
                bool pixel_on =
                    ((glyph[glyph_row * char_width + col] & glyph_mask) != 0U);

                g_st7789_dma_buffer[buffered_bytes++] =
                    pixel_on ? color_high : bg_high;
                g_st7789_dma_buffer[buffered_bytes++] =
                    pixel_on ? color_low : bg_low;

                if (buffered_bytes >= ST7789_DMA_BUFFER_SIZE) {
                    st7789_flush_pixel_buffer(
                        g_st7789_dma_buffer, &buffered_bytes);
                }
            }
        }
    }

    st7789_flush_pixel_buffer(g_st7789_dma_buffer, &buffered_bytes);
    st7789_deselect();
}

void ST7789_ShowAsciiStringScaled(uint16_t x, uint16_t y, const char *str,
    uint8_t scale, uint16_t color, uint16_t bg_color)
{
    uint16_t visible_chars = 0U;
    uint16_t draw_width;
    uint16_t draw_height;
    uint16_t row;
    uint16_t ch_index;
    uint8_t color_high = (uint8_t) (color >> 8);
    uint8_t color_low = (uint8_t) color;
    uint8_t bg_high = (uint8_t) (bg_color >> 8);
    uint8_t bg_low = (uint8_t) bg_color;
    uint32_t buffered_bytes = 0U;

    if ((str == NULL) || (scale == 0U) || (x >= ST7789_WIDTH) ||
        (y >= ST7789_HEIGHT)) {
        return;
    }

    while ((str[visible_chars] != '\0') &&
        (str[visible_chars] != '\n') &&
        ((uint32_t) x + (uint32_t) (visible_chars + 1U) * 8U * scale <=
            ST7789_WIDTH)) {
        visible_chars++;
    }
    if (visible_chars == 0U) {
        return;
    }

    draw_width = (uint16_t) (visible_chars * 8U * scale);
    draw_height = (uint16_t) (16U * scale);
    if ((uint32_t) y + draw_height > ST7789_HEIGHT) {
        draw_height = (uint16_t) (ST7789_HEIGHT - y);
    }

    ST7789_SetWindow(x, y, (uint16_t) (x + draw_width - 1U),
        (uint16_t) (y + draw_height - 1U));
    st7789_select();
    DL_GPIO_setPins(LCD_CTRL_PORT, LCD_CTRL_LCD_DC_PIN);

    for (row = 0U; row < draw_height; row++) {
        uint8_t source_row = (uint8_t) (row / scale);
        uint8_t glyph_row = (uint8_t) (source_row / 8U);
        uint8_t glyph_mask = (uint8_t) (1U << (source_row % 8U));

        for (ch_index = 0U; ch_index < visible_chars; ch_index++) {
            const uint8_t *glyph =
                OLED_F8x16[st7789_ascii_index(str[ch_index])];
            uint8_t col;

            for (col = 0U; col < 8U; col++) {
                bool pixel_on =
                    ((glyph[glyph_row * 8U + col] & glyph_mask) != 0U);
                uint8_t repeat;

                for (repeat = 0U; repeat < scale; repeat++) {
                    g_st7789_dma_buffer[buffered_bytes++] =
                        pixel_on ? color_high : bg_high;
                    g_st7789_dma_buffer[buffered_bytes++] =
                        pixel_on ? color_low : bg_low;
                    if (buffered_bytes >= ST7789_DMA_BUFFER_SIZE) {
                        st7789_flush_pixel_buffer(
                            g_st7789_dma_buffer, &buffered_bytes);
                    }
                }
            }
        }
    }

    st7789_flush_pixel_buffer(g_st7789_dma_buffer, &buffered_bytes);
    st7789_deselect();
}

void DMA_IRQHandler(void)
{
    switch (DL_DMA_getPendingInterrupt(DMA)) {
        case DL_DMA_EVENT_IIDX_DMACH2:
            DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
            DL_DMA_disableChannel(DMA, ST7789_DMA_CHAN_ID);
            g_st7789_dma_busy = false;
            break;
        default:
            break;
    }
}

void ST7789_ShowChar(uint16_t x, uint16_t y, char ch, uint8_t font_size,
    uint16_t color, uint16_t bg_color)
{
    uint8_t index = st7789_ascii_index(ch);

    if (font_size == ST7789_8X16) {
        ST7789_ShowImage(x, y, 8U, 16U, OLED_F8x16[index], color, bg_color);
    } else if (font_size == ST7789_6X8) {
        ST7789_ShowImage(x, y, 6U, 8U, OLED_F6x8[index], color, bg_color);
    }
}

void ST7789_ShowString(uint16_t x, uint16_t y, const char *str,
    uint8_t font_size, uint16_t color, uint16_t bg_color)
{
    uint16_t start_x = x;
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;
    uint8_t char_height = st7789_font_height(font_size);

    if ((str == NULL) || (char_height == 0U)) {
        return;
    }

    while (*str != '\0') {
        if (*str == '\n') {
            cursor_x = start_x;
            cursor_y = (uint16_t) (cursor_y + char_height);
            str++;
            continue;
        }

        if (cursor_y >= ST7789_HEIGHT) {
            break;
        }
        if (cursor_x < ST7789_WIDTH) {
            ST7789_ShowChar(cursor_x, cursor_y, *str, font_size, color,
                bg_color);
        }

        cursor_x = (uint16_t) (cursor_x + font_size);
        str++;
    }
}

void ST7789_ShowNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length,
    uint8_t font_size, uint16_t color, uint16_t bg_color)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        ST7789_ShowChar((uint16_t) (x + i * font_size), y,
            (char) (number / st7789_pow_u32(10U, (uint32_t) (length - i - 1U)) %
                         10U +
                     '0'),
            font_size, color, bg_color);
    }
}

void ST7789_ShowSignedNum(uint16_t x, uint16_t y, int32_t number,
    uint8_t length, uint8_t font_size, uint16_t color, uint16_t bg_color)
{
    uint32_t mag;
    uint8_t i;

    if (number >= 0) {
        ST7789_ShowChar(x, y, '+', font_size, color, bg_color);
        mag = (uint32_t) number;
    } else {
        ST7789_ShowChar(x, y, '-', font_size, color, bg_color);
        mag = (uint32_t) (-(number + 1)) + 1U;
    }

    for (i = 0U; i < length; i++) {
        ST7789_ShowChar((uint16_t) (x + (i + 1U) * font_size), y,
            (char) (mag / st7789_pow_u32(10U, (uint32_t) (length - i - 1U)) %
                         10U +
                     '0'),
            font_size, color, bg_color);
    }
}

void ST7789_ShowHexNum(uint16_t x, uint16_t y, uint32_t number,
    uint8_t length, uint8_t font_size, uint16_t color, uint16_t bg_color)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        uint8_t digit = (uint8_t) (number /
                                      st7789_pow_u32(
                                          16U, (uint32_t) (length - i - 1U)) %
                                  16U);
        char ch = (digit < 10U) ?
            (char) ((uint8_t) '0' + digit) :
            (char) ((uint8_t) 'A' + digit - 10U);

        ST7789_ShowChar((uint16_t) (x + i * font_size), y, ch, font_size,
            color, bg_color);
    }
}

void ST7789_ShowBinNum(uint16_t x, uint16_t y, uint32_t number,
    uint8_t length, uint8_t font_size, uint16_t color, uint16_t bg_color)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        ST7789_ShowChar((uint16_t) (x + i * font_size), y,
            (char) (number / st7789_pow_u32(2U, (uint32_t) (length - i - 1U)) %
                         2U +
                     '0'),
            font_size, color, bg_color);
    }
}

void ST7789_ShowFloatNum(uint16_t x, uint16_t y, double number,
    uint8_t int_length, uint8_t fra_length, uint8_t font_size, uint16_t color,
    uint16_t bg_color)
{
    uint32_t pow_num;
    uint32_t int_num;
    uint32_t fra_num;

    if (number >= 0.0) {
        ST7789_ShowChar(x, y, '+', font_size, color, bg_color);
    } else {
        ST7789_ShowChar(x, y, '-', font_size, color, bg_color);
        number = -number;
    }

    int_num = (uint32_t) number;
    number -= (double) int_num;
    pow_num = st7789_pow_u32(10U, fra_length);
    fra_num = (uint32_t) (number * (double) pow_num + 0.5);
    if (fra_num >= pow_num) {
        int_num++;
        fra_num -= pow_num;
    }

    ST7789_ShowNum((uint16_t) (x + font_size), y, int_num, int_length,
        font_size, color, bg_color);
    ST7789_ShowChar((uint16_t) (x + (int_length + 1U) * font_size), y, '.',
        font_size, color, bg_color);
    ST7789_ShowNum((uint16_t) (x + (int_length + 2U) * font_size), y, fra_num,
        fra_length, font_size, color, bg_color);
}

void ST7789_ShowChinese(uint16_t x, uint16_t y, const char *chinese,
    uint16_t color, uint16_t bg_color)
{
    uint16_t glyph_index = 0U;

    if (chinese == NULL) {
        return;
    }

    while (*chinese != '\0') {
        char glyph[OLED_CHN_CHAR_WIDTH + 1U];
        uint8_t i;

        /* 当前字库按 UTF-8 三字节汉字索引查表。 */
        for (i = 0U; i < OLED_CHN_CHAR_WIDTH; i++) {
            if (chinese[i] == '\0') {
                return;
            }
            glyph[i] = chinese[i];
        }
        glyph[OLED_CHN_CHAR_WIDTH] = '\0';

        ST7789_ShowImage((uint16_t) (x + glyph_index * 16U), y, 16U, 16U,
            st7789_find_chinese(glyph), color, bg_color);
        chinese += OLED_CHN_CHAR_WIDTH;
        glyph_index++;
    }
}

void ST7789_Printf(uint16_t x, uint16_t y, uint8_t font_size, uint16_t color,
    uint16_t bg_color, const char *format, ...)
{
    char buffer[ST7789_PRINTF_BUFFER_SIZE];
    va_list args;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    (void) vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    ST7789_ShowString(x, y, buffer, font_size, color, bg_color);
}

void ST7789_PrintfFast(uint16_t x, uint16_t y, uint8_t font_size,
    uint16_t color, uint16_t bg_color, const char *format, ...)
{
    char buffer[ST7789_PRINTF_BUFFER_SIZE];
    va_list args;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    (void) vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    ST7789_ShowAsciiStringFast(
        x, y, buffer, font_size, color, bg_color);
}

void ST7789_DrawTestPattern(void)
{
    uint16_t band = (uint16_t) (ST7789_WIDTH / 8U);

    ST7789_FillRect(0U, 0U, band, ST7789_HEIGHT, ST7789_COLOR_BLACK);
    ST7789_FillRect(band, 0U, band, ST7789_HEIGHT, ST7789_COLOR_BLUE);
    ST7789_FillRect((uint16_t) (band * 2U), 0U, band, ST7789_HEIGHT,
        ST7789_COLOR_GREEN);
    ST7789_FillRect((uint16_t) (band * 3U), 0U, band, ST7789_HEIGHT,
        ST7789_COLOR_CYAN);
    ST7789_FillRect((uint16_t) (band * 4U), 0U, band, ST7789_HEIGHT,
        ST7789_COLOR_RED);
    ST7789_FillRect((uint16_t) (band * 5U), 0U, band, ST7789_HEIGHT,
        ST7789_COLOR_MAGENTA);
    ST7789_FillRect((uint16_t) (band * 6U), 0U, band, ST7789_HEIGHT,
        ST7789_COLOR_YELLOW);
    ST7789_FillRect((uint16_t) (band * 7U), 0U,
        (uint16_t) (ST7789_WIDTH - (band * 7U)), ST7789_HEIGHT,
        ST7789_COLOR_WHITE);
}
