#include "oled.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR            0x3CU
#endif

#ifndef OLED_I2C_WAIT_SPINS
#define OLED_I2C_WAIT_SPINS      200000U
#endif

#define OLED_I2C_FIFO_BYTES      8U
#define OLED_I2C_DATA_CHUNK      (OLED_I2C_FIFO_BYTES - 1U)

static uint8_t g_oled_display_buf[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t g_oled_i2c_ready;
static uint8_t g_oled_i2c_addr = OLED_I2C_ADDR;

static uint32_t oled_pow_u32(uint32_t base, uint32_t exp);
static uint8_t oled_point_in_polygon(uint8_t nvert, const int16_t *vertx, const int16_t *verty, int16_t testx, int16_t testy);
static uint8_t oled_is_in_angle(int16_t x, int16_t y, int16_t start_angle, int16_t end_angle);
static void oled_i2c_init(void);
static void oled_i2c_recover_bus(void);
static uint8_t oled_i2c_write_addr(uint8_t addr, const uint8_t *data, uint8_t length);
static uint8_t oled_i2c_write(const uint8_t *data, uint8_t length);
static void oled_write_command(uint8_t command);
static void oled_write_data_stream(const uint8_t *data, uint8_t count);
static void oled_set_cursor(uint8_t page, uint8_t x);
static uint8_t oled_ascii_index(char ch);
static const uint8_t *oled_find_chinese(const char *utf8_char);

static uint32_t oled_pow_u32(uint32_t base, uint32_t exp)
{
    uint32_t result = 1U;

    while (exp-- > 0U) {
        result *= base;
    }

    return result;
}

static uint8_t oled_point_in_polygon(uint8_t nvert, const int16_t *vertx,
    const int16_t *verty, int16_t testx, int16_t testy)
{
    int16_t i;
    int16_t j;
    uint8_t inside = 0U;

    for (i = 0, j = (int16_t) (nvert - 1U); i < (int16_t) nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) /
                            (verty[j] - verty[i]) +
                        vertx[i])) {
            inside = (uint8_t) !inside;
        }
    }

    return inside;
}

static uint8_t oled_is_in_angle(
    int16_t x, int16_t y, int16_t start_angle, int16_t end_angle)
{
    int16_t point_angle;

    point_angle = (int16_t) (atan2((double) y, (double) x) * 180.0 / 3.14159265358979323846);
    if (start_angle < end_angle) {
        return (uint8_t) ((point_angle >= start_angle) &&
                          (point_angle <= end_angle));
    }

    return (uint8_t) ((point_angle >= start_angle) ||
                      (point_angle <= end_angle));
}

static void oled_i2c_init(void)
{
    if (g_oled_i2c_ready != 0U) {
        return;
    }

    DL_I2C_disableController(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_setControllerTXFIFOThreshold(
        I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(
        I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_0_INST);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_enableController(I2C_0_INST);

    g_oled_i2c_ready = 1U;
}

static void oled_i2c_recover_bus(void)
{
    g_oled_i2c_ready = 0U;
    SYSCFG_DL_I2C_0_init();
    oled_i2c_init();
}

static uint8_t oled_i2c_write_addr(uint8_t addr, const uint8_t *data, uint8_t length)
{
    uint32_t spins;
    uint8_t retry;

    if ((data == NULL) || (length == 0U)) {
        return 0U;
    }

    oled_i2c_init();

    for (retry = 0U; retry < 2U; retry++) {
        spins = OLED_I2C_WAIT_SPINS;
        while ((DL_I2C_getControllerStatus(I2C_0_INST) &
                   DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
            if (spins-- == 0U) {
                break;
            }
        }

        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_clearInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        DL_I2C_resetControllerTransfer(I2C_0_INST);
        (void) DL_I2C_fillControllerTXFIFO(I2C_0_INST, (uint8_t *) data, length);
        DL_I2C_startControllerTransfer(I2C_0_INST, addr,
            DL_I2C_CONTROLLER_DIRECTION_TX, length);

        delay_cpu_cycles(16U);

        spins = OLED_I2C_WAIT_SPINS;
        while ((DL_I2C_getControllerStatus(I2C_0_INST) &
                   DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
            if (spins-- == 0U) {
                break;
            }
        }

        if ((DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) {
            return 1U;
        }

        oled_i2c_recover_bus();
    }

    return 0U;
}

static uint8_t oled_i2c_write(const uint8_t *data, uint8_t length)
{
    return oled_i2c_write_addr(g_oled_i2c_addr, data, length);
}

static void oled_write_command(uint8_t command)
{
    uint8_t packet[2];

    packet[0] = 0x00U;
    packet[1] = command;
    (void) oled_i2c_write(packet, 2U);
}

static void oled_write_data_stream(const uint8_t *data, uint8_t count)
{
    uint8_t packet[OLED_I2C_FIFO_BYTES];
    uint8_t sent = 0U;

    while (sent < count) {
        uint8_t i;
        uint8_t chunk = (uint8_t) (count - sent);

        if (chunk > OLED_I2C_DATA_CHUNK) {
            chunk = OLED_I2C_DATA_CHUNK;
        }

        packet[0] = 0x40U;
        for (i = 0U; i < chunk; i++) {
            packet[i + 1U] = data[sent + i];
        }

        (void) oled_i2c_write(packet, (uint8_t) (chunk + 1U));
        sent = (uint8_t) (sent + chunk);
    }
}

static void oled_set_cursor(uint8_t page, uint8_t x)
{
    oled_write_command((uint8_t) (0xB0U | (page & 0x07U)));
    oled_write_command((uint8_t) (0x10U | ((x >> 4) & 0x0FU)));
    oled_write_command((uint8_t) (0x00U | (x & 0x0FU)));
}

static uint8_t oled_ascii_index(char ch)
{
    if ((ch < ' ') || (ch > '~')) {
        return (uint8_t) ('?' - ' ');
    }

    return (uint8_t) (ch - ' ');
}

static const uint8_t *oled_find_chinese(const char *utf8_char)
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

void OLED_Init(void)
{
    static const uint8_t probe_packet[2] = {0x00U, 0xAEU};
    static const uint8_t init_cmds[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x20U, 0x02U,
        0x40U, 0xA1U,
        0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x30U,
        0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
    };
    uint32_t i;

    delay_us(100000U);
    oled_i2c_recover_bus();

    g_oled_i2c_addr = OLED_I2C_ADDR;
    if (oled_i2c_write_addr(g_oled_i2c_addr, probe_packet, 2U) == 0U) {
        g_oled_i2c_addr = 0x3DU;
        (void) oled_i2c_write_addr(g_oled_i2c_addr, probe_packet, 2U);
    }

    for (i = 0U; i < (sizeof(init_cmds) / sizeof(init_cmds[0])); i++) {
        oled_write_command(init_cmds[i]);
    }

    OLED_Clear();
    OLED_Update();
}

void OLED_Update(void)
{
    uint8_t page;

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        oled_set_cursor(page, 0U);
        oled_write_data_stream(g_oled_display_buf[page], OLED_WIDTH);
    }
}

void OLED_UpdateArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint8_t page_start;
    uint8_t page_end;
    uint8_t page;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT) || (width == 0U) ||
        (height == 0U)) {
        return;
    }

    if ((uint16_t) x + width > OLED_WIDTH) {
        width = (uint8_t) (OLED_WIDTH - x);
    }
    if ((uint16_t) y + height > OLED_HEIGHT) {
        height = (uint8_t) (OLED_HEIGHT - y);
    }

    page_start = (uint8_t) (y / 8U);
    page_end = (uint8_t) ((y + height - 1U) / 8U);

    for (page = page_start; page <= page_end; page++) {
        oled_set_cursor(page, x);
        oled_write_data_stream(&g_oled_display_buf[page][x], width);
    }
}

void OLED_Clear(void)
{
    (void) memset(g_oled_display_buf, 0, sizeof(g_oled_display_buf));
}

void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint8_t xi;
    uint8_t yi;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT) || (width == 0U) ||
        (height == 0U)) {
        return;
    }

    if ((uint16_t) x + width > OLED_WIDTH) {
        width = (uint8_t) (OLED_WIDTH - x);
    }
    if ((uint16_t) y + height > OLED_HEIGHT) {
        height = (uint8_t) (OLED_HEIGHT - y);
    }

    for (yi = y; yi < (uint8_t) (y + height); yi++) {
        for (xi = x; xi < (uint8_t) (x + width); xi++) {
            g_oled_display_buf[yi / 8U][xi] &=
                (uint8_t) ~(1U << (yi % 8U));
        }
    }
}

void OLED_Reverse(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        for (x = 0U; x < OLED_WIDTH; x++) {
            g_oled_display_buf[page][x] ^= 0xFFU;
        }
    }
}

void OLED_ReverseArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint8_t xi;
    uint8_t yi;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT) || (width == 0U) ||
        (height == 0U)) {
        return;
    }

    if ((uint16_t) x + width > OLED_WIDTH) {
        width = (uint8_t) (OLED_WIDTH - x);
    }
    if ((uint16_t) y + height > OLED_HEIGHT) {
        height = (uint8_t) (OLED_HEIGHT - y);
    }

    for (yi = y; yi < (uint8_t) (y + height); yi++) {
        for (xi = x; xi < (uint8_t) (x + width); xi++) {
            g_oled_display_buf[yi / 8U][xi] ^= (uint8_t) (1U << (yi % 8U));
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t font_size)
{
    uint8_t index = oled_ascii_index(ch);

    if (font_size == OLED_8X16) {
        OLED_ShowImage(x, y, 8U, 16U, OLED_F8x16[index]);
    } else if (font_size == OLED_6X8) {
        OLED_ShowImage(x, y, 6U, 8U, OLED_F6x8[index]);
    }
}

void OLED_ShowString(
    uint8_t x, uint8_t y, const char *str, uint8_t font_size)
{
    uint8_t i = 0U;

    if (str == NULL) {
        return;
    }

    while (str[i] != '\0') {
        OLED_ShowChar((uint8_t) (x + i * font_size), y, str[i], font_size);
        i++;
    }
}

void OLED_ShowNum(
    uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        OLED_ShowChar((uint8_t) (x + i * font_size), y,
            (char) (number / oled_pow_u32(10U, (uint32_t) (length - i - 1U)) %
                         10U +
                     '0'),
            font_size);
    }
}

void OLED_ShowSignedNum(
    uint8_t x, uint8_t y, int32_t number, uint8_t length, uint8_t font_size)
{
    uint32_t mag;
    uint8_t i;

    if (number >= 0) {
        OLED_ShowChar(x, y, '+', font_size);
        mag = (uint32_t) number;
    } else {
        OLED_ShowChar(x, y, '-', font_size);
        mag = (uint32_t) (-(number + 1)) + 1U;
    }

    for (i = 0U; i < length; i++) {
        OLED_ShowChar((uint8_t) (x + (i + 1U) * font_size), y,
            (char) (mag / oled_pow_u32(10U, (uint32_t) (length - i - 1U)) %
                         10U +
                     '0'),
            font_size);
    }
}

void OLED_ShowHexNum(
    uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        uint8_t digit =
            (uint8_t) (number / oled_pow_u32(16U, (uint32_t) (length - i - 1U)) %
                       16U);

        OLED_ShowChar((uint8_t) (x + i * font_size), y,
            (char) ((digit < 10U) ? (digit + '0') : (digit - 10U + 'A')),
            font_size);
    }
}

void OLED_ShowBinNum(
    uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        OLED_ShowChar((uint8_t) (x + i * font_size), y,
            (char) (number / oled_pow_u32(2U, (uint32_t) (length - i - 1U)) %
                         2U +
                     '0'),
            font_size);
    }
}

void OLED_ShowFloatNum(uint8_t x, uint8_t y, double number,
    uint8_t int_length, uint8_t fra_length, uint8_t font_size)
{
    uint32_t pow_num;
    uint32_t int_num;
    uint32_t fra_num;

    if (number >= 0.0) {
        OLED_ShowChar(x, y, '+', font_size);
    } else {
        OLED_ShowChar(x, y, '-', font_size);
        number = -number;
    }

    int_num = (uint32_t) number;
    number -= (double) int_num;
    pow_num = oled_pow_u32(10U, fra_length);
    fra_num = (uint32_t) llround(number * pow_num);
    int_num += fra_num / pow_num;
    fra_num %= pow_num;

    OLED_ShowNum((uint8_t) (x + font_size), y, int_num, int_length, font_size);
    OLED_ShowChar((uint8_t) (x + (int_length + 1U) * font_size), y, '.',
        font_size);
    OLED_ShowNum((uint8_t) (x + (int_length + 2U) * font_size), y, fra_num,
        fra_length, font_size);
}

void OLED_ShowChinese(uint8_t x, uint8_t y, const char *chinese)
{
    uint8_t index = 0U;
    uint8_t char_pos = 0U;
    char glyph[OLED_CHN_CHAR_WIDTH + 1U];

    if (chinese == NULL) {
        return;
    }

    (void) memset(glyph, 0, sizeof(glyph));

    while (chinese[index] != '\0') {
        glyph[char_pos++] = chinese[index++];
        if (char_pos >= OLED_CHN_CHAR_WIDTH) {
            const uint8_t *bitmap;
            uint8_t glyph_index =
                (uint8_t) (index / OLED_CHN_CHAR_WIDTH - 1U);

            glyph[OLED_CHN_CHAR_WIDTH] = '\0';
            bitmap = oled_find_chinese(glyph);
            OLED_ShowImage((uint8_t) (x + glyph_index * 16U), y, 16U, 16U,
                bitmap);
            char_pos = 0U;
            (void) memset(glyph, 0, sizeof(glyph));
        }
    }
}

void OLED_ShowImage(
    uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image)
{
    uint8_t xi;
    uint8_t page;
    uint8_t page_count;

    if ((image == NULL) || (x >= OLED_WIDTH) || (y >= OLED_HEIGHT) ||
        (width == 0U) || (height == 0U)) {
        return;
    }

    page_count = (uint8_t) ((height - 1U) / 8U + 1U);
    OLED_ClearArea(x, y, width, height);

    for (page = 0U; page < page_count; page++) {
        for (xi = 0U; xi < width; xi++) {
            uint8_t dst_x = (uint8_t) (x + xi);
            uint8_t dst_page = (uint8_t) (y / 8U + page);
            uint8_t src = image[page * width + xi];

            if (dst_x >= OLED_WIDTH) {
                break;
            }
            if (dst_page >= OLED_PAGE_COUNT) {
                return;
            }

            g_oled_display_buf[dst_page][dst_x] |=
                (uint8_t) (src << (y % 8U));

            if ((dst_page + 1U) >= OLED_PAGE_COUNT) {
                continue;
            }

            g_oled_display_buf[dst_page + 1U][dst_x] |=
                (uint8_t) (src >> (8U - (y % 8U)));
        }
    }
}

void OLED_Printf(
    uint8_t x, uint8_t y, uint8_t font_size, const char *format, ...)
{
    char buffer[32];
    va_list args;

    va_start(args, format);
    (void) vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    OLED_ShowString(x, y, buffer, font_size);
}

void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }

    g_oled_display_buf[y / 8U][x] |= (uint8_t) (1U << (y % 8U));
}

uint8_t OLED_GetPoint(uint8_t x, uint8_t y)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return 0U;
    }

    return (uint8_t) ((g_oled_display_buf[y / 8U][x] & (1U << (y % 8U))) != 0U);
}

void OLED_DrawLine(uint8_t x0_in, uint8_t y0_in, uint8_t x1_in, uint8_t y1_in)
{
    int16_t x0 = x0_in;
    int16_t y0 = y0_in;
    int16_t x1 = x1_in;
    int16_t y1 = y1_in;
    int16_t dx = (int16_t) abs(x1 - x0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = (int16_t) -abs(y1 - y0);
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy;

    while (1) {
        OLED_DrawPoint((uint8_t) x0, (uint8_t) y0);
        if ((x0 == x1) && (y0 == y1)) {
            break;
        }

        if ((2 * err) >= dy) {
            err += dy;
            x0 += sx;
        }
        if ((2 * err) <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void OLED_DrawRectangle(
    uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t is_filled)
{
    uint8_t xi;
    uint8_t yi;

    if ((width == 0U) || (height == 0U)) {
        return;
    }

    if (is_filled == OLED_UNFILLED) {
        for (xi = x; xi < (uint8_t) (x + width); xi++) {
            OLED_DrawPoint(xi, y);
            OLED_DrawPoint(xi, (uint8_t) (y + height - 1U));
        }
        for (yi = y; yi < (uint8_t) (y + height); yi++) {
            OLED_DrawPoint(x, yi);
            OLED_DrawPoint((uint8_t) (x + width - 1U), yi);
        }
    } else {
        for (xi = x; xi < (uint8_t) (x + width); xi++) {
            for (yi = y; yi < (uint8_t) (y + height); yi++) {
                OLED_DrawPoint(xi, yi);
            }
        }
    }
}

void OLED_DrawTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
    uint8_t x2, uint8_t y2, uint8_t is_filled)
{
    int16_t vx[3];
    int16_t vy[3];
    uint8_t minx = x0;
    uint8_t miny = y0;
    uint8_t maxx = x0;
    uint8_t maxy = y0;
    uint8_t xi;
    uint8_t yi;

    if (is_filled == OLED_UNFILLED) {
        OLED_DrawLine(x0, y0, x1, y1);
        OLED_DrawLine(x0, y0, x2, y2);
        OLED_DrawLine(x1, y1, x2, y2);
        return;
    }

    if (x1 < minx) {
        minx = x1;
    }
    if (x2 < minx) {
        minx = x2;
    }
    if (y1 < miny) {
        miny = y1;
    }
    if (y2 < miny) {
        miny = y2;
    }
    if (x1 > maxx) {
        maxx = x1;
    }
    if (x2 > maxx) {
        maxx = x2;
    }
    if (y1 > maxy) {
        maxy = y1;
    }
    if (y2 > maxy) {
        maxy = y2;
    }

    vx[0] = x0;
    vx[1] = x1;
    vx[2] = x2;
    vy[0] = y0;
    vy[1] = y1;
    vy[2] = y2;

    for (xi = minx; xi <= maxx; xi++) {
        for (yi = miny; yi <= maxy; yi++) {
            if (oled_point_in_polygon(3U, vx, vy, xi, yi) != 0U) {
                OLED_DrawPoint(xi, yi);
            }
        }
    }
}

void OLED_DrawCircle(
    uint8_t x, uint8_t y, uint8_t radius, uint8_t is_filled)
{
    int16_t px = 0;
    int16_t py = radius;
    int16_t d = 1 - radius;
    int16_t j;

    OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
    OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y + px));
    OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
    OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y - px));

    if (is_filled != 0U) {
        for (j = -py; j < py; j++) {
            OLED_DrawPoint(x, (uint8_t) (y + j));
        }
    }

    while (px < py) {
        px++;
        if (d < 0) {
            d += (int16_t) (2 * px + 1);
        } else {
            py--;
            d += (int16_t) (2 * (px - py) + 1);
        }

        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
        OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y + px));
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
        OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y - px));
        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y - py));
        OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y - px));
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + py));
        OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y + px));

        if (is_filled != 0U) {
            for (j = -py; j < py; j++) {
                OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + j));
                OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + j));
            }
            for (j = -px; j < px; j++) {
                OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y + j));
                OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y + j));
            }
        }
    }
}

void OLED_DrawEllipse(
    uint8_t x, uint8_t y, uint8_t a_in, uint8_t b_in, uint8_t is_filled)
{
    int16_t px = 0;
    int16_t py = b_in;
    int16_t a = a_in;
    int16_t b = b_in;
    double d1 = (double) b * b + (double) a * a * (-(double) b + 0.5);
    double d2;
    int16_t j;

    OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
    OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
    OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + py));
    OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y - py));

    if (is_filled != 0U) {
        for (j = -py; j < py; j++) {
            OLED_DrawPoint(x, (uint8_t) (y + j));
        }
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

        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + py));
        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y - py));

        if (is_filled != 0U) {
            for (j = -py; j < py; j++) {
                OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + j));
                OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + j));
            }
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

        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + py));
        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y - py));

        if (is_filled != 0U) {
            for (j = -py; j < py; j++) {
                OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + j));
                OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + j));
            }
        }
    }
}

void OLED_DrawArc(uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle,
    int16_t end_angle, uint8_t is_filled)
{
    int16_t px = 0;
    int16_t py = radius;
    int16_t d = 1 - radius;
    int16_t j;

    if (oled_is_in_angle(px, py, start_angle, end_angle) != 0U) {
        OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
    }
    if (oled_is_in_angle(-px, -py, start_angle, end_angle) != 0U) {
        OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
    }
    if (oled_is_in_angle(py, px, start_angle, end_angle) != 0U) {
        OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y + px));
    }
    if (oled_is_in_angle(-py, -px, start_angle, end_angle) != 0U) {
        OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y - px));
    }

    if (is_filled != 0U) {
        for (j = -py; j < py; j++) {
            if (oled_is_in_angle(0, j, start_angle, end_angle) != 0U) {
                OLED_DrawPoint(x, (uint8_t) (y + j));
            }
        }
    }

    while (px < py) {
        px++;
        if (d < 0) {
            d += (int16_t) (2 * px + 1);
        } else {
            py--;
            d += (int16_t) (2 * (px - py) + 1);
        }

        if (oled_is_in_angle(px, py, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + py));
        }
        if (oled_is_in_angle(py, px, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y + px));
        }
        if (oled_is_in_angle(-px, -py, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y - py));
        }
        if (oled_is_in_angle(-py, -px, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y - px));
        }
        if (oled_is_in_angle(px, -py, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y - py));
        }
        if (oled_is_in_angle(py, -px, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y - px));
        }
        if (oled_is_in_angle(-px, py, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + py));
        }
        if (oled_is_in_angle(-py, px, start_angle, end_angle) != 0U) {
            OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y + px));
        }

        if (is_filled != 0U) {
            for (j = -py; j < py; j++) {
                if (oled_is_in_angle(px, j, start_angle, end_angle) != 0U) {
                    OLED_DrawPoint((uint8_t) (x + px), (uint8_t) (y + j));
                }
                if (oled_is_in_angle(-px, j, start_angle, end_angle) != 0U) {
                    OLED_DrawPoint((uint8_t) (x - px), (uint8_t) (y + j));
                }
            }
            for (j = -px; j < px; j++) {
                if (oled_is_in_angle(py, j, start_angle, end_angle) != 0U) {
                    OLED_DrawPoint((uint8_t) (x + py), (uint8_t) (y + j));
                }
                if (oled_is_in_angle(-py, j, start_angle, end_angle) != 0U) {
                    OLED_DrawPoint((uint8_t) (x - py), (uint8_t) (y + j));
                }
            }
        }
    }
}
