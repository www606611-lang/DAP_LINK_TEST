#include "k230_vision_link.h"

#include "vision_uart.h"

#include <stddef.h>

#define K230_VISION_FRAME_CAPACITY 20U

static char g_frame[K230_VISION_FRAME_CAPACITY];
static uint8_t g_frame_length;
static bool g_collecting;
static k230_vision_snapshot_t g_snapshot;

static bool k230_vision_parse_frame(uint32_t now_ms);
static bool k230_vision_parse_decimal(const char *text,
    uint8_t length, uint8_t *index, uint16_t maximum,
    uint16_t *value);

void K230VisionLink_Init(uint32_t now_ms)
{
    g_frame_length = 0U;
    g_collecting = false;
    g_snapshot.online = false;
    g_snapshot.target_valid = false;
    g_snapshot.cx = 0U;
    g_snapshot.cy = 0U;
    g_snapshot.error_x = 0;
    g_snapshot.error_y = 0;
    g_snapshot.frame_sequence = 0U;
    g_snapshot.last_frame_ms = now_ms;
    g_snapshot.parse_error_count = 0U;
    g_snapshot.resync_count = 0U;
    g_snapshot.timeout_count = 0U;
    g_snapshot.received_byte_count = 0U;
    g_snapshot.overflow_count = 0U;
}

void K230VisionLink_Task(uint32_t now_ms)
{
    vision_uart_stats_t uart_stats;
    uint8_t byte;

    while (VisionUart_ReadByte(&byte)) {
        if (byte == (uint8_t) '@') {
            if (g_collecting) {
                g_snapshot.resync_count++;
            }
            g_collecting = true;
            g_frame_length = 0U;
        } else if (g_collecting && (byte == (uint8_t) '#')) {
            if (!k230_vision_parse_frame(now_ms)) {
                g_snapshot.parse_error_count++;
            }
            g_collecting = false;
            g_frame_length = 0U;
        } else if (g_collecting) {
            if (g_frame_length < (K230_VISION_FRAME_CAPACITY - 1U)) {
                g_frame[g_frame_length++] = (char) byte;
            } else {
                g_snapshot.parse_error_count++;
                g_collecting = false;
                g_frame_length = 0U;
            }
        }
    }

    if (VisionUart_GetStats(&uart_stats)) {
        g_snapshot.received_byte_count =
            uart_stats.received_byte_count;
        g_snapshot.overflow_count = uart_stats.overflow_count;
    }

    if (g_snapshot.online &&
        ((uint32_t) (now_ms - g_snapshot.last_frame_ms) >=
            K230_VISION_OFFLINE_TIMEOUT_MS)) {
        g_snapshot.online = false;
        g_snapshot.target_valid = false;
        g_snapshot.timeout_count++;
    }
}

bool K230VisionLink_GetSnapshot(k230_vision_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    *snapshot = g_snapshot;
    return true;
}

static bool k230_vision_parse_frame(uint32_t now_ms)
{
    uint8_t index = 0U;
    uint16_t valid;
    uint16_t cx;
    uint16_t cy;

    if (!k230_vision_parse_decimal(g_frame, g_frame_length,
            &index, 1U, &valid) ||
        (index >= g_frame_length) || (g_frame[index++] != ',') ||
        !k230_vision_parse_decimal(g_frame, g_frame_length,
            &index, K230_VISION_IMAGE_WIDTH - 1U, &cx) ||
        (index >= g_frame_length) || (g_frame[index++] != ',') ||
        !k230_vision_parse_decimal(g_frame, g_frame_length,
            &index, K230_VISION_IMAGE_HEIGHT - 1U, &cy) ||
        (index != g_frame_length)) {
        return false;
    }

    g_snapshot.online = true;
    g_snapshot.target_valid = valid != 0U;
    g_snapshot.cx = cx;
    g_snapshot.cy = cy;
    g_snapshot.error_x = (int16_t) ((int16_t) cx -
        K230_VISION_CENTER_X);
    g_snapshot.error_y = (int16_t) ((int16_t) cy -
        K230_VISION_CENTER_Y);
    g_snapshot.frame_sequence++;
    g_snapshot.last_frame_ms = now_ms;
    return true;
}

static bool k230_vision_parse_decimal(const char *text,
    uint8_t length, uint8_t *index, uint16_t maximum,
    uint16_t *value)
{
    uint32_t parsed = 0U;
    uint8_t start;

    if ((text == NULL) || (index == NULL) || (value == NULL) ||
        (*index >= length)) {
        return false;
    }

    start = *index;
    while ((*index < length) &&
        (text[*index] >= '0') && (text[*index] <= '9')) {
        parsed = (parsed * 10U) +
            (uint32_t) (text[*index] - '0');
        if (parsed > maximum) {
            return false;
        }
        (*index)++;
    }
    if (*index == start) {
        return false;
    }

    *value = (uint16_t) parsed;
    return true;
}
