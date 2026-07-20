#include "k230_vision_link.h"
#include "vision_uart.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MOCK_RX_CAPACITY 512U

static uint8_t g_mock_rx[MOCK_RX_CAPACITY];
static uint16_t g_mock_head;
static uint16_t g_mock_tail;
static vision_uart_stats_t g_mock_stats;

static void mock_reset(void)
{
    g_mock_head = 0U;
    g_mock_tail = 0U;
    g_mock_stats.received_byte_count = 0U;
    g_mock_stats.overflow_count = 0U;
}

static void mock_feed(const char *text)
{
    while (*text != '\0') {
        assert(g_mock_head < MOCK_RX_CAPACITY);
        g_mock_rx[g_mock_head++] = (uint8_t) *text++;
        g_mock_stats.received_byte_count++;
    }
}

bool VisionUart_ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_mock_tail == g_mock_head)) {
        return false;
    }
    *byte = g_mock_rx[g_mock_tail++];
    return true;
}

bool VisionUart_GetStats(vision_uart_stats_t *stats)
{
    if (stats == NULL) {
        return false;
    }
    *stats = g_mock_stats;
    return true;
}

static k230_vision_snapshot_t get_snapshot(void)
{
    k230_vision_snapshot_t snapshot;

    assert(K230VisionLink_GetSnapshot(&snapshot));
    return snapshot;
}

static void test_fragmented_valid_frame(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(0U);
    mock_feed("noise@1,203");
    K230VisionLink_Task(10U);
    assert(!get_snapshot().online);

    mock_feed(",117#");
    K230VisionLink_Task(20U);
    snapshot = get_snapshot();
    assert(snapshot.online);
    assert(snapshot.target_valid);
    assert(snapshot.cx == 203U);
    assert(snapshot.cy == 117U);
    assert(snapshot.error_x == 3);
    assert(snapshot.error_y == -3);
    assert(snapshot.frame_sequence == 1U);
    assert(snapshot.last_frame_ms == 20U);
}

static void test_lost_target_is_still_online(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(100U);
    mock_feed("@0,000,000#");
    K230VisionLink_Task(110U);
    snapshot = get_snapshot();
    assert(snapshot.online);
    assert(!snapshot.target_valid);
    assert(snapshot.frame_sequence == 1U);
    assert(snapshot.cx == 0U);
    assert(snapshot.cy == 0U);
}

static void test_malformed_and_partial_do_not_refresh_link(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(0U);
    mock_feed("@1,200,120#");
    K230VisionLink_Task(10U);
    mock_feed("garbage@1,201");
    K230VisionLink_Task(100U);
    snapshot = get_snapshot();
    assert(snapshot.last_frame_ms == 10U);
    assert(snapshot.online);

    K230VisionLink_Task(160U);
    snapshot = get_snapshot();
    assert(!snapshot.online);
    assert(!snapshot.target_valid);
    assert(snapshot.timeout_count == 1U);
}

static void test_ranges_and_valid_field_are_strict(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(0U);
    mock_feed("@2,200,120#@1,400,120#@1,399,240#");
    K230VisionLink_Task(1U);
    snapshot = get_snapshot();
    assert(!snapshot.online);
    assert(snapshot.frame_sequence == 0U);
    assert(snapshot.parse_error_count == 3U);
}

static void test_resync_and_concatenated_frames(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(0U);
    mock_feed("@1,20@1,100,110#@1,399,239#");
    K230VisionLink_Task(5U);
    snapshot = get_snapshot();
    assert(snapshot.resync_count == 1U);
    assert(snapshot.parse_error_count == 0U);
    assert(snapshot.frame_sequence == 2U);
    assert(snapshot.cx == 399U);
    assert(snapshot.cy == 239U);
}

static void test_oversize_frame_recovers(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(0U);
    mock_feed("@1234567890123456789012345#@1,200,120#");
    g_mock_stats.overflow_count = 2U;
    K230VisionLink_Task(7U);
    snapshot = get_snapshot();
    assert(snapshot.parse_error_count == 1U);
    assert(snapshot.frame_sequence == 1U);
    assert(snapshot.overflow_count == 2U);
    assert(snapshot.received_byte_count == strlen(
        "@1234567890123456789012345#@1,200,120#"));
}

static void test_timeout_is_counted_once_per_online_period(void)
{
    k230_vision_snapshot_t snapshot;

    mock_reset();
    K230VisionLink_Init(0U);
    mock_feed("@1,200,120#");
    K230VisionLink_Task(1U);
    K230VisionLink_Task(151U);
    K230VisionLink_Task(300U);
    snapshot = get_snapshot();
    assert(!snapshot.online);
    assert(snapshot.timeout_count == 1U);
}

int main(void)
{
    test_fragmented_valid_frame();
    test_lost_target_is_still_online();
    test_malformed_and_partial_do_not_refresh_link();
    test_ranges_and_valid_field_are_strict();
    test_resync_and_concatenated_frames();
    test_oversize_frame_recovers();
    test_timeout_is_counted_once_per_online_period();
    return 0;
}
