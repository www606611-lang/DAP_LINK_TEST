#include "chassis_radio_link.h"
#include "radio_uart.h"
#include "wire_protocol.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

#define MOCK_CAPACITY 1024U

static uint8_t g_rx[MOCK_CAPACITY];
static uint16_t g_rx_head;
static uint16_t g_rx_tail;
static uint8_t g_tx[MOCK_CAPACITY];
static uint16_t g_tx_length;
static radio_uart_stats_t g_uart_stats;

static void mock_reset(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_tx_length = 0U;
    memset(&g_uart_stats, 0, sizeof(g_uart_stats));
}

static void mock_feed_frame(uint8_t type, uint16_t sequence,
    uint8_t role, bool corrupt_crc)
{
    uint8_t payload[] = {role, 0U, 0U, 0U, 0U};
    uint8_t frame[WIRE_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t length = WireProtocol_Encode(type, sequence, payload,
        sizeof(payload), frame, sizeof(frame));
    uint16_t index;

    assert(length != 0U);
    if (corrupt_crc) {
        frame[length - 1U] ^= 0x80U;
    }
    for (index = 0U; index < length; index++) {
        assert(g_rx_head < MOCK_CAPACITY);
        g_rx[g_rx_head++] = frame[index];
        g_uart_stats.received_byte_count++;
    }
}

bool RadioUart_ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_rx_tail == g_rx_head)) {
        return false;
    }
    *byte = g_rx[g_rx_tail++];
    return true;
}

bool RadioUart_Write(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) ||
        ((uint32_t) g_tx_length + length > MOCK_CAPACITY)) {
        g_uart_stats.tx_rejected_count++;
        return false;
    }
    memcpy(&g_tx[g_tx_length], data, length);
    g_tx_length = (uint16_t) (g_tx_length + length);
    g_uart_stats.transmitted_byte_count += length;
    return true;
}

bool RadioUart_GetStats(radio_uart_stats_t *stats)
{
    if (stats == NULL) {
        return false;
    }
    *stats = g_uart_stats;
    return true;
}

static chassis_radio_snapshot_t snapshot(void)
{
    chassis_radio_snapshot_t value;
    assert(ChassisRadioLink_GetSnapshot(&value));
    return value;
}

static void test_two_role_online_sequence_and_timeout(void)
{
    chassis_radio_snapshot_t state;

    mock_reset();
    ChassisRadioLink_Init(0U);
    mock_feed_frame(WIRE_MESSAGE_HELLO, 1U,
        WIRE_ROLE_ESP32, false);
    ChassisRadioLink_Task(10U);
    state = snapshot();
    assert(state.esp32_online);
    assert(!state.k230_online);
    assert(!state.online);

    mock_feed_frame(WIRE_MESSAGE_HEARTBEAT, 7U,
        WIRE_ROLE_K230, false);
    ChassisRadioLink_Task(20U);
    state = snapshot();
    assert(state.online);
    assert(state.rx_frame_count == 2U);

    mock_feed_frame(WIRE_MESSAGE_HEARTBEAT, 7U,
        WIRE_ROLE_K230, false);
    mock_feed_frame(WIRE_MESSAGE_HEARTBEAT, 6U,
        WIRE_ROLE_K230, false);
    ChassisRadioLink_Task(30U);
    state = snapshot();
    assert(state.duplicate_count == 1U);
    assert(state.out_of_order_count == 1U);
    assert(state.rx_frame_count == 2U);

    ChassisRadioLink_Task(1010U);
    state = snapshot();
    assert(!state.esp32_online);
    assert(state.k230_online);
    assert(!state.online);
    assert(state.timeout_count == 1U);

    mock_feed_frame(WIRE_MESSAGE_HELLO, 0U,
        WIRE_ROLE_ESP32, false);
    ChassisRadioLink_Task(1020U);
    state = snapshot();
    assert(state.esp32_online);
    assert(state.rx_frame_count == 3U);
    mock_feed_frame(WIRE_MESSAGE_HEARTBEAT, 1U,
        WIRE_ROLE_ESP32, false);
    ChassisRadioLink_Task(1030U);
    assert(snapshot().rx_frame_count == 4U);
}

static void test_corruption_and_shadow_have_no_link_ownership(void)
{
    chassis_radio_snapshot_t state;

    mock_reset();
    ChassisRadioLink_Init(0U);
    mock_feed_frame(WIRE_MESSAGE_HEARTBEAT, 1U,
        WIRE_ROLE_K230, true);
    mock_feed_frame(WIRE_MESSAGE_CONTROL_SHADOW, 2U,
        WIRE_ROLE_K230, false);
    mock_feed_frame(WIRE_MESSAGE_EMERGENCY_STOP, 3U,
        WIRE_ROLE_K230, false);
    ChassisRadioLink_Task(5U);
    state = snapshot();
    assert(!state.online);
    assert(!state.k230_online);
    assert(state.crc_error_count == 1U);
    assert(state.shadow_command_count == 2U);
    assert(state.rx_frame_count == 0U);
}

static void test_outbound_is_only_presence_and_status(void)
{
    wire_parser_t parser;
    wire_frame_t frame;
    uint16_t index;
    uint32_t completed = 0U;

    mock_reset();
    ChassisRadioLink_Init(0U);
    ChassisRadioLink_SetStatusFlags(CHASSIS_RADIO_STATUS_HIGH_Z);
    ChassisRadioLink_Task(0U);
    assert(g_tx_length != 0U);

    WireProtocol_ParserInit(&parser);
    for (index = 0U; index < g_tx_length; index++) {
        if (WireProtocol_ParserFeed(&parser, g_tx[index], &frame)) {
            assert((frame.type == WIRE_MESSAGE_HELLO) ||
                (frame.type == WIRE_MESSAGE_STATUS));
            assert(frame.payload[0] == WIRE_ROLE_CHASSIS);
            if (frame.type == WIRE_MESSAGE_STATUS) {
                assert((frame.payload[1] &
                    CHASSIS_RADIO_STATUS_HIGH_Z) != 0U);
            }
            completed++;
        }
    }
    assert(completed == 2U);
    assert(snapshot().tx_frame_count == 2U);
}

int main(void)
{
    test_two_role_online_sequence_and_timeout();
    test_corruption_and_shadow_have_no_link_ownership();
    test_outbound_is_only_presence_and_status();
    return 0;
}
