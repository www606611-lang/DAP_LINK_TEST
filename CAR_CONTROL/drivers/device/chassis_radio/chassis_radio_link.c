#include "chassis_radio_link.h"

#include "radio_uart.h"
#include "wire_protocol.h"

#include <stddef.h>

#define CHASSIS_RADIO_RX_BUDGET       128U
#define CHASSIS_RADIO_HELLO_MS        1000U
#define CHASSIS_RADIO_HEARTBEAT_MS    250U
#define CHASSIS_RADIO_STATUS_MS       500U

typedef struct {
    bool seen;
    uint16_t sequence;
    uint32_t last_rx_ms;
} chassis_radio_peer_t;

static wire_parser_t g_parser;
static chassis_radio_snapshot_t g_snapshot;
static chassis_radio_peer_t g_esp32;
static chassis_radio_peer_t g_k230;
static uint16_t g_tx_sequence;
static uint32_t g_last_presence_tx_ms;
static uint32_t g_last_status_tx_ms;

static void chassis_radio_handle_frame(const wire_frame_t *frame,
    uint32_t now_ms);
static bool chassis_radio_accept_sequence(chassis_radio_peer_t *peer,
    uint16_t sequence, uint32_t now_ms);
static void chassis_radio_update_timeouts(uint32_t now_ms);
static bool chassis_radio_send(uint8_t type, const uint8_t *payload,
    uint8_t payload_length);
static void chassis_radio_send_presence(uint32_t now_ms);
static void chassis_radio_send_status(uint32_t now_ms);
static void chassis_radio_write_u32(uint8_t *output, uint32_t value);

void ChassisRadioLink_Init(uint32_t now_ms)
{
    WireProtocol_ParserInit(&g_parser);
    g_snapshot.online = false;
    g_snapshot.esp32_online = false;
    g_snapshot.k230_online = false;
    g_snapshot.last_role = 0U;
    g_snapshot.status_flags = CHASSIS_RADIO_STATUS_HIGH_Z;
    g_snapshot.last_frame_ms = now_ms;
    g_snapshot.rx_frame_count = 0U;
    g_snapshot.tx_frame_count = 0U;
    g_snapshot.duplicate_count = 0U;
    g_snapshot.out_of_order_count = 0U;
    g_snapshot.shadow_command_count = 0U;
    g_snapshot.unsupported_count = 0U;
    g_snapshot.timeout_count = 0U;
    g_snapshot.crc_error_count = 0U;
    g_snapshot.length_error_count = 0U;
    g_snapshot.version_error_count = 0U;
    g_snapshot.resync_count = 0U;
    g_snapshot.received_byte_count = 0U;
    g_snapshot.rx_overflow_count = 0U;
    g_snapshot.transmitted_byte_count = 0U;
    g_snapshot.tx_rejected_count = 0U;
    g_esp32.seen = false;
    g_esp32.sequence = 0U;
    g_esp32.last_rx_ms = now_ms;
    g_k230.seen = false;
    g_k230.sequence = 0U;
    g_k230.last_rx_ms = now_ms;
    g_tx_sequence = 0U;
    g_last_presence_tx_ms = now_ms - CHASSIS_RADIO_HELLO_MS;
    g_last_status_tx_ms = now_ms - CHASSIS_RADIO_STATUS_MS;
}

void ChassisRadioLink_SetStatusFlags(uint8_t status_flags)
{
    g_snapshot.status_flags = status_flags;
}

void ChassisRadioLink_Task(uint32_t now_ms)
{
    radio_uart_stats_t uart_stats;
    wire_frame_t frame;
    uint16_t processed = 0U;
    uint8_t byte;

    while ((processed < CHASSIS_RADIO_RX_BUDGET) &&
        RadioUart_ReadByte(&byte)) {
        processed++;
        if (WireProtocol_ParserFeed(&g_parser, byte, &frame)) {
            chassis_radio_handle_frame(&frame, now_ms);
        }
    }

    chassis_radio_update_timeouts(now_ms);
    chassis_radio_send_presence(now_ms);
    chassis_radio_send_status(now_ms);

    g_snapshot.crc_error_count = g_parser.stats.crc_error_count;
    g_snapshot.length_error_count = g_parser.stats.length_error_count;
    g_snapshot.version_error_count = g_parser.stats.version_error_count;
    g_snapshot.resync_count = g_parser.stats.resync_count;
    if (RadioUart_GetStats(&uart_stats)) {
        g_snapshot.received_byte_count = uart_stats.received_byte_count;
        g_snapshot.rx_overflow_count = uart_stats.rx_overflow_count;
        g_snapshot.transmitted_byte_count =
            uart_stats.transmitted_byte_count;
        g_snapshot.tx_rejected_count = uart_stats.tx_rejected_count;
    }
}

bool ChassisRadioLink_GetSnapshot(chassis_radio_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static void chassis_radio_handle_frame(const wire_frame_t *frame,
    uint32_t now_ms)
{
    chassis_radio_peer_t *peer;
    uint8_t role;

    if (frame == NULL) {
        return;
    }
    if ((frame->type == WIRE_MESSAGE_CONTROL_SHADOW) ||
        (frame->type == WIRE_MESSAGE_EMERGENCY_STOP)) {
        g_snapshot.shadow_command_count++;
        return;
    }
    if ((frame->type != WIRE_MESSAGE_HELLO) &&
        (frame->type != WIRE_MESSAGE_HEARTBEAT) &&
        (frame->type != WIRE_MESSAGE_STATUS)) {
        g_snapshot.unsupported_count++;
        return;
    }
    if (frame->length < 1U) {
        g_snapshot.unsupported_count++;
        return;
    }

    role = frame->payload[0];
    if (role == WIRE_ROLE_ESP32) {
        peer = &g_esp32;
    } else if (role == WIRE_ROLE_K230) {
        peer = &g_k230;
    } else {
        g_snapshot.unsupported_count++;
        return;
    }
    if (frame->type == WIRE_MESSAGE_HELLO) {
        peer->seen = true;
        peer->sequence = frame->sequence;
        peer->last_rx_ms = now_ms;
    } else if (!chassis_radio_accept_sequence(peer,
            frame->sequence, now_ms)) {
        return;
    }

    g_snapshot.last_role = role;
    g_snapshot.last_frame_ms = now_ms;
    g_snapshot.rx_frame_count++;
}

static bool chassis_radio_accept_sequence(chassis_radio_peer_t *peer,
    uint16_t sequence, uint32_t now_ms)
{
    if (!peer->seen) {
        peer->seen = true;
    } else if (sequence == peer->sequence) {
        g_snapshot.duplicate_count++;
        return false;
    } else if (!WireProtocol_SequenceIsNewer(sequence, peer->sequence)) {
        g_snapshot.out_of_order_count++;
        return false;
    }

    peer->sequence = sequence;
    peer->last_rx_ms = now_ms;
    return true;
}

static void chassis_radio_update_timeouts(uint32_t now_ms)
{
    bool old_esp32_online = g_snapshot.esp32_online;
    bool old_k230_online = g_snapshot.k230_online;

    g_snapshot.esp32_online = g_esp32.seen &&
        ((uint32_t) (now_ms - g_esp32.last_rx_ms) <
            CHASSIS_RADIO_OFFLINE_TIMEOUT_MS);
    g_snapshot.k230_online = g_k230.seen &&
        ((uint32_t) (now_ms - g_k230.last_rx_ms) <
            CHASSIS_RADIO_OFFLINE_TIMEOUT_MS);
    if ((old_esp32_online && !g_snapshot.esp32_online) ||
        (old_k230_online && !g_snapshot.k230_online)) {
        g_snapshot.timeout_count++;
    }
    g_snapshot.online = g_snapshot.esp32_online &&
        g_snapshot.k230_online;
}

static bool chassis_radio_send(uint8_t type, const uint8_t *payload,
    uint8_t payload_length)
{
    uint8_t frame[WIRE_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t length = WireProtocol_Encode(type, g_tx_sequence,
        payload, payload_length, frame, sizeof(frame));

    if ((length == 0U) || !RadioUart_Write(frame, length)) {
        return false;
    }
    g_tx_sequence++;
    g_snapshot.tx_frame_count++;
    return true;
}

static void chassis_radio_send_presence(uint32_t now_ms)
{
    uint32_t interval = g_snapshot.esp32_online ?
        CHASSIS_RADIO_HEARTBEAT_MS : CHASSIS_RADIO_HELLO_MS;
    uint8_t payload[5];

    if ((uint32_t) (now_ms - g_last_presence_tx_ms) < interval) {
        return;
    }
    payload[0] = WIRE_ROLE_CHASSIS;
    chassis_radio_write_u32(&payload[1], now_ms);
    g_last_presence_tx_ms = now_ms;
    (void) chassis_radio_send(g_snapshot.esp32_online ?
        WIRE_MESSAGE_HEARTBEAT : WIRE_MESSAGE_HELLO,
        payload, sizeof(payload));
}

static void chassis_radio_send_status(uint32_t now_ms)
{
    uint8_t payload[6];

    if ((uint32_t) (now_ms - g_last_status_tx_ms) <
        CHASSIS_RADIO_STATUS_MS) {
        return;
    }
    payload[0] = WIRE_ROLE_CHASSIS;
    payload[1] = g_snapshot.status_flags;
    chassis_radio_write_u32(&payload[2], now_ms);
    g_last_status_tx_ms = now_ms;
    (void) chassis_radio_send(WIRE_MESSAGE_STATUS,
        payload, sizeof(payload));
}

static void chassis_radio_write_u32(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t) value;
    output[1] = (uint8_t) (value >> 8);
    output[2] = (uint8_t) (value >> 16);
    output[3] = (uint8_t) (value >> 24);
}
