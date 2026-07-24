#ifndef DRIVERS_UTILITY_WIRE_PROTOCOL_H
#define DRIVERS_UTILITY_WIRE_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define WIRE_PROTOCOL_MAGIC_0          0xA5U
#define WIRE_PROTOCOL_MAGIC_1          0x5AU
#define WIRE_PROTOCOL_VERSION          1U
#define WIRE_PROTOCOL_MAX_PAYLOAD      64U
#define WIRE_PROTOCOL_FRAME_OVERHEAD   9U
#define WIRE_PROTOCOL_MAX_FRAME_SIZE   \
    (WIRE_PROTOCOL_FRAME_OVERHEAD + WIRE_PROTOCOL_MAX_PAYLOAD)

typedef enum {
    WIRE_MESSAGE_HELLO = 0x01,
    WIRE_MESSAGE_HEARTBEAT = 0x02,
    WIRE_MESSAGE_CONTROL_SHADOW = 0x10,
    WIRE_MESSAGE_EMERGENCY_STOP = 0x11,
    WIRE_MESSAGE_STATUS = 0x20,
    WIRE_MESSAGE_ACK = 0x7E,
    WIRE_MESSAGE_NACK = 0x7F
} wire_message_type_t;

typedef enum {
    WIRE_ROLE_K230 = 1,
    WIRE_ROLE_ESP32 = 2,
    WIRE_ROLE_CHASSIS = 3
} wire_role_t;

typedef struct {
    uint8_t type;
    uint16_t sequence;
    uint8_t length;
    uint8_t payload[WIRE_PROTOCOL_MAX_PAYLOAD];
} wire_frame_t;

typedef struct {
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t version_error_count;
    uint32_t resync_count;
} wire_parser_stats_t;

typedef struct {
    uint8_t state;
    wire_frame_t frame;
    uint8_t payload_index;
    uint16_t crc;
    uint16_t received_crc;
    wire_parser_stats_t stats;
} wire_parser_t;

uint16_t WireProtocol_Crc16Ccitt(const uint8_t *data,
    uint16_t length, uint16_t initial);
uint16_t WireProtocol_Encode(uint8_t type, uint16_t sequence,
    const uint8_t *payload, uint8_t payload_length,
    uint8_t *output, uint16_t output_capacity);
bool WireProtocol_SequenceIsNewer(uint16_t sequence,
    uint16_t previous);
void WireProtocol_ParserInit(wire_parser_t *parser);
bool WireProtocol_ParserFeed(wire_parser_t *parser, uint8_t byte,
    wire_frame_t *completed_frame);

#endif
