#include "wire_protocol.h"

#include <stddef.h>

enum {
    WIRE_STATE_WAIT_MAGIC_0 = 0,
    WIRE_STATE_WAIT_MAGIC_1,
    WIRE_STATE_READ_VERSION,
    WIRE_STATE_READ_TYPE,
    WIRE_STATE_READ_SEQUENCE_0,
    WIRE_STATE_READ_SEQUENCE_1,
    WIRE_STATE_READ_LENGTH,
    WIRE_STATE_READ_PAYLOAD,
    WIRE_STATE_READ_CRC_0,
    WIRE_STATE_READ_CRC_1
};

static void wire_protocol_reset_frame(wire_parser_t *parser);
static void wire_protocol_resync(wire_parser_t *parser, uint8_t byte);
static void wire_protocol_update_crc(wire_parser_t *parser, uint8_t byte);

uint16_t WireProtocol_Crc16Ccitt(const uint8_t *data,
    uint16_t length, uint16_t initial)
{
    uint16_t crc = initial;
    uint16_t index;

    if ((data == NULL) && (length != 0U)) {
        return crc;
    }

    for (index = 0U; index < length; index++) {
        uint8_t bit;

        crc ^= (uint16_t) data[index] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t) ((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t) (crc << 1);
            }
        }
    }
    return crc;
}
uint16_t WireProtocol_Encode(uint8_t type, uint16_t sequence,
    const uint8_t *payload, uint8_t payload_length,
    uint8_t *output, uint16_t output_capacity)
{
    uint16_t crc;
    uint16_t index;
    uint16_t frame_length = (uint16_t)
        (WIRE_PROTOCOL_FRAME_OVERHEAD + payload_length);

    if ((payload_length > WIRE_PROTOCOL_MAX_PAYLOAD) ||
        (output == NULL) || (output_capacity < frame_length) ||
        ((payload_length != 0U) && (payload == NULL))) {
        return 0U;
    }

    output[0] = WIRE_PROTOCOL_MAGIC_0;
    output[1] = WIRE_PROTOCOL_MAGIC_1;
    output[2] = WIRE_PROTOCOL_VERSION;
    output[3] = type;
    output[4] = (uint8_t) (sequence & 0xFFU);
    output[5] = (uint8_t) (sequence >> 8);
    output[6] = payload_length;
    for (index = 0U; index < payload_length; index++) {
        output[7U + index] = payload[index];
    }
    crc = WireProtocol_Crc16Ccitt(&output[2],
        (uint16_t) (5U + payload_length), 0xFFFFU);
    output[7U + payload_length] = (uint8_t) (crc & 0xFFU);
    output[8U + payload_length] = (uint8_t) (crc >> 8);
    return frame_length;
}

bool WireProtocol_SequenceIsNewer(uint16_t sequence,
    uint16_t previous)
{
    uint16_t delta = (uint16_t) (sequence - previous);
    return (delta != 0U) && (delta < 0x8000U);
}

void WireProtocol_ParserInit(wire_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    parser->stats.valid_frame_count = 0U;
    parser->stats.crc_error_count = 0U;
    parser->stats.length_error_count = 0U;
    parser->stats.version_error_count = 0U;
    parser->stats.resync_count = 0U;
    wire_protocol_reset_frame(parser);
}

bool WireProtocol_ParserFeed(wire_parser_t *parser, uint8_t byte,
    wire_frame_t *completed_frame)
{
    uint8_t index;

    if ((parser == NULL) || (completed_frame == NULL)) {
        return false;
    }

    switch (parser->state) {
        case WIRE_STATE_WAIT_MAGIC_0:
            if (byte == WIRE_PROTOCOL_MAGIC_0) {
                parser->state = WIRE_STATE_WAIT_MAGIC_1;
            }
            return false;

        case WIRE_STATE_WAIT_MAGIC_1:
            if (byte == WIRE_PROTOCOL_MAGIC_1) {
                parser->state = WIRE_STATE_READ_VERSION;
            } else if (byte != WIRE_PROTOCOL_MAGIC_0) {
                parser->state = WIRE_STATE_WAIT_MAGIC_0;
                parser->stats.resync_count++;
            }
            return false;

        case WIRE_STATE_READ_VERSION:
            if (byte != WIRE_PROTOCOL_VERSION) {
                parser->stats.version_error_count++;
                wire_protocol_resync(parser, byte);
                return false;
            }
            parser->crc = 0xFFFFU;
            wire_protocol_update_crc(parser, byte);
            parser->state = WIRE_STATE_READ_TYPE;
            return false;

        case WIRE_STATE_READ_TYPE:
            parser->frame.type = byte;
            wire_protocol_update_crc(parser, byte);
            parser->state = WIRE_STATE_READ_SEQUENCE_0;
            return false;

        case WIRE_STATE_READ_SEQUENCE_0:
            parser->frame.sequence = byte;
            wire_protocol_update_crc(parser, byte);
            parser->state = WIRE_STATE_READ_SEQUENCE_1;
            return false;

        case WIRE_STATE_READ_SEQUENCE_1:
            parser->frame.sequence |= (uint16_t) byte << 8;
            wire_protocol_update_crc(parser, byte);
            parser->state = WIRE_STATE_READ_LENGTH;
            return false;

        case WIRE_STATE_READ_LENGTH:
            if (byte > WIRE_PROTOCOL_MAX_PAYLOAD) {
                parser->stats.length_error_count++;
                wire_protocol_resync(parser, byte);
                return false;
            }
            parser->frame.length = byte;
            parser->payload_index = 0U;
            wire_protocol_update_crc(parser, byte);
            parser->state = (byte == 0U) ?
                WIRE_STATE_READ_CRC_0 : WIRE_STATE_READ_PAYLOAD;
            return false;

        case WIRE_STATE_READ_PAYLOAD:
            parser->frame.payload[parser->payload_index++] = byte;
            wire_protocol_update_crc(parser, byte);
            if (parser->payload_index >= parser->frame.length) {
                parser->state = WIRE_STATE_READ_CRC_0;
            }
            return false;

        case WIRE_STATE_READ_CRC_0:
            parser->received_crc = byte;
            parser->state = WIRE_STATE_READ_CRC_1;
            return false;

        case WIRE_STATE_READ_CRC_1:
            parser->received_crc |= (uint16_t) byte << 8;
            if (parser->received_crc != parser->crc) {
                parser->stats.crc_error_count++;
                wire_protocol_resync(parser, byte);
                return false;
            }
            completed_frame->type = parser->frame.type;
            completed_frame->sequence = parser->frame.sequence;
            completed_frame->length = parser->frame.length;
            for (index = 0U; index < parser->frame.length; index++) {
                completed_frame->payload[index] =
                    parser->frame.payload[index];
            }
            parser->stats.valid_frame_count++;
            wire_protocol_reset_frame(parser);
            return true;

        default:
            wire_protocol_reset_frame(parser);
            return false;
    }
}

static void wire_protocol_reset_frame(wire_parser_t *parser)
{
    parser->state = WIRE_STATE_WAIT_MAGIC_0;
    parser->frame.type = 0U;
    parser->frame.sequence = 0U;
    parser->frame.length = 0U;
    parser->payload_index = 0U;
    parser->crc = 0xFFFFU;
    parser->received_crc = 0U;
}

static void wire_protocol_resync(wire_parser_t *parser, uint8_t byte)
{
    parser->stats.resync_count++;
    wire_protocol_reset_frame(parser);
    if (byte == WIRE_PROTOCOL_MAGIC_0) {
        parser->state = WIRE_STATE_WAIT_MAGIC_1;
    }
}

static void wire_protocol_update_crc(wire_parser_t *parser, uint8_t byte)
{
    parser->crc = WireProtocol_Crc16Ccitt(&byte, 1U, parser->crc);
}
