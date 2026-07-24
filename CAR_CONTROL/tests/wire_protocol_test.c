#include "wire_protocol.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

static uint16_t make_frame(uint8_t type, uint16_t sequence,
    const uint8_t *payload, uint8_t payload_length, uint8_t *output)
{
    uint16_t length = WireProtocol_Encode(type, sequence, payload,
        payload_length, output, WIRE_PROTOCOL_MAX_FRAME_SIZE);
    assert(length != 0U);
    return length;
}
static void test_crc_known_vector(void)
{
    static const uint8_t vector[] = "123456789";
    assert(WireProtocol_Crc16Ccitt(vector,
        (uint16_t) (sizeof(vector) - 1U), 0xFFFFU) == 0x29B1U);
}

static void test_fragmented_and_concatenated_frames(void)
{
    uint8_t first[WIRE_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t second[WIRE_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t payload[] = {WIRE_ROLE_K230, 0x11U, 0x22U};
    uint16_t first_length = make_frame(WIRE_MESSAGE_HEARTBEAT,
        0x1234U, payload, sizeof(payload), first);
    uint16_t second_length = make_frame(WIRE_MESSAGE_STATUS,
        0x1235U, NULL, 0U, second);
    wire_parser_t parser;
    wire_frame_t frame;
    uint16_t index;

    WireProtocol_ParserInit(&parser);
    for (index = 0U; index < first_length; index++) {
        bool completed = WireProtocol_ParserFeed(&parser,
            first[index], &frame);
        assert(completed == (index == (first_length - 1U)));
    }
    assert(frame.type == WIRE_MESSAGE_HEARTBEAT);
    assert(frame.sequence == 0x1234U);
    assert(frame.length == sizeof(payload));
    assert(memcmp(frame.payload, payload, sizeof(payload)) == 0);
    for (index = 0U; index < second_length; index++) {
        (void) WireProtocol_ParserFeed(&parser, second[index], &frame);
    }
    assert(frame.type == WIRE_MESSAGE_STATUS);
    assert(frame.sequence == 0x1235U);
    assert(parser.stats.valid_frame_count == 2U);
}

static void test_bad_crc_oversize_and_version_recover(void)
{
    uint8_t bad[WIRE_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t good[WIRE_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t oversize[] = {
        WIRE_PROTOCOL_MAGIC_0, WIRE_PROTOCOL_MAGIC_1,
        WIRE_PROTOCOL_VERSION, WIRE_MESSAGE_HELLO, 0U, 0U,
        WIRE_PROTOCOL_MAX_PAYLOAD + 1U
    };
    uint8_t bad_version[] = {
        WIRE_PROTOCOL_MAGIC_0, WIRE_PROTOCOL_MAGIC_1,
        WIRE_PROTOCOL_VERSION + 1U
    };
    uint16_t bad_length = make_frame(WIRE_MESSAGE_HELLO,
        1U, NULL, 0U, bad);
    uint16_t good_length = make_frame(WIRE_MESSAGE_HEARTBEAT,
        2U, NULL, 0U, good);
    wire_parser_t parser;
    wire_frame_t frame;
    uint16_t index;

    bad[bad_length - 1U] ^= 0x80U;
    WireProtocol_ParserInit(&parser);
    for (index = 0U; index < bad_length; index++) {
        assert(!WireProtocol_ParserFeed(&parser, bad[index], &frame));
    }
    for (index = 0U; index < sizeof(oversize); index++) {
        assert(!WireProtocol_ParserFeed(&parser, oversize[index], &frame));
    }
    for (index = 0U; index < sizeof(bad_version); index++) {
        assert(!WireProtocol_ParserFeed(&parser,
            bad_version[index], &frame));
    }
    for (index = 0U; index < good_length; index++) {
        (void) WireProtocol_ParserFeed(&parser, good[index], &frame);
    }
    assert(frame.sequence == 2U);
    assert(parser.stats.valid_frame_count == 1U);
    assert(parser.stats.crc_error_count == 1U);
    assert(parser.stats.length_error_count == 1U);
    assert(parser.stats.version_error_count == 1U);
}

static void test_sequence_wrap_rules(void)
{
    assert(WireProtocol_SequenceIsNewer(0U, 0xFFFFU));
    assert(WireProtocol_SequenceIsNewer(8U, 7U));
    assert(!WireProtocol_SequenceIsNewer(7U, 7U));
    assert(!WireProtocol_SequenceIsNewer(6U, 7U));
    assert(!WireProtocol_SequenceIsNewer(0x8007U, 7U));
}

int main(void)
{
    test_crc_known_vector();
    test_fragmented_and_concatenated_frames();
    test_bad_crc_oversize_and_version_recover();
    test_sequence_wrap_rules();
    return 0;
}
