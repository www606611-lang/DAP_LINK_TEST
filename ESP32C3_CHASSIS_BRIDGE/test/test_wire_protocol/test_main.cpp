#include <unity.h>

#include "WireProtocol.h"

using namespace WireProtocol;

void setUp() {}
void tearDown() {}

namespace {

void testCrcKnownVector() {
    static const uint8_t input[] = {'1', '2', '3', '4', '5',
                                    '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16Ccitt(input, sizeof(input)));
}

void testFragmentedFrameRoundTrip() {
    static const uint8_t payload[] = {kRoleK230, 0x11, 0x22, 0x33, 0x44};
    uint8_t bytes[kMaxFrameSize] = {};
    const size_t length = encodeFrame(kHeartbeat, 0x1234, payload,
                                      sizeof(payload), bytes, sizeof(bytes));
    TEST_ASSERT_EQUAL_UINT(14, length);

    Parser parser;
    Frame frame{};
    bool completed = false;
    for (size_t index = 0; index < length; ++index) {
        completed = parser.feed(bytes[index], frame);
        TEST_ASSERT_EQUAL(index + 1 == length, completed);
    }
    TEST_ASSERT_EQUAL_HEX8(kHeartbeat, frame.type);
    TEST_ASSERT_EQUAL_HEX16(0x1234, frame.sequence);
    TEST_ASSERT_EQUAL_UINT8(sizeof(payload), frame.length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, frame.payload, sizeof(payload));
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().validFrames);
}

void testBadCrcResynchronizes() {
    uint8_t bad[kMaxFrameSize] = {};
    uint8_t good[kMaxFrameSize] = {};
    size_t badLength = encodeFrame(kHello, 1, nullptr, 0, bad, sizeof(bad));
    const size_t goodLength =
        encodeFrame(kStatus, 2, nullptr, 0, good, sizeof(good));
    bad[badLength - 1] ^= 0x80;

    Parser parser;
    Frame frame{};
    for (size_t index = 0; index < badLength; ++index) {
        TEST_ASSERT_FALSE(parser.feed(bad[index], frame));
    }
    bool completed = false;
    for (size_t index = 0; index < goodLength; ++index) {
        completed = parser.feed(good[index], frame);
    }
    TEST_ASSERT_TRUE(completed);
    TEST_ASSERT_EQUAL_HEX8(kStatus, frame.type);
    TEST_ASSERT_EQUAL_HEX16(2, frame.sequence);
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().crcErrors);
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().validFrames);
}

void testOversizeIsRejectedThenRecovers() {
    const uint8_t prefix[] = {kMagic0, kMagic1, kVersion, kHello, 0, 0,
                              static_cast<uint8_t>(kMaxPayload + 1)};
    uint8_t good[kMaxFrameSize] = {};
    const size_t goodLength =
        encodeFrame(kHeartbeat, 7, nullptr, 0, good, sizeof(good));
    Parser parser;
    Frame frame{};
    for (uint8_t value : prefix) {
        TEST_ASSERT_FALSE(parser.feed(value, frame));
    }
    bool completed = false;
    for (size_t index = 0; index < goodLength; ++index) {
        completed = parser.feed(good[index], frame);
    }
    TEST_ASSERT_TRUE(completed);
    TEST_ASSERT_EQUAL_HEX16(7, frame.sequence);
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().lengthErrors);
}

void testSequenceWrapRules() {
    TEST_ASSERT_TRUE(sequenceIsNewer(0, 0xFFFF));
    TEST_ASSERT_TRUE(sequenceIsNewer(8, 7));
    TEST_ASSERT_FALSE(sequenceIsNewer(7, 7));
    TEST_ASSERT_FALSE(sequenceIsNewer(6, 7));
    TEST_ASSERT_FALSE(sequenceIsNewer(0x8007, 7));
}

}  // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(testCrcKnownVector);
    RUN_TEST(testFragmentedFrameRoundTrip);
    RUN_TEST(testBadCrcResynchronizes);
    RUN_TEST(testOversizeIsRejectedThenRecovers);
    RUN_TEST(testSequenceWrapRules);
    return UNITY_END();
}
