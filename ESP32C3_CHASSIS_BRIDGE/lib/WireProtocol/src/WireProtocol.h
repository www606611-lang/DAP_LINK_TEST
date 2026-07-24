#pragma once

#include <stddef.h>
#include <stdint.h>

namespace WireProtocol {

constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxPayload = 64;
constexpr size_t kFrameOverhead = 9;
constexpr size_t kMaxFrameSize = kFrameOverhead + kMaxPayload;

enum MessageType : uint8_t {
    kHello = 0x01,
    kHeartbeat = 0x02,
    kControlShadow = 0x10,
    kEmergencyStop = 0x11,
    kStatus = 0x20,
    kAck = 0x7E,
    kNack = 0x7F,
};

enum Role : uint8_t {
    kRoleK230 = 1,
    kRoleEsp32 = 2,
    kRoleChassis = 3,
};

struct Frame {
    uint8_t type = 0;
    uint16_t sequence = 0;
    uint8_t length = 0;
    uint8_t payload[kMaxPayload] = {};
};

struct ParserStats {
    uint32_t validFrames = 0;
    uint32_t crcErrors = 0;
    uint32_t lengthErrors = 0;
    uint32_t versionErrors = 0;
    uint32_t resyncs = 0;
};

uint16_t crc16Ccitt(const uint8_t* data, size_t length,
                    uint16_t initial = 0xFFFF);

size_t encodeFrame(uint8_t type, uint16_t sequence, const uint8_t* payload,
                   size_t payloadLength, uint8_t* output,
                   size_t outputCapacity);

bool sequenceIsNewer(uint16_t sequence, uint16_t previous);

class Parser {
public:
    Parser();

    bool feed(uint8_t value, Frame& completedFrame);
    void reset();
    const ParserStats& stats() const;

private:
    enum class State : uint8_t {
        WaitMagic0,
        WaitMagic1,
        ReadVersion,
        ReadType,
        ReadSequence0,
        ReadSequence1,
        ReadLength,
        ReadPayload,
        ReadCrc0,
        ReadCrc1,
    };

    void resync(uint8_t value);
    void updateCrc(uint8_t value);

    State state_ = State::WaitMagic0;
    Frame frame_{};
    uint8_t payloadIndex_ = 0;
    uint16_t crc_ = 0xFFFF;
    uint16_t receivedCrc_ = 0;
    ParserStats stats_{};
};

}  // namespace WireProtocol
