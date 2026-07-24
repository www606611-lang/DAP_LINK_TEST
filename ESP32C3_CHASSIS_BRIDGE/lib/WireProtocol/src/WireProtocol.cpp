#include "WireProtocol.h"

#include <string.h>

namespace WireProtocol {

uint16_t crc16Ccitt(const uint8_t* data, size_t length, uint16_t initial) {
    uint16_t crc = initial;
    for (size_t index = 0; index < length; ++index) {
        crc ^= static_cast<uint16_t>(data[index]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U)
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

size_t encodeFrame(uint8_t type, uint16_t sequence, const uint8_t* payload,
                   size_t payloadLength, uint8_t* output,
                   size_t outputCapacity) {
    if (payloadLength > kMaxPayload || output == nullptr ||
        outputCapacity < kFrameOverhead + payloadLength ||
        (payloadLength > 0 && payload == nullptr)) {
        return 0;
    }

    output[0] = kMagic0;
    output[1] = kMagic1;
    output[2] = kVersion;
    output[3] = type;
    output[4] = static_cast<uint8_t>(sequence & 0xFFU);
    output[5] = static_cast<uint8_t>(sequence >> 8);
    output[6] = static_cast<uint8_t>(payloadLength);
    if (payloadLength > 0) {
        memcpy(output + 7, payload, payloadLength);
    }

    const uint16_t crc = crc16Ccitt(output + 2, 5 + payloadLength);
    output[7 + payloadLength] = static_cast<uint8_t>(crc & 0xFFU);
    output[8 + payloadLength] = static_cast<uint8_t>(crc >> 8);
    return kFrameOverhead + payloadLength;
}

bool sequenceIsNewer(uint16_t sequence, uint16_t previous) {
    const uint16_t delta = static_cast<uint16_t>(sequence - previous);
    return delta != 0 && delta < 0x8000U;
}

Parser::Parser() { reset(); }

void Parser::reset() {
    state_ = State::WaitMagic0;
    frame_ = Frame{};
    payloadIndex_ = 0;
    crc_ = 0xFFFF;
    receivedCrc_ = 0;
}

const ParserStats& Parser::stats() const { return stats_; }

void Parser::updateCrc(uint8_t value) {
    crc_ = crc16Ccitt(&value, 1, crc_);
}

void Parser::resync(uint8_t value) {
    ++stats_.resyncs;
    reset();
    if (value == kMagic0) {
        state_ = State::WaitMagic1;
    }
}

bool Parser::feed(uint8_t value, Frame& completedFrame) {
    switch (state_) {
        case State::WaitMagic0:
            if (value == kMagic0) {
                state_ = State::WaitMagic1;
            }
            return false;

        case State::WaitMagic1:
            if (value == kMagic1) {
                state_ = State::ReadVersion;
            } else if (value != kMagic0) {
                state_ = State::WaitMagic0;
                ++stats_.resyncs;
            }
            return false;

        case State::ReadVersion:
            if (value != kVersion) {
                ++stats_.versionErrors;
                resync(value);
                return false;
            }
            crc_ = 0xFFFF;
            updateCrc(value);
            state_ = State::ReadType;
            return false;

        case State::ReadType:
            frame_.type = value;
            updateCrc(value);
            state_ = State::ReadSequence0;
            return false;

        case State::ReadSequence0:
            frame_.sequence = value;
            updateCrc(value);
            state_ = State::ReadSequence1;
            return false;

        case State::ReadSequence1:
            frame_.sequence |= static_cast<uint16_t>(value) << 8;
            updateCrc(value);
            state_ = State::ReadLength;
            return false;

        case State::ReadLength:
            if (value > kMaxPayload) {
                ++stats_.lengthErrors;
                resync(value);
                return false;
            }
            frame_.length = value;
            payloadIndex_ = 0;
            updateCrc(value);
            state_ = value == 0 ? State::ReadCrc0 : State::ReadPayload;
            return false;

        case State::ReadPayload:
            frame_.payload[payloadIndex_++] = value;
            updateCrc(value);
            if (payloadIndex_ >= frame_.length) {
                state_ = State::ReadCrc0;
            }
            return false;

        case State::ReadCrc0:
            receivedCrc_ = value;
            state_ = State::ReadCrc1;
            return false;

        case State::ReadCrc1:
            receivedCrc_ |= static_cast<uint16_t>(value) << 8;
            if (receivedCrc_ != crc_) {
                ++stats_.crcErrors;
                resync(value);
                return false;
            }
            completedFrame = frame_;
            ++stats_.validFrames;
            reset();
            return true;
    }

    reset();
    return false;
}

}  // namespace WireProtocol
