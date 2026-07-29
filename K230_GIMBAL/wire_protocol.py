"""Bounded binary framing shared by K230, ESP32-C3, and Tianmengxing."""

MAGIC_0 = 0xA5
MAGIC_1 = 0x5A
VERSION = 1
MAX_PAYLOAD = 64

TYPE_HELLO = 0x01
TYPE_HEARTBEAT = 0x02
TYPE_CONTROL_SHADOW = 0x10
TYPE_EMERGENCY_STOP = 0x11
TYPE_STATUS = 0x20
TYPE_H_MISSION_COMMAND = 0x30
TYPE_H_BALL_STATUS = 0x31
TYPE_ACK = 0x7E
TYPE_NACK = 0x7F

ROLE_K230 = 1
ROLE_ESP32 = 2
ROLE_CHASSIS = 3

H_MISSION_COMMAND_PAYLOAD_SIZE = 16
H_BALL_STATUS_PAYLOAD_SIZE = 26

H_BALL_FLAG_MEASUREMENT_VALID = 1 << 0
H_BALL_FLAG_PREDICTED = 1 << 1
H_BALL_FLAG_ARMED = 1 << 2
H_BALL_FLAG_FAULT = 1 << 3
H_BALL_FLAG_FINISHED = 1 << 4


def crc16_ccitt(data, initial=0xFFFF):
    crc = initial & 0xFFFF
    for value in data:
        crc ^= int(value) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type, sequence, payload=b""):
    if not 0 <= int(message_type) <= 0xFF:
        raise ValueError("message_type out of range")
    if not 0 <= int(sequence) <= 0xFFFF:
        raise ValueError("sequence out of range")
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")

    body = bytearray(5 + len(payload))
    body[0] = VERSION
    body[1] = int(message_type)
    body[2] = int(sequence) & 0xFF
    body[3] = (int(sequence) >> 8) & 0xFF
    body[4] = len(payload)
    body[5:] = payload
    crc = crc16_ccitt(body)
    return bytes((MAGIC_0, MAGIC_1)) + bytes(body) + bytes(
        (crc & 0xFF, (crc >> 8) & 0xFF)
    )


def encode_u32_le(value):
    value = int(value) & 0xFFFFFFFF
    return bytes(
        (
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        )
    )


def encode_u16_le(value):
    value = _checked_int(value, 0, 0xFFFF, "u16")
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def encode_i16_le(value):
    value = _checked_int(value, -0x8000, 0x7FFF, "i16") & 0xFFFF
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def decode_u16_le(data, offset=0):
    _require_bytes(data, offset, 2)
    return int(data[offset]) | (int(data[offset + 1]) << 8)


def decode_i16_le(data, offset=0):
    value = decode_u16_le(data, offset)
    return value - 0x10000 if value & 0x8000 else value


def decode_u32_le(data, offset=0):
    _require_bytes(data, offset, 4)
    return (
        int(data[offset])
        | (int(data[offset + 1]) << 8)
        | (int(data[offset + 2]) << 16)
        | (int(data[offset + 3]) << 24)
    )


def encode_h_mission_command(command):
    payload = bytearray(H_MISSION_COMMAND_PAYLOAD_SIZE)
    payload[0] = ROLE_CHASSIS
    payload[1] = _checked_int(command["profile"], 0, 0xFF, "profile")
    payload[2] = _checked_int(command["action"], 0, 0xFF, "action")
    payload[3] = _checked_int(command.get("flags", 0), 0, 0xFF, "flags")
    payload[4:6] = encode_u16_le(command["mission_sequence"])
    payload[6:8] = encode_i16_le(command["target_x_0p1mm"])
    payload[8:12] = encode_u32_le(command["command_time_ms"])
    payload[12:14] = encode_i16_le(
        command.get("chassis_accel_mm_s2", 0)
    )
    payload[14:16] = encode_u16_le(command["lease_ms"])
    return bytes(payload)


def decode_h_mission_command(payload):
    payload = bytes(payload)
    if (
        len(payload) != H_MISSION_COMMAND_PAYLOAD_SIZE
        or payload[0] != ROLE_CHASSIS
    ):
        raise ValueError("invalid H mission command")
    return {
        "profile": payload[1],
        "action": payload[2],
        "flags": payload[3],
        "mission_sequence": decode_u16_le(payload, 4),
        "target_x_0p1mm": decode_i16_le(payload, 6),
        "command_time_ms": decode_u32_le(payload, 8),
        "chassis_accel_mm_s2": decode_i16_le(payload, 12),
        "lease_ms": decode_u16_le(payload, 14),
    }


def encode_h_ball_status(status):
    payload = bytearray(H_BALL_STATUS_PAYLOAD_SIZE)
    payload[0] = ROLE_K230
    payload[1] = _checked_int(status["profile"], 0, 0xFF, "profile")
    payload[2] = _checked_int(
        status["controller_state"], 0, 0xFF, "controller_state"
    )
    payload[3] = _checked_int(status["quality"], 0, 100, "quality")
    payload[4] = _checked_int(status["flags"], 0, 0xFF, "flags")
    payload[5] = 0
    payload[6:8] = encode_u16_le(status["mission_sequence"])
    payload[8:12] = encode_u32_le(status["sample_time_ms"])
    payload[12:14] = encode_i16_le(status["ball_x_0p1mm"])
    payload[14:16] = encode_i16_le(status["ball_v_0p1mm_s"])
    payload[16:18] = encode_i16_le(status["target_x_0p1mm"])
    payload[18:20] = encode_i16_le(status["error_0p1mm"])
    payload[20:22] = encode_i16_le(status["beam_angle_mdeg"])
    payload[22:24] = encode_u16_le(status["observation_age_ms"])
    payload[24:26] = encode_u16_le(status["max_abs_error_0p1mm"])
    return bytes(payload)


def decode_h_ball_status(payload):
    payload = bytes(payload)
    if (
        len(payload) != H_BALL_STATUS_PAYLOAD_SIZE
        or payload[0] != ROLE_K230
    ):
        raise ValueError("invalid H ball status")
    return {
        "profile": payload[1],
        "controller_state": payload[2],
        "quality": payload[3],
        "flags": payload[4],
        "mission_sequence": decode_u16_le(payload, 6),
        "sample_time_ms": decode_u32_le(payload, 8),
        "ball_x_0p1mm": decode_i16_le(payload, 12),
        "ball_v_0p1mm_s": decode_i16_le(payload, 14),
        "target_x_0p1mm": decode_i16_le(payload, 16),
        "error_0p1mm": decode_i16_le(payload, 18),
        "beam_angle_mdeg": decode_i16_le(payload, 20),
        "observation_age_ms": decode_u16_le(payload, 22),
        "max_abs_error_0p1mm": decode_u16_le(payload, 24),
    }


def _checked_int(value, minimum, maximum, name):
    value = int(value)
    if value < minimum or value > maximum:
        raise ValueError("%s out of range" % name)
    return value


def _require_bytes(data, offset, length):
    if offset < 0 or len(data) < offset + length:
        raise ValueError("truncated integer")


def sequence_is_newer(sequence, previous):
    if previous is None:
        return True
    delta = (int(sequence) - int(previous)) & 0xFFFF
    return 0 < delta < 0x8000


class FrameParser:
    """Incremental parser with bounded payload storage and byte resync."""

    WAIT_MAGIC_0 = 0
    WAIT_MAGIC_1 = 1
    READ_VERSION = 2
    READ_TYPE = 3
    READ_SEQUENCE_0 = 4
    READ_SEQUENCE_1 = 5
    READ_LENGTH = 6
    READ_PAYLOAD = 7
    READ_CRC_0 = 8
    READ_CRC_1 = 9

    def __init__(self):
        self.valid_count = 0
        self.crc_error_count = 0
        self.length_error_count = 0
        self.version_error_count = 0
        self.resync_count = 0
        self._payload = bytearray(MAX_PAYLOAD)
        self.reset()

    def reset(self):
        self._state = self.WAIT_MAGIC_0
        self._message_type = 0
        self._sequence = 0
        self._length = 0
        self._payload_index = 0
        self._crc = 0xFFFF
        self._received_crc = 0

    def feed(self, data):
        frames = []
        for value in data:
            frame = self.feed_byte(value)
            if frame is not None:
                frames.append(frame)
        return frames

    def feed_byte(self, value):
        value = int(value) & 0xFF

        if self._state == self.WAIT_MAGIC_0:
            if value == MAGIC_0:
                self._state = self.WAIT_MAGIC_1
            return None

        if self._state == self.WAIT_MAGIC_1:
            if value == MAGIC_1:
                self._state = self.READ_VERSION
            elif value != MAGIC_0:
                self._state = self.WAIT_MAGIC_0
                self.resync_count += 1
            return None

        if self._state == self.READ_VERSION:
            if value != VERSION:
                self.version_error_count += 1
                self._resync(value)
                return None
            self._crc = crc16_ccitt((value,))
            self._state = self.READ_TYPE
            return None

        if self._state == self.READ_TYPE:
            self._message_type = value
            self._crc = crc16_ccitt((value,), self._crc)
            self._state = self.READ_SEQUENCE_0
            return None

        if self._state == self.READ_SEQUENCE_0:
            self._sequence = value
            self._crc = crc16_ccitt((value,), self._crc)
            self._state = self.READ_SEQUENCE_1
            return None

        if self._state == self.READ_SEQUENCE_1:
            self._sequence |= value << 8
            self._crc = crc16_ccitt((value,), self._crc)
            self._state = self.READ_LENGTH
            return None

        if self._state == self.READ_LENGTH:
            if value > MAX_PAYLOAD:
                self.length_error_count += 1
                self._resync(value)
                return None
            self._length = value
            self._payload_index = 0
            self._crc = crc16_ccitt((value,), self._crc)
            self._state = self.READ_PAYLOAD if value else self.READ_CRC_0
            return None

        if self._state == self.READ_PAYLOAD:
            self._payload[self._payload_index] = value
            self._payload_index += 1
            self._crc = crc16_ccitt((value,), self._crc)
            if self._payload_index >= self._length:
                self._state = self.READ_CRC_0
            return None

        if self._state == self.READ_CRC_0:
            self._received_crc = value
            self._state = self.READ_CRC_1
            return None

        self._received_crc |= value << 8
        if self._received_crc != self._crc:
            self.crc_error_count += 1
            self._resync(value)
            return None

        frame = (
            self._message_type,
            self._sequence,
            bytes(self._payload[: self._length]),
        )
        self.valid_count += 1
        self.reset()
        return frame

    def _resync(self, value):
        self.resync_count += 1
        self.reset()
        if value == MAGIC_0:
            self._state = self.WAIT_MAGIC_1
