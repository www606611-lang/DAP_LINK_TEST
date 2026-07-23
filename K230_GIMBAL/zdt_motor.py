"""ZDT CAN protocol primitives and bounded read-only discovery."""

import time


CHECK_BYTE = 0x6B
READ_OPTION_STATUS = 0x1A
READ_VERSION = 0x1F
READ_BUS_VOLTAGE = 0x24
READ_ENCODER = 0x31
READ_REALTIME_POSITION = 0x36
READ_MOTOR_FLAGS = 0x3A
COMMAND_ENABLE = 0xF3
COMMAND_SPEED = 0xF6
COMMAND_POSITION_X = 0xFB
COMMAND_POSITION_EMM = 0xFD
COMMAND_STOP = 0xFE
COMMAND_SYNC = 0xFF
POSITION_MODE_RELATIVE_TARGET = 0
POSITION_MODE_ABSOLUTE = 1
POSITION_MODE_RELATIVE_CURRENT = 2
MAX_CAN_PAYLOAD = 8


class ZdtTimeout(Exception):
    pass


class ZdtCommandError(Exception):
    pass


def validate_address(address, allow_broadcast=False):
    address = int(address)
    minimum = 0 if allow_broadcast else 1
    if not minimum <= address <= 255:
        raise ValueError("ZDT address must be in range 1..255")
    return address


def extended_id(address, packet_index=0):
    address = validate_address(address, allow_broadcast=True)
    packet_index = int(packet_index)
    if not 0 <= packet_index <= 255:
        raise ValueError("ZDT packet index must be in range 0..255")
    return (address << 8) | packet_index


def decode_extended_id(can_id):
    can_id = int(can_id)
    if not 0 <= can_id <= 0x1FFFFFFF:
        raise ValueError("extended CAN identifier out of range")
    return (can_id >> 8) & 0xFF, can_id & 0xFF


def build_read_command(address, function_code):
    address = validate_address(address)
    function_code = int(function_code)
    if not 0 <= function_code <= 0xFF:
        raise ValueError("function code out of range")
    return bytes((address, function_code, CHECK_BYTE))


def read_position_command(address):
    return build_read_command(address, READ_REALTIME_POSITION)


def split_serial_command(address, serial_command):
    """Convert address-prefixed ZDT serial bytes into extended CAN frames."""
    address = validate_address(address, allow_broadcast=True)
    serial_command = bytes(serial_command)
    if len(serial_command) < 3:
        raise ValueError("ZDT command is too short")
    if serial_command[0] != address:
        raise ValueError("ZDT command address mismatch")
    if serial_command[-1] != CHECK_BYTE:
        raise ValueError("unsupported ZDT check byte")

    function_code = serial_command[1]
    payload = serial_command[2:]
    frames = []
    packet_index = 0
    data_capacity = MAX_CAN_PAYLOAD - 1
    for offset in range(0, len(payload), data_capacity):
        frames.append(
            (
                extended_id(address, packet_index),
                bytes((function_code,))
                + payload[offset : offset + data_capacity],
            )
        )
        packet_index += 1
    return frames


def read_position_frames(address):
    return split_serial_command(address, read_position_command(address))


def read_frames(address, function_code):
    return split_serial_command(
        address, build_read_command(address, function_code)
    )


def build_enable_command(address, enabled, sync=False):
    address = validate_address(address)
    return bytes(
        (
            address,
            COMMAND_ENABLE,
            0xAB,
            0x01 if enabled else 0x00,
            0x01 if sync else 0x00,
            CHECK_BYTE,
        )
    )


def build_stop_command(address, sync=False):
    address = validate_address(address)
    return bytes(
        (
            address,
            COMMAND_STOP,
            0x98,
            0x01 if sync else 0x00,
            CHECK_BYTE,
        )
    )


def validate_position_mode(position_mode):
    position_mode = int(position_mode)
    if position_mode not in (
        POSITION_MODE_RELATIVE_TARGET,
        POSITION_MODE_ABSOLUTE,
        POSITION_MODE_RELATIVE_CURRENT,
    ):
        raise ValueError("ZDT position mode must be 0, 1, or 2")
    return position_mode


def build_x_position_command(
    address,
    degrees,
    speed_rpm,
    position_mode=POSITION_MODE_RELATIVE_CURRENT,
    sync=False,
):
    address = validate_address(address)
    degrees = float(degrees)
    position_mode = validate_position_mode(position_mode)
    speed_units = int(round(abs(float(speed_rpm)) * 10.0))
    angle_units = int(round(abs(degrees) * 10.0))
    if not 1 <= speed_units <= 30000:
        raise ValueError("X speed must be in range 0.1..3000.0 RPM")
    minimum_angle = 0 if position_mode == POSITION_MODE_ABSOLUTE else 1
    if not minimum_angle <= angle_units <= 0xFFFFFFFF:
        raise ValueError("X position angle is outside the bounded range")
    direction = 0x01 if degrees < 0.0 else 0x00
    return bytes(
        (
            address,
            COMMAND_POSITION_X,
            direction,
            (speed_units >> 8) & 0xFF,
            speed_units & 0xFF,
            (angle_units >> 24) & 0xFF,
            (angle_units >> 16) & 0xFF,
            (angle_units >> 8) & 0xFF,
            angle_units & 0xFF,
            position_mode,
            0x01 if sync else 0x00,
            CHECK_BYTE,
        )
    )


def build_x_relative_command(address, degrees, speed_rpm, sync=False):
    return build_x_position_command(
        address,
        degrees,
        speed_rpm,
        POSITION_MODE_RELATIVE_CURRENT,
        sync,
    )


def build_emm_position_command(
    address,
    pulses,
    speed_rpm,
    acceleration=10,
    position_mode=POSITION_MODE_RELATIVE_CURRENT,
    sync=False,
):
    address = validate_address(address)
    pulses = int(pulses)
    position_mode = validate_position_mode(position_mode)
    speed_rpm = int(round(abs(float(speed_rpm))))
    acceleration = int(acceleration)
    pulse_count = abs(pulses)
    if not 1 <= speed_rpm <= 3000:
        raise ValueError("Emm speed must be in range 1..3000 RPM")
    if not 0 <= acceleration <= 0xFF:
        raise ValueError("Emm acceleration must be in range 0..255")
    minimum_pulses = 0 if position_mode == POSITION_MODE_ABSOLUTE else 1
    if not minimum_pulses <= pulse_count <= 0xFFFFFFFF:
        raise ValueError("Emm pulse count is outside the bounded range")
    direction = 0x01 if pulses < 0 else 0x00
    return bytes(
        (
            address,
            COMMAND_POSITION_EMM,
            direction,
            (speed_rpm >> 8) & 0xFF,
            speed_rpm & 0xFF,
            acceleration,
            (pulse_count >> 24) & 0xFF,
            (pulse_count >> 16) & 0xFF,
            (pulse_count >> 8) & 0xFF,
            pulse_count & 0xFF,
            position_mode,
            0x01 if sync else 0x00,
            CHECK_BYTE,
        )
    )


def build_emm_relative_command(
    address, pulses, speed_rpm, acceleration=10, sync=False
):
    return build_emm_position_command(
        address,
        pulses,
        speed_rpm,
        acceleration,
        POSITION_MODE_RELATIVE_CURRENT,
        sync,
    )


def build_x_speed_command(
    address, speed_rpm, acceleration_rpm_s=100, sync=False
):
    address = validate_address(address)
    speed_rpm = float(speed_rpm)
    acceleration_rpm_s = int(acceleration_rpm_s)
    speed_units = int(round(abs(speed_rpm) * 10.0))
    if not 0 <= speed_units <= 30000:
        raise ValueError("X speed must be in range 0..3000.0 RPM")
    if not 0 <= acceleration_rpm_s <= 0xFFFF:
        raise ValueError("X acceleration must be in range 0..65535 RPM/S")
    direction = 0x01 if speed_rpm < 0.0 else 0x00
    return bytes(
        (
            address,
            COMMAND_SPEED,
            direction,
            (acceleration_rpm_s >> 8) & 0xFF,
            acceleration_rpm_s & 0xFF,
            (speed_units >> 8) & 0xFF,
            speed_units & 0xFF,
            0x01 if sync else 0x00,
            CHECK_BYTE,
        )
    )


def build_emm_speed_command(
    address, speed_rpm, acceleration=10, sync=False
):
    address = validate_address(address)
    speed_rpm = float(speed_rpm)
    acceleration = int(acceleration)
    speed_units = int(round(abs(speed_rpm)))
    if not 0 <= speed_units <= 3000:
        raise ValueError("Emm speed must be in range 0..3000 RPM")
    if not 0 <= acceleration <= 0xFF:
        raise ValueError("Emm acceleration must be in range 0..255")
    direction = 0x01 if speed_rpm < 0.0 else 0x00
    return bytes(
        (
            address,
            COMMAND_SPEED,
            direction,
            (speed_units >> 8) & 0xFF,
            speed_units & 0xFF,
            acceleration,
            0x01 if sync else 0x00,
            CHECK_BYTE,
        )
    )


def build_sync_trigger_command():
    return bytes((0x00, COMMAND_SYNC, 0x66, CHECK_BYTE))


class ResponseAssembler:
    """Reassemble one address' ordered extended-CAN response packets."""

    def __init__(self, address):
        self.address = validate_address(address)
        self.reset()

    def reset(self):
        self.next_packet = 0
        self.function_code = None
        self.payload = bytearray()

    def feed(self, can_id, data):
        address, packet_index = decode_extended_id(can_id)
        data = bytes(data)
        if address != self.address or len(data) > MAX_CAN_PAYLOAD:
            return None
        if packet_index == 0:
            self.reset()
        if packet_index != self.next_packet:
            self.reset()
            return None

        if packet_index == 0:
            if not data:
                return None
            self.function_code = data[0]
        elif data and data[0] == self.function_code:
            data = data[1:]

        self.payload.extend(data)
        self.next_packet += 1
        if len(data) < MAX_CAN_PAYLOAD or (
            data and data[-1] == CHECK_BYTE
        ):
            response = bytes((self.address,)) + bytes(self.payload)
            self.reset()
            return response
        return None


class ZdtReadOnlyClient:
    """Bounded query client containing no enable, stop, or motion commands."""

    def __init__(self, can_controller):
        self.can = can_controller

    def query_position(self, address, timeout_ms=120):
        return self.query(address, READ_REALTIME_POSITION, timeout_ms)

    def query_position_degrees(self, address, firmware, timeout_ms=120):
        return parse_position_degrees(
            self.query_position(address, timeout_ms)["raw"], firmware
        )

    def query_motor_flags(self, address, timeout_ms=120):
        return parse_motor_flags(
            self.query(address, READ_MOTOR_FLAGS, timeout_ms)["raw"]
        )

    def query_bus_voltage(self, address, timeout_ms=120):
        return parse_bus_voltage(
            self.query(address, READ_BUS_VOLTAGE, timeout_ms)["raw"]
        )

    def query(self, address, function_code, timeout_ms=120):
        address = validate_address(address)
        self._drain_receive_queue()
        for can_id, data in read_frames(address, function_code):
            self.can.send(
                can_id,
                data,
                extended=True,
                timeout_ms=timeout_ms,
            )

        assembler = ResponseAssembler(address)
        deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
        while True:
            message = self.can.receive()
            if message is not None and message.get("extended", False):
                frame_address, _packet_index = decode_extended_id(
                    message["id"]
                )
                if frame_address == address:
                    response = assembler.feed(
                        message["id"], message["data"]
                    )
                    if (
                        response is not None
                        and len(response) >= 3
                        and response[1] == int(function_code)
                    ):
                        return {
                            "address": address,
                            "raw": response,
                            "can_id": message["id"],
                        }
            if _ticks_diff(_ticks_ms(), deadline) >= 0:
                raise ZdtTimeout("ZDT address %d response timeout" % address)
            _sleep_ms(1)

    def query_profile(self, address, timeout_ms=120):
        options_raw = self.query(
            address, READ_OPTION_STATUS, timeout_ms
        )["raw"]
        options = parse_option_status(options_raw)
        version = parse_version(
            self.query(address, READ_VERSION, timeout_ms)["raw"]
        )
        bus_mv = parse_bus_voltage(
            self.query(address, READ_BUS_VOLTAGE, timeout_ms)["raw"]
        )
        encoder_deg = parse_encoder_degrees(
            self.query(address, READ_ENCODER, timeout_ms)["raw"]
        )
        position_deg = parse_position_degrees(
            self.query(address, READ_REALTIME_POSITION, timeout_ms)["raw"],
            options["firmware"],
        )
        flags = parse_motor_flags(
            self.query(address, READ_MOTOR_FLAGS, timeout_ms)["raw"]
        )
        profile = {
            "address": validate_address(address),
            "version": version,
            "bus_mv": bus_mv,
            "encoder_deg": encoder_deg,
            "position_deg": position_deg,
        }
        profile.update(options)
        profile.update(flags)
        return profile

    def _drain_receive_queue(self):
        for _ in range(8):
            if self.can.receive() is None:
                return


class ZdtCommandClient:
    """Bounded motion commands with mandatory receive acknowledgements."""

    def __init__(self, can_controller):
        self.can = can_controller
        self.reader = ZdtReadOnlyClient(can_controller)

    def enable(self, address, enabled=True, timeout_ms=120, sync=False):
        return self._command(
            build_enable_command(address, enabled, sync), timeout_ms
        )

    def stop(self, address, timeout_ms=120, sync=False):
        return self._command(
            build_stop_command(address, sync), timeout_ms
        )

    def stop_no_reply(self, address, timeout_ms=20, sync=False):
        command = build_stop_command(address, sync)
        for can_id, data in split_serial_command(address, command):
            self.can.send(
                can_id, data, extended=True, timeout_ms=timeout_ms
            )

    def move_x_relative(
        self, address, degrees, speed_rpm, timeout_ms=120, sync=False
    ):
        return self._command(
            build_x_relative_command(address, degrees, speed_rpm, sync),
            timeout_ms,
        )

    def move_x_position(
        self,
        address,
        degrees,
        speed_rpm,
        position_mode=POSITION_MODE_RELATIVE_CURRENT,
        timeout_ms=120,
        sync=False,
    ):
        return self._command(
            build_x_position_command(
                address,
                degrees,
                speed_rpm,
                position_mode,
                sync,
            ),
            timeout_ms,
        )

    def move_emm_relative(
        self,
        address,
        pulses,
        speed_rpm,
        acceleration=10,
        timeout_ms=120,
        sync=False,
    ):
        return self._command(
            build_emm_relative_command(
                address, pulses, speed_rpm, acceleration, sync
            ),
            timeout_ms,
        )

    def move_emm_position(
        self,
        address,
        pulses,
        speed_rpm,
        acceleration=10,
        position_mode=POSITION_MODE_RELATIVE_CURRENT,
        timeout_ms=120,
        sync=False,
    ):
        return self._command(
            build_emm_position_command(
                address,
                pulses,
                speed_rpm,
                acceleration,
                position_mode,
                sync,
            ),
            timeout_ms,
        )

    def move_x_speed(
        self,
        address,
        speed_rpm,
        acceleration_rpm_s=100,
        timeout_ms=120,
        sync=False,
    ):
        return self._command(
            build_x_speed_command(
                address, speed_rpm, acceleration_rpm_s, sync
            ),
            timeout_ms,
        )

    def move_emm_speed(
        self,
        address,
        speed_rpm,
        acceleration=10,
        timeout_ms=120,
        sync=False,
    ):
        return self._command(
            build_emm_speed_command(
                address, speed_rpm, acceleration, sync
            ),
            timeout_ms,
        )

    def trigger_sync(self, timeout_ms=120):
        command = build_sync_trigger_command()
        self.reader._drain_receive_queue()
        for can_id, data in split_serial_command(0, command):
            self.can.send(
                can_id, data, extended=True, timeout_ms=timeout_ms
            )

        deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
        while True:
            message = self.can.receive()
            if message is not None and message.get("extended", False):
                response_address, packet = decode_extended_id(message["id"])
                data = bytes(message["data"])
                if (
                    response_address != 0
                    and packet == 0
                    and len(data) == 3
                    and data[0] == COMMAND_SYNC
                    and data[2] == CHECK_BYTE
                ):
                    if data[1] in (0x02, 0x9F):
                        return bytes((response_address,)) + data
                    raise ZdtCommandError(
                        "ZDT sync trigger rejected: %s" %
                        format_hex(data)
                    )
            if _ticks_diff(_ticks_ms(), deadline) >= 0:
                raise ZdtTimeout("ZDT sync trigger response timeout")
            _sleep_ms(1)

    def _command(self, serial_command, timeout_ms):
        serial_command = bytes(serial_command)
        address = validate_address(serial_command[0])
        function_code = serial_command[1]
        self.reader._drain_receive_queue()
        for can_id, data in split_serial_command(address, serial_command):
            self.can.send(
                can_id, data, extended=True, timeout_ms=timeout_ms
            )
            _sleep_ms(2)

        assembler = ResponseAssembler(address)
        deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
        while True:
            message = self.can.receive()
            if message is not None and message.get("extended", False):
                frame_address, _packet = decode_extended_id(message["id"])
                if frame_address == address:
                    response = assembler.feed(
                        message["id"], message["data"]
                    )
                    if (
                        response is not None
                        and len(response) == 4
                        and response[1] == function_code
                    ):
                        if response[2] in (0x02, 0x9F):
                            return response
                        raise ZdtCommandError(
                            "ZDT command 0x%02X rejected: %s" %
                            (function_code, format_hex(response))
                        )
            if _ticks_diff(_ticks_ms(), deadline) >= 0:
                raise ZdtTimeout(
                    "ZDT command 0x%02X response timeout" % function_code
                )
            _sleep_ms(1)


def parse_option_status(response):
    response = bytes(response)
    if len(response) not in (4, 5):
        raise ValueError(
            "ZDT option response length mismatch: len=%d raw=%s" %
            (len(response), format_hex(response))
        )
    if response[1] != READ_OPTION_STATUS or response[-1] != CHECK_BYTE:
        raise ValueError(
            "ZDT option response format mismatch: raw=%s" %
            format_hex(response)
        )
    option_word = int.from_bytes(response[2:-1], "big")
    flags = option_word & 0xFF
    return {
        "option_word": option_word,
        "option_flags": flags,
        "motor_step_deg": 0.9 if flags & 0x01 else 1.8,
        "firmware": "Emm" if flags & 0x02 else "X",
        "closed_loop": bool(flags & 0x04),
        "positive_direction": "CCW" if flags & 0x10 else "CW",
        "button_locked": bool(flags & 0x20),
        "scaled_input": bool(flags & 0x80),
    }


def parse_version(response):
    response = _validated_response(response, READ_VERSION, 7)
    firmware_version = (response[2] << 8) | response[3]
    return {
        "firmware_raw": firmware_version,
        "hardware_series": "Y" if response[4] >> 4 else "X",
        "hardware_type": response[4] & 0x0F,
        "hardware_version": response[5],
    }


def parse_bus_voltage(response):
    response = _validated_response(response, READ_BUS_VOLTAGE, 5)
    return (response[2] << 8) | response[3]


def parse_encoder_degrees(response):
    response = _validated_response(response, READ_ENCODER, 5)
    encoder = (response[2] << 8) | response[3]
    return encoder * 360.0 / 65536.0


def parse_position_degrees(response, firmware):
    response = _validated_response(response, READ_REALTIME_POSITION, 8)
    magnitude = int.from_bytes(response[3:7], "big")
    if firmware == "Emm":
        degrees = magnitude * 360.0 / 65536.0
    elif firmware == "X":
        degrees = magnitude / 10.0
    else:
        raise ValueError("unknown ZDT firmware type")
    return -degrees if response[2] else degrees


def parse_motor_flags(response):
    response = _validated_response(response, READ_MOTOR_FLAGS, 4)
    flags = response[2]
    return {
        "motor_flags": flags,
        "enabled": bool(flags & 0x01),
        "position_reached": bool(flags & 0x02),
        "stalled": bool(flags & 0x04),
        "stall_protected": bool(flags & 0x08),
        "left_limit": bool(flags & 0x10),
        "right_limit": bool(flags & 0x20),
        "power_loss_recorded": bool(flags & 0x80),
    }


def _validated_response(response, function_code, expected_length):
    response = bytes(response)
    if len(response) != int(expected_length):
        raise ValueError(
            "ZDT response length mismatch for 0x%02X: len=%d raw=%s" %
            (function_code, len(response), format_hex(response))
        )
    if response[1] != int(function_code) or response[-1] != CHECK_BYTE:
        raise ValueError(
            "ZDT response format mismatch for 0x%02X: raw=%s" %
            (function_code, format_hex(response))
        )
    return response


def format_hex(data):
    return " ".join("%02X" % value for value in bytes(data))


def _sleep_ms(duration_ms):
    sleep_ms = getattr(time, "sleep_ms", None)
    if sleep_ms is not None:
        sleep_ms(int(duration_ms))
    else:
        time.sleep(int(duration_ms) / 1000.0)


def _ticks_ms():
    ticks_ms = getattr(time, "ticks_ms", None)
    if ticks_ms is not None:
        return ticks_ms()
    return int(time.monotonic() * 1000)


def _ticks_add(value, delta):
    ticks_add = getattr(time, "ticks_add", None)
    if ticks_add is not None:
        return ticks_add(value, delta)
    return value + delta


def _ticks_diff(value, reference):
    ticks_diff = getattr(time, "ticks_diff", None)
    if ticks_diff is not None:
        return ticks_diff(value, reference)
    return value - reference
