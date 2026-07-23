import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import config
import chassis_radio
from chassis_radio import ChassisRadio
from gimbal_control import (
    EMM_PULSES_PER_REVOLUTION,
    GimbalControlError,
    GimbalSupervisor,
)
from mcp2515 import MCP2515
from vision import letterbox_param, map_target_center, nms_detections, select_target
from wire_protocol import (
    FrameParser,
    ROLE_CHASSIS,
    ROLE_ESP32,
    ROLE_K230,
    TYPE_HEARTBEAT,
    TYPE_HELLO,
    TYPE_STATUS,
    crc16_ccitt,
    encode_frame,
    sequence_is_newer,
)
from zdt_motor import (
    POSITION_MODE_ABSOLUTE,
    ResponseAssembler,
    ZdtCommandClient,
    ZdtReadOnlyClient,
    ZdtTimeout,
    build_emm_position_command,
    build_emm_relative_command,
    build_emm_speed_command,
    build_enable_command,
    build_stop_command,
    build_sync_trigger_command,
    build_x_position_command,
    build_x_relative_command,
    build_x_speed_command,
    decode_extended_id,
    extended_id,
    parse_bus_voltage,
    parse_encoder_degrees,
    parse_motor_flags,
    parse_option_status,
    parse_position_degrees,
    parse_version,
    read_position_frames,
    split_serial_command,
)


class FakeWlan:
    def isconnected(self):
        return True


class FakeSocket:
    def __init__(self, packets=()):
        self.packets = list(packets)
        self.sent = []

    def recvfrom(self, _capacity):
        if not self.packets:
            raise OSError()
        return self.packets.pop(0), ("192.168.4.1", 4210)

    def sendto(self, packet, address):
        self.sent.append((packet, address))
        return len(packet)


class FakeRuntimeSocket(FakeSocket):
    def __init__(self):
        super().__init__()
        self.bound_address = None
        self.timeout = None

    def bind(self, address):
        self.bound_address = address

    def settimeout(self, timeout):
        self.timeout = timeout

    def close(self):
        pass


class FakeSocketModule:
    AF_INET = 2
    SOCK_DGRAM = 2

    def __init__(self):
        self.instance = FakeRuntimeSocket()

    def socket(self, _family, _kind):
        return self.instance


class FakeMcp2515Transport:
    def __init__(self):
        self.registers = bytearray(256)
        self.reset()

    def reset(self):
        self.registers[:] = bytes(256)
        self.registers[MCP2515.REG_CANSTAT] = MCP2515.MODE_CONFIG
        self.registers[MCP2515.REG_CANCTRL] = MCP2515.MODE_CONFIG

    def transfer(self, tx_data):
        tx_data = bytes(tx_data)
        response = bytearray(len(tx_data))
        command = tx_data[0]
        if command == MCP2515.CMD_RESET:
            self.reset()
        elif command == MCP2515.CMD_READ:
            address = tx_data[1]
            for index in range(2, len(tx_data)):
                response[index] = self.registers[(address + index - 2) & 0xFF]
        elif command == MCP2515.CMD_WRITE:
            address = tx_data[1]
            for index, value in enumerate(tx_data[2:]):
                self.registers[(address + index) & 0xFF] = value
        elif command == MCP2515.CMD_BIT_MODIFY:
            address, mask, value = tx_data[1:4]
            self.registers[address] = (
                (self.registers[address] & ~mask) | (value & mask)
            )
            if address == MCP2515.REG_CANCTRL:
                self.registers[MCP2515.REG_CANSTAT] = (
                    self.registers[MCP2515.REG_CANCTRL]
                    & MCP2515.MODE_MASK
                )
        elif command == MCP2515.CMD_RTS_TX0:
            source = MCP2515.REG_TXB0SIDH
            target = MCP2515.REG_RXB0SIDH
            for index in range(13):
                self.registers[target + index] = self.registers[source + index]
            self.registers[MCP2515.REG_TXB0CTRL] &= ~MCP2515.TXBCTRL_TXREQ
            self.registers[MCP2515.REG_CANINTF] |= MCP2515.CANINTF_RX0IF
        elif command == MCP2515.CMD_READ_STATUS:
            response[1] = 0
        else:
            raise AssertionError("unexpected SPI command 0x%02X" % command)
        return bytes(response)


class FakeZdtCanController:
    def __init__(self, response=None):
        self.response_template = response
        self.response = None
        self.sent = []

    def send(self, can_id, data, extended=True, timeout_ms=0):
        self.sent.append((can_id, bytes(data), extended, timeout_ms))
        self.response = self.response_template

    def receive(self):
        response = self.response
        self.response = None
        return response


class FakeGimbalReader:
    def __init__(self):
        self.positions = {1: 10.0, 2: 0.0}
        self.bus_mv = {1: 12000, 2: 12000}
        self.flags = {
            1: self._normal_flags(),
            2: self._normal_flags(),
        }

    @staticmethod
    def _normal_flags():
        return {
            "enabled": True,
            "position_reached": True,
            "stalled": False,
            "stall_protected": False,
            "left_limit": False,
            "right_limit": False,
            "power_loss_recorded": False,
            "motor_flags": 3,
        }

    def query_profile(self, address, _timeout_ms):
        profile = {
            "address": address,
            "firmware": "X" if address == 1 else "Emm",
            "closed_loop": True,
            "bus_mv": self.bus_mv[address],
            "position_deg": self.positions[address],
        }
        profile.update(self.flags[address])
        return profile

    def query_position_degrees(self, address, _firmware, _timeout_ms):
        return self.positions[address]

    def query_motor_flags(self, address, _timeout_ms):
        return dict(self.flags[address])

    def query_bus_voltage(self, address, _timeout_ms):
        return self.bus_mv[address]


class FakeGimbalMotion:
    def __init__(self, reader, apply_commands=True):
        self.reader = reader
        self.apply_commands = apply_commands
        self.calls = []
        self.pending = {}

    def stop(self, address, timeout_ms, sync=False):
        self.calls.append(("stop", address, timeout_ms, sync))

    def stop_no_reply(self, address, timeout_ms, sync=False):
        self.calls.append(("stop_no_reply", address, timeout_ms, sync))

    def enable(self, address, enabled, timeout_ms, sync=False):
        self.calls.append(("enable", address, enabled, timeout_ms, sync))
        self.reader.flags[address]["enabled"] = enabled

    def move_x_position(
        self,
        address,
        degrees,
        speed_rpm,
        position_mode,
        timeout_ms,
        sync,
    ):
        self.calls.append(
            (
                "move_x_position",
                address,
                degrees,
                speed_rpm,
                position_mode,
                timeout_ms,
                sync,
            )
        )
        self.pending[address] = float(degrees)

    def move_emm_position(
        self,
        address,
        pulses,
        speed_rpm,
        acceleration,
        position_mode,
        timeout_ms,
        sync,
    ):
        self.calls.append(
            (
                "move_emm_position",
                address,
                pulses,
                speed_rpm,
                acceleration,
                position_mode,
                timeout_ms,
                sync,
            )
        )
        self.pending[address] = (
            pulses * 360.0 / EMM_PULSES_PER_REVOLUTION
        )

    def trigger_sync(self, timeout_ms):
        self.calls.append(("trigger_sync", timeout_ms))
        for address in (1, 2):
            self.reader.flags[address]["position_reached"] = False
        if self.apply_commands:
            self.reader.positions.update(self.pending)
            for address in (1, 2):
                self.reader.flags[address]["position_reached"] = True
        self.pending = {}
        return bytes.fromhex("01 FF 02 6B")


class RuntimeLogicTest(unittest.TestCase):
    def make_gimbal_supervisor(self, apply_commands=True):
        reader = FakeGimbalReader()
        motion = FakeGimbalMotion(reader, apply_commands)
        supervisor = GimbalSupervisor(
            None,
            config.GIMBAL_YAW_CAN_ADDRESS,
            config.GIMBAL_PITCH_CAN_ADDRESS,
            config.GIMBAL_YAW_SESSION_MIN_DEG,
            config.GIMBAL_YAW_SESSION_MAX_DEG,
            config.GIMBAL_PITCH_SESSION_MIN_DEG,
            config.GIMBAL_PITCH_SESSION_MAX_DEG,
            config.GIMBAL_YAW_MAX_RPM,
            config.GIMBAL_PITCH_MAX_RPM,
            config.GIMBAL_COMMAND_LEASE_MS,
            config.GIMBAL_FEEDBACK_POLL_MS,
            config.GIMBAL_VOLTAGE_POLL_MS,
            config.GIMBAL_POSITION_TOLERANCE_DEG,
            reader,
            motion,
        )
        return supervisor, reader, motion

    def test_radio_enabled_while_motion_gates_are_disabled(self):
        self.assertFalse(config.CHASSIS_RADIO_ENABLED)
        self.assertTrue(config.CAN_ENABLED)
        self.assertTrue(config.ZDT_DISCOVERY_ENABLED)
        self.assertFalse(config.GIMBAL_MOTION_ENABLED)
        self.assertEqual(config.GIMBAL_YAW_CAN_ADDRESS, 1)
        self.assertEqual(config.GIMBAL_PITCH_CAN_ADDRESS, 2)
        self.assertEqual(config.GIMBAL_YAW_POSITIVE_DIRECTION, "CW")
        self.assertEqual(config.GIMBAL_PITCH_POSITIVE_DIRECTION, "UP")
        self.assertEqual(config.GIMBAL_YAW_SESSION_MIN_DEG, -16.0)
        self.assertEqual(config.GIMBAL_YAW_SESSION_MAX_DEG, 16.0)
        self.assertEqual(config.GIMBAL_PITCH_SESSION_MIN_DEG, -14.0)
        self.assertEqual(config.GIMBAL_PITCH_SESSION_MAX_DEG, 14.0)

    def test_gimbal_supervisor_requires_arm_and_enforces_limits(self):
        supervisor, _reader, motion = self.make_gimbal_supervisor()
        with self.assertRaises(GimbalControlError):
            supervisor.command_offsets(1.0, 1.0, 0)

        snapshot = supervisor.arm(0)
        self.assertEqual(snapshot["state"], GimbalSupervisor.STATE_ARMED)
        with self.assertRaises(ValueError):
            supervisor.command_offsets(17.0, 0.0, 0)

        supervisor.command_offsets(8.0, -7.0, 0)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_MOVING)
        self.assertEqual(
            [call[0] for call in motion.calls[-3:]],
            ["move_x_position", "move_emm_position", "trigger_sync"],
        )
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.last_event, "REACHED")
        self.assertAlmostEqual(supervisor.snapshot()["offsets"]["yaw"], 8.0)
        self.assertAlmostEqual(
            supervisor.snapshot()["offsets"]["pitch"], -7.0, places=1
        )

    def test_gimbal_supervisor_lease_expiry_stops_and_disarms(self):
        supervisor, _reader, motion = self.make_gimbal_supervisor(False)
        supervisor.arm(0)
        supervisor.command_offsets(8.0, 7.0, 0, lease_ms=100)
        supervisor.task(100)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_DISARMED)
        self.assertEqual(supervisor.last_event, "LEASE_EXPIRED")
        self.assertEqual(supervisor.lease_expired_count, 1)
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )

    def test_gimbal_supervisor_fault_latches_until_explicit_clear(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor()
        supervisor.arm(0)
        reader.flags[1]["stalled"] = True
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_FAULT)
        self.assertEqual(supervisor.fault_code, "FEEDBACK_FAILED")
        with self.assertRaises(GimbalControlError):
            supervisor.command_offsets(0.0, 0.0, 100)
        self.assertFalse(supervisor.clear_fault(100))
        reader.flags[1]["stalled"] = False
        self.assertTrue(supervisor.clear_fault(100))
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_DISARMED)

    def test_gimbal_supervisor_release_disables_both_axes(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor()
        supervisor.arm(0)
        supervisor.release()
        self.assertFalse(reader.flags[1]["enabled"])
        self.assertFalse(reader.flags[2]["enabled"])
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_DISARMED)

    def test_mcp2515_extended_identifier_round_trip(self):
        for can_id in (0, 0x100, 0x001ABCDE, 0x1FFFFFFF):
            encoded = MCP2515.encode_identifier(can_id, extended=True)
            decoded, extended = MCP2515.decode_identifier(encoded)
            self.assertTrue(extended)
            self.assertEqual(decoded, can_id)

    def test_mcp2515_8mhz_500k_loopback_then_listen_only(self):
        transport = FakeMcp2515Transport()
        controller = MCP2515(transport, 8_000_000, 500_000)
        controller.initialize(MCP2515.MODE_LOOPBACK)
        self.assertEqual(
            transport.registers[MCP2515.REG_CNF1 : MCP2515.REG_CNF1 + 1],
            bytes((0x00,)),
        )
        self.assertEqual(transport.registers[MCP2515.REG_CNF2], 0x91)
        self.assertEqual(transport.registers[MCP2515.REG_CNF3], 0x01)
        self.assertTrue(controller.loopback_self_test(rounds=4))
        controller.set_mode(MCP2515.MODE_LISTEN_ONLY)
        self.assertEqual(controller.mode(), MCP2515.MODE_LISTEN_ONLY)
        self.assertEqual(controller.tx_count, 4)
        self.assertEqual(controller.rx_count, 4)
        self.assertEqual(
            transport.registers[MCP2515.REG_CANINTF],
            0,
        )

    def test_zdt_read_position_and_extended_id(self):
        self.assertEqual(read_position_frames(1), [(0x0100, b"\x36\x6b")])
        self.assertEqual(extended_id(2, 3), 0x0203)
        self.assertEqual(decode_extended_id(0x0203), (2, 3))

    def test_zdt_long_command_matches_manual_fragmentation(self):
        serial_command = bytes.fromhex(
            "01 FD 01 0F A0 00 00 01 FA 00 00 00 6B"
        )
        self.assertEqual(
            split_serial_command(1, serial_command),
            [
                (0x0100, bytes.fromhex("FD 01 0F A0 00 00 01 FA")),
                (0x0101, bytes.fromhex("FD 00 00 00 6B")),
            ],
        )

    def test_zdt_x_probe_command_is_five_degrees_at_five_rpm(self):
        command = build_x_relative_command(1, 5.0, 5.0)
        self.assertEqual(
            command,
            bytes.fromhex("01 FB 00 00 32 00 00 00 32 02 00 6B"),
        )
        self.assertEqual(
            split_serial_command(1, command),
            [
                (0x0100, bytes.fromhex("FB 00 00 32 00 00 00 32")),
                (0x0101, bytes.fromhex("FB 02 00 6B")),
            ],
        )

    def test_zdt_absolute_position_commands_allow_zero_and_sync(self):
        self.assertEqual(
            build_x_position_command(
                1, 0.0, 3.0, POSITION_MODE_ABSOLUTE, True
            ),
            bytes.fromhex(
                "01 FB 00 00 1E 00 00 00 00 01 01 6B"
            ),
        )
        self.assertEqual(
            build_emm_position_command(
                2, 0, 2, 10, POSITION_MODE_ABSOLUTE, True
            ),
            bytes.fromhex(
                "02 FD 00 00 02 0A 00 00 00 00 01 01 6B"
            ),
        )

    def test_zdt_x_and_emm_speed_commands_match_manual_layouts(self):
        self.assertEqual(
            build_x_speed_command(1, -3.0, 100),
            bytes.fromhex("01 F6 01 00 64 00 1E 00 6B"),
        )
        self.assertEqual(
            build_emm_speed_command(2, 2.0, 10),
            bytes.fromhex("02 F6 00 00 02 0A 00 6B"),
        )

    def test_zdt_sync_trigger_uses_broadcast_and_accepts_axis_ack(self):
        command = build_sync_trigger_command()
        self.assertEqual(command, bytes.fromhex("00 FF 66 6B"))
        self.assertEqual(
            split_serial_command(0, command),
            [(0x0000, bytes.fromhex("FF 66 6B"))],
        )
        controller = FakeZdtCanController(
            {
                "id": 0x0100,
                "extended": True,
                "data": bytes.fromhex("FF 02 6B"),
            }
        )
        self.assertEqual(
            ZdtCommandClient(controller).trigger_sync(10),
            bytes.fromhex("01 FF 02 6B"),
        )

    def test_zdt_emm_enable_stop_and_relative_commands(self):
        self.assertEqual(
            build_enable_command(2, True),
            bytes.fromhex("02 F3 AB 01 00 6B"),
        )
        self.assertEqual(
            build_stop_command(2), bytes.fromhex("02 FE 98 00 6B")
        )
        self.assertEqual(
            build_emm_relative_command(2, -44, 5, 10),
            bytes.fromhex(
                "02 FD 01 00 05 0A 00 00 00 2C 02 00 6B"
            ),
        )

    def test_zdt_command_client_requires_and_accepts_motor_ack(self):
        controller = FakeZdtCanController(
            {
                "id": 0x0100,
                "extended": True,
                "data": bytes.fromhex("FB 02 6B"),
            }
        )
        response = ZdtCommandClient(controller).move_x_relative(
            1, 5.0, 5.0, 10
        )
        self.assertEqual(response, bytes.fromhex("01 FB 02 6B"))
        self.assertEqual(
            [(can_id, data) for can_id, data, _ext, _timeout in controller.sent],
            [
                (0x0100, bytes.fromhex("FB 00 00 32 00 00 00 32")),
                (0x0101, bytes.fromhex("FB 02 00 6B")),
            ],
        )

    def test_zdt_response_reassembly(self):
        assembler = ResponseAssembler(1)
        self.assertIsNone(
            assembler.feed(0x0100, bytes.fromhex("36 00 00 00 00 00 00 00"))
        )
        response = assembler.feed(0x0101, bytes.fromhex("36 00 6B"))
        self.assertEqual(
            response,
            bytes.fromhex("01 36 00 00 00 00 00 00 00 00 6B"),
        )

    def test_zdt_profile_parsers_match_manual_units(self):
        options = parse_option_status(bytes.fromhex("01 1A 00 06 6B"))
        self.assertEqual(options["firmware"], "Emm")
        self.assertTrue(options["closed_loop"])
        self.assertEqual(options["option_word"], 0x0006)
        self.assertEqual(
            parse_version(bytes.fromhex("01 1F 00 C8 03 14 6B")),
            {
                "firmware_raw": 200,
                "hardware_series": "X",
                "hardware_type": 3,
                "hardware_version": 20,
            },
        )
        self.assertEqual(
            parse_bus_voltage(bytes.fromhex("01 24 5D C0 6B")), 24000
        )
        self.assertAlmostEqual(
            parse_encoder_degrees(bytes.fromhex("01 31 40 00 6B")), 90.0
        )
        self.assertAlmostEqual(
            parse_position_degrees(
                bytes.fromhex("01 36 01 00 00 00 10 6B"), "X"
            ),
            -1.6,
        )
        flags = parse_motor_flags(bytes.fromhex("01 3A 83 6B"))
        self.assertTrue(flags["enabled"])
        self.assertTrue(flags["position_reached"])
        self.assertFalse(flags["stalled"])

    def test_zdt_read_only_client_queries_one_position(self):
        controller = FakeZdtCanController(
            {
                "id": 0x0100,
                "extended": True,
                "data": bytes.fromhex("36 00 00 00 01 6B"),
            }
        )
        response = ZdtReadOnlyClient(controller).query_position(1, 10)
        self.assertEqual(
            controller.sent,
            [(0x0100, b"\x36\x6b", True, 10)],
        )
        self.assertEqual(response["raw"], bytes.fromhex("01 36 00 00 00 01 6B"))

    def test_zdt_read_only_client_timeout_is_bounded(self):
        controller = FakeZdtCanController()
        with self.assertRaises(ZdtTimeout):
            ZdtReadOnlyClient(controller).query_position(2, 0)

    def test_crc_known_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_frame_round_trip_fragmented(self):
        packet = encode_frame(TYPE_HEARTBEAT, 0x1234, b"abc")
        parser = FrameParser()
        self.assertEqual(parser.feed(packet[:4]), [])
        self.assertEqual(
            parser.feed(packet[4:]),
            [(TYPE_HEARTBEAT, 0x1234, b"abc")],
        )

    def test_bad_crc_resynchronizes_to_next_frame(self):
        bad = bytearray(encode_frame(TYPE_HEARTBEAT, 1, b"bad"))
        bad[-1] ^= 0x80
        good = encode_frame(TYPE_HEARTBEAT, 2, b"good")
        parser = FrameParser()
        self.assertEqual(
            parser.feed(bytes(bad) + good),
            [(TYPE_HEARTBEAT, 2, b"good")],
        )
        self.assertEqual(parser.crc_error_count, 1)

    def test_sequence_wrap_and_duplicate_rules(self):
        self.assertTrue(sequence_is_newer(0, 0xFFFF))
        self.assertTrue(sequence_is_newer(20, 10))
        self.assertFalse(sequence_is_newer(10, 10))
        self.assertFalse(sequence_is_newer(9, 10))

    def test_radio_tracks_role_sequences_independently(self):
        packets = (
            encode_frame(TYPE_HELLO, 10, bytes((ROLE_ESP32,))),
            encode_frame(TYPE_STATUS, 0xFFFF, bytes((ROLE_CHASSIS,))),
            encode_frame(TYPE_HEARTBEAT, 10, bytes((ROLE_ESP32,))),
            encode_frame(TYPE_HEARTBEAT, 9, bytes((ROLE_ESP32,))),
        )
        radio = ChassisRadio()
        radio.sock = FakeSocket(packets)
        radio._receive(100)
        self.assertEqual(radio.rx_frame_count, 2)
        self.assertEqual(radio.duplicate_count, 1)
        self.assertEqual(radio.out_of_order_count, 1)
        self.assertTrue(radio._role_online(ROLE_ESP32, 100))
        self.assertTrue(radio._role_online(ROLE_CHASSIS, 100))

        radio.sock = FakeSocket(
            (encode_frame(TYPE_STATUS, 0, bytes((ROLE_CHASSIS,))),)
        )
        radio._receive(200)
        self.assertEqual(radio.rx_frame_count, 3)

        radio.sock = FakeSocket(
            (encode_frame(TYPE_HELLO, 0, bytes((ROLE_CHASSIS,))),)
        )
        radio._receive(300)
        self.assertEqual(radio.last_rx_sequence[ROLE_CHASSIS], 0)
        self.assertEqual(radio.rx_frame_count, 4)

    def test_online_radio_task_sends_k230_heartbeat(self):
        radio = ChassisRadio()
        radio.wlan = FakeWlan()
        radio.sock = FakeSocket(
            (encode_frame(TYPE_HELLO, 1, bytes((ROLE_ESP32,))),)
        )
        radio.task(1000)
        self.assertEqual(radio.state, ChassisRadio.STATE_ONLINE)
        self.assertEqual(len(radio.sock.sent), 1)
        frames = FrameParser().feed(radio.sock.sent[0][0])
        self.assertEqual(frames[0][0], TYPE_HEARTBEAT)
        self.assertEqual(frames[0][2][0], ROLE_K230)

    def test_runtime_socket_has_bounded_receive_timeout(self):
        fake_socket_module = FakeSocketModule()
        original_socket_module = chassis_radio.socket
        chassis_radio.socket = fake_socket_module
        try:
            radio = ChassisRadio()
            radio._open_socket()
        finally:
            chassis_radio.socket = original_socket_module

        self.assertIs(radio.sock, fake_socket_module.instance)
        self.assertEqual(
            radio.sock.bound_address,
            ("0.0.0.0", config.CHASSIS_RADIO_LOCAL_PORT),
        )
        self.assertEqual(radio.sock.timeout, 0.001)

    def test_letterbox_matches_accepted_pipeline(self):
        self.assertEqual(
            letterbox_param((640, 384), (320, 320)),
            (64, 64, 0, 0, 0.5),
        )

    def test_coordinate_mapping_preserves_center_and_bounds(self):
        self.assertEqual(map_target_center(320, 192), (200, 120))
        self.assertEqual(map_target_center(0, 0), (0, 0))
        self.assertEqual(map_target_center(640, 384), (399, 239))

    def test_highest_confidence_detection_is_published(self):
        detections = [
            [0, 0, 100, 100, 0.50, 0],
            [270, 142, 370, 242, 0.90, 0],
        ]
        self.assertEqual(select_target(detections), (200, 120))

    def test_nms_suppresses_only_same_class_overlap(self):
        detections = [
            [0, 0, 100, 100, 0.90, 0],
            [5, 5, 95, 95, 0.80, 0],
            [5, 5, 95, 95, 0.70, 1],
        ]
        kept = nms_detections(detections, 0.45)
        self.assertEqual(len(kept), 2)
        self.assertEqual([item[5] for item in kept], [0, 1])


if __name__ == "__main__":
    unittest.main()
