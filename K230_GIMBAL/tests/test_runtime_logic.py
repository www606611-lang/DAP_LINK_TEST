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
    run_emm_axis_probe,
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
    ResponseAssembler,
    ZdtCommandClient,
    ZdtReadOnlyClient,
    ZdtTimeout,
    build_emm_relative_command,
    build_enable_command,
    build_stop_command,
    build_x_relative_command,
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


class FakeEmmMotorController:
    def __init__(self):
        self.sent = []
        self.response = None
        self.position_reads = 0

    def send(self, can_id, data, extended=True, timeout_ms=0):
        data = bytes(data)
        self.sent.append((can_id, data, extended, timeout_ms))
        function_code = data[0]
        response_data = None
        if function_code == 0x1A:
            response_data = bytes.fromhex("1A 00 06 6B")
        elif function_code == 0x1F:
            response_data = bytes.fromhex("1F 00 C8 13 0A 6B")
        elif function_code == 0x24:
            response_data = bytes.fromhex("24 2E 84 6B")
        elif function_code == 0x31:
            response_data = bytes.fromhex("31 00 00 6B")
        elif function_code == 0x36:
            self.position_reads += 1
            position = 0 if self.position_reads == 1 else 0x05AE
            response_data = bytes(
                (
                    0x36,
                    0x00,
                    (position >> 24) & 0xFF,
                    (position >> 16) & 0xFF,
                    (position >> 8) & 0xFF,
                    position & 0xFF,
                    0x6B,
                )
            )
        elif function_code == 0x3A:
            response_data = bytes.fromhex("3A 03 6B")
        elif function_code in (0xF3, 0xFD, 0xFE):
            response_data = bytes((function_code, 0x02, 0x6B))
        if response_data is not None:
            self.response = {
                "id": 0x0200,
                "extended": True,
                "data": response_data,
            }

    def receive(self):
        response = self.response
        self.response = None
        return response


class RuntimeLogicTest(unittest.TestCase):
    def test_radio_enabled_while_motion_gates_are_disabled(self):
        self.assertFalse(config.CHASSIS_RADIO_ENABLED)
        self.assertTrue(config.CAN_ENABLED)
        self.assertTrue(config.ZDT_DISCOVERY_ENABLED)
        self.assertFalse(config.GIMBAL_MOTION_ENABLED)
        self.assertEqual(config.GIMBAL_YAW_CAN_ADDRESS, 1)
        self.assertEqual(config.GIMBAL_PITCH_CAN_ADDRESS, 2)
        self.assertEqual(config.GIMBAL_YAW_POSITIVE_DIRECTION, "CW")
        self.assertEqual(config.GIMBAL_PITCH_POSITIVE_DIRECTION, "UP")

    def test_emm_axis_probe_uses_manual_3200_pulses_per_revolution(self):
        controller = FakeEmmMotorController()
        result = run_emm_axis_probe(
            controller, 2, pulses=71, speed_rpm=3, lease_ms=500
        )
        self.assertEqual(EMM_PULSES_PER_REVOLUTION, 3200)
        self.assertTrue(result["reached"])
        self.assertEqual(result["requested_pulses"], 71)
        self.assertAlmostEqual(result["requested_deg"], 7.9875)
        self.assertAlmostEqual(result["delta_deg"], 7.98706, places=3)

    def test_emm_axis_probe_rejects_unbounded_motion(self):
        with self.assertRaises(ValueError):
            run_emm_axis_probe(None, 2, pulses=73)

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
