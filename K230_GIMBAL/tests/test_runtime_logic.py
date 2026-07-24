import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import config
import chassis_radio
import gimbal_control
from chassis_radio import ChassisRadio
from gimbal_control import (
    EMM_PULSES_PER_REVOLUTION,
    GimbalControlError,
    GimbalSupervisor,
    TargetTracker,
)
from mcp2515 import MCP2515, MCP2515RuntimeGate
from vision import (
    TargetSelector,
    letterbox_param,
    map_target_center,
    nms_detections,
    select_target,
)
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

    def move_x_speed(
        self,
        address,
        speed_rpm,
        acceleration_rpm_s,
        timeout_ms,
        sync,
    ):
        self.calls.append(
            (
                "move_x_speed",
                address,
                speed_rpm,
                acceleration_rpm_s,
                timeout_ms,
                sync,
            )
        )

    def move_emm_speed(
        self,
        address,
        speed_rpm,
        acceleration,
        timeout_ms,
        sync,
    ):
        self.calls.append(
            (
                "move_emm_speed",
                address,
                speed_rpm,
                acceleration,
                timeout_ms,
                sync,
            )
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
    def make_gimbal_supervisor(
        self,
        apply_commands=True,
        yaw_continuous=False,
        pitch_continuous=False,
    ):
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
            yaw_continuous=yaw_continuous,
            pitch_continuous=pitch_continuous,
        )
        return supervisor, reader, motion

    def make_target_tracker(self, apply_commands=True):
        supervisor, reader, motion = self.make_gimbal_supervisor(
            apply_commands
        )
        tracker = TargetTracker(
            supervisor,
            config.TARGET_COORD_WIDTH,
            config.TARGET_COORD_HEIGHT,
            config.TRACKING_DEADBAND_X,
            config.TRACKING_DEADBAND_Y,
            config.TRACKING_DEADBAND_HYSTERESIS_X,
            config.TRACKING_DEADBAND_HYSTERESIS_Y,
            config.TRACKING_YAW_RPM_PER_PIXEL,
            config.TRACKING_PITCH_RPM_PER_PIXEL,
            config.TRACKING_FILTER_ALPHA,
            config.TRACKING_UPDATE_MS,
            config.TRACKING_MISSING_STOP_MS,
            config.TRACKING_LOST_TIMEOUT_MS,
            config.TRACKING_MIN_YAW_RPM,
            config.TRACKING_MIN_PITCH_RPM,
            config.TRACKING_MAX_YAW_RPM,
            config.TRACKING_MAX_PITCH_RPM,
            config.TRACKING_YAW_ACCELERATION_RPM_S,
            config.TRACKING_PITCH_ACCELERATION,
            config.TRACKING_SPEED_CHANGE_RPM,
            config.TRACKING_COMMAND_REFRESH_MS,
            config.TRACKING_COMMAND_LEASE_MS,
        )
        return tracker, supervisor, reader, motion

    def test_independent_gimbal_tracking_gate_is_enabled(self):
        self.assertFalse(config.CHASSIS_RADIO_ENABLED)
        self.assertTrue(config.CAN_ENABLED)
        self.assertTrue(config.ZDT_DISCOVERY_ENABLED)
        self.assertTrue(config.GIMBAL_MOTION_ENABLED)
        self.assertEqual(config.GIMBAL_YAW_CAN_ADDRESS, 1)
        self.assertEqual(config.GIMBAL_PITCH_CAN_ADDRESS, 2)
        self.assertEqual(config.GIMBAL_YAW_POSITIVE_DIRECTION, "CW")
        self.assertEqual(config.GIMBAL_PITCH_POSITIVE_DIRECTION, "UP")
        self.assertTrue(config.GIMBAL_YAW_CONTINUOUS)
        self.assertTrue(config.GIMBAL_PITCH_CONTINUOUS)
        self.assertIsNone(config.GIMBAL_YAW_CENTER_DEG)
        self.assertIsNone(config.GIMBAL_PITCH_CENTER_DEG)
        self.assertLess(
            config.GIMBAL_RECOVERY_PITCH_MIN_DEG,
            config.GIMBAL_PITCH_SESSION_MIN_DEG,
        )
        self.assertGreater(
            config.GIMBAL_RECOVERY_PITCH_MAX_DEG,
            config.GIMBAL_PITCH_SESSION_MAX_DEG,
        )
        self.assertEqual(config.GIMBAL_YAW_SESSION_MIN_DEG, -180.0)
        self.assertEqual(config.GIMBAL_YAW_SESSION_MAX_DEG, 180.0)
        self.assertEqual(config.GIMBAL_PITCH_SESSION_MIN_DEG, -32.0)
        self.assertEqual(config.GIMBAL_PITCH_SESSION_MAX_DEG, 22.0)
        self.assertGreater(
            config.TRACKING_COMMAND_LEASE_MS,
            config.TRACKING_MISSING_STOP_MS,
        )
        self.assertLess(
            config.TRACKING_COMMAND_REFRESH_MS,
            config.TRACKING_COMMAND_LEASE_MS,
        )
        self.assertLess(config.TRACKING_UPDATE_MS, 50)
        self.assertGreater(config.TRACKING_FILTER_ALPHA, 0.5)

    def test_gimbal_supervisor_requires_arm_and_enforces_limits(self):
        supervisor, _reader, motion = self.make_gimbal_supervisor()
        with self.assertRaises(GimbalControlError):
            supervisor.command_offsets(1.0, 1.0, 0)

        snapshot = supervisor.arm(0)
        self.assertEqual(snapshot["state"], GimbalSupervisor.STATE_ARMED)
        with self.assertRaises(ValueError):
            supervisor.command_offsets(
                config.GIMBAL_YAW_SESSION_MAX_DEG + 1.0, 0.0, 0
            )

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

    def test_gimbal_supervisor_lease_expiry_stops_and_remains_armed(self):
        supervisor, _reader, motion = self.make_gimbal_supervisor(False)
        supervisor.arm(0)
        supervisor.command_offsets(8.0, 7.0, 0, lease_ms=100)
        supervisor.task(100)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.last_event, "LEASE_EXPIRED")
        self.assertEqual(supervisor.lease_expired_count, 1)
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )

    def test_gimbal_command_lease_starts_after_can_commands_complete(self):
        supervisor, _reader, _motion = self.make_gimbal_supervisor(False)
        original_ticks_ms = getattr(gimbal_control.time, "ticks_ms", None)
        had_ticks_ms = hasattr(gimbal_control.time, "ticks_ms")
        gimbal_control.time.ticks_ms = lambda: 400
        try:
            supervisor.arm(100)
            supervisor.command_speeds(2.0, -2.0, 100, lease_ms=300)
            self.assertEqual(supervisor.command_started_ms, 400)
            self.assertEqual(supervisor.lease_deadline_ms, 700)
            supervisor.task(699)
            self.assertEqual(supervisor.state, GimbalSupervisor.STATE_MOVING)
            supervisor.task(700)
            self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        finally:
            if had_ticks_ms:
                gimbal_control.time.ticks_ms = original_ticks_ms
            else:
                delattr(gimbal_control.time, "ticks_ms")

    def test_gimbal_supervisor_fault_latches_until_explicit_clear(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor()
        supervisor.arm(0)
        reader.flags[1]["stalled"] = True
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.last_event, "FEEDBACK_RETRY")
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS * 2)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_FAULT)
        self.assertEqual(supervisor.fault_code, "FEEDBACK_FAILED")
        with self.assertRaises(GimbalControlError):
            supervisor.command_offsets(0.0, 0.0, 100)
        self.assertFalse(supervisor.clear_fault(100))
        reader.flags[1]["stalled"] = False
        self.assertTrue(supervisor.clear_fault(100))
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_DISARMED)

    def test_gimbal_supervisor_recovers_one_feedback_error(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor()
        supervisor.arm(0)
        original_query = reader.query_position_degrees
        remaining_failures = [1]

        def flaky_query(address, firmware, timeout_ms):
            if remaining_failures[0]:
                remaining_failures[0] -= 1
                raise RuntimeError("transient CAN error")
            return original_query(address, firmware, timeout_ms)

        reader.query_position_degrees = flaky_query
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.last_event, "FEEDBACK_RETRY")
        self.assertEqual(supervisor.feedback_error_streak, 1)
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS * 2)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.feedback_error_streak, 0)
        self.assertEqual(supervisor.feedback_retry_count, 1)

    def test_gimbal_supervisor_release_disables_both_axes(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor()
        supervisor.arm(0)
        supervisor.release()
        self.assertFalse(reader.flags[1]["enabled"])
        self.assertFalse(reader.flags[2]["enabled"])
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_DISARMED)

    def test_gimbal_supervisor_hold_preserves_arm_and_origin(self):
        supervisor, _reader, motion = self.make_gimbal_supervisor()
        supervisor.arm(0)
        origin = dict(supervisor.origins)
        supervisor.command_offsets(4.0, 3.0, 0)
        supervisor.hold("TARGET_LOST")
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.origins, origin)
        self.assertIsNone(supervisor.lease_deadline_ms)
        self.assertEqual(supervisor.last_event, "TARGET_LOST")
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )

    def test_gimbal_supervisor_fixed_origin_survives_rearm(self):
        reader = FakeGimbalReader()
        motion = FakeGimbalMotion(reader)
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
            0.0,
            0.0,
        )
        first = supervisor.arm(0)
        self.assertEqual(first["origins"], {"yaw": 0.0, "pitch": 0.0})
        self.assertEqual(first["offsets"], {"yaw": 10.0, "pitch": 0.0})
        supervisor.stop()
        reader.positions[1] = 12.0
        second = supervisor.arm(100)
        self.assertEqual(second["origins"], {"yaw": 0.0, "pitch": 0.0})
        self.assertEqual(second["offsets"], {"yaw": 12.0, "pitch": 0.0})

    def test_gimbal_supervisor_recovers_from_gravity_drop(self):
        reader = FakeGimbalReader()
        reader.positions[1] = 3.0
        reader.positions[2] = -30.0
        motion = FakeGimbalMotion(reader)
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
            0.0,
            0.0,
        )
        snapshot = supervisor.recover_to_center(
            0,
            config.GIMBAL_RECOVERY_YAW_MIN_DEG,
            config.GIMBAL_RECOVERY_YAW_MAX_DEG,
            config.GIMBAL_RECOVERY_PITCH_MIN_DEG,
            config.GIMBAL_RECOVERY_PITCH_MAX_DEG,
            config.GIMBAL_RECOVERY_YAW_RPM,
            config.GIMBAL_RECOVERY_PITCH_RPM,
            config.GIMBAL_RECOVERY_LEASE_MS,
        )
        self.assertTrue(snapshot["recovery_active"])
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_MOVING)
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        final = supervisor.snapshot()
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(final["event"], "RECOVERED")
        self.assertFalse(final["recovery_active"])
        self.assertAlmostEqual(final["offsets"]["yaw"], 0.0)
        self.assertAlmostEqual(final["offsets"]["pitch"], 0.0)

    def test_gimbal_supervisor_rejects_position_beyond_recovery_limit(self):
        reader = FakeGimbalReader()
        reader.positions[2] = -40.0
        motion = FakeGimbalMotion(reader)
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
            reader=reader,
            motion=motion,
            yaw_origin_deg=0.0,
            pitch_origin_deg=0.0,
        )
        with self.assertRaises(GimbalControlError):
            supervisor.recover_to_center(
                0,
                config.GIMBAL_RECOVERY_YAW_MIN_DEG,
                config.GIMBAL_RECOVERY_YAW_MAX_DEG,
                config.GIMBAL_RECOVERY_PITCH_MIN_DEG,
                config.GIMBAL_RECOVERY_PITCH_MAX_DEG,
                config.GIMBAL_RECOVERY_YAW_RPM,
                config.GIMBAL_RECOVERY_PITCH_RPM,
                config.GIMBAL_RECOVERY_LEASE_MS,
            )
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_FAULT)

    def test_target_tracker_maps_image_error_to_bounded_axis_speeds(self):
        tracker, supervisor, _reader, motion = self.make_target_tracker()
        tracker.start(0)
        tracker.task((399, 0), 0)
        self.assertEqual(tracker.state, TargetTracker.STATE_TRACKING)
        self.assertEqual(tracker.raw_error_x, 199)
        self.assertEqual(tracker.raw_error_y, 120)
        self.assertAlmostEqual(tracker.command_yaw_rpm, -10.0)
        self.assertAlmostEqual(tracker.command_pitch_rpm, -6.0)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_MOVING)
        self.assertEqual(supervisor.motion_mode, "speed")
        self.assertEqual(
            [call[0] for call in motion.calls[-3:]],
            ["move_x_speed", "move_emm_speed", "trigger_sync"],
        )

    def test_target_tracker_deadband_holds_once_and_uses_exit_hysteresis(self):
        tracker, _supervisor, _reader, motion = self.make_target_tracker()
        tracker.start(0)
        calls_before = len(motion.calls)
        tracker.task((200, 120), 0)
        self.assertEqual(tracker.state, TargetTracker.STATE_LOCKED)
        self.assertEqual(len(motion.calls), calls_before + 2)
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )
        tracker.task((220, 120), config.TRACKING_UPDATE_MS)
        self.assertEqual(tracker.state, TargetTracker.STATE_LOCKED)
        self.assertEqual(len(motion.calls), calls_before + 2)
        tracker.task((240, 120), config.TRACKING_UPDATE_MS * 2)
        self.assertEqual(tracker.state, TargetTracker.STATE_TRACKING)
        self.assertEqual(motion.calls[-1][0], "trigger_sync")

    def test_target_tracker_synchronizes_zero_speed_for_locked_axis(self):
        tracker, _supervisor, _reader, motion = self.make_target_tracker()
        tracker.start(0)
        tracker.task((200, 0), 0)
        self.assertEqual(
            [call[0] for call in motion.calls[-3:]],
            ["move_x_speed", "move_emm_speed", "trigger_sync"],
        )
        self.assertEqual(motion.calls[-3][2], 0.0)
        self.assertLess(motion.calls[-2][2], 0.0)

    def test_target_tracker_close_holds_both_enabled_axes(self):
        tracker, supervisor, reader, motion = self.make_target_tracker()
        tracker.start(0)
        tracker.close()
        self.assertEqual(tracker.state, TargetTracker.STATE_OFF)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertTrue(reader.flags[1]["enabled"])
        self.assertTrue(reader.flags[2]["enabled"])
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )

    def test_target_tracker_loss_holds_and_reacquires_without_rebase(self):
        tracker, supervisor, _reader, motion = self.make_target_tracker()
        tracker.start(0)
        origin = dict(supervisor.origins)
        tracker.task((300, 120), 0)
        tracker.task(None, config.TRACKING_MISSING_STOP_MS)
        self.assertEqual(tracker.state, TargetTracker.STATE_SEARCHING)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )
        tracker.task(None, config.TRACKING_LOST_TIMEOUT_MS)
        self.assertEqual(tracker.state, TargetTracker.STATE_LOST)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.origins, origin)
        self.assertEqual(tracker.lost_count, 1)
        self.assertEqual(
            [call[0] for call in motion.calls[-2:]], ["stop", "stop"]
        )
        tracker.task(
            (300, 120),
            config.TRACKING_LOST_TIMEOUT_MS + config.TRACKING_UPDATE_MS,
        )
        self.assertEqual(tracker.state, TargetTracker.STATE_TRACKING)
        self.assertEqual(supervisor.origins, origin)
        self.assertEqual(tracker.last_event, "SPEED_COMMAND")

    def test_target_tracker_holds_same_speed_until_refresh_is_due(self):
        tracker, _supervisor, _reader, motion = self.make_target_tracker()
        tracker.start(0)
        tracker.task((399, 120), 0)
        calls_after_command = len(motion.calls)
        tracker.task((399, 120), config.TRACKING_UPDATE_MS)
        self.assertEqual(len(motion.calls), calls_after_command)
        self.assertEqual(tracker.last_event, "SPEED_HOLD")
        tracker.task((399, 120), config.TRACKING_COMMAND_REFRESH_MS)
        self.assertEqual(len(motion.calls), calls_after_command + 3)

    def test_target_tracker_recovers_after_lease_stop(self):
        tracker, supervisor, _reader, _motion = self.make_target_tracker()
        tracker.start(0)
        tracker.task((300, 120), 0)
        commands_before = tracker.command_count
        supervisor.lease_deadline_ms = 100
        tracker.task((300, 120), 100)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_MOVING)
        self.assertEqual(tracker.state, TargetTracker.STATE_TRACKING)
        self.assertEqual(tracker.command_count, commands_before + 1)
        self.assertEqual(tracker.lease_recovery_count, 1)

    def test_motion_response_timing_uses_encoder_feedback(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor(False)
        supervisor.arm(0)
        supervisor.command_speeds(2.0, 0.0, 0, lease_ms=300)
        reader.positions[1] += 1.0
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        timing = supervisor.snapshot()["timing"]
        self.assertEqual(timing["motion_response_ms"]["yaw"], 50)
        self.assertIsNone(timing["motion_response_ms"]["pitch"])

    def test_continuous_yaw_speed_ignores_position_limit(self):
        supervisor, _reader, motion = self.make_gimbal_supervisor(
            yaw_continuous=True
        )
        supervisor.arm(0)
        supervisor.positions["yaw"] = 1000.0
        snapshot = supervisor.command_speeds(5.0, 0.0, 0, lease_ms=300)
        self.assertTrue(snapshot["yaw_continuous"])
        self.assertFalse(snapshot["speed_limited"]["yaw"])
        self.assertEqual(snapshot["speed_commands"]["yaw"], 5.0)
        self.assertEqual(
            [call[0] for call in motion.calls[-3:]],
            ["move_x_speed", "move_emm_speed", "trigger_sync"],
        )

    def test_continuous_pitch_speed_ignores_position_limit(self):
        supervisor, _reader, _motion = self.make_gimbal_supervisor(
            yaw_continuous=True,
            pitch_continuous=True,
        )
        supervisor.arm(0)
        supervisor.positions["pitch"] = 1000.0
        snapshot = supervisor.command_speeds(0.0, 5.0, 0, lease_ms=300)
        self.assertTrue(snapshot["pitch_continuous"])
        self.assertFalse(snapshot["speed_limited"]["pitch"])
        self.assertEqual(snapshot["speed_commands"]["pitch"], 5.0)

    def test_continuous_axes_ignore_motor_limit_flags(self):
        supervisor, reader, _motion = self.make_gimbal_supervisor(
            yaw_continuous=True,
            pitch_continuous=True,
        )
        supervisor.arm(0)
        reader.flags[1]["left_limit"] = True
        reader.flags[2]["right_limit"] = True
        supervisor.task(config.GIMBAL_FEEDBACK_POLL_MS)
        self.assertEqual(supervisor.state, GimbalSupervisor.STATE_ARMED)
        self.assertEqual(supervisor.fault_code, "")

    def test_pitch_speed_is_blocked_only_at_mechanical_limit(self):
        supervisor, _reader, _motion = self.make_gimbal_supervisor(
            yaw_continuous=True
        )
        supervisor.arm(0)
        supervisor.positions["pitch"] = supervisor.pitch_max_deg
        snapshot = supervisor.command_speeds(2.0, 3.0, 0, lease_ms=300)
        self.assertFalse(snapshot["speed_limited"]["yaw"])
        self.assertTrue(snapshot["speed_limited"]["pitch"])
        self.assertEqual(snapshot["speed_commands"]["yaw"], 2.0)
        self.assertEqual(snapshot["speed_commands"]["pitch"], 0.0)

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

    def test_mcp2515_runtime_gate_transfers_active_ownership(self):
        class GateConfig:
            MCP2515_SCK_PIN = 15
            MCP2515_MOSI_PIN = 16
            MCP2515_MISO_PIN = 17
            MCP2515_CS_PIN = 14
            MCP2515_INT_PIN = 19
            MCP2515_SPI_BAUDRATE = 5_000_000
            MCP2515_OSC_HZ = 8_000_000
            CAN_BITRATE = 500_000
            MCP2515_SELF_TEST_ROUNDS = 2
            ZDT_DISCOVERY_ENABLED = False

        transport = FakeMcp2515Transport()
        gate = MCP2515RuntimeGate(GateConfig, lambda *_args: transport)
        self.assertTrue(gate.start())
        controller = gate.activate_normal()
        self.assertIs(controller, gate.controller)
        self.assertEqual(gate.state, MCP2515RuntimeGate.STATE_ACTIVE)
        self.assertEqual(controller.mode(), MCP2515.MODE_NORMAL)
        self.assertEqual(gate.state_text(), "CAN ACTIVE")

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

    def test_target_selector_holds_nearby_low_confidence_edge_detection(self):
        selector = TargetSelector(0.45, 0.25, 180, 500)
        self.assertEqual(
            selector.select([[400, 140, 500, 240, 0.80, 0]], 0),
            map_target_center(450, 190),
        )
        self.assertEqual(
            selector.select(
                [
                    [500, 130, 639, 250, 0.28, 0],
                    [0, 0, 80, 80, 0.44, 0],
                ],
                50,
            ),
            map_target_center(569, 190),
        )

    def test_target_selector_requires_acquire_confidence_after_timeout(self):
        selector = TargetSelector(0.45, 0.25, 180, 500)
        self.assertIsNone(
            selector.select([[0, 100, 80, 180, 0.30, 0]], 0)
        )
        self.assertEqual(
            selector.select([[0, 100, 80, 180, 0.60, 0]], 10),
            map_target_center(40, 140),
        )
        self.assertIsNone(selector.select([], 20))
        self.assertIsNone(
            selector.select([[20, 100, 100, 180, 0.30, 0]], 600)
        )

    def test_target_selector_does_not_jump_to_distant_box_while_locked(self):
        selector = TargetSelector(0.45, 0.25, 100, 500)
        selector.select([[270, 142, 370, 242, 0.80, 0]], 0)
        self.assertIsNone(
            selector.select([[0, 0, 80, 80, 0.95, 0]], 50)
        )
        self.assertEqual(
            selector.select([[0, 0, 80, 80, 0.95, 0]], 500),
            map_target_center(40, 40),
        )

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
