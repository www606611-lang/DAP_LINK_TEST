"""Bounded two-axis gimbal supervision above the ZDT protocol layer."""

import time

from zdt_motor import (
    POSITION_MODE_ABSOLUTE,
    ZdtCommandClient,
    ZdtReadOnlyClient,
    ZdtTimeout,
)


MIN_BUS_MV = 10000
EMM_PULSES_PER_REVOLUTION = 3200


def _command_completion_ms(fallback_ms):
    runtime_ticks_ms = getattr(time, "ticks_ms", None)
    if runtime_ticks_ms is not None:
        return int(runtime_ticks_ms())
    return int(fallback_ms)


class GimbalControlError(Exception):
    pass


class GimbalSupervisor:
    STATE_DISARMED = 0
    STATE_ARMED = 1
    STATE_MOVING = 2
    STATE_FAULT = 3

    def __init__(
        self,
        can_controller,
        yaw_address,
        pitch_address,
        yaw_min_deg,
        yaw_max_deg,
        pitch_min_deg,
        pitch_max_deg,
        yaw_max_rpm,
        pitch_max_rpm,
        command_lease_ms=400,
        poll_interval_ms=50,
        voltage_interval_ms=500,
        position_tolerance_deg=1.5,
        reader=None,
        motion=None,
        yaw_origin_deg=None,
        pitch_origin_deg=None,
        yaw_continuous=False,
        pitch_continuous=False,
    ):
        self.can = can_controller
        self.yaw_address = int(yaw_address)
        self.pitch_address = int(pitch_address)
        if self.yaw_address == self.pitch_address:
            raise ValueError("gimbal axes require different CAN addresses")

        self.yaw_min_deg = float(yaw_min_deg)
        self.yaw_max_deg = float(yaw_max_deg)
        self.pitch_min_deg = float(pitch_min_deg)
        self.pitch_max_deg = float(pitch_max_deg)
        self.yaw_continuous = bool(yaw_continuous)
        self.pitch_continuous = bool(pitch_continuous)
        if (
            not self.yaw_continuous
            and not self.yaw_min_deg < 0.0 < self.yaw_max_deg
        ):
            raise ValueError("yaw software limits must span zero")
        if (
            not self.pitch_continuous
            and not self.pitch_min_deg < 0.0 < self.pitch_max_deg
        ):
            raise ValueError("pitch software limits must span zero")

        self.yaw_max_rpm = float(yaw_max_rpm)
        self.pitch_max_rpm = float(pitch_max_rpm)
        if self.yaw_max_rpm <= 0.0 or self.pitch_max_rpm < 1.0:
            raise ValueError("gimbal speed limits are invalid")

        self.command_lease_ms = _validate_lease(command_lease_ms)
        self.poll_interval_ms = int(poll_interval_ms)
        self.voltage_interval_ms = int(voltage_interval_ms)
        self.position_tolerance_deg = float(position_tolerance_deg)
        if self.poll_interval_ms < 20:
            raise ValueError("gimbal poll interval is below 20 ms")
        if self.voltage_interval_ms < self.poll_interval_ms:
            raise ValueError("gimbal voltage interval is too short")
        if self.position_tolerance_deg <= 0.0:
            raise ValueError("gimbal position tolerance must be positive")

        self.reader = reader or ZdtReadOnlyClient(can_controller)
        self.motion = motion or ZdtCommandClient(can_controller)
        self.fixed_origins = {
            "yaw": (
                None if yaw_origin_deg is None else float(yaw_origin_deg)
            ),
            "pitch": (
                None
                if pitch_origin_deg is None
                else float(pitch_origin_deg)
            ),
        }
        self.state = self.STATE_DISARMED
        self.fault_code = ""
        self.fault_detail = ""
        self.last_event = "INIT"
        self.origins = {"yaw": 0.0, "pitch": 0.0}
        self.positions = {"yaw": 0.0, "pitch": 0.0}
        self.targets = {"yaw": 0.0, "pitch": 0.0}
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.motion_mode = "idle"
        self.flags = {"yaw": {}, "pitch": {}}
        self.bus_mv = {"yaw": 0, "pitch": 0}
        self.firmware = {"yaw": "X", "pitch": "Emm"}
        self.lease_deadline_ms = None
        self.command_started_ms = 0
        self.command_completed_ms = 0
        self.last_can_command_ms = 0
        self.max_can_command_ms = 0
        self.motion_probe = None
        self.motion_response_ms = {"yaw": None, "pitch": None}
        self.last_poll_ms = 0
        self.last_voltage_ms = 0
        self.lease_expired_count = 0
        self.feedback_error_streak = 0
        self.feedback_retry_count = 0
        self.command_retry_count = 0
        self.last_feedback_error = ""
        self.recovery_active = False
        self.recovery_limits = None

    def arm(self, now_ms):
        now_ms = int(now_ms)
        if self.state == self.STATE_FAULT:
            raise GimbalControlError("clear the latched fault before arming")
        if self.state == self.STATE_MOVING:
            raise GimbalControlError("stop the active command before arming")

        try:
            self._stop_all()
            yaw_profile = self.reader.query_profile(self.yaw_address, 150)
            pitch_profile = self.reader.query_profile(
                self.pitch_address, 150
            )
            self._validate_profile(yaw_profile, "X")
            self._validate_profile(pitch_profile, "Emm")
            yaw_origin = self.fixed_origins["yaw"]
            if yaw_origin is None:
                yaw_origin = yaw_profile["position_deg"]
            pitch_origin = self.fixed_origins["pitch"]
            if pitch_origin is None:
                pitch_origin = pitch_profile["position_deg"]
            yaw_offset = yaw_profile["position_deg"] - yaw_origin
            pitch_offset = pitch_profile["position_deg"] - pitch_origin
            if not self.pitch_continuous:
                pitch_offset = _wrapped_degrees(pitch_offset)
            if (
                not self.yaw_continuous
                and not self.yaw_min_deg <= yaw_offset <= self.yaw_max_deg
            ):
                raise GimbalControlError("yaw starts outside software limit")
            if not self.pitch_continuous and not (
                self.pitch_min_deg <= pitch_offset <= self.pitch_max_deg
            ):
                raise GimbalControlError("pitch starts outside software limit")
            self.motion.enable(self.yaw_address, True, 150)
            self.motion.enable(self.pitch_address, True, 150)
        except BaseException as exc:
            self._latch_fault("ARM_FAILED", exc)
            raise GimbalControlError(self.fault_detail)

        self.origins["yaw"] = yaw_origin
        self.origins["pitch"] = pitch_origin
        self.positions["yaw"] = yaw_profile["position_deg"]
        self.positions["pitch"] = pitch_profile["position_deg"]
        offsets = self._position_offsets()
        self.targets["yaw"] = offsets["yaw"]
        self.targets["pitch"] = offsets["pitch"]
        self.flags["yaw"] = _profile_flags(yaw_profile)
        self.flags["pitch"] = _profile_flags(pitch_profile)
        self.bus_mv["yaw"] = yaw_profile["bus_mv"]
        self.bus_mv["pitch"] = pitch_profile["bus_mv"]
        self.firmware["yaw"] = yaw_profile["firmware"]
        self.firmware["pitch"] = pitch_profile["firmware"]
        self.lease_deadline_ms = None
        self.last_poll_ms = now_ms
        self.last_voltage_ms = now_ms
        self.feedback_error_streak = 0
        self.last_feedback_error = ""
        self.recovery_active = False
        self.recovery_limits = None
        self.motion_mode = "idle"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.state = self.STATE_ARMED
        self.last_event = "ARMED"
        return self.snapshot()

    def recover_to_center(
        self,
        now_ms,
        yaw_recovery_min_deg,
        yaw_recovery_max_deg,
        pitch_recovery_min_deg,
        pitch_recovery_max_deg,
        yaw_rpm,
        pitch_rpm,
        lease_ms=2000,
    ):
        if (
            self.fixed_origins["yaw"] is None
            or self.fixed_origins["pitch"] is None
        ):
            raise GimbalControlError("recovery requires fixed origins")
        if self.state == self.STATE_MOVING:
            raise GimbalControlError("stop active motion before recovery")

        recovery_limits = {
            "yaw_min": float(yaw_recovery_min_deg),
            "yaw_max": float(yaw_recovery_max_deg),
            "pitch_min": float(pitch_recovery_min_deg),
            "pitch_max": float(pitch_recovery_max_deg),
        }
        if not (
            recovery_limits["yaw_min"] < self.yaw_min_deg
            and recovery_limits["yaw_max"] > self.yaw_max_deg
            and recovery_limits["pitch_min"] < self.pitch_min_deg
            and recovery_limits["pitch_max"] > self.pitch_max_deg
        ):
            raise ValueError("recovery limits must contain session limits")

        now_ms = int(now_ms)
        try:
            self._stop_all()
            yaw_profile = self.reader.query_profile(self.yaw_address, 150)
            pitch_profile = self.reader.query_profile(
                self.pitch_address, 150
            )
            self._validate_profile(yaw_profile, "X")
            self._validate_profile(pitch_profile, "Emm")
            self.origins.update(self.fixed_origins)
            self.positions["yaw"] = yaw_profile["position_deg"]
            self.positions["pitch"] = pitch_profile["position_deg"]
            offsets = self._position_offsets()
            self._validate_recovery_offsets(offsets, recovery_limits)
            self.motion.enable(self.yaw_address, True, 150)
            self.motion.enable(self.pitch_address, True, 150)
        except BaseException as exc:
            self._latch_fault("RECOVERY_ARM_FAILED", exc)
            raise GimbalControlError(self.fault_detail)

        self.targets.update(offsets)
        self.flags["yaw"] = _profile_flags(yaw_profile)
        self.flags["pitch"] = _profile_flags(pitch_profile)
        self.bus_mv["yaw"] = yaw_profile["bus_mv"]
        self.bus_mv["pitch"] = pitch_profile["bus_mv"]
        self.firmware["yaw"] = yaw_profile["firmware"]
        self.firmware["pitch"] = pitch_profile["firmware"]
        self.lease_deadline_ms = None
        self.last_poll_ms = now_ms
        self.last_voltage_ms = now_ms
        self.feedback_error_streak = 0
        self.last_feedback_error = ""
        self.state = self.STATE_ARMED
        self.recovery_limits = recovery_limits
        if (
            abs(offsets["yaw"]) <= self.position_tolerance_deg
            and abs(offsets["pitch"]) <= self.position_tolerance_deg
        ):
            self.recovery_active = False
            self.recovery_limits = None
            self.last_event = "CENTERED"
            return self.snapshot()

        self.recovery_active = True
        try:
            self.command_offsets(
                0.0,
                0.0,
                now_ms,
                yaw_rpm,
                pitch_rpm,
                lease_ms,
            )
        except BaseException:
            self.recovery_active = False
            self.recovery_limits = None
            raise
        self.last_event = "RECOVERY_COMMAND"
        return self.snapshot()

    def command_offsets(
        self,
        yaw_deg,
        pitch_deg,
        now_ms,
        yaw_rpm=None,
        pitch_rpm=None,
        lease_ms=None,
        command_yaw=True,
        command_pitch=True,
    ):
        if self.state not in (self.STATE_ARMED, self.STATE_MOVING):
            raise GimbalControlError("gimbal command requires explicit arm")

        yaw_deg = float(yaw_deg)
        pitch_deg = float(pitch_deg)
        self._validate_target(yaw_deg, pitch_deg)
        yaw_rpm = (
            self.yaw_max_rpm if yaw_rpm is None else float(yaw_rpm)
        )
        pitch_rpm = (
            self.pitch_max_rpm if pitch_rpm is None else float(pitch_rpm)
        )
        command_yaw = bool(command_yaw)
        command_pitch = bool(command_pitch)
        if not command_yaw and not command_pitch:
            raise ValueError("gimbal command requires at least one axis")
        if not 0.0 < yaw_rpm <= self.yaw_max_rpm:
            raise ValueError("yaw command speed exceeds validated limit")
        if not 1.0 <= pitch_rpm <= self.pitch_max_rpm:
            raise ValueError("pitch command speed exceeds validated limit")
        lease_ms = _validate_lease(
            self.command_lease_ms if lease_ms is None else lease_ms
        )
        now_ms = int(now_ms)

        yaw_target_deg = self.origins["yaw"] + yaw_deg
        pitch_target_deg = self.origins["pitch"] + pitch_deg
        pitch_target_pulses = int(
            round(
                pitch_target_deg
                * EMM_PULSES_PER_REVOLUTION
                / 360.0
            )
        )
        command_begin_ms = _command_completion_ms(now_ms)
        try:
            if command_yaw:
                self.motion.move_x_position(
                    self.yaw_address,
                    yaw_target_deg,
                    yaw_rpm,
                    POSITION_MODE_ABSOLUTE,
                    150,
                    True,
                )
            if command_pitch:
                self.motion.move_emm_position(
                    self.pitch_address,
                    pitch_target_pulses,
                    pitch_rpm,
                    10,
                    POSITION_MODE_ABSOLUTE,
                    150,
                    True,
                )
            self.motion.trigger_sync(150)
        except BaseException as exc:
            self._latch_fault("COMMAND_FAILED", exc)
            raise GimbalControlError(self.fault_detail)

        if command_yaw:
            self.targets["yaw"] = yaw_deg
        if command_pitch:
            self.targets["pitch"] = pitch_deg
        completion_ms = _command_completion_ms(now_ms)
        self._record_command_timing(command_begin_ms, completion_ms)
        self.command_started_ms = completion_ms
        self.lease_deadline_ms = _ticks_add(completion_ms, lease_ms)
        self.motion_mode = "position"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.state = self.STATE_MOVING
        self.last_event = "COMMAND"
        return self.snapshot()

    def command_speeds(
        self,
        yaw_rpm,
        pitch_rpm,
        now_ms,
        yaw_acceleration_rpm_s=120,
        pitch_acceleration=10,
        lease_ms=None,
    ):
        if self.state not in (self.STATE_ARMED, self.STATE_MOVING):
            raise GimbalControlError("gimbal speed command requires explicit arm")

        yaw_rpm = float(yaw_rpm)
        pitch_rpm = float(pitch_rpm)
        if abs(yaw_rpm) > self.yaw_max_rpm:
            raise ValueError("yaw speed exceeds validated limit")
        if abs(pitch_rpm) > self.pitch_max_rpm:
            raise ValueError("pitch speed exceeds validated limit")
        yaw_acceleration_rpm_s = int(yaw_acceleration_rpm_s)
        pitch_acceleration = int(pitch_acceleration)
        if not 0 <= yaw_acceleration_rpm_s <= 0xFFFF:
            raise ValueError("yaw acceleration is outside protocol range")
        if not 0 <= pitch_acceleration <= 0xFF:
            raise ValueError("pitch acceleration is outside protocol range")
        lease_ms = _validate_lease(
            self.command_lease_ms if lease_ms is None else lease_ms
        )
        now_ms = int(now_ms)

        offsets = self._position_offsets()
        limited_yaw = False
        limited_pitch = False
        if not self.yaw_continuous:
            if (
                offsets["yaw"] <= self.yaw_min_deg and yaw_rpm < 0.0
            ) or (
                offsets["yaw"] >= self.yaw_max_deg and yaw_rpm > 0.0
            ):
                yaw_rpm = 0.0
                limited_yaw = True
        if not self.pitch_continuous and ((
            offsets["pitch"] <= self.pitch_min_deg and pitch_rpm < 0.0
        ) or (
            offsets["pitch"] >= self.pitch_max_deg and pitch_rpm > 0.0
        )):
            pitch_rpm = 0.0
            limited_pitch = True

        was_stopped = (
            abs(self.speed_commands["yaw"]) < 0.05
            and abs(self.speed_commands["pitch"]) < 0.05
        )
        command_begin_ms = _command_completion_ms(now_ms)
        try:
            self._stage_speed_with_retry(
                self.motion.move_x_speed,
                self.yaw_address,
                yaw_rpm,
                yaw_acceleration_rpm_s,
                150,
                True,
            )
            self._stage_speed_with_retry(
                self.motion.move_emm_speed,
                self.pitch_address,
                pitch_rpm,
                pitch_acceleration,
                150,
                True,
            )
            self.motion.trigger_sync(150)
        except BaseException as exc:
            self._latch_fault("SPEED_COMMAND_FAILED", exc)
            raise GimbalControlError(self.fault_detail)

        self.speed_commands["yaw"] = yaw_rpm
        self.speed_commands["pitch"] = pitch_rpm
        self.speed_limited["yaw"] = limited_yaw
        self.speed_limited["pitch"] = limited_pitch
        completion_ms = _command_completion_ms(now_ms)
        self._record_command_timing(command_begin_ms, completion_ms)
        if was_stopped and (abs(yaw_rpm) >= 0.05 or abs(pitch_rpm) >= 0.05):
            self.motion_probe = {
                "started_ms": completion_ms,
                "positions": dict(self.positions),
                "axes": {
                    "yaw": abs(yaw_rpm) >= 0.05,
                    "pitch": abs(pitch_rpm) >= 0.05,
                },
            }
            self.motion_response_ms = {"yaw": None, "pitch": None}
        self.command_started_ms = completion_ms
        self.lease_deadline_ms = _ticks_add(completion_ms, lease_ms)
        self.motion_mode = "speed"
        self.state = self.STATE_MOVING
        if limited_yaw or limited_pitch:
            self.last_event = "SPEED_LIMIT"
        else:
            self.last_event = "SPEED_COMMAND"
        return self.snapshot()

    def task(self, now_ms):
        now_ms = int(now_ms)
        if self.state == self.STATE_MOVING and self._lease_expired(now_ms):
            self._stop_all()
            self.state = self.STATE_ARMED
            self.lease_deadline_ms = None
            self.recovery_active = False
            self.recovery_limits = None
            self.motion_mode = "idle"
            self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
            self.speed_limited = {"yaw": False, "pitch": False}
            self.motion_probe = None
            self.lease_expired_count += 1
            self.last_event = "LEASE_EXPIRED"
            return self.state
        if self.state not in (self.STATE_ARMED, self.STATE_MOVING):
            return self.state
        if _ticks_diff(now_ms, self.last_poll_ms) < self.poll_interval_ms:
            return self.state

        self.last_poll_ms = now_ms
        try:
            self._update_feedback(now_ms)
            self.feedback_error_streak = 0
            self.last_feedback_error = ""
        except BaseException as exc:
            self.feedback_error_streak += 1
            self.feedback_retry_count += 1
            self.last_feedback_error = repr(exc)
            if self.feedback_error_streak >= 2:
                self._latch_fault("FEEDBACK_FAILED", exc)
            else:
                self.last_event = "FEEDBACK_RETRY"
        return self.state

    def stop(self, reason="STOP"):
        self._stop_all()
        self.state = self.STATE_DISARMED
        self.lease_deadline_ms = None
        self.recovery_active = False
        self.recovery_limits = None
        self.motion_mode = "idle"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.motion_probe = None
        self.last_event = str(reason)

    def hold(self, reason="HOLD"):
        if self.state == self.STATE_FAULT:
            return
        self._stop_all()
        self.state = self.STATE_ARMED
        self.lease_deadline_ms = None
        self.recovery_active = False
        self.recovery_limits = None
        self.motion_mode = "idle"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.motion_probe = None
        self.last_event = str(reason)

    def release(self):
        try:
            self._stop_all()
            self.motion.enable(self.yaw_address, False, 150)
            self.motion.enable(self.pitch_address, False, 150)
        except BaseException as exc:
            self._latch_fault("RELEASE_FAILED", exc)
            raise GimbalControlError(self.fault_detail)
        self.state = self.STATE_DISARMED
        self.lease_deadline_ms = None
        self.recovery_active = False
        self.recovery_limits = None
        self.motion_mode = "idle"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.last_event = "RELEASED"

    def clear_fault(self, now_ms):
        if self.state != self.STATE_FAULT:
            return True
        try:
            self._stop_all()
            yaw_profile = self.reader.query_profile(self.yaw_address, 150)
            pitch_profile = self.reader.query_profile(
                self.pitch_address, 150
            )
            self._validate_profile(yaw_profile, "X")
            self._validate_profile(pitch_profile, "Emm")
        except BaseException as exc:
            self.fault_detail = repr(exc)
            return False

        self.fault_code = ""
        self.fault_detail = ""
        self.state = self.STATE_DISARMED
        self.lease_deadline_ms = None
        self.last_poll_ms = int(now_ms)
        self.feedback_error_streak = 0
        self.last_feedback_error = ""
        self.last_event = "FAULT_CLEARED"
        self.motion_mode = "idle"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        return True

    def state_text(self):
        if self.state == self.STATE_ARMED:
            return "GIMBAL ARMED"
        if self.state == self.STATE_MOVING:
            return "GIMBAL MOVING"
        if self.state == self.STATE_FAULT:
            return "GIMBAL FAULT"
        return "GIMBAL SAFE"

    def snapshot(self):
        return {
            "state": self.state,
            "state_text": self.state_text(),
            "fault": self.fault_code,
            "fault_detail": self.fault_detail,
            "event": self.last_event,
            "origins": dict(self.origins),
            "positions": dict(self.positions),
            "offsets": self._position_offsets(),
            "targets": dict(self.targets),
            "speed_commands": dict(self.speed_commands),
            "speed_limited": dict(self.speed_limited),
            "motion_mode": self.motion_mode,
            "yaw_continuous": self.yaw_continuous,
            "pitch_continuous": self.pitch_continuous,
            "flags": {
                "yaw": dict(self.flags["yaw"]),
                "pitch": dict(self.flags["pitch"]),
            },
            "bus_mv": dict(self.bus_mv),
            "lease_active": self.lease_deadline_ms is not None,
            "lease_expired_count": self.lease_expired_count,
            "feedback_error_streak": self.feedback_error_streak,
            "feedback_retry_count": self.feedback_retry_count,
            "command_retry_count": self.command_retry_count,
            "last_feedback_error": self.last_feedback_error,
            "recovery_active": self.recovery_active,
            "timing": {
                "can_command_ms": self.last_can_command_ms,
                "can_command_max_ms": self.max_can_command_ms,
                "motion_response_ms": dict(self.motion_response_ms),
            },
        }

    def _update_feedback(self, now_ms):
        self.positions["yaw"] = self.reader.query_position_degrees(
            self.yaw_address, self.firmware["yaw"], 150
        )
        self.positions["pitch"] = self.reader.query_position_degrees(
            self.pitch_address, self.firmware["pitch"], 150
        )
        self._observe_motion_response(_command_completion_ms(now_ms))
        self.flags["yaw"] = self.reader.query_motor_flags(
            self.yaw_address, 150
        )
        self.flags["pitch"] = self.reader.query_motor_flags(
            self.pitch_address, 150
        )
        if _ticks_diff(now_ms, self.last_voltage_ms) >= (
            self.voltage_interval_ms
        ):
            self.bus_mv["yaw"] = self.reader.query_bus_voltage(
                self.yaw_address, 150
            )
            self.bus_mv["pitch"] = self.reader.query_bus_voltage(
                self.pitch_address, 150
            )
            self.last_voltage_ms = now_ms

        for axis in ("yaw", "pitch"):
            flags = self.flags[axis]
            if flags.get("stalled") or flags.get("stall_protected"):
                raise GimbalControlError("%s axis stalled" % axis)
            continuous = (
                self.yaw_continuous if axis == "yaw"
                else self.pitch_continuous
            )
            if not continuous and (
                flags.get("left_limit") or flags.get("right_limit")
            ):
                raise GimbalControlError(
                    "%s hardware limit asserted" % axis
                )
            if self.bus_mv[axis] < MIN_BUS_MV:
                raise GimbalControlError(
                    "%s bus voltage below limit" % axis
                )

        offsets = self._position_offsets()
        margin = self.position_tolerance_deg
        if self.recovery_active:
            self._validate_recovery_offsets(offsets, self.recovery_limits)
        else:
            if not self.yaw_continuous and not (
                self.yaw_min_deg - margin
                <= offsets["yaw"]
                <= self.yaw_max_deg + margin
            ):
                raise GimbalControlError("yaw software limit exceeded")
            if not self.pitch_continuous and not (
                self.pitch_min_deg - margin
                <= offsets["pitch"]
                <= self.pitch_max_deg + margin
            ):
                raise GimbalControlError("pitch software limit exceeded")

        if self.state == self.STATE_MOVING and self.motion_mode == "position":
            reached = (
                self.flags["yaw"].get("position_reached", False)
                and self.flags["pitch"].get("position_reached", False)
            )
            settled_long_enough = _ticks_diff(
                now_ms, self.command_started_ms
            ) >= self.poll_interval_ms
            if reached and settled_long_enough:
                yaw_error = abs(offsets["yaw"] - self.targets["yaw"])
                pitch_error = abs(
                    offsets["pitch"] - self.targets["pitch"]
                )
                if (
                    yaw_error <= self.position_tolerance_deg
                    and pitch_error <= self.position_tolerance_deg
                ):
                    recovered = self.recovery_active
                    self.state = self.STATE_ARMED
                    self.lease_deadline_ms = None
                    self.recovery_active = False
                    self.recovery_limits = None
                    self.motion_mode = "idle"
                    self.last_event = "RECOVERED" if recovered else "REACHED"

    def _validate_target(self, yaw_deg, pitch_deg):
        if (
            not self.yaw_continuous
            and not self.yaw_min_deg <= yaw_deg <= self.yaw_max_deg
        ):
            raise ValueError("yaw target exceeds software limit")
        if (
            not self.pitch_continuous
            and not self.pitch_min_deg <= pitch_deg <= self.pitch_max_deg
        ):
            raise ValueError("pitch target exceeds software limit")

    def _validate_profile(self, profile, expected_firmware):
        if profile["firmware"] != expected_firmware:
            raise GimbalControlError(
                "expected %s firmware at address %d" %
                (expected_firmware, profile["address"])
            )
        if not profile["closed_loop"]:
            raise GimbalControlError("motor is not in closed-loop mode")
        if profile["bus_mv"] < MIN_BUS_MV:
            raise GimbalControlError("motor bus voltage is below limit")
        if profile["stalled"] or profile["stall_protected"]:
            raise GimbalControlError("motor stall state blocks arming")

    @staticmethod
    def _validate_recovery_offsets(offsets, limits):
        if not limits["yaw_min"] <= offsets["yaw"] <= limits["yaw_max"]:
            raise GimbalControlError("yaw is outside recovery limit")
        if not (
            limits["pitch_min"]
            <= offsets["pitch"]
            <= limits["pitch_max"]
        ):
            raise GimbalControlError("pitch is outside recovery limit")

    def _position_offsets(self):
        pitch_offset = self.positions["pitch"] - self.origins["pitch"]
        if not self.pitch_continuous:
            pitch_offset = _wrapped_degrees(pitch_offset)
        return {
            "yaw": self.positions["yaw"] - self.origins["yaw"],
            "pitch": pitch_offset,
        }

    def _lease_expired(self, now_ms):
        return (
            self.lease_deadline_ms is not None
            and _ticks_diff(now_ms, self.lease_deadline_ms) >= 0
        )

    def _record_command_timing(self, begin_ms, completion_ms):
        duration_ms = max(0, _ticks_diff(completion_ms, begin_ms))
        self.command_completed_ms = completion_ms
        self.last_can_command_ms = duration_ms
        self.max_can_command_ms = max(self.max_can_command_ms, duration_ms)

    def _observe_motion_response(self, feedback_ms):
        if self.motion_probe is None:
            return
        for axis in ("yaw", "pitch"):
            if not self.motion_probe["axes"][axis]:
                continue
            if self.motion_response_ms[axis] is not None:
                continue
            moved_deg = abs(
                self.positions[axis]
                - self.motion_probe["positions"][axis]
            )
            if moved_deg >= 0.1:
                self.motion_response_ms[axis] = max(
                    0,
                    _ticks_diff(
                        feedback_ms, self.motion_probe["started_ms"]
                    ),
                )
        if all(
            not self.motion_probe["axes"][axis]
            or self.motion_response_ms[axis] is not None
            for axis in ("yaw", "pitch")
        ):
            self.motion_probe = None

    def _stop_all(self):
        for address in (self.yaw_address, self.pitch_address):
            try:
                self.motion.stop(address, 150)
            except BaseException:
                try:
                    self.motion.stop_no_reply(address, 20)
                except BaseException:
                    pass

    def _stage_speed_with_retry(self, operation, *args):
        try:
            return operation(*args)
        except ZdtTimeout:
            self.command_retry_count += 1
            return operation(*args)

    def _latch_fault(self, code, detail):
        self._stop_all()
        self.state = self.STATE_FAULT
        self.fault_code = str(code)
        self.fault_detail = repr(detail)
        self.lease_deadline_ms = None
        self.recovery_active = False
        self.recovery_limits = None
        self.motion_mode = "idle"
        self.speed_commands = {"yaw": 0.0, "pitch": 0.0}
        self.speed_limited = {"yaw": False, "pitch": False}
        self.last_event = "FAULT"


class TargetTracker:
    """Supervised velocity visual servo for two continuous gimbal axes."""

    STATE_OFF = 0
    STATE_SEARCHING = 1
    STATE_TRACKING = 2
    STATE_LOCKED = 3
    STATE_LOST = 4
    STATE_FAULT = 5

    def __init__(
        self,
        supervisor,
        coordinate_width,
        coordinate_height,
        deadband_x,
        deadband_y,
        deadband_hysteresis_x,
        deadband_hysteresis_y,
        yaw_rpm_per_pixel,
        pitch_rpm_per_pixel,
        filter_alpha,
        update_interval_ms,
        missing_stop_ms,
        lost_timeout_ms,
        min_yaw_rpm,
        min_pitch_rpm,
        max_yaw_rpm,
        max_pitch_rpm,
        yaw_acceleration_rpm_s,
        pitch_acceleration,
        speed_change_threshold_rpm,
        command_refresh_ms,
        command_lease_ms,
    ):
        self.supervisor = supervisor
        self.coordinate_width = int(coordinate_width)
        self.coordinate_height = int(coordinate_height)
        if self.coordinate_width < 2 or self.coordinate_height < 2:
            raise ValueError("tracking coordinate space is too small")
        self.center_x = self.coordinate_width // 2
        self.center_y = self.coordinate_height // 2
        self.deadband_x = int(deadband_x)
        self.deadband_y = int(deadband_y)
        self.deadband_hysteresis_x = int(deadband_hysteresis_x)
        self.deadband_hysteresis_y = int(deadband_hysteresis_y)
        if (
            self.deadband_x < 0
            or self.deadband_y < 0
            or self.deadband_hysteresis_x < 0
            or self.deadband_hysteresis_y < 0
        ):
            raise ValueError("tracking deadband must be non-negative")
        self.yaw_rpm_per_pixel = float(yaw_rpm_per_pixel)
        self.pitch_rpm_per_pixel = float(pitch_rpm_per_pixel)
        if self.yaw_rpm_per_pixel == 0.0 or self.pitch_rpm_per_pixel == 0.0:
            raise ValueError("tracking speed gain must be non-zero")
        self.filter_alpha = float(filter_alpha)
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("tracking filter alpha must be in (0, 1]")
        self.update_interval_ms = int(update_interval_ms)
        self.missing_stop_ms = int(missing_stop_ms)
        self.lost_timeout_ms = int(lost_timeout_ms)
        if self.update_interval_ms < 30:
            raise ValueError("tracking update interval is below 30 ms")
        if not self.update_interval_ms <= self.missing_stop_ms:
            raise ValueError("tracking missing-target stop is too short")
        if self.lost_timeout_ms <= self.missing_stop_ms:
            raise ValueError("tracking loss timeout is too short")
        self.min_yaw_rpm = float(min_yaw_rpm)
        self.min_pitch_rpm = float(min_pitch_rpm)
        self.max_yaw_rpm = float(max_yaw_rpm)
        self.max_pitch_rpm = float(max_pitch_rpm)
        if not 0.0 <= self.min_yaw_rpm <= self.max_yaw_rpm:
            raise ValueError("tracking yaw speed range is invalid")
        if not 0.0 <= self.min_pitch_rpm <= self.max_pitch_rpm:
            raise ValueError("tracking pitch speed range is invalid")
        if self.max_yaw_rpm > supervisor.yaw_max_rpm:
            raise ValueError("tracking yaw speed exceeds supervisor limit")
        if self.max_pitch_rpm > supervisor.pitch_max_rpm:
            raise ValueError("tracking pitch speed exceeds supervisor limit")
        self.yaw_acceleration_rpm_s = int(yaw_acceleration_rpm_s)
        self.pitch_acceleration = int(pitch_acceleration)
        self.speed_change_threshold_rpm = float(speed_change_threshold_rpm)
        self.command_refresh_ms = int(command_refresh_ms)
        if not 0 <= self.yaw_acceleration_rpm_s <= 0xFFFF:
            raise ValueError("tracking yaw acceleration is invalid")
        if not 0 <= self.pitch_acceleration <= 0xFF:
            raise ValueError("tracking pitch acceleration is invalid")
        if self.speed_change_threshold_rpm < 0.0:
            raise ValueError("tracking speed threshold is invalid")
        self.command_lease_ms = _validate_lease(command_lease_ms)
        if not self.command_refresh_ms < self.command_lease_ms:
            raise ValueError("tracking refresh must precede lease expiry")
        if self.command_lease_ms <= self.missing_stop_ms:
            raise ValueError("tracking lease must exceed missing-target stop")

        self.state = self.STATE_OFF
        self.last_event = "INIT"
        self.last_seen_ms = None
        self.last_command_ms = None
        self.filtered_error_x = None
        self.filtered_error_y = None
        self.yaw_locked = False
        self.pitch_locked = False
        self.raw_error_x = 0
        self.raw_error_y = 0
        self.command_yaw_rpm = 0.0
        self.command_pitch_rpm = 0.0
        self.stopped_for_gap = False
        self.command_count = 0
        self.lost_count = 0
        self.lease_recovery_count = 0
        self.last_control_latency_ms = 0
        self.max_control_latency_ms = 0

    def start(self, now_ms, arm_supervisor=True):
        now_ms = int(now_ms)
        if arm_supervisor:
            self.supervisor.arm(now_ms)
        else:
            if self.supervisor.state != self.supervisor.STATE_ARMED:
                raise GimbalControlError("tracker requires armed supervisor")
        self.state = self.STATE_SEARCHING
        self.last_event = "STARTED"
        self.last_seen_ms = now_ms
        self.last_command_ms = _ticks_add(
            now_ms, -self.update_interval_ms
        )
        self.filtered_error_x = None
        self.filtered_error_y = None
        self.yaw_locked = False
        self.pitch_locked = False
        self.raw_error_x = 0
        self.raw_error_y = 0
        self.command_yaw_rpm = 0.0
        self.command_pitch_rpm = 0.0
        self.stopped_for_gap = False
        return self.snapshot(now_ms)

    def task(self, target, now_ms):
        now_ms = int(now_ms)
        self.supervisor.task(now_ms)
        if self.supervisor.state == self.supervisor.STATE_FAULT:
            self.state = self.STATE_FAULT
            self.last_event = "SUPERVISOR_FAULT"
            return self.state
        if self.state == self.STATE_OFF:
            return self.state
        if self.supervisor.state == self.supervisor.STATE_DISARMED:
            self.state = self.STATE_FAULT
            self.last_event = "SUPERVISOR_DISARMED"
            return self.state
        if self.supervisor.last_event == "LEASE_EXPIRED":
            self.command_yaw_rpm = 0.0
            self.command_pitch_rpm = 0.0
            self.last_command_ms = _ticks_add(
                now_ms, -self.command_refresh_ms
            )
            self.lease_recovery_count += 1

        if target is None:
            self._handle_missing_target(now_ms)
            return self.state

        target_x = int(target[0])
        target_y = int(target[1])
        self.raw_error_x = target_x - self.center_x
        self.raw_error_y = self.center_y - target_y
        self.last_seen_ms = now_ms
        if self.state == self.STATE_LOST or self.stopped_for_gap:
            self.filtered_error_x = None
            self.filtered_error_y = None
            self.yaw_locked = False
            self.pitch_locked = False
            self.stopped_for_gap = False
            self.last_event = "REACQUIRED"

        self.filtered_error_x = self._filtered(
            self.filtered_error_x, self.raw_error_x
        )
        self.filtered_error_y = self._filtered(
            self.filtered_error_y, self.raw_error_y
        )
        error_x, self.yaw_locked = self._apply_deadband(
            self.filtered_error_x,
            self.deadband_x,
            self.deadband_hysteresis_x,
            self.yaw_locked,
        )
        error_y, self.pitch_locked = self._apply_deadband(
            self.filtered_error_y,
            self.deadband_y,
            self.deadband_hysteresis_y,
            self.pitch_locked,
        )
        if error_x == 0.0 and error_y == 0.0:
            if self.state != self.STATE_LOCKED:
                self.supervisor.hold("TRACK_LOCK")
                self.command_yaw_rpm = 0.0
                self.command_pitch_rpm = 0.0
            self.state = self.STATE_LOCKED
            self.last_event = "LOCKED"
            return self.state
        if _ticks_diff(now_ms, self.last_command_ms) < (
            self.update_interval_ms
        ):
            self.state = self.STATE_TRACKING
            return self.state

        requested_yaw_rpm = self._speed_from_error(
            error_x,
            self.yaw_rpm_per_pixel,
            self.min_yaw_rpm,
            self.max_yaw_rpm,
            0.1,
        )
        requested_pitch_rpm = self._speed_from_error(
            error_y,
            self.pitch_rpm_per_pixel,
            self.min_pitch_rpm,
            self.max_pitch_rpm,
            1.0,
        )
        speed_changed = (
            abs(requested_yaw_rpm - self.command_yaw_rpm)
            >= self.speed_change_threshold_rpm
            or abs(requested_pitch_rpm - self.command_pitch_rpm)
            >= self.speed_change_threshold_rpm
        )
        refresh_due = _ticks_diff(now_ms, self.last_command_ms) >= (
            self.command_refresh_ms
        )
        if not speed_changed and not refresh_due:
            self.state = self.STATE_TRACKING
            self.last_event = "SPEED_HOLD"
            return self.state

        snapshot = self.supervisor.command_speeds(
            requested_yaw_rpm,
            requested_pitch_rpm,
            now_ms,
            self.yaw_acceleration_rpm_s,
            self.pitch_acceleration,
            self.command_lease_ms,
        )
        self.last_control_latency_ms = max(
            0,
            _ticks_diff(self.supervisor.command_completed_ms, now_ms),
        )
        self.max_control_latency_ms = max(
            self.max_control_latency_ms, self.last_control_latency_ms
        )
        self.command_yaw_rpm = requested_yaw_rpm
        self.command_pitch_rpm = requested_pitch_rpm
        self.last_command_ms = now_ms
        self.command_count += 1
        self.state = self.STATE_TRACKING
        self.last_event = (
            "SPEED_LIMIT"
            if snapshot["speed_limited"]["yaw"]
            or snapshot["speed_limited"]["pitch"]
            else "SPEED_COMMAND"
        )
        return self.state

    def close(self):
        try:
            self.supervisor.hold("TRACKER_CLOSED")
        finally:
            self.state = self.STATE_OFF
            self.last_event = "CLOSED"

    def state_text(self):
        if self.state == self.STATE_SEARCHING:
            return "TRACK SEARCH"
        if self.state == self.STATE_TRACKING:
            if self.last_event == "SPEED_LIMIT":
                return "TRACK LIMIT"
            return "TRACK MOVE"
        if self.state == self.STATE_LOCKED:
            return "TRACK LOCK"
        if self.state == self.STATE_LOST:
            return "TRACK LOST"
        if self.state == self.STATE_FAULT:
            return "TRACK FAULT"
        return "TRACK OFF"

    def snapshot(self, now_ms=None):
        age_ms = None
        if now_ms is not None and self.last_seen_ms is not None:
            age_ms = max(0, _ticks_diff(int(now_ms), self.last_seen_ms))
        return {
            "state": self.state,
            "state_text": self.state_text(),
            "event": self.last_event,
            "error": {"x": self.raw_error_x, "y": self.raw_error_y},
            "filtered_error": {
                "x": self.filtered_error_x,
                "y": self.filtered_error_y,
            },
            "speed_rpm": {
                "yaw": self.command_yaw_rpm,
                "pitch": self.command_pitch_rpm,
            },
            "target_age_ms": age_ms,
            "command_count": self.command_count,
            "lost_count": self.lost_count,
            "lease_recovery_count": self.lease_recovery_count,
            "timing": {
                "control_ms": self.last_control_latency_ms,
                "control_max_ms": self.max_control_latency_ms,
            },
            "supervisor": self.supervisor.snapshot(),
        }

    def _handle_missing_target(self, now_ms):
        if self.last_seen_ms is None:
            self.last_seen_ms = now_ms
        age_ms = _ticks_diff(now_ms, self.last_seen_ms)
        if age_ms >= self.missing_stop_ms and not self.stopped_for_gap:
            self.supervisor.hold("TARGET_GAP")
            self.command_yaw_rpm = 0.0
            self.command_pitch_rpm = 0.0
            self.stopped_for_gap = True
            self.last_event = "TARGET_GAP_STOP"
        if age_ms < self.lost_timeout_ms:
            self.state = self.STATE_SEARCHING
            return
        if self.state != self.STATE_LOST:
            self.lost_count += 1
        self.state = self.STATE_LOST
        self.last_event = "TARGET_LOST"

    def _filtered(self, previous, current):
        if previous is None:
            return float(current)
        return (
            self.filter_alpha * float(current)
            + (1.0 - self.filter_alpha) * previous
        )

    @staticmethod
    def _speed_from_error(error, gain, minimum_rpm, maximum_rpm, quantum):
        speed_rpm = _clamp(
            float(error) * gain, -maximum_rpm, maximum_rpm
        )
        if speed_rpm != 0.0 and abs(speed_rpm) < minimum_rpm:
            speed_rpm = minimum_rpm if speed_rpm > 0.0 else -minimum_rpm
        return round(speed_rpm / quantum) * quantum

    @staticmethod
    def _apply_deadband(value, deadband, hysteresis, locked):
        limit = deadband + hysteresis if locked else deadband
        if abs(value) <= limit:
            return 0.0, True
        return value, False


def _profile_flags(profile):
    return {
        "enabled": profile["enabled"],
        "position_reached": profile["position_reached"],
        "stalled": profile["stalled"],
        "stall_protected": profile["stall_protected"],
        "left_limit": profile["left_limit"],
        "right_limit": profile["right_limit"],
    }


def _validate_lease(lease_ms):
    lease_ms = int(lease_ms)
    if not 100 <= lease_ms <= 2000:
        raise ValueError("gimbal lease must be in range 100..2000 ms")
    return lease_ms


def _wrapped_degrees(degrees):
    degrees = float(degrees)
    while degrees > 180.0:
        degrees -= 360.0
    while degrees <= -180.0:
        degrees += 360.0
    return degrees


def _clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


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
