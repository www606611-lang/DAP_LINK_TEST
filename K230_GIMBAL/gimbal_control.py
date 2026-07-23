"""Bounded two-axis gimbal supervision above the ZDT protocol layer."""

import time

from zdt_motor import (
    POSITION_MODE_ABSOLUTE,
    ZdtCommandClient,
    ZdtReadOnlyClient,
)


MIN_BUS_MV = 10000
EMM_PULSES_PER_REVOLUTION = 3200


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
        if not self.yaw_min_deg < 0.0 < self.yaw_max_deg:
            raise ValueError("yaw software limits must span zero")
        if not self.pitch_min_deg < 0.0 < self.pitch_max_deg:
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
        self.state = self.STATE_DISARMED
        self.fault_code = ""
        self.fault_detail = ""
        self.last_event = "INIT"
        self.origins = {"yaw": 0.0, "pitch": 0.0}
        self.positions = {"yaw": 0.0, "pitch": 0.0}
        self.targets = {"yaw": 0.0, "pitch": 0.0}
        self.flags = {"yaw": {}, "pitch": {}}
        self.bus_mv = {"yaw": 0, "pitch": 0}
        self.firmware = {"yaw": "X", "pitch": "Emm"}
        self.lease_deadline_ms = None
        self.command_started_ms = 0
        self.last_poll_ms = 0
        self.last_voltage_ms = 0
        self.lease_expired_count = 0

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
            self.motion.enable(self.yaw_address, True, 150)
            self.motion.enable(self.pitch_address, True, 150)
        except BaseException as exc:
            self._latch_fault("ARM_FAILED", exc)
            raise GimbalControlError(self.fault_detail)

        self.origins["yaw"] = yaw_profile["position_deg"]
        self.origins["pitch"] = pitch_profile["position_deg"]
        self.positions.update(self.origins)
        self.targets["yaw"] = 0.0
        self.targets["pitch"] = 0.0
        self.flags["yaw"] = _profile_flags(yaw_profile)
        self.flags["pitch"] = _profile_flags(pitch_profile)
        self.bus_mv["yaw"] = yaw_profile["bus_mv"]
        self.bus_mv["pitch"] = pitch_profile["bus_mv"]
        self.firmware["yaw"] = yaw_profile["firmware"]
        self.firmware["pitch"] = pitch_profile["firmware"]
        self.lease_deadline_ms = None
        self.last_poll_ms = now_ms
        self.last_voltage_ms = now_ms
        self.state = self.STATE_ARMED
        self.last_event = "ARMED"
        return self.snapshot()

    def command_offsets(
        self,
        yaw_deg,
        pitch_deg,
        now_ms,
        yaw_rpm=None,
        pitch_rpm=None,
        lease_ms=None,
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
        try:
            self.motion.move_x_position(
                self.yaw_address,
                yaw_target_deg,
                yaw_rpm,
                POSITION_MODE_ABSOLUTE,
                150,
                True,
            )
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

        self.targets["yaw"] = yaw_deg
        self.targets["pitch"] = pitch_deg
        self.command_started_ms = now_ms
        self.lease_deadline_ms = _ticks_add(now_ms, lease_ms)
        self.state = self.STATE_MOVING
        self.last_event = "COMMAND"
        return self.snapshot()

    def task(self, now_ms):
        now_ms = int(now_ms)
        if self.state == self.STATE_MOVING and self._lease_expired(now_ms):
            self._stop_all()
            self.state = self.STATE_DISARMED
            self.lease_deadline_ms = None
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
        except BaseException as exc:
            self._latch_fault("FEEDBACK_FAILED", exc)
        return self.state

    def stop(self, reason="STOP"):
        self._stop_all()
        self.state = self.STATE_DISARMED
        self.lease_deadline_ms = None
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
        self.last_event = "FAULT_CLEARED"
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
            "flags": {
                "yaw": dict(self.flags["yaw"]),
                "pitch": dict(self.flags["pitch"]),
            },
            "bus_mv": dict(self.bus_mv),
            "lease_active": self.lease_deadline_ms is not None,
            "lease_expired_count": self.lease_expired_count,
        }

    def _update_feedback(self, now_ms):
        self.positions["yaw"] = self.reader.query_position_degrees(
            self.yaw_address, self.firmware["yaw"], 150
        )
        self.positions["pitch"] = self.reader.query_position_degrees(
            self.pitch_address, self.firmware["pitch"], 150
        )
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
            if flags.get("left_limit") or flags.get("right_limit"):
                raise GimbalControlError(
                    "%s hardware limit asserted" % axis
                )
            if self.bus_mv[axis] < MIN_BUS_MV:
                raise GimbalControlError(
                    "%s bus voltage below limit" % axis
                )

        offsets = self._position_offsets()
        margin = self.position_tolerance_deg
        if not (
            self.yaw_min_deg - margin
            <= offsets["yaw"]
            <= self.yaw_max_deg + margin
        ):
            raise GimbalControlError("yaw software limit exceeded")
        if not (
            self.pitch_min_deg - margin
            <= offsets["pitch"]
            <= self.pitch_max_deg + margin
        ):
            raise GimbalControlError("pitch software limit exceeded")

        if self.state == self.STATE_MOVING:
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
                    self.state = self.STATE_ARMED
                    self.lease_deadline_ms = None
                    self.last_event = "REACHED"

    def _validate_target(self, yaw_deg, pitch_deg):
        if not self.yaw_min_deg <= yaw_deg <= self.yaw_max_deg:
            raise ValueError("yaw target exceeds software limit")
        if not self.pitch_min_deg <= pitch_deg <= self.pitch_max_deg:
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

    def _position_offsets(self):
        return {
            "yaw": self.positions["yaw"] - self.origins["yaw"],
            "pitch": _wrapped_degrees(
                self.positions["pitch"] - self.origins["pitch"]
            ),
        }

    def _lease_expired(self, now_ms):
        return (
            self.lease_deadline_ms is not None
            and _ticks_diff(now_ms, self.lease_deadline_ms) >= 0
        )

    def _stop_all(self):
        for address in (self.yaw_address, self.pitch_address):
            try:
                self.motion.stop(address, 150)
            except BaseException:
                try:
                    self.motion.stop_no_reply(address, 20)
                except BaseException:
                    pass

    def _latch_fault(self, code, detail):
        self._stop_all()
        self.state = self.STATE_FAULT
        self.fault_code = str(code)
        self.fault_detail = repr(detail)
        self.lease_deadline_ms = None
        self.last_event = "FAULT"


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
