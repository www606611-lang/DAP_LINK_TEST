"""Supervised gimbal motion built on the ZDT device API."""

import time

from zdt_motor import (
    ZdtCommandClient,
    ZdtReadOnlyClient,
    parse_position_degrees,
)


MIN_BUS_MV = 10000
MAX_AXIS_PROBE_DEGREES = 8.0
MAX_AXIS_PROBE_RPM = 10.0
EMM_PULSES_PER_REVOLUTION = 3200
MAX_EMM_AXIS_PROBE_PULSES = 72


def run_axis_probe(
    can_controller,
    address,
    degrees=5.0,
    speed_rpm=5.0,
    lease_ms=1200,
):
    degrees = float(degrees)
    speed_rpm = float(speed_rpm)
    lease_ms = int(lease_ms)
    if abs(degrees) > MAX_AXIS_PROBE_DEGREES or degrees == 0.0:
        raise ValueError("axis probe angle exceeds commissioning limit")
    if not 0.0 < speed_rpm <= MAX_AXIS_PROBE_RPM:
        raise ValueError("axis probe speed exceeds commissioning limit")
    if not 200 <= lease_ms <= 2000:
        raise ValueError("axis probe lease is outside bounded range")

    reader = ZdtReadOnlyClient(can_controller)
    motion = ZdtCommandClient(can_controller)
    profile = reader.query_profile(address, 150)
    _require_motion_ready(profile)
    if profile["firmware"] != "X":
        raise ValueError("first axis probe requires X firmware")

    before_deg = profile["position_deg"]
    result = {
        "address": int(address),
        "firmware": profile["firmware"],
        "before_deg": before_deg,
        "requested_deg": degrees,
        "speed_rpm": speed_rpm,
        "reached": False,
    }
    motion.stop(address, 150)
    motion.enable(address, True, 150)
    try:
        motion.move_x_relative(address, degrees, speed_rpm, 150)
        deadline = _ticks_add(_ticks_ms(), lease_ms)
        while _ticks_diff(_ticks_ms(), deadline) < 0:
            _sleep_ms(30)
            position_raw = reader.query_position(address, 150)["raw"]
            after_deg = _parse_x_position(position_raw)
            flags = reader.query(address, 0x3A, 150)["raw"]
            result["after_deg"] = after_deg
            result["delta_deg"] = after_deg - before_deg
            result["flags_raw"] = _format_hex(flags)
            position_error = abs(result["delta_deg"] - degrees)
            if (
                len(flags) >= 4
                and (flags[2] & 0x02)
                and position_error <= 0.8
            ):
                result["reached"] = True
                break
        if "after_deg" not in result:
            raise RuntimeError("axis probe produced no position feedback")
        return result
    finally:
        try:
            motion.stop(address, 150)
        except Exception:
            motion.stop_no_reply(address, 20)


def run_emm_axis_probe(
    can_controller,
    address,
    pulses=71,
    speed_rpm=3.0,
    acceleration=10,
    lease_ms=1500,
):
    pulses = int(pulses)
    speed_rpm = float(speed_rpm)
    acceleration = int(acceleration)
    lease_ms = int(lease_ms)
    if pulses == 0 or abs(pulses) > MAX_EMM_AXIS_PROBE_PULSES:
        raise ValueError("Emm axis probe pulses exceed commissioning limit")
    if not 0.0 < speed_rpm <= MAX_AXIS_PROBE_RPM:
        raise ValueError("Emm axis probe speed exceeds commissioning limit")
    if not 0 <= acceleration <= 0xFF:
        raise ValueError("Emm axis probe acceleration is outside range")
    if not 200 <= lease_ms <= 2000:
        raise ValueError("Emm axis probe lease is outside bounded range")

    reader = ZdtReadOnlyClient(can_controller)
    motion = ZdtCommandClient(can_controller)
    profile = reader.query_profile(address, 150)
    _require_motion_ready(profile)
    if profile["firmware"] != "Emm":
        raise ValueError("Emm axis probe requires Emm firmware")

    before_deg = profile["position_deg"]
    requested_deg = (
        pulses * 360.0 / EMM_PULSES_PER_REVOLUTION
    )
    result = {
        "address": int(address),
        "firmware": profile["firmware"],
        "before_deg": before_deg,
        "requested_pulses": pulses,
        "requested_deg": requested_deg,
        "speed_rpm": speed_rpm,
        "reached": False,
    }
    motion.stop(address, 150)
    motion.enable(address, True, 150)
    try:
        motion.move_emm_relative(
            address, pulses, speed_rpm, acceleration, 150
        )
        deadline = _ticks_add(_ticks_ms(), lease_ms)
        while _ticks_diff(_ticks_ms(), deadline) < 0:
            _sleep_ms(30)
            position_raw = reader.query_position(address, 150)["raw"]
            after_deg = parse_position_degrees(position_raw, "Emm")
            flags = reader.query(address, 0x3A, 150)["raw"]
            result["after_deg"] = after_deg
            result["delta_deg"] = _wrapped_degrees(
                after_deg - before_deg
            )
            result["flags_raw"] = _format_hex(flags)
            position_error = abs(
                result["delta_deg"] - requested_deg
            )
            if (
                len(flags) >= 4
                and (flags[2] & 0x02)
                and position_error <= 1.0
            ):
                result["reached"] = True
                break
        if "after_deg" not in result:
            raise RuntimeError("Emm axis probe produced no position feedback")
        return result
    finally:
        try:
            motion.stop(address, 150)
        except Exception:
            motion.stop_no_reply(address, 20)


def _require_motion_ready(profile):
    if not profile["closed_loop"]:
        raise RuntimeError("ZDT motor is not in closed-loop mode")
    if profile["bus_mv"] < MIN_BUS_MV:
        raise RuntimeError("ZDT bus voltage is below commissioning limit")
    if profile["stalled"] or profile["stall_protected"]:
        raise RuntimeError("ZDT stall state blocks motion")


def _parse_x_position(response):
    response = bytes(response)
    magnitude = int.from_bytes(response[3:7], "big") / 10.0
    return -magnitude if response[2] else magnitude


def _wrapped_degrees(degrees):
    degrees = float(degrees)
    while degrees > 180.0:
        degrees -= 360.0
    while degrees <= -180.0:
        degrees += 360.0
    return degrees


def _format_hex(data):
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
