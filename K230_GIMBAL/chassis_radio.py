"""Nonblocking K230 WLAN/UDP endpoint for the chassis wireless bridge."""

import config
from wire_protocol import (
    FrameParser,
    ROLE_CHASSIS,
    ROLE_ESP32,
    ROLE_K230,
    TYPE_HEARTBEAT,
    TYPE_HELLO,
    TYPE_STATUS,
    encode_frame,
    encode_u32_le,
    sequence_is_newer,
)

try:
    import network
    import socket

    NETWORK_RUNTIME = True
except ImportError:
    network = None
    socket = None
    NETWORK_RUNTIME = False


class ChassisRadio:
    STATE_OFF = 0
    STATE_WIFI_WAIT = 1
    STATE_PEER_WAIT = 2
    STATE_ONLINE = 3

    def __init__(self):
        self.state = self.STATE_OFF
        self.wlan = None
        self.sock = None
        self.parser = FrameParser()
        self.tx_sequence = 0
        self.last_rx_sequence = [None, None, None, None]
        self.last_rx_by_role_ms = [0, 0, 0, 0]
        self.last_rx_ms = 0
        self.last_tx_ms = 0
        self.last_connect_attempt_ms = 0
        self.rx_frame_count = 0
        self.tx_frame_count = 0
        self.duplicate_count = 0
        self.out_of_order_count = 0
        self.unsupported_count = 0
        self.socket_error_count = 0

    def open(self, now_ms):
        if not NETWORK_RUNTIME:
            raise RuntimeError("chassis radio requires CanMV network runtime")
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        try:
            self.wlan.config(reconnects=-1)
        except Exception:
            pass
        self.state = self.STATE_WIFI_WAIT
        self._connect(now_ms)

    def task(self, now_ms):
        if self.wlan is None:
            return

        if not self.wlan.isconnected():
            self.state = self.STATE_WIFI_WAIT
            self._close_socket()
            if _elapsed(now_ms, self.last_connect_attempt_ms) >= (
                config.CHASSIS_RADIO_RECONNECT_MS
            ):
                self._connect(now_ms)
            return

        if self.sock is None:
            self._open_socket()
            if self.sock is None:
                return
            self.state = self.STATE_PEER_WAIT

        self._receive(now_ms)
        bridge_online = self._role_online(ROLE_ESP32, now_ms)
        self.state = (
            self.STATE_ONLINE if bridge_online else self.STATE_PEER_WAIT
        )

        interval = (
            config.CHASSIS_RADIO_HEARTBEAT_MS
            if bridge_online
            else config.CHASSIS_RADIO_HELLO_MS
        )
        if _elapsed(now_ms, self.last_tx_ms) >= interval:
            message_type = (
                TYPE_HEARTBEAT if bridge_online else TYPE_HELLO
            )
            payload = bytes((ROLE_K230,)) + encode_u32_le(now_ms)
            self._send(message_type, payload, now_ms)

    def close(self):
        self._close_socket()
        if self.wlan is not None:
            try:
                self.wlan.disconnect()
                self.wlan.active(False)
            except Exception:
                pass
            self.wlan = None
        self.state = self.STATE_OFF

    def state_text(self):
        if self.state == self.STATE_WIFI_WAIT:
            return "WIFI WAIT"
        if self.state == self.STATE_PEER_WAIT:
            return "RADIO WAIT"
        if self.state == self.STATE_ONLINE:
            return "RADIO ONLINE"
        return "RADIO OFF"

    def snapshot(self, now_ms):
        bridge_online = self._role_online(ROLE_ESP32, now_ms)
        chassis_online = self._role_online(ROLE_CHASSIS, now_ms)
        return {
            "state": self.state,
            "online": bridge_online,
            "end_to_end": bridge_online and chassis_online,
            "bridge_online": bridge_online,
            "chassis_online": chassis_online,
            "age_ms": _elapsed(now_ms, self.last_rx_ms),
            "rx_frames": self.rx_frame_count,
            "tx_frames": self.tx_frame_count,
            "duplicates": self.duplicate_count,
            "out_of_order": self.out_of_order_count,
            "unsupported": self.unsupported_count,
            "crc_errors": self.parser.crc_error_count,
            "length_errors": self.parser.length_error_count,
            "socket_errors": self.socket_error_count,
        }

    def _connect(self, now_ms):
        self.last_connect_attempt_ms = now_ms
        try:
            self.wlan.connect(
                config.CHASSIS_WIFI_SSID,
                config.CHASSIS_WIFI_PASSWORD,
            )
        except Exception:
            self.socket_error_count += 1

    def _open_socket(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", config.CHASSIS_RADIO_LOCAL_PORT))
            self.sock.settimeout(0.001)
        except Exception:
            self.socket_error_count += 1
            self._close_socket()

    def _close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _receive(self, now_ms):
        for _ in range(4):
            try:
                data, _address = self.sock.recvfrom(96)
            except OSError:
                break
            except Exception:
                self.socket_error_count += 1
                break

            frames = self.parser.feed(data)
            for message_type, sequence, payload in frames:
                if (
                    message_type not in
                    (TYPE_HELLO, TYPE_HEARTBEAT, TYPE_STATUS)
                    or len(payload) < 1
                ):
                    self.unsupported_count += 1
                    continue
                role = payload[0]
                if role not in (ROLE_ESP32, ROLE_CHASSIS):
                    self.unsupported_count += 1
                    continue
                previous = self.last_rx_sequence[role]
                if message_type == TYPE_HELLO:
                    self.last_rx_sequence[role] = sequence
                    self.last_rx_by_role_ms[role] = now_ms
                    self.last_rx_ms = now_ms
                    self.rx_frame_count += 1
                    continue
                if previous is not None and sequence == previous:
                    self.duplicate_count += 1
                    continue
                if not sequence_is_newer(sequence, previous):
                    self.out_of_order_count += 1
                    continue
                self.last_rx_sequence[role] = sequence
                self.last_rx_by_role_ms[role] = now_ms
                self.last_rx_ms = now_ms
                self.rx_frame_count += 1

    def _send(self, message_type, payload, now_ms):
        packet = encode_frame(message_type, self.tx_sequence, payload)
        self.tx_sequence = (self.tx_sequence + 1) & 0xFFFF
        try:
            self.sock.sendto(
                packet,
                (
                    config.CHASSIS_RADIO_HOST,
                    config.CHASSIS_RADIO_REMOTE_PORT,
                ),
            )
            self.tx_frame_count += 1
            self.last_tx_ms = now_ms
        except Exception:
            self.socket_error_count += 1

    def _role_online(self, role, now_ms):
        return (
            self.last_rx_sequence[role] is not None
            and _elapsed(now_ms, self.last_rx_by_role_ms[role])
            < config.CHASSIS_RADIO_OFFLINE_MS
        )


def _elapsed(now_ms, previous_ms):
    try:
        import time

        return time.ticks_diff(now_ms, previous_ms)
    except (ImportError, AttributeError):
        return int(now_ms) - int(previous_ms)
