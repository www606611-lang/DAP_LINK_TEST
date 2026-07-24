"""MCP2515 transport, bounded controller driver, and safe K2 runtime gate."""

import time


try:
    from machine import FPIOA, Pin, SPI

    CANMV_MACHINE_RUNTIME = True
except ImportError:
    FPIOA = None
    Pin = None
    SPI = None
    CANMV_MACHINE_RUNTIME = False


class MCP2515Error(Exception):
    pass


class MCP2515Timeout(MCP2515Error):
    pass


class CanMvSpiTransport:
    """One-command-per-CS QSPI0 adapter for the Lushan Pi pin map."""

    def __init__(
        self,
        sck_pin,
        mosi_pin,
        miso_pin,
        cs_pin,
        int_pin,
        baudrate=5_000_000,
    ):
        if not CANMV_MACHINE_RUNTIME:
            raise RuntimeError("CanMV machine runtime is required")

        self.fpioa = FPIOA()
        self.fpioa.set_function(int(cs_pin), FPIOA.QSPI0_CS0)
        self.fpioa.set_function(int(sck_pin), FPIOA.QSPI0_CLK)
        self.fpioa.set_function(int(mosi_pin), FPIOA.QSPI0_D0)
        self.fpioa.set_function(int(miso_pin), FPIOA.QSPI0_D1)

        gpio_function = getattr(FPIOA, "GPIO%d" % int(int_pin))
        self.fpioa.set_function(int(int_pin), gpio_function)
        self.int_pin = Pin(int(int_pin), Pin.IN, pull=Pin.PULL_UP)
        self.spi = SPI(
            1,
            baudrate=int(baudrate),
            polarity=0,
            phase=0,
            bits=8,
        )

    def transfer(self, tx_data):
        tx_data = bytes(tx_data)
        rx_data = bytearray(len(tx_data))
        self.spi.write_readinto(tx_data, rx_data)
        return bytes(rx_data)

    def interrupt_asserted(self):
        return self.int_pin.value() == 0

    def close(self):
        deinit = getattr(self.spi, "deinit", None)
        if deinit is not None:
            deinit()


class MCP2515:
    CMD_RESET = 0xC0
    CMD_READ = 0x03
    CMD_WRITE = 0x02
    CMD_BIT_MODIFY = 0x05
    CMD_READ_STATUS = 0xA0
    CMD_RTS_TX0 = 0x81

    REG_CANSTAT = 0x0E
    REG_CANCTRL = 0x0F
    REG_TEC = 0x1C
    REG_REC = 0x1D
    REG_CNF3 = 0x28
    REG_CNF2 = 0x29
    REG_CNF1 = 0x2A
    REG_CANINTE = 0x2B
    REG_CANINTF = 0x2C
    REG_EFLG = 0x2D
    REG_TXB0CTRL = 0x30
    REG_TXB0SIDH = 0x31
    REG_RXB0CTRL = 0x60
    REG_RXB0SIDH = 0x61
    REG_RXB1CTRL = 0x70
    REG_RXB1SIDH = 0x71

    MODE_NORMAL = 0x00
    MODE_SLEEP = 0x20
    MODE_LOOPBACK = 0x40
    MODE_LISTEN_ONLY = 0x60
    MODE_CONFIG = 0x80
    MODE_MASK = 0xE0

    CANINTF_RX0IF = 0x01
    CANINTF_RX1IF = 0x02
    CANINTF_TX0IF = 0x04
    TXBCTRL_TXREQ = 0x08
    TXBCTRL_ERROR_MASK = 0x70

    # 8 MHz, 500 kbit/s, 8 TQ, 75% sample point, single sampling.
    BIT_TIMING = {
        (8_000_000, 500_000): (0x00, 0x91, 0x01),
    }

    def __init__(self, transport, oscillator_hz=8_000_000, bitrate=500_000):
        self.transport = transport
        self.oscillator_hz = int(oscillator_hz)
        self.bitrate = int(bitrate)
        self.tx_count = 0
        self.rx_count = 0
        self.timeout_count = 0
        self.error_count = 0

    def reset(self):
        self._transfer((self.CMD_RESET,))
        _sleep_ms(10)

    def read_registers(self, address, count=1):
        count = int(count)
        if count <= 0:
            return b""
        response = self._transfer(
            bytes((self.CMD_READ, int(address) & 0xFF)) + bytes(count)
        )
        return response[2:]

    def read_register(self, address):
        return self.read_registers(address, 1)[0]

    def write_registers(self, address, values):
        values = bytes(values)
        self._transfer(
            bytes((self.CMD_WRITE, int(address) & 0xFF)) + values
        )

    def write_register(self, address, value):
        self.write_registers(address, (int(value) & 0xFF,))

    def bit_modify(self, address, mask, value):
        self._transfer(
            (
                self.CMD_BIT_MODIFY,
                int(address) & 0xFF,
                int(mask) & 0xFF,
                int(value) & 0xFF,
            )
        )

    def read_status(self):
        return self._transfer((self.CMD_READ_STATUS, 0x00))[1]

    def mode(self):
        return self.read_register(self.REG_CANSTAT) & self.MODE_MASK

    def set_mode(self, mode, timeout_ms=20):
        mode = int(mode) & self.MODE_MASK
        self.bit_modify(self.REG_CANCTRL, self.MODE_MASK, mode)
        deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
        while self.mode() != mode:
            if _ticks_diff(_ticks_ms(), deadline) >= 0:
                self.timeout_count += 1
                raise MCP2515Timeout("mode transition timeout: 0x%02X" % mode)
            _sleep_ms(1)

    def initialize(self, target_mode=MODE_LISTEN_ONLY):
        timing = self.BIT_TIMING.get((self.oscillator_hz, self.bitrate))
        if timing is None:
            raise ValueError(
                "unsupported MCP2515 timing: %d Hz / %d bit/s"
                % (self.oscillator_hz, self.bitrate)
            )

        self.reset()
        if self.mode() != self.MODE_CONFIG:
            self.set_mode(self.MODE_CONFIG)

        cnf1, cnf2, cnf3 = timing
        self.write_register(self.REG_CNF1, cnf1)
        self.write_register(self.REG_CNF2, cnf2)
        self.write_register(self.REG_CNF3, cnf3)
        self.write_register(self.REG_RXB0CTRL, 0x60)
        self.write_register(self.REG_RXB1CTRL, 0x60)
        self.write_register(self.REG_CANINTE, 0x03)
        self.write_register(self.REG_CANINTF, 0x00)
        self.write_register(self.REG_EFLG, 0x00)

        if self.read_register(self.REG_CNF1) != cnf1:
            raise MCP2515Error("CNF1 readback mismatch")
        if self.read_register(self.REG_CNF2) != cnf2:
            raise MCP2515Error("CNF2 readback mismatch")
        if self.read_register(self.REG_CNF3) != cnf3:
            raise MCP2515Error("CNF3 readback mismatch")
        self.set_mode(target_mode)

    @staticmethod
    def encode_identifier(can_id, extended=True):
        can_id = int(can_id)
        if extended:
            if not 0 <= can_id <= 0x1FFFFFFF:
                raise ValueError("extended CAN identifier out of range")
            return bytes(
                (
                    (can_id >> 21) & 0xFF,
                    ((can_id >> 13) & 0xE0)
                    | 0x08
                    | ((can_id >> 16) & 0x03),
                    (can_id >> 8) & 0xFF,
                    can_id & 0xFF,
                )
            )
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("standard CAN identifier out of range")
        return bytes(((can_id >> 3) & 0xFF, (can_id << 5) & 0xE0, 0, 0))

    @staticmethod
    def decode_identifier(register_bytes):
        sidh, sidl, eid8, eid0 = bytes(register_bytes[:4])
        extended = bool(sidl & 0x08)
        if extended:
            can_id = (
                (sidh << 21)
                | ((sidl & 0xE0) << 13)
                | ((sidl & 0x03) << 16)
                | (eid8 << 8)
                | eid0
            )
        else:
            can_id = (sidh << 3) | (sidl >> 5)
        return can_id, extended

    def send(self, can_id, data=b"", extended=True, timeout_ms=20):
        data = bytes(data)
        if len(data) > 8:
            raise ValueError("classic CAN payload exceeds 8 bytes")
        if self.read_register(self.REG_TXB0CTRL) & self.TXBCTRL_TXREQ:
            raise MCP2515Error("TX buffer 0 is busy")

        header = self.encode_identifier(can_id, extended)
        self.write_registers(
            self.REG_TXB0SIDH,
            header + bytes((len(data),)) + data,
        )
        self._transfer((self.CMD_RTS_TX0,))

        deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
        while True:
            tx_control = self.read_register(self.REG_TXB0CTRL)
            if not (tx_control & self.TXBCTRL_TXREQ):
                if tx_control & self.TXBCTRL_ERROR_MASK:
                    self.bit_modify(
                        self.REG_TXB0CTRL, self.TXBCTRL_ERROR_MASK, 0
                    )
                    self.bit_modify(
                        self.REG_CANINTF, self.CANINTF_TX0IF, 0
                    )
                    self.error_count += 1
                    raise MCP2515Error(
                        "CAN transmit error: 0x%02X" % tx_control
                    )
                self.bit_modify(
                    self.REG_CANINTF, self.CANINTF_TX0IF, 0
                )
                self.tx_count += 1
                return
            if _ticks_diff(_ticks_ms(), deadline) >= 0:
                self.bit_modify(
                    self.REG_TXB0CTRL, self.TXBCTRL_TXREQ, 0
                )
                self.bit_modify(
                    self.REG_CANINTF, self.CANINTF_TX0IF, 0
                )
                self.timeout_count += 1
                raise MCP2515Timeout("CAN transmit timeout")
            _sleep_ms(1)

    def receive(self):
        flags = self.read_register(self.REG_CANINTF)
        if flags & self.CANINTF_RX0IF:
            base = self.REG_RXB0SIDH
            flag = self.CANINTF_RX0IF
            buffer_index = 0
        elif flags & self.CANINTF_RX1IF:
            base = self.REG_RXB1SIDH
            flag = self.CANINTF_RX1IF
            buffer_index = 1
        else:
            return None

        raw = self.read_registers(base, 13)
        can_id, extended = self.decode_identifier(raw[:4])
        length = raw[4] & 0x0F
        if length > 8:
            self.error_count += 1
            self.bit_modify(self.REG_CANINTF, flag, 0)
            raise MCP2515Error("invalid received DLC: %d" % length)
        message = {
            "id": can_id,
            "extended": extended,
            "rtr": bool(raw[4] & 0x40),
            "data": bytes(raw[5 : 5 + length]),
            "buffer": buffer_index,
        }
        self.bit_modify(self.REG_CANINTF, flag, 0)
        self.rx_count += 1
        return message

    def loopback_self_test(self, rounds=4, timeout_ms=30):
        self.set_mode(self.MODE_LOOPBACK)
        self.write_register(self.REG_CANINTF, 0)
        for index in range(int(rounds)):
            can_id = 0x001ABCDE + index
            payload = bytes((0xA5, index & 0xFF, 0x5A, 0xC3))
            self.send(can_id, payload, extended=True, timeout_ms=timeout_ms)
            deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
            message = self.receive()
            while message is None:
                if _ticks_diff(_ticks_ms(), deadline) >= 0:
                    self.timeout_count += 1
                    raise MCP2515Timeout("loopback receive timeout")
                _sleep_ms(1)
                message = self.receive()
            if (
                message["id"] != can_id
                or not message["extended"]
                or message["data"] != payload
            ):
                self.error_count += 1
                raise MCP2515Error("loopback payload mismatch")
        return True

    def snapshot(self):
        return {
            "mode": self.mode(),
            "canstat": self.read_register(self.REG_CANSTAT),
            "canctrl": self.read_register(self.REG_CANCTRL),
            "tec": self.read_register(self.REG_TEC),
            "rec": self.read_register(self.REG_REC),
            "eflg": self.read_register(self.REG_EFLG),
            "canintf": self.read_register(self.REG_CANINTF),
            "tx": self.tx_count,
            "rx": self.rx_count,
            "timeouts": self.timeout_count,
            "errors": self.error_count,
        }

    def _transfer(self, tx_data):
        tx_data = bytes(tx_data)
        response = bytes(self.transport.transfer(tx_data))
        if len(response) != len(tx_data):
            self.error_count += 1
            raise MCP2515Error("SPI response length mismatch")
        return response


class MCP2515RuntimeGate:
    """Boot-time internal loopback followed by receive-only bus observation."""

    STATE_OFF = 0
    STATE_LISTEN = 1
    STATE_FAILED = 2
    STATE_ACTIVE = 3

    def __init__(self, config_module, transport_factory=CanMvSpiTransport):
        self.config = config_module
        self.transport_factory = transport_factory
        self.transport = None
        self.controller = None
        self.state = self.STATE_OFF
        self.last_error = ""
        self.observed_frames = 0
        self.last_message = None
        self.discovery = {}

    def start(self):
        try:
            self.transport = self.transport_factory(
                self.config.MCP2515_SCK_PIN,
                self.config.MCP2515_MOSI_PIN,
                self.config.MCP2515_MISO_PIN,
                self.config.MCP2515_CS_PIN,
                self.config.MCP2515_INT_PIN,
                self.config.MCP2515_SPI_BAUDRATE,
            )
            self.controller = MCP2515(
                self.transport,
                self.config.MCP2515_OSC_HZ,
                self.config.CAN_BITRATE,
            )
            self.controller.initialize(MCP2515.MODE_LOOPBACK)
            self.controller.loopback_self_test(
                self.config.MCP2515_SELF_TEST_ROUNDS
            )
            print("CAN loopback PASS")
            if getattr(self.config, "ZDT_DISCOVERY_ENABLED", False):
                self._discover_zdt()
            else:
                self.controller.set_mode(MCP2515.MODE_LISTEN_ONLY)
            self.state = self.STATE_LISTEN
            print("CAN mode=LISTEN_ONLY")
            print("CAN status:", self.controller.snapshot())
        except BaseException as exc:
            self.last_error = repr(exc)
            self.state = self.STATE_FAILED
            print("CAN gate failed:", self.last_error)
        return self.state == self.STATE_LISTEN

    def task(self):
        if self.state != self.STATE_LISTEN:
            return
        try:
            interrupt_asserted = getattr(
                self.transport, "interrupt_asserted", None
            )
            if interrupt_asserted is not None and not interrupt_asserted():
                return
            for _ in range(4):
                message = self.controller.receive()
                if message is None:
                    break
                self.last_message = message
                self.observed_frames += 1
        except BaseException as exc:
            self.last_error = repr(exc)
            self.state = self.STATE_FAILED
            print("CAN listen failed:", self.last_error)

    def activate_normal(self):
        if self.state != self.STATE_LISTEN or self.controller is None:
            raise MCP2515Error("CAN gate is not ready for active ownership")
        self.controller.set_mode(MCP2515.MODE_NORMAL)
        self.state = self.STATE_ACTIVE
        print("CAN mode=NORMAL owner=GIMBAL")
        return self.controller

    def state_text(self):
        if self.state == self.STATE_ACTIVE:
            return "CAN ACTIVE"
        if self.state == self.STATE_LISTEN:
            if self.discovery:
                parts = []
                for address in self.config.ZDT_DISCOVERY_ADDRESSES:
                    result = self.discovery.get(int(address))
                    parts.append(
                        "%d%s" %
                        (int(address), "OK" if result and result["ok"] else "--")
                    )
                return "ZDT " + " ".join(parts)
            return "CAN LISTEN"
        if self.state == self.STATE_FAILED:
            return "CAN FAIL"
        return "CAN OFF"

    def snapshot(self):
        snapshot = {
            "state": self.state,
            "observed": self.observed_frames,
            "error": self.last_error,
            "zdt": self.discovery,
        }
        if self.controller is not None:
            snapshot.update(self.controller.snapshot())
        return snapshot

    def close(self):
        if self.controller is not None:
            try:
                self.controller.set_mode(MCP2515.MODE_LISTEN_ONLY)
            except Exception:
                pass
        if self.transport is not None:
            try:
                self.transport.close()
            except Exception:
                pass
        self.state = self.STATE_OFF

    def _discover_zdt(self):
        from zdt_motor import ZdtReadOnlyClient, format_hex

        client = ZdtReadOnlyClient(self.controller)
        self.controller.set_mode(MCP2515.MODE_NORMAL)
        try:
            for configured_address in self.config.ZDT_DISCOVERY_ADDRESSES:
                address = int(configured_address)
                try:
                    profile = client.query_profile(
                        address,
                        self.config.ZDT_DISCOVERY_TIMEOUT_MS,
                    )
                    position_response = client.query_position(
                        address,
                        self.config.ZDT_DISCOVERY_TIMEOUT_MS,
                    )
                    self.discovery[address] = {
                        "ok": True,
                        "raw": format_hex(position_response["raw"]),
                        "profile": profile,
                    }
                    print(
                        "ZDT address=%d profile=%s" % (address, profile)
                    )
                except BaseException as exc:
                    self.discovery[address] = {
                        "ok": False,
                        "error": repr(exc),
                    }
                    print("ZDT address=%d query failed: %r" % (address, exc))
        finally:
            self.controller.set_mode(MCP2515.MODE_LISTEN_ONLY)


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
