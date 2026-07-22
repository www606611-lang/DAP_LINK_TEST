"""UART2 compatibility link from the K230 to the Tianmengxing chassis."""

import config


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def encode_target_packet(valid, cx, cy):
    """Return the validated @valid,cx,cy# compatibility frame."""
    x = clamp(int(cx), 0, config.UART_COORD_WIDTH - 1)
    y = clamp(int(cy), 0, config.UART_COORD_HEIGHT - 1)
    return "@%d,%03d,%03d#" % (1 if valid else 0, x, y)


class ChassisLink:
    """Own UART setup and the last published target coordinates."""

    def __init__(self, uart=None):
        self.uart = uart
        self.last_cx = config.UART_COORD_WIDTH // 2
        self.last_cy = config.UART_COORD_HEIGHT // 2
        self.last_valid = False

    def open(self):
        if self.uart is not None:
            return self.uart

        from machine import FPIOA, UART

        fpioa = FPIOA()
        fpioa.set_function(config.UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(config.UART_RX_PIN, FPIOA.UART2_RXD)
        self.uart = UART(
            UART.UART2,
            baudrate=config.UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        print(
            "uart2 ready tx=io%d rx=io%d baud=%d" %
            (config.UART_TX_PIN, config.UART_RX_PIN, config.UART_BAUDRATE)
        )
        return self.uart

    def publish(self, valid, cx=None, cy=None):
        if valid:
            if cx is None or cy is None:
                raise ValueError("valid target requires cx and cy")
            self.last_cx = clamp(int(cx), 0, config.UART_COORD_WIDTH - 1)
            self.last_cy = clamp(int(cy), 0, config.UART_COORD_HEIGHT - 1)

        self.last_valid = bool(valid)
        packet = encode_target_packet(
            self.last_valid, self.last_cx, self.last_cy)
        if self.uart is not None:
            self.uart.write(packet)
        return self.last_cx, self.last_cy, self.last_valid

    def close(self):
        if self.uart is not None:
            self.uart.deinit()
            self.uart = None
