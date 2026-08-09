"""串口传输层测试。"""

from gasrobot_base.serial_transport import SerialTransport


class _FakeSerial:
    def __init__(self) -> None:
        self.port = None
        self.is_open = False
        self.dtr = True
        self.rts = True
        self.in_waiting = 3
        self.input_reset = False
        self.output_reset = False
        self.closed = False

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False
        self.closed = True

    def reset_input_buffer(self) -> None:
        self.input_reset = True

    def reset_output_buffer(self) -> None:
        self.output_reset = True

    def read(self, size: int) -> bytes:
        return b"abc"[:size]

    def write(self, data: bytes) -> int:
        return len(data)


def test_transport_manages_connection_and_nonblocking_io(monkeypatch):
    connection = _FakeSerial()
    monkeypatch.setattr(
        "gasrobot_base.serial_transport.serial.Serial",
        lambda **_kwargs: connection,
    )
    transport = SerialTransport("/dev/ttyTEST", 115200, 0.0)

    transport.connect()

    assert transport.is_open
    assert connection.port == "/dev/ttyTEST"
    assert connection.dtr is False
    assert connection.rts is False
    assert connection.input_reset
    assert connection.output_reset
    assert transport.read_available() == b"abc"
    assert transport.write(b"1234") == 4

    transport.close()
    assert not transport.is_open
    assert connection.closed
