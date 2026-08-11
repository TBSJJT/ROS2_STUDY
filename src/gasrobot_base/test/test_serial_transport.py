"""串口传输层测试。"""

from gasrobot_base.serial_transport import SerialTransport


class _FakeSerial:
    """模拟 pyserial 的最小接口，避免测试依赖真实串口硬件。"""

    def __init__(self) -> None:
        """初始化可供断言检查的串口状态。"""

        self.port = None
        self.is_open = False
        self.dtr = True
        self.rts = True
        self.in_waiting = 3
        self.input_reset = False
        self.output_reset = False
        self.closed = False

    def open(self) -> None:
        """模拟打开串口。"""

        self.is_open = True

    def close(self) -> None:
        """模拟关闭串口并记录关闭动作。"""

        self.is_open = False
        self.closed = True

    def reset_input_buffer(self) -> None:
        """记录输入缓冲区复位动作。"""

        self.input_reset = True

    def reset_output_buffer(self) -> None:
        """记录输出缓冲区复位动作。"""

        self.output_reset = True

    def read(self, size: int) -> bytes:
        """按请求长度返回固定测试数据。"""

        return b"abc"[:size]

    def write(self, data: bytes) -> int:
        """模拟完整写入并返回字节数。"""

        return len(data)


def test_transport_manages_connection_and_nonblocking_io(monkeypatch):
    """验证传输层管理控制线、缓冲区、读写和关闭状态。"""

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
