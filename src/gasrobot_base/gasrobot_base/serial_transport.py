"""与 ROS 无关的 pyserial 连接生命周期封装。"""

import time
from typing import Optional

import serial
from serial import SerialException


class SerialTransport:
    """负责串口打开、关闭和非阻塞读写。"""

    def __init__(
        self,
        port: str,
        baud: int,
        startup_delay: float,
    ) -> None:
        """保存串口连接参数，但暂不访问硬件。"""

        # 这里只保存连接参数，构造对象时不立即访问硬件。
        self.port = port
        self.baud = baud
        self.startup_delay = startup_delay
        self._serial: Optional[serial.Serial] = None

    @property
    def is_open(self) -> bool:
        """返回串口是否已经成功打开。"""

        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        """按底盘所需串口参数建立连接并清空缓冲区。"""

        self.close()
        connection = serial.Serial(
            port=None,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.0,
            write_timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        connection.port = self.port

        try:
            # 某些 USB 转串口会用 DTR/RTS 复位下位机，因此打开前后均关闭。
            self._disable_control_lines(connection)
            connection.open()
            self._disable_control_lines(connection)
            time.sleep(max(0.0, self.startup_delay))
            connection.reset_input_buffer()
            connection.reset_output_buffer()
        except (OSError, SerialException):
            # 连接初始化失败时关闭半打开句柄，再把原异常交给节点处理。
            if connection.is_open:
                connection.close()
            raise

        self._serial = connection

    @staticmethod
    def _disable_control_lines(connection: serial.Serial) -> None:
        """尽力关闭 DTR/RTS；不支持控制线的串口设备可以继续使用。"""

        try:
            connection.dtr = False
            connection.rts = False
        except (OSError, SerialException):
            # 关闭阶段不传播异常，保证节点退出流程能够继续执行。
            pass

    def close(self) -> None:
        """关闭串口并释放对象。"""

        connection = self._serial
        self._serial = None
        if connection is None:
            return
        try:
            if connection.is_open:
                connection.close()
        except (OSError, SerialException):
            pass

    def read_available(self) -> bytes:
        """非阻塞读取当前缓冲区中的全部字节。"""

        if not self.is_open:
            return b""

        # timeout=0 保证该读取不会阻塞 ROS 执行器线程。
        waiting = self._serial.in_waiting
        return self._serial.read(waiting) if waiting > 0 else b""

    def write(self, data: bytes) -> int:
        """写入完整协议帧并返回实际写入字节数。"""

        if not self.is_open:
            return 0
        return self._serial.write(data)
