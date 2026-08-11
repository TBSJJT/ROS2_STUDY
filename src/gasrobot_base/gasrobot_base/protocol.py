"""STM32 底盘二进制协议的编码、解码与流式拆帧。"""

import math
from dataclasses import dataclass
from typing import List

from gasrobot_base.models import RawFeedback, VelocityCommand


# 控制帧与反馈帧共用帧头、帧尾，长度不同且方向固定。
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
COMMAND_FRAME_SIZE = 9
FEEDBACK_FRAME_SIZE = 23
YAW_CENTIDEGREES_PER_DEGREE = 100.0


class ProtocolError(ValueError):
    """表示收到的字节序列不符合底盘协议。"""


@dataclass(frozen=True)
class VelocityLimits:
    """发送到底盘前使用的速度限幅。"""

    linear_x: float
    linear_y: float
    angular_z: float


def _finite_or_zero(value: float) -> float:
    """将 NaN 和无穷值转换为零，防止异常指令进入下位机。"""

    return value if math.isfinite(value) else 0.0


def _clamp(value: float, limit: float) -> float:
    """先过滤非有限值，再按给定的正负对称范围限幅。"""

    value = _finite_or_zero(value)
    return max(-limit, min(limit, value))


def _read_i16_be(data: bytes, offset: int) -> int:
    """从指定偏移读取一个有符号大端 16 位整数。"""

    return int.from_bytes(
        data[offset:offset + 2],
        byteorder="big",
        signed=True,
    )


def _write_i16_be(buffer: bytearray, offset: int, value: int) -> None:
    """限幅后向指定偏移写入一个有符号大端 16 位整数。"""

    bounded = max(-32768, min(32767, int(value)))
    encoded = bounded.to_bytes(2, byteorder="big", signed=True)
    buffer[offset:offset + 2] = encoded


def decode_yaw_centidegrees(frame: bytes, offset: int) -> float:
    """将有符号大端0.01度航向解码为弧度。"""

    yaw_degrees = (
        _read_i16_be(frame, offset)
        / YAW_CENTIDEGREES_PER_DEGREE
    )
    return math.radians(yaw_degrees)


def encode_velocity_command(
    command: VelocityCommand,
    limits: VelocityLimits,
) -> bytes:
    """将 ROS 三轴速度编码为 9 字节 STM32 控制帧。"""

    linear_x = _clamp(command.linear_x, limits.linear_x)
    linear_y = _clamp(command.linear_y, limits.linear_y)
    angular_z = _clamp(command.angular_z, limits.angular_z)

    frame = bytearray(COMMAND_FRAME_SIZE)
    frame[0] = FRAME_HEADER

    # 线速度由 m/s 转为 mm/s，保留三位小数的协议精度。
    _write_i16_be(frame, 1, round(linear_x * 1000.0))
    _write_i16_be(frame, 3, round(linear_y * 1000.0))

    # 下位机旋转方向与 ROS 相反，协议边界统一完成符号转换。
    _write_i16_be(frame, 5, round(-angular_z * 1000.0))
    frame[7] = sum(frame[1:7]) & 0xFF
    frame[8] = FRAME_TAIL
    return bytes(frame)


def decode_feedback_frame(frame: bytes) -> RawFeedback:
    """校验并解码一帧 23 字节 STM32 反馈。"""

    if len(frame) != FEEDBACK_FRAME_SIZE:
        raise ProtocolError("反馈帧长度不正确")
    if frame[0] != FRAME_HEADER:
        raise ProtocolError("反馈帧头不正确")
    if frame[-1] != FRAME_TAIL:
        raise ProtocolError("反馈帧尾不正确")
    if frame[21] != (sum(frame[1:21]) & 0xFF):
        raise ProtocolError("反馈帧校验和不正确")

    # 只有完成长度、边界字节和校验和检查后才进行字段解码。
    return RawFeedback(
        linear_x=_read_i16_be(frame, 1) / 1000.0,
        linear_y=_read_i16_be(frame, 3) / 1000.0,
        angular_z=-_read_i16_be(frame, 5) / 1000.0,
        yaw=decode_yaw_centidegrees(frame, 19),
        acceleration_raw=(
            _read_i16_be(frame, 7),
            _read_i16_be(frame, 9),
            _read_i16_be(frame, 11),
        ),
        gyroscope_raw=(
            _read_i16_be(frame, 13),
            _read_i16_be(frame, 15),
            _read_i16_be(frame, 17),
        ),
    )


class FeedbackStreamParser:
    """从任意分片的串口字节流中恢复完整反馈帧。"""

    def __init__(self, max_buffer_size: int = 2048) -> None:
        """创建带有最大缓存保护的流式解析器。"""

        # 缓存跨越多次串口读取的残帧，避免假设一次读取就是一帧。
        self._buffer = bytearray()
        self.max_buffer_size = max_buffer_size
        self.bad_frame_count = 0

    def clear(self) -> None:
        """清空尚未组成完整帧的缓存。"""

        self._buffer.clear()

    def feed(self, data: bytes) -> List[RawFeedback]:
        """追加串口数据并返回本次解析出的全部反馈。"""

        if data:
            self._buffer.extend(data)

        feedback_items = []
        header = bytes((FRAME_HEADER,))

        while self._buffer:
            # 丢弃帧头之前的线路噪声，并把本次重同步计入坏帧统计。
            header_index = self._buffer.find(header)
            if header_index < 0:
                self.bad_frame_count += 1
                self._buffer.clear()
                break

            if header_index > 0:
                del self._buffer[:header_index]
                self.bad_frame_count += 1

            if len(self._buffer) < FEEDBACK_FRAME_SIZE:
                break

            frame = bytes(self._buffer[:FEEDBACK_FRAME_SIZE])
            try:
                feedback = decode_feedback_frame(frame)
            except ProtocolError:
                # 校验失败时只滑动一个字节，继续搜索下一处可能的帧头。
                del self._buffer[0]
                self.bad_frame_count += 1
                continue

            del self._buffer[:FEEDBACK_FRAME_SIZE]
            feedback_items.append(feedback)

        if len(self._buffer) > self.max_buffer_size:
            # 长时间无法成帧时主动释放缓存，防止异常数据无限占用内存。
            self._buffer.clear()
            self.bad_frame_count += 1

        return feedback_items
