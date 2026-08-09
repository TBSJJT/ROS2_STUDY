"""STM32 底盘二进制协议的编码、解码与流式拆帧。"""

import math
from dataclasses import dataclass
from typing import List

from gasrobot_base.models import RawFeedback, VelocityCommand


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
COMMAND_FRAME_SIZE = 9
FEEDBACK_FRAME_SIZE = 21


class ProtocolError(ValueError):
    """表示收到的字节序列不符合底盘协议。"""


@dataclass(frozen=True)
class VelocityLimits:
    """发送到底盘前使用的速度限幅。"""

    linear_x: float
    linear_y: float
    angular_z: float


def _finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def _clamp(value: float, limit: float) -> float:
    value = _finite_or_zero(value)
    return max(-limit, min(limit, value))


def _read_i16_be(data: bytes, offset: int) -> int:
    return int.from_bytes(
        data[offset:offset + 2],
        byteorder="big",
        signed=True,
    )


def _write_i16_be(buffer: bytearray, offset: int, value: int) -> None:
    bounded = max(-32768, min(32767, int(value)))
    encoded = bounded.to_bytes(2, byteorder="big", signed=True)
    buffer[offset:offset + 2] = encoded


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
    _write_i16_be(frame, 1, round(linear_x * 1000.0))
    _write_i16_be(frame, 3, round(linear_y * 1000.0))

    # 下位机旋转方向与 ROS 相反，协议边界统一完成符号转换。
    _write_i16_be(frame, 5, round(-angular_z * 1000.0))
    frame[7] = sum(frame[1:7]) & 0xFF
    frame[8] = FRAME_TAIL
    return bytes(frame)


def decode_feedback_frame(frame: bytes) -> RawFeedback:
    """校验并解码一帧 21 字节 STM32 反馈。"""

    if len(frame) != FEEDBACK_FRAME_SIZE:
        raise ProtocolError("反馈帧长度不正确")
    if frame[0] != FRAME_HEADER:
        raise ProtocolError("反馈帧头不正确")
    if frame[-1] != FRAME_TAIL:
        raise ProtocolError("反馈帧尾不正确")
    if frame[19] != (sum(frame[1:19]) & 0xFF):
        raise ProtocolError("反馈帧校验和不正确")

    return RawFeedback(
        linear_x=_read_i16_be(frame, 1) / 1000.0,
        linear_y=_read_i16_be(frame, 3) / 1000.0,
        angular_z=-_read_i16_be(frame, 5) / 1000.0,
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
                del self._buffer[0]
                self.bad_frame_count += 1
                continue

            del self._buffer[:FEEDBACK_FRAME_SIZE]
            feedback_items.append(feedback)

        if len(self._buffer) > self.max_buffer_size:
            self._buffer.clear()
            self.bad_frame_count += 1

        return feedback_items
