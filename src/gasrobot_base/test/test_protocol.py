"""STM32 底盘协议单元测试。"""

import math

import pytest

from gasrobot_base.models import VelocityCommand
from gasrobot_base.protocol import (
    FeedbackStreamParser,
    ProtocolError,
    VelocityLimits,
    decode_feedback_frame,
    encode_velocity_command,
)


def _write_i16(frame: bytearray, offset: int, value: int) -> None:
    """在测试帧的指定位置写入有符号大端 16 位整数。"""

    frame[offset:offset + 2] = value.to_bytes(
        2,
        byteorder="big",
        signed=True,
    )


def _feedback_frame(
    yaw_byte: int = 64,
    checksum_delta: int = 0,
) -> bytes:
    """构造一帧可调航向和校验偏差的 22 字节反馈。"""

    frame = bytearray(22)
    frame[0] = 0x7B
    for offset, value in zip(
        range(1, 19, 2),
        (1200, -350, -500, 4096, 0, -4096, 131, -262, 393),
    ):
        _write_i16(frame, offset, value)
    frame[19] = yaw_byte
    frame[20] = (sum(frame[1:20]) + checksum_delta) & 0xFF
    frame[21] = 0x7D
    return bytes(frame)


def test_encode_velocity_command_applies_limits_and_ros_sign():
    """验证控制帧限幅、单位换算和旋转方向转换。"""

    frame = encode_velocity_command(
        VelocityCommand(2.0, -0.25, 0.75),
        VelocityLimits(1.5, 1.0, 1.0),
    )

    assert frame[0] == 0x7B
    assert frame[-1] == 0x7D
    assert int.from_bytes(frame[1:3], "big", signed=True) == 1500
    assert int.from_bytes(frame[3:5], "big", signed=True) == -250
    assert int.from_bytes(frame[5:7], "big", signed=True) == -750
    assert frame[7] == sum(frame[1:7]) & 0xFF


def test_encode_velocity_command_filters_non_finite_values():
    """验证非有限速度不会被发送到底盘。"""

    frame = encode_velocity_command(
        VelocityCommand(math.nan, math.inf, -math.inf),
        VelocityLimits(1.0, 1.0, 1.0),
    )

    assert frame[1:7] == bytes(6)


def test_decode_feedback_frame_converts_units_and_sign():
    """验证反馈帧各字段的单位、符号和原始计数。"""

    feedback = decode_feedback_frame(_feedback_frame())

    assert feedback.linear_x == pytest.approx(1.2)
    assert feedback.linear_y == pytest.approx(-0.35)
    assert feedback.angular_z == pytest.approx(0.5)
    assert feedback.yaw == pytest.approx(math.pi / 2.0)
    assert feedback.acceleration_raw == (4096, 0, -4096)
    assert feedback.gyroscope_raw == (131, -262, 393)


@pytest.mark.parametrize(
    ("yaw_byte", "expected"),
    (
        (0, 0.0),
        (64, math.pi / 2.0),
        (128, math.pi),
        (192, -math.pi / 2.0),
        (255, -2.0 * math.pi / 256.0),
    ),
)
def test_decode_feedback_frame_decodes_full_circle_yaw(
    yaw_byte: int,
    expected: float,
):
    """验证单字节航向在整周关键位置的解码结果。"""

    feedback = decode_feedback_frame(_feedback_frame(yaw_byte=yaw_byte))

    assert feedback.yaw == pytest.approx(expected)


def test_decode_feedback_frame_rejects_bad_checksum():
    """验证校验和错误的反馈帧会被拒绝。"""

    with pytest.raises(ProtocolError, match="校验和"):
        decode_feedback_frame(_feedback_frame(checksum_delta=1))


def test_stream_parser_handles_noise_fragmentation_and_recovery():
    """验证解析器可处理噪声、分片、坏帧并恢复同步。"""

    parser = FeedbackStreamParser()
    good_frame = _feedback_frame()
    bad_frame = _feedback_frame(checksum_delta=1)

    assert parser.feed(b"\x00\x01" + good_frame[:7]) == []
    first_batch = parser.feed(good_frame[7:] + bad_frame + good_frame)

    assert len(first_batch) == 2
    assert first_batch[0].linear_x == pytest.approx(1.2)
    assert first_batch[1].gyroscope_raw[2] == 393
    assert parser.bad_frame_count >= 2
