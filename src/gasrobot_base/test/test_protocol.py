#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 底盘二进制协议的单元测试。

本测试文件验证底盘串口通信协议的编码和解码功能：
1. encode_velocity_command()：ROS 速度指令 → 9 字节二进制控制帧
2. decode_feedback_frame()：23 字节二进制反馈帧 → RawFeedback 数据对象
3. FeedbackStreamParser：从连续字节流中提取完整反馈帧

协议帧格式（参考 protocol.py）：
  控制帧（上位机→下位机）：9 字节
    字节0：帧头 0x7B
    字节1-2：X 线速度（mm/s，有符号 16 位大端）
    字节3-4：Y 线速度（mm/s，有符号 16 位大端）
    字节5-6：Z 角速度（mm/s，有符号 16 位大端，ROS 方向→下位机方向取反）
    字节7：校验和（字节1-6 和的低 8 位）
    字节8：帧尾 0x7D

  反馈帧（下位机→上位机）：23 字节
    字节0：帧头 0x7B
    字节1-2：X 线速度（有符号 16 位大端）
    字节3-4：Y 线速度（有符号 16 位大端）
    字节5-6：Z 角速度     ...
    字节7-8：加速度 X 原始值
    字节9-10：加速度 Y 原始值
    字节11-12：加速度 Z 原始值
    字节13-14：陀螺仪 X 原始值
    字节15-16：陀螺仪 Y 原始值
    字节17-18：陀螺仪 Z 原始值
    字节19-20：航向（0.01 度单位，有符号 16 位大端）
    字节21：校验和
    字节22：帧尾 0x7D
"""

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


# -----------------------------------------------------------------------
# 辅助函数
# -----------------------------------------------------------------------
def _write_i16(frame: bytearray, offset: int, value: int) -> None:
    """
    在测试帧的指定位置写入一个有符号大端 16 位整数。

    这个函数和 protocol.py 中的 _write_i16_be 功能相同，
    但在测试文件中独立定义，以便测试能够自己构造帧数据。

    大端序（Big-endian）也叫"网络字节序"：
    高位字节存储在低地址，低位字节存储在高地址。
    例如数值 0x1234：
        大端序：字节 [0x12, 0x34]
        小端序：字节 [0x34, 0x12]
    STM32 使用大端序。

    参数:
        frame:  要修改的字节数组
        offset: 写入的起始位置（字节索引）
        value:  要写入的 16 位有符号整数（-32768 ~ 32767）

    """
    # value.to_bytes(2, byteorder="big", signed=True)
    # 把整数转为 2 字节的 bytes 对象
    # byteorder="big"：使用大端序
    # signed=True：允许负数（使用补码表示）
    # frame[offset:offset + 2] = ...：把 2 个字节写入数组的指定位置
    frame[offset:offset + 2] = value.to_bytes(
        2,
        byteorder="big",
        signed=True,
    )


def _feedback_frame(
    yaw_centidegrees: int = 9000,
    checksum_delta: int = 0,
) -> bytes:
    """
    构造一帧可调航向和校验偏差的 23 字节测试反馈帧。

    默认构造的反馈帧字段值：
    - X 线速度: 1200 → 1.2 m/s
    - Y 线速度: -350 → -0.35 m/s
    - Z 角速度: -500 → 0.5 m/s（ROS 方向：下位机值取反）
    - 加速度: (4096, 0, -4096) → (+1g, 0g, -1g)
    - 陀螺仪: (131, -262, 393) → (+1 dps, -2 dps, +3 dps)
    - 航向: 默认 9000 厘度 = 90.00° = π/2 弧度

    参数:
        yaw_centidegrees: 航向值，单位为 0.01 度（厘度）
                          例如 9000 = 90.00 度，18000 = 180.00 度
        checksum_delta:   校验和的偏移量
                          0 = 正确的校验和
                          != 0 → 故意的校验错误，用于测试错误检测

    返回:
        23 字节的反馈帧

    """
    # 创建 23 字节的字节数组（mutable bytes）
    frame = bytearray(23)

    # 字节 0：帧头 0x7B
    frame[0] = 0x7B

    # 字节 1-18：写入 9 个有符号 16 位整数字段
    # zip(range(1, 19, 2), (...))：
    #   range(1, 19, 2) 产生 [1, 3, 5, 7, 9, 11, 13, 15, 17]
    #   即每个 16 位字段的起始偏移（每 2 字节一跳）
    # zip 把偏移和值配对：(1, 1200), (3, -350), (5, -500), ...
    for offset, value in zip(
        range(1, 19, 2),
        (1200, -350, -500,    # 三轴速度 (mm/s)
         4096, 0, -4096,      # 三轴加速度 (LSB)
         131, -262, 393),     # 三轴陀螺仪 (LSB)
    ):
        _write_i16(frame, offset, value)

    # 字节 19-20：航向值（厘度）
    _write_i16(frame, 19, yaw_centidegrees)

    # 字节 21：校验和 = (字节 1 到 20 所有字节之和) 取低 8 位
    # 加上 checksum_delta 偏移量，用于测试错误检测
    frame[21] = (sum(frame[1:21]) + checksum_delta) & 0xFF

    # 字节 22：帧尾 0x7D
    frame[22] = 0x7D

    # bytes(frame)：把可变的 bytearray 转为不可变的 bytes
    return bytes(frame)


# -----------------------------------------------------------------------
# 测试 1：控制帧编码
# -----------------------------------------------------------------------
def test_encode_velocity_command_applies_limits_and_ros_sign():
    """
    验证控制帧的速度限幅、单位换算和旋转方向转换。

    编码过程：
    1. 限幅（clamping）：确保速度不超过硬件/协议允许的最大值
    2. 单位换算：m/s → mm/s（乘以 1000，保留 3 位小数精度）
    3. 旋转方向转换：ROS 的逆时针为正 → 下位机的约定方向（取反）

    测试场景：
    - 输入：vx=2.0, vy=-0.25, wz=0.75 (m/s 和 rad/s)
    - 限幅：(1.5, 1.0, 1.0)
    - vx 被限幅到 1.5
    - 编码后：1500 mm/s, -250 mm/s, -750 mm/s（注意 wz 取反）

    """
    # 创建一个速度指令和一个限幅配置
    # 注意 linear_x=2.0 超过了限幅 1.5，会被截断
    frame = encode_velocity_command(
        VelocityCommand(2.0, -0.25, 0.75),
        VelocityLimits(1.5, 1.0, 1.0),
    )

    # 断言帧头和帧尾
    assert frame[0] == 0x7B   # 帧头
    assert frame[-1] == 0x7D  # 帧尾（frame[8]）

    # 断言三轴速度编码
    # int.from_bytes(frame[1:3], "big", signed=True)：
    #   从 frame 的第 1-2 字节（左闭右开）解码有符号大端 16 位整数
    # vx=1.5 m/s → 1500 mm/s（限幅生效）
    assert int.from_bytes(frame[1:3], "big", signed=True) == 1500
    # vy=-0.25 m/s → -250 mm/s
    assert int.from_bytes(frame[3:5], "big", signed=True) == -250
    # wz=0.75 rad/s → 取反 → -750（ROS→下位机方向转换）
    assert int.from_bytes(frame[5:7], "big", signed=True) == -750

    # 断言校验和：字节 1-6 所有值相加后取低 8 位
    assert frame[7] == sum(frame[1:7]) & 0xFF


# -----------------------------------------------------------------------
# 测试 2：NaN/Inf 过滤
# -----------------------------------------------------------------------
def test_encode_velocity_command_filters_non_finite_values():
    """
    验证非有限速度值（NaN、Inf）不会被发送到底盘。

    非有限值说明：
    - NaN (Not a Number)：不是数字，如 0/0 的结果
    - Inf (Infinity)：无穷大，如 1/0 的结果
    - 这些值可能来自 ROS 消息的未初始化字段或计算错误

    如果 NaN/Inf 被编码为二进制帧发送到底盘：
    - 可能被误解为巨大的速度值 → 底盘失控
    - 可能触发下位机故障保护 → 底盘突然停机

    所以编码函数必须过滤这些值，将其转为 0（安全值）。

    测试通过传入 NaN、Inf、-Inf 三个非有限值来验证过滤逻辑。

    """
    # math.nan：Not a Number（不是数字）
    # math.inf：正无穷大
    # -math.inf：负无穷大
    frame = encode_velocity_command(
        VelocityCommand(math.nan, math.inf, -math.inf),
        VelocityLimits(1.0, 1.0, 1.0),
    )

    # 字节 1 到 6 应该全部为 0（所有非有限值被过滤为零）
    # bytes(6) 创建一个 6 字节的全零 bytes 对象
    assert frame[1:7] == bytes(6)


# -----------------------------------------------------------------------
# 测试 3：反馈帧解码
# -----------------------------------------------------------------------
def test_decode_feedback_frame_converts_units_and_sign():
    """
    验证反馈帧各字段的单位转换、符号修正和原始值提取。

    解码过程（与编码互为逆过程，但多了 IMU 原始数据字段）：
    1. 帧头/帧尾/校验和验证
    2. 速度：mm/s → m/s（除以 1000），角速度取反
    3. 航向：厘度 → 度 → 弧度
    4. IMU 数据：保留原始 LSB 计数，不做转换

    """
    # 解码默认参数构造的反馈帧
    feedback = decode_feedback_frame(_feedback_frame())

    # --- 断言底盘速度 ---
    # X 线速度：1200 → 1.2 m/s
    assert feedback.linear_x == pytest.approx(1.2)
    # Y 线速度：-350 → -0.35 m/s
    assert feedback.linear_y == pytest.approx(-0.35)
    # Z 角速度：-500 → 取反 → 0.5 rad/s
    assert feedback.angular_z == pytest.approx(0.5)

    # --- 断言航向 ---
    # 9000 厘度 = 90.00 度 = π/2 弧度
    assert feedback.yaw == pytest.approx(math.pi / 2.0)

    # --- 断言 IMU 原始值（解码后保留为原始 LSB 计数）---
    assert feedback.acceleration_raw == (4096, 0, -4096)
    assert feedback.gyroscope_raw == (131, -262, 393)


# -----------------------------------------------------------------------
# 测试 4：全周期航向解码
# -----------------------------------------------------------------------
@pytest.mark.parametrize(
    ("yaw_centidegrees", "expected"),
    (
        # 测试不同的航向值及其期望的弧度结果
        (0, 0.0),                                    # 0 度
        (9000, math.pi / 2.0),                       # 90 度
        (18000, math.pi),                            # 180 度
        (-9000, -math.pi / 2.0),                     # -90 度
        (-17999, math.radians(-179.99)),             # -179.99 度（接近 -π）
    ),
)
def test_decode_feedback_frame_decodes_full_circle_yaw(
    yaw_centidegrees: int,
    expected: float,
):
    """
    验证双字节航向在整个 360 度范围内的关键位置解码正确。

    使用 @pytest.mark.parametrize 参数化测试：
    同一个测试会运行 5 次，每次使用不同的 (yaw_centidegrees, expected) 组合。
    覆盖了 0°、90°、180°、-90° 和边界值 -179.99°。

    航向编码使用 16 位有符号整数表示 0.01 度单位（厘度）：
    - 范围：-327.68° ~ +327.67°
    - 精度：0.01°
    - 完全覆盖 360 度全周期

    参数:
        yaw_centidegrees: 航向的厘度值（0.01 度单位）
        expected:         期望解码出的弧度值

    """
    # 用指定航向构造反馈帧并解码
    feedback = decode_feedback_frame(
        _feedback_frame(yaw_centidegrees=yaw_centidegrees)
    )

    # 断言解码的航向弧度值与期望一致
    assert feedback.yaw == pytest.approx(expected)


# -----------------------------------------------------------------------
# 测试 5：校验和错误被拒绝
# -----------------------------------------------------------------------
def test_decode_feedback_frame_rejects_bad_checksum():
    """
    验证校验和错误的反馈帧会被 ProtocolError 异常拒绝。

    校验和的作用：
    串口通信中数据可能因电磁干扰、接触不良等原因损坏。
    校验和是一种简单的错误检测机制：
    - 发送方计算数据字节的和，作为校验字节附加在帧中
    - 接收方重新计算和，与收到的校验字节比对
    - 如果不匹配，说明数据在传输中损坏了，应该丢弃这帧

    这虽然不能纠正错误（需要更复杂的纠错码），
    但可以防止损坏的数据被当作正确数据使用。

    测试通过 checksum_delta=1 故意让校验和不正确。

    """
    # pytest.raises(ProtocolError, match="校验和")：
    #   期望 with 块中的代码抛出 ProtocolError 异常
    #   且异常消息中包含"校验和"关键字
    with pytest.raises(ProtocolError, match="校验和"):
        # checksum_delta=1 使校验和偏移 1，与实际计算值不匹配
        decode_feedback_frame(_feedback_frame(checksum_delta=1))


# -----------------------------------------------------------------------
# 测试 6：流式解析器
# -----------------------------------------------------------------------
def test_stream_parser_handles_noise_fragmentation_and_recovery():
    """
    验证 FeedbackStreamParser 能处理线路噪声、数据分片、坏帧并恢复同步。

    串口通信的挑战：
    1. 分片到达：一次 read() 不一定刚好读到完整的一帧
       - 可能只读到半帧，下次再读到另一半
    2. 线路噪声：字节流中可能出现不在帧内的随机字节
    3. 数据损坏：校验和错误导致某些帧不可用
    4. 多帧粘连：一次读取可能包含好几个完整的帧

    FeedbackStreamParser 需要应对以上所有情况：
    - 缓存不完整的帧片段，等待更多数据
    - 跳过噪声字节（丢弃帧头之前的内容）
    - 对校验失败的帧，逐字节滑动查找下一个帧头
    - 一次返回所有解析出的完整帧

    测试场景：
    1. 喂入噪声 + 不完整的帧前 7 字节 → 应该返回空列表（等完整帧）
    2. 继续喂入剩余字节 + 一个坏帧 + 一个好的完整帧
       → 应该返回 2 个帧（丢弃坏帧）
    3. 检查 bad_frame_count 是否正确统计

    """
    parser = FeedbackStreamParser()
    # 一个完整的好帧
    good_frame = _feedback_frame()
    # 一个校验和不对的坏帧
    bad_frame = _feedback_frame(checksum_delta=1)

    # --- 步骤 1：喂入噪声 + 半帧 ---
    # b"\x00\x01"：2 字节噪声（不是帧头 0x7B）
    # good_frame[:7]：好帧的前 7 字节（不完整，长度不够 23）
    # 期望返回空列表，因为没有完整的帧可用
    assert parser.feed(b"\x00\x01" + good_frame[:7]) == []

    # --- 步骤 2：喂入剩余数据 ---
    # good_frame[7:]：好帧的剩余 16 字节（7+16=23，完整）
    # bad_frame：一个完整的坏帧（会被丢弃）
    # good_frame：另一帧完整的好帧
    first_batch = parser.feed(good_frame[7:] + bad_frame + good_frame)

    # 期望返回 2 个好帧（坏帧在校验失败后被丢弃）
    assert len(first_batch) == 2

    # 断言第 1 帧的 X 线速度正确
    assert first_batch[0].linear_x == pytest.approx(1.2)
    # 断言第 2 帧的陀螺仪 Z 轴原始值正确
    assert first_batch[1].gyroscope_raw[2] == 393

    # 断言坏帧计数至少为 2：
    # - 开头 2 字节噪声被丢弃 → 至少 +1
    # - 或者中间那个坏帧 → +1
    assert parser.bad_frame_count >= 2
